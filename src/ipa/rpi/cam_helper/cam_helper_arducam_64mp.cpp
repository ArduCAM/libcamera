/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Based on cam_helper_imx477.cpp
 * Copyright (C) 2020, Raspberry Pi (Trading) Limited
 *
 * cam_helper_arducam_64mp.cpp - camera helper for arducam_64mp sensor
 * Copyright (C) 2021, Arducam Technology co., Ltd.
 */

#include <assert.h>
#include <cmath>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include <libcamera/base/log.h>
#include <controller/pdaf_data.h>

#include "cam_helper.h"
#include "md_parser.h"

#define ALIGN_UP(x,a)    (((x)+(a)-1)&~(a-1))

using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;

namespace libcamera {
LOG_DECLARE_CATEGORY(IPARPI)
}

/*
 * We care about two gain registers and a pair of exposure registers. Their
 * I2C addresses from the Sony IMX519 datasheet:
 */
constexpr uint32_t expHiReg = 0x0202;
constexpr uint32_t expLoReg = 0x0203;
constexpr uint32_t gainHiReg = 0x0204;
constexpr uint32_t gainLoReg = 0x0205;
constexpr uint32_t frameLengthHiReg = 0x0340;
constexpr uint32_t frameLengthLoReg = 0x0341;
constexpr uint32_t lineLengthHiReg = 0x0342;
constexpr uint32_t lineLengthLoReg = 0x0343;
constexpr std::initializer_list<uint32_t> registerList =
	{ expHiReg, expLoReg, gainHiReg, gainLoReg, frameLengthHiReg, frameLengthLoReg, 
		lineLengthHiReg, lineLengthLoReg };

class CamHelperArducam64MP : public CamHelper
{
public:
	CamHelperArducam64MP();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gain_code) const override;
	void prepare(libcamera::Span<const uint8_t> buffer, Metadata &metadata) override;
	std::pair<uint32_t, uint32_t> getBlanking(libcamera::utils::Duration &exposure,
							  libcamera::utils::Duration minFrameDuration,
							  libcamera::utils::Duration maxFrameDuration) const override;
	void getDelays(int &exposureDelay, int &gainDelay,
			       int &vblankDelay, int &hblankDelay) const override;
	bool sensorEmbeddedDataPresent() const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 48;
	/* Maximum frame length allowable for long exposure calculations. */
	static constexpr int frameLengthMax = 0xffff;
	/* Largest long exposure scale factor given as a left shift on the frame length. */
	static constexpr int longExposureShiftMax = 7;

	static constexpr int pdafStatsRows = 12;
	static constexpr int pdafStatsCols = 16;

	void populateMetadata(const MdParser::RegisterMap &registers,
			      Metadata &metadata) const override;
	static bool parsePdafData(const uint8_t *ptr, size_t len, unsigned bpp,
				  PdafRegions &pdaf);
};

CamHelperArducam64MP::CamHelperArducam64MP()
	: CamHelper(std::make_unique<MdParserSmia>(registerList), frameIntegrationDiff)
{
}

uint32_t CamHelperArducam64MP::gainCode(double gain) const
{
	return static_cast<uint32_t>(1024 - 1024 / gain);
}

double CamHelperArducam64MP::gain(uint32_t gain_code) const
{
	return 1024.0 / (1024 - gain_code);
}

void CamHelperArducam64MP::prepare(libcamera::Span<const uint8_t> buffer, Metadata &metadata)
{
	MdParser::RegisterMap registers;
	DeviceStatus deviceStatus;

	LOG(IPARPI, Debug) << "Embedded buffer size: " << buffer.size();

	size_t bytesPerLine = (mode_.width * mode_.bitdepth) >> 3;
	bytesPerLine = ALIGN_UP(bytesPerLine, 16);

	if (metadata.get("device.status", deviceStatus)) {
		LOG(IPARPI, Error) << "DeviceStatus not found from DelayedControls";
		return;
	}

	parseEmbeddedData(buffer, metadata);

	if (buffer.size() > 2 * bytesPerLine) {
		PdafRegions pdaf;
		if (parsePdafData(&buffer[2 * bytesPerLine],
				  buffer.size() - 2 * bytesPerLine,
				  mode_.bitdepth, pdaf))
			metadata.set("pdaf.regions", pdaf);
	}

	/*
	 * The DeviceStatus struct is first populated with values obtained from
	 * DelayedControls. If this reports frame length is > frameLengthMax,
	 * it means we are using a long exposure mode. Since the long exposure
	 * scale factor is not returned back through embedded data, we must rely
	 * on the existing exposure lines and frame length values returned by
	 * DelayedControls.
	 *
	 * Otherwise, all values are updated with what is reported in the
	 * embedded data.
	 */
	if (deviceStatus.frameLength > frameLengthMax) {
		DeviceStatus parsedDeviceStatus;

		metadata.get("device.status", parsedDeviceStatus);
		parsedDeviceStatus.shutterSpeed = deviceStatus.shutterSpeed;
		parsedDeviceStatus.frameLength = deviceStatus.frameLength;
		metadata.get("device.status", parsedDeviceStatus);

		LOG(IPARPI, Debug) << "Metadata updated for long exposure: "
				   << parsedDeviceStatus;
	}
}

std::pair<uint32_t, uint32_t> CamHelperArducam64MP::getBlanking(libcamera::utils::Duration &exposure,
							  libcamera::utils::Duration minFrameDuration,
							  libcamera::utils::Duration maxFrameDuration) const
{
	uint32_t frameLength, expLines;
	unsigned int shift = 0;

	auto [vblank, hblank] = CamHelper::getBlanking(exposure, minFrameDuration,
						       maxFrameDuration);

	frameLength = mode_.height + vblank;
	Duration lineLength = hblankToLineLength(hblank);
	/*
	 * Check if the frame length calculated needs to be setup for long
	 * exposure mode. This will require us to use a long exposure scale
	 * factor provided by a shift operation in the sensor.
	 */
	while (frameLength > frameLengthMax) {
		if (++shift > longExposureShiftMax) {
			shift = longExposureShiftMax;
			frameLength = frameLengthMax;
			break;
		}
		frameLength >>= 1;
	}

	if (shift) {
		/* Account for any rounding in the scaled frame length value. */
		frameLength <<= shift;
		expLines = exposureLines(exposure, lineLength);
		expLines = std::min(expLines, frameLength - frameIntegrationDiff);
		exposure = CamHelperArducam64MP::exposure(expLines, lineLength);
	}

	return { frameLength - mode_.height, hblank };
}

void CamHelperArducam64MP::getDelays(int &exposureDelay, int &gainDelay,
			       int &vblankDelay, int &hblankDelay) const
{
	exposureDelay = 2;
	gainDelay = 2;
	vblankDelay = 3;
	hblankDelay = 3;
}

bool CamHelperArducam64MP::sensorEmbeddedDataPresent() const
{
	return true;
}

void CamHelperArducam64MP::populateMetadata(const MdParser::RegisterMap &registers,
				       Metadata &metadata) const
{
	DeviceStatus deviceStatus;
	deviceStatus.lineLength = lineLengthPckToDuration(registers.at(lineLengthHiReg) * 256 +
							  registers.at(lineLengthLoReg));
	deviceStatus.shutterSpeed = exposure(registers.at(expHiReg) * 256 + registers.at(expLoReg), 
										deviceStatus.lineLength);
	deviceStatus.analogueGain = gain(registers.at(gainHiReg) * 256 + registers.at(gainLoReg));
	deviceStatus.frameLength = registers.at(frameLengthHiReg) * 256 + registers.at(frameLengthLoReg);

	metadata.set("device.status", deviceStatus);
}

bool CamHelperArducam64MP::parsePdafData(const uint8_t *ptr, size_t len,
				    unsigned bpp, PdafRegions &pdaf)
{
	size_t step = bpp >> 1; /* bytes per PDAF grid entry */

	if (bpp < 10 || bpp > 12 || len < 194 * step || ptr[0] != 0 || ptr[1] >= 0x40) {
		LOG(IPARPI, Error) << "PDAF data in unsupported format";
		return false;
	}

	pdaf.init({ pdafStatsCols, pdafStatsRows });

	ptr += 2 * step;
	for (unsigned i = 0; i < pdafStatsRows; ++i) {
		for (unsigned j = 0; j < pdafStatsCols; ++j) {
			unsigned c = (ptr[0] << 3) | (ptr[1] >> 5);
			int p = (((ptr[1] & 0x0F) - (ptr[1] & 0x10)) << 6) | (ptr[2] >> 2);
			PdafData pdafData;
			pdafData.conf = c;
			pdafData.phase = c ? p : 0;
			pdaf.set(libcamera::Point(j, i), { pdafData, 1, 0 });
			ptr += step;
		}
	}

	return true;
}

static CamHelper *create()
{
	return new CamHelperArducam64MP();
}

static RegisterCamHelper reg("arducam_64mp", &create);
