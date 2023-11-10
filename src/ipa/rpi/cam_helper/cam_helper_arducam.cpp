/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * cam_helper_Arducam.cpp - camera information for Arducam sensor
 */

#include <assert.h>

#include "cam_helper.h"
#include "arducam/arducam_pivariety.hpp"

using namespace RPiController;

class CamHelperArducam : public CamHelper
{
public:
	CamHelperArducam();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gain_code) const override;
	void getDelays(int &exposureDelay, int &gainDelay,
			       int &vblankDelay, int &hblankDelay) const override;
	unsigned int hideFramesModeSwitch() const override;
	unsigned int mistrustFramesStartup() const override;
	unsigned int mistrustFramesModeSwitch() const override;
	std::string getTuningData() override;
	void setI2C(int16_t i2c_bus, int16_t i2c_addr) override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 4;
	int exposure_delay_ = 2;
	int gain_delay_ = 2;
	int vblank_delay_ = 2;
	int hide_frames_mode_switch_ = 2;
	std::string tuning_data = std::string();
};

/*
 * Arducam doesn't output metadata, so we have to use the "unicam parser" which
 * works by counting frames.
 */

CamHelperArducam::CamHelperArducam()
	: CamHelper(nullptr, frameIntegrationDiff)
{
}

void CamHelperArducam::setI2C(int16_t i2c_bus, int16_t i2c_addr) {
	CamHelper::setI2C(i2c_bus, i2c_addr);
	int bus = 10;

	if (i2c_bus_ != -1) {
		bus = i2c_bus_;
	}

	Arducam::ArducamUtils arducam(bus);
	tuning_data = arducam.readJson();

	if (!arducam.convert(tuning_data)){
		printf("old firmware data.\n");
		arducam.writeJson(tuning_data);
	}

	exposure_delay_ = DEFAULT(arducam.readInfo(Arducam::EXPOSURE_DELAY), 2);
	gain_delay_ =  DEFAULT(arducam.readInfo(Arducam::GAIN_DELAY), 2);
	vblank_delay_ =  DEFAULT(arducam.readInfo(Arducam::VBLANK_DELAY), 2);
	uint32_t version = arducam.getVersion();
	if (version != UNKNOWN
		&& (version >> 16) >= 1
		&& (version & 0xFFFF) > 1) {
			hide_frames_mode_switch_ =
				DEFAULT(arducam.readInfo(Arducam::HIDE_FRAMES), 2);
	}
}

uint32_t CamHelperArducam::gainCode(double gain) const
{
	return static_cast<uint32_t>(gain * 100);
}

double CamHelperArducam::gain(uint32_t gain_code) const
{
	return static_cast<double>(gain_code / 100.0);
}

void CamHelperArducam::getDelays(int &exposureDelay, int &gainDelay,
			       int &vblankDelay, int &hblankDelay) const
{
	exposureDelay = 2;
	gainDelay = 2;
	vblankDelay = 3;
	hblankDelay = 3;
}

unsigned int CamHelperArducam::hideFramesModeSwitch() const
{
	/*
	 * After a mode switch, we get a couple of under-exposed frames which
	 * we don't want shown.
	 */
	return hide_frames_mode_switch_;
}

unsigned int CamHelperArducam::mistrustFramesStartup() const
{
	/*
	 * First couple of frames are under-exposed and are no good for control
	 * algos.
	 */
	return 2;
}

unsigned int CamHelperArducam::mistrustFramesModeSwitch() const
{
	/*
	 * First couple of frames are under-exposed even after a simple
	 * mode switch, and are no good for control algos.
	 */
	return 2;
}

std::string CamHelperArducam::getTuningData()
{
	if (target_ == "pisp") {
		Arducam::ArducamUtils::convertPISP(tuning_data);
	}
	return tuning_data;
}

static CamHelper *create()
{
	return new CamHelperArducam();
}

static RegisterCamHelper reg("arducam-pivariety", &create);
static RegisterCamHelper regAlias("arducam_pivariety", &create);