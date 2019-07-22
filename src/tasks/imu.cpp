/*
 * Copyright (c) 2018, Niklas Hauser
 *
 * This file is part of the Navimet project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "imu.hpp"
#include "../board.hpp"
#include <modm/processing/timer.hpp>
#include <modm/math/geometry/angle.hpp>
#include <modm/driver/inertial/bno055.hpp>
using namespace modm::literals;

#undef	MODM_LOG_LEVEL
#define	MODM_LOG_LEVEL modm::log::INFO

namespace
{
using Scl = Board::D5;
using Sda = Board::D4;
using Addr = Board::D3;
using Int = Board::D2;
using Master = I2cMaster1;
// using Master = BitBangI2cMaster<Scl, Sda>;

static constexpr uint8_t addr = 1;
modm::bno055::Data data;
modm::Bno055<Master> device(data, modm::bno055::addr(addr));
modm::ShortPeriodicTimer timer{1000};
}


namespace navimet
{

void
ImuTask::initialize()
{
	Master::connect<Sda::Sda, Scl::Scl>();
	// Master::connect<Sda::BitBang, Scl::BitBang>();
	Master::initialize<Board::SystemClock, 100_kHz>();

	Addr::setOutput(addr);
	Int::setInput(Int::InputType::PullUp);
	Int::setInputTrigger(Int::InputTrigger::FallingEdge);
	Int::enableExternalInterrupt();

	MODM_LOG_INFO << "IMU initialized." << modm::endl;
}

bool
ImuTask::update()
{
	PT_BEGIN();

	// Wait until BNO055 has finished booting
	PT_WAIT_UNTIL(timer.execute());
	timer.restart(33);

	if (PT_CALL(device.configure())) {
		MODM_LOG_INFO << "IMU configured." << modm::endl;
	} else {
		MODM_LOG_ERROR << "IMU configuring failed!" << modm::endl;
	}

	if (not PT_CALL(device.enableExternalClock())) {
		MODM_LOG_ERROR << "IMU external clock failed!!" << modm::endl;
	}

	static uint8_t device_id[3];
	if (PT_CALL(device.readRegister(device.Register::SW_REV_ID_LSB, device_id, 3))) {
		MODM_LOG_INFO << "IMU bootloader v" << device_id[2];
		MODM_LOG_INFO << " firmware v0." << device_id[1] << ".0." << device_id[0] << modm::endl;
	}


	while(1)
	{
		PT_WAIT_UNTIL(timer.execute());

		PT_CALL(device.readData());
		{
			m_heading = modm::toRadian(data.heading());
#if 0
			static modm::Timeout dbgtmr{200};
			if (dbgtmr.isExpired()) {
				MODM_LOG_DEBUG.printf("%lu ms: %d\n", modm::Clock::now().getTime(),
			    	                  int(modm::toDegree(m_heading)));
				dbgtmr.restart(200);
			}
#endif
		}

		PT_YIELD();
	}

	PT_END();
}

}
