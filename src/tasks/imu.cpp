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
#include <modm/board.hpp>
#include <invensense/modm/driver/mpu_device.hpp>

#undef	MODM_LOG_LEVEL
#define	MODM_LOG_LEVEL modm::log::INFO

namespace imu
{
using Int = Board::D12;
using Scl = Board::D15;
using Sda = Board::D14;
using Master = I2cMaster1;

using Device = modm::MpuDevice<I2cMaster1, Int>;
}

MODM_ISR(EXTI9_5)
{
	if (imu::Int::getExternalInterruptFlag())
	{
		imu::Device::interrupt();
		imu::Int::acknowledgeExternalInterruptFlag();
	}
}

namespace navimet
{

void
ImuTask::initialize()
{
	imu::Master::connect<imu::Sda::Sda, imu::Scl::Scl>();
	imu::Master::initialize<Board::systemClock, imu::Master::Baudrate::Fast>();

	// invensense::emd::set_uart_handler([](uint8_t c)
	// {
	// 	Board::stlink::Uart::write(c);
	// 	return true;
	// });

	imu::Device::configure();

	int result = (~imu::Device::calibrate()) & 0b111;
	if (result & 0b001) {
		MODM_LOG_ERROR << "Gyro failed." << modm::endl;
	}
	if (result & 0b010) {
		MODM_LOG_ERROR << "Accel failed." << modm::endl;
	}
	if (result & 0b100) {
		MODM_LOG_ERROR << "Compass failed." << modm::endl;
	}

	imu::Device::setSampleRate(100);

	MODM_LOG_INFO << "IMU task initialized." << modm::endl;
}

bool
ImuTask::update()
{
	PT_BEGIN();

	while(1)
	{
		imu::Device::update();

		if (imu::Device::execute())
		{
			imu::Device::Quaternion quad = imu::Device::getQuaternion();
			MODM_LOG_DEBUG.printf("%lu ms, %u, %u: %2.2f %2.2f %2.2f %2.2f\n",
			         quad.time().getTime(), quad.accuracy(), quad.is_new(),
			         quad.w, quad.x, quad.y, quad.z);
			imu::Device::Heading head = imu::Device::getHeading();
			MODM_LOG_DEBUG.printf("%lu ms, %u, %u: %7.3f\n",
			         head.time().getTime(), head.accuracy(), head.is_new(),
			         head.heading);

			if (head.is_new()) {
				m_heading = head.heading;
			}
		}

		PT_YIELD();
	}

	PT_END();
}

}