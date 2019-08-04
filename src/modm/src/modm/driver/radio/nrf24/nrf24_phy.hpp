/*
 * Copyright (c) 2014, Kevin Läufer
 * Copyright (c) 2014, Niklas Hauser
 * Copyright (c) 2014-2015, Daniel Krebs
 * Copyright (c) 2015, Sascha Schade
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_NRF24_PHY_HPP
#define MODM_NRF24_PHY_HPP

#include <stdint.h>
#include <modm/debug/logger.hpp>

#include <modm/architecture/interface/delay.hpp>
#include "nrf24_definitions.hpp"


namespace modm
{

/**
 * Hardware abstraction layer for nRF24L01+
 *
 * @ingroup  modm_driver_nrf24
 * @author   Daniel Krebs
 */
template <typename Spi, typename Csn, typename Ce>
class Nrf24Phy : public Nrf24Register
{

public:

	/**
	 * Initialize nRF24-HAL
	 *
	 * Call this function before using this class!
	 *
	 * @param payload_length    Set fixed payload length in bytes , set 0 for dynamic payload length
	 *
	 * Note: Dynamic payload length is *not yet implemented*
	 */
	static void
	initialize(uint8_t payload_length = max_payload_length)
	{
		// When payload length is configured to 0, length will be polled every
		// time a packet is fetched from Rx Fifo

		/* For now always use fixed payload size */
		payload_len = (payload_length == 0) ? max_payload_length : payload_length;
	}

	/// Get the maximum payload size the hardware can transmit in one packet
	static uint16_t constexpr
	getMaxPayload()
	{ return max_payload_length; }


	/// Read simple 8 bit register
	static uint8_t
	readRegister(NrfRegister_t reg);


	/// Write simple 8 bit register
	static void
	writeRegister(NrfRegister_t, uint8_t data);


	/**
	 * Set bits inside a register
	 *
	 * Only set individual bits and leave the rest untouched
	 *
	 * @param reg   Register where to set bits
	 * @param flags Which bits to set
	 */
	static void
	setBits(NrfRegister_t reg, Flags_t flags);


	/**
	 * Clear bits inside a register
	 *
	 * Only clear individual bits and leave the rest untouched.
	 *
	 * Note: To clear RX_DR, TX_DS, MAX_RT you must use setBits() !
	 *
	 * @param reg   Register where to clear bits
	 * @param flags Which bits to set
	 */
	static void
	clearBits(NrfRegister_t reg, Flags_t flags);


	static void
	clearInterrupt(InterruptFlag_t flag);

	/**
	 * Read received payload
	 *
	 * Used in RX mode.
	 * Payload is deleted from FIFO after it is read.
	 *
	 *  @param buffer   buffer where to put payload, should be 32 byte wide
	 *  @return         length of received payload
	 */
	static uint8_t
	readRxPayload(uint8_t* buffer);


	/**
	 * Write payload to be send
	 *
	 *  @param buffer   Buffer from where to read the payload
	 *  @param len      How many bytes the payload is wide
	 */
	static void
	writeTxPayload(uint8_t* buffer, uint8_t len);


	/// Same as writeTxPayload() but disable auto ACK for this packet
	static void
	writeTxPayloadNoAck(uint8_t* buffer, uint8_t len);


	/**
	 * Write payload to be transmitted together with ACK packet
	 *
	 * Used in RX mode.
	 * Maximum three ACK packet payloads can be pending. Payloads with
	 * same Pipe are handled using first in - first out principle. Write
	 * payload: 1– 32 bytes. A write operation always starts at byte 0.
	 *
	 * @param pipe Pipe for which the ACK payload is destined, must be in range 0 to 5
	 * @param buffer Buffer from where to read the payload
	 * @param len How many bytes the payload is wide
	 */
	static void
	writeAckPayload(Pipe_t pipe, uint8_t* buffer, uint8_t len);


	/// Send a high pulse of 10us length on Ce pin (blocking)
	static void
	pulseCe();

	/// Set Ce pin high
	/// If Ce was high before the pin won't be set low before waiting for 10us.
	static void
	setCe()
	{ Ce::set(); }

	/// Set Ce pin low
	static void
	resetCe()
	{ Ce::reset(); }

	/// Flush Tx Fifo
	/// Used in Tx mode
	static void
	flushTxFifo()
	{ writeCommandNoData(Command::FLUSH_TX); }


	/**
	 * Flush Rx Fifo
	 *
	 * Used in Rx mode.
	 * Should not be executed during transmission of
	 * acknowledge, that is, acknowledge package will
	 * not be completed.
	 */
	static void
	flushRxFifo()
	{ writeCommandNoData(Command::FLUSH_RX); }


	/**
	 * Reuse last transmitted payload
	 *
	 * Used in PTX mode.
	 * TX payload reuse is active until W_TX_PAYLOAD or FLUSH TX is
	 * executed. TX payload reuse must not be activated or deactivated
	 * during package transmission.
	 */
	static void
	reuseTxPayload()
	{ writeCommandNoData(Command::REUSE_TX_PL);	}


	/// Read Rx payload width for top of Rx Fifo
	/// Note: Flush RX FIFO if the read value is larger than 32 bytes.
	static uint8_t
	readRxPayloadWidth()
	{ return writeCommandSingleData(Command::R_RX_PL_WID, 0x00); }


	/**
	 * Read new status
	 *
	 * Note: status will be automatically updated every time a command is
	 *       issued, so it might not be necessary to call this explicitly.
	 *
	 *  @return Status register
	 */
	static uint8_t
	readStatus();


	/// Read Fifo status register
	/// @return Fifo Status register
	static uint8_t
	readFifoStatus()
	{ return readRegister(NrfRegister::FIFO_STATUS); }


	/**
	 * Set Rx address for a pipe
	 *
	 * Note: pipe 0 and pipe 1 have a 5 byte wide address while pipes 2 to 5
	 *       share the upper 4 bytes with pipe 1, so when setting the address
	 *       of a pipes 2 to 5, only 1 byte (LSB) will be written to SPI.
	 *
	 * @param pipe     Pipe number, must be in range 0 to 5
	 * @param address  Address for which packets will be put into pipe, see
	 *                 description concerning differences by pipe
	 */
	static void
	setRxAddress(Pipe_t pipe, uint64_t address);


	/**
	 * Set Tx address
	 *
	 * Used in PTX mode only.
	 * Set RX_ADDR_P0 equal to this address to handle
	 * automatic acknowledge if this is a PTX device with
	 * Enhanced ShockBurst enabled.
	 *
	 * @param address   Address where to send packets
	 */
	static void
	setTxAddress(uint64_t address);


	/**
	 * Get Rx address of pipe
	 *
	 *  @param pipe     Pipe number
	 *  @return         Address set for pipe
	 */
	static uint64_t
	getRxAddress(Pipe_t pipe);


	/// Get Tx address
	static uint64_t
	getTxAddress();


	///
	static uint16_t
	getPayloadLength()
	{ return payload_len; }

public:
	static void
	dumpRegisters();


private:
	static uint8_t
	writeCommandSingleData(Command_t cmd, uint8_t data);

	static void
	writeCommandNoData(Command_t cmd);

	/**
	 * Read and write multiple bytes via SPI
	 *
	 * Supplying NULL as argv or retv is allowed.
	 *  argv = nullptr  -> 0x00 is send
	 *  retv = nullptr  -> return values are discarded
	 */
	static void
	writeCommandMultiData(Command_t cmd, uint8_t* argv, uint8_t* retv, uint8_t argc);

	/// Read content any address register. Special handling is needed since sizes
	/// differ: 40-bit vs 8-bit
	static uint64_t
	readAddressRegister(NrfRegister_t reg);

private:
	/// How much payload is allowed at maximum
	static constexpr uint8_t max_payload_length = 32;

	/// Max. size of Rx/Tx addresses in bytes
	static constexpr uint8_t max_address_size = 5;

	/// Number of Rx pipes (0 to 5)
	static constexpr uint8_t rx_pipe_count = 6;

	/// Current content of status register, will be updated during most commands
	static inline uint8_t status;

	/// Fixed length of payload in bytes. 0 means dynamic payload
	static inline uint8_t payload_len;
};

} // namespace modm

#include "nrf24_phy_impl.hpp"

#endif
