// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    er2055.c

    GI 512 bit electrically alterable read-only memory.

***************************************************************************/

#include "emu.h"
#include "machine/er2055.h"

#include "logmacro.h"


//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

// device type definition
DEFINE_DEVICE_TYPE(ER2055, er2055_device, "er2055", "ER2055 EAROM")

void er2055_device::er2055_map(address_map &map)
{
	map(0x0000, 0x003f).ram();
}



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  er2055_device - constructor
//-------------------------------------------------

er2055_device::er2055_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, ER2055, tag, owner, clock),
		device_memory_interface(mconfig, *this),
		device_nvram_interface(mconfig, *this),
		m_region(*this, DEVICE_SELF),
		m_space_config("EAROM", ENDIANNESS_BIG, 8, 6, 0, address_map_constructor(FUNC(er2055_device::er2055_map), this)),
		m_control_state(0),
		m_address(0),
		m_data(0)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void er2055_device::device_start()
{
	save_item(NAME(m_control_state));
	save_item(NAME(m_address));
	save_item(NAME(m_data));

	m_control_state = 0;
}


//-------------------------------------------------
//  memory_space_config - return a description of
//  any address spaces owned by this device
//-------------------------------------------------

device_memory_interface::space_config_vector er2055_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(0, &m_space_config)
	};
}


//-------------------------------------------------
//  nvram_default - called to initialize NVRAM to
//  its default state
//-------------------------------------------------

void er2055_device::nvram_default()
{
	// default to all-0xff
	for (int byte = 0; byte < SIZE_DATA; byte++)
		space(AS_PROGRAM).write_byte(byte, 0xff);

	// populate from a memory region if present
	if (m_region != nullptr)
	{
		if (m_region->bytes() != SIZE_DATA)
			fatalerror("er2055 region '%s' wrong size (expected size = 0x40)\n", tag());
		if (m_region->bytewidth() != 1)
			fatalerror("er2055 region '%s' needs to be an 8-bit region\n", tag());

		uint8_t *default_data = m_region->base();
		for (int byte = 0; byte < SIZE_DATA; byte++)
			space(AS_PROGRAM).write_byte(byte, default_data[byte]);
	}
}


//-------------------------------------------------
//  nvram_read - called to read NVRAM from the
//  .nv file
//-------------------------------------------------

void er2055_device::nvram_read(emu_file &file)
{
	uint8_t buffer[SIZE_DATA];
	file.read(buffer, sizeof(buffer));
	for (int byte = 0; byte < SIZE_DATA; byte++)
		space(AS_PROGRAM).write_byte(byte, buffer[byte]);
}


//-------------------------------------------------
//  nvram_write - called to write NVRAM to the
//  .nv file
//-------------------------------------------------

void er2055_device::nvram_write(emu_file &file)
{
	uint8_t buffer[SIZE_DATA];
	for (int byte = 0; byte < SIZE_DATA; byte++)
		buffer[byte] = space(AS_PROGRAM).read_byte(byte);
	file.write(buffer, sizeof(buffer));
}



//**************************************************************************
//  I/O OPERATIONS
//**************************************************************************

//-------------------------------------------------
//  set_control - set the control lines; these
//  must be done simultaneously because the chip
//  reacts to various combinations
//-------------------------------------------------

void er2055_device::set_control(uint8_t cs1, uint8_t cs2, uint8_t c1, uint8_t c2)
{
	// create a new composite control state
	uint8_t oldstate = m_control_state;
	m_control_state = oldstate & CK;
	m_control_state |= (c1 != 0) ? C1 : 0;
	m_control_state |= (c2 != 0) ? C2 : 0;
	m_control_state |= (cs1 != 0) ? CS1 : 0;
	m_control_state |= (cs2 != 0) ? CS2 : 0;

	// if not selected, or if change from previous, we're done
	if ((m_control_state & (CS1 | CS2)) != (CS1 | CS2) || m_control_state == oldstate)
		return;

	update_state();
}


//-------------------------------------------------
//  update_state - update internal state following
//  a transition on clock, control or chip select
//  lines based on what mode we're in
//-------------------------------------------------

void er2055_device::update_state()
{
	switch (m_control_state & (C1 | C2))
	{
		// write mode; erasing is required, so we perform an AND against previous
		// data to simulate incorrect behavior if erasing was not done
		case 0:
			space(AS_PROGRAM).write_byte(m_address, space(AS_PROGRAM).read_byte(m_address) & m_data);
			LOG("Write %02X = %02X\n", m_address, m_data);
			break;

		// erase mode
		case C2:
			space(AS_PROGRAM).write_byte(m_address, 0xff);
			LOG("Erase %02X\n", m_address);
			break;
	}
}


//-------------------------------------------------
//  set_clk - set the CLK line, pulses on which
//  are required for every read operation and for
//  successive write or erase operations
//-------------------------------------------------

WRITE_LINE_MEMBER(er2055_device::set_clk)
{
	uint8_t oldstate = m_control_state;
	if (state)
		m_control_state |= CK;
	else
		m_control_state &= ~CK;

	// updates occur on falling edge when chip is selected
	if ((m_control_state & (CS1 | CS2)) == (CS1 | CS2) && (m_control_state != oldstate) && !state)
	{
		// read mode (C2 is "Don't Care")
		if ((m_control_state & C1) == C1)
		{
			m_data = space(AS_PROGRAM).read_byte(m_address);
			LOG("Read %02X = %02X\n", m_address, m_data);
		}

		update_state();
	}
}
