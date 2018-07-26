// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Coleco Adam Serial/Parallel Interface emulation

**********************************************************************/

#include "emu.h"
#include "spi.h"



//**************************************************************************
//  MACROS / CONSTANTS
//**************************************************************************

#define M6801_TAG       "m6801"
#define MC2661_TAG      "mc2661"
#define RS232_TAG       "rs232"



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(ADAM_SPI, adam_spi_device, "adam_spi", "Adam SPI")


//-------------------------------------------------
//  ROM( adam_spi )
//-------------------------------------------------

ROM_START( adam_spi )
	ROM_REGION( 0x800, M6801_TAG, 0 )
	ROM_LOAD( "spi.bin", 0x000, 0x800, CRC(4ba30352) SHA1(99fe5aebd505a208bea6beec5d7322b15426e9c1) )
ROM_END


//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

const tiny_rom_entry *adam_spi_device::device_rom_region() const
{
	return ROM_NAME( adam_spi );
}


//-------------------------------------------------
//  ADDRESS_MAP( adam_spi_mem )
//-------------------------------------------------

void adam_spi_device::adam_spi_mem(address_map &map)
{
	map(0x0000, 0x001f).rw(M6801_TAG, FUNC(m6801_cpu_device::m6801_io_r), FUNC(m6801_cpu_device::m6801_io_w));
	map(0x0080, 0x00ff).ram();
	map(0xf800, 0xffff).rom().region(M6801_TAG, 0);
}


//-------------------------------------------------
//  ADDRESS_MAP( adam_spi_io )
//-------------------------------------------------

void adam_spi_device::adam_spi_io(address_map &map)
{
	map(M6801_PORT2, M6801_PORT2).rw(FUNC(adam_spi_device::p2_r), FUNC(adam_spi_device::p2_w));
}


//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_START(adam_spi_device::device_add_mconfig)
	MCFG_DEVICE_ADD(M6801_TAG, M6801, XTAL(4'000'000))
	MCFG_DEVICE_PROGRAM_MAP(adam_spi_mem)
	MCFG_DEVICE_IO_MAP(adam_spi_io)
	MCFG_DEVICE_DISABLE()

	MCFG_DEVICE_ADD(MC2661_TAG, MC2661, XTAL(4'915'200))

	MCFG_DEVICE_ADD(RS232_TAG, RS232_PORT, default_rs232_devices, nullptr)

	centronics_device &centronics(CENTRONICS(config, "centronics", centronics_devices, "printer"));
	centronics.set_data_input_buffer("cent_data_in");
	INPUT_BUFFER(config, "cent_data_in");
	MCFG_CENTRONICS_OUTPUT_LATCH_ADD("cent_data_out", "centronics")
MACHINE_CONFIG_END



//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  adam_spi_device - constructor
//-------------------------------------------------

adam_spi_device::adam_spi_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, ADAM_SPI, tag, owner, clock)
	, device_adamnet_card_interface(mconfig, *this)
	, m_maincpu(*this, M6801_TAG)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void adam_spi_device::device_start()
{
}


//-------------------------------------------------
//  adamnet_reset_w -
//-------------------------------------------------

void adam_spi_device::adamnet_reset_w(int state)
{
	m_maincpu->set_input_line(INPUT_LINE_RESET, state);
}


//-------------------------------------------------
//  p2_r -
//-------------------------------------------------

READ8_MEMBER( adam_spi_device::p2_r )
{
	/*

	    bit     description

	    0       mode bit 0
	    1       mode bit 1
	    2       mode bit 2
	    3       NET RXD
	    4

	*/

	uint8_t data = M6801_MODE_7;

	// NET RXD
	data |= m_bus->rxd_r(this) << 3;

	return data;
}


//-------------------------------------------------
//  p2_w -
//-------------------------------------------------

WRITE8_MEMBER( adam_spi_device::p2_w )
{
	/*

	    bit     description

	    0
	    1
	    2
	    3
	    4       NET TXD

	*/

	m_bus->txd_w(this, BIT(data, 4));
}
