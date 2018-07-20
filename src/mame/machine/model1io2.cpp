// license: BSD-3-Clause
// copyright-holders: Dirk Best
/***************************************************************************

    Sega Model 1 I/O Board (Advanced)

    Used by:
    - Wing War (R360) (837-10859)
    - NetMerc (837-11659)
    - Virtua Cop (837-11130 with 837-11131)

    Diagnostic LCD:

    It's possible to attach a small LCD to the board and enable a
    diagnostic mode. To try this in MAME, enable the 'Diagnostic'
    view, then map keys to I/O board buttons. Hold button 0 and
    press F3 to reset. The main screen will display 'I/O board error'
    but the LCD at the bottom is now active. Control it with board
    buttons 0 (up), 1 (down) and 2 (select).

    It's also possible to show some debug values while the game is
    running. To do this, hold board button '1' and push reset. Use
    buttons 0 and 1 to scroll the screen.

    Debug mode:

    You can attach a terminal to SIO channel B. Attach it to the
    MAME slot option 'ioboard:cn8'. You need to enable both JP3
    and JP4 jumpers and hold board button 0 at startup.

    Default settings are 9600-8-N-1. It will output "RS".
    You can then enter the following commands:

    - DT[word1][word2]: Returns word2 bytes from location word1
    - GO: ?
    - IN[byte1]: Return value from I/O port byte1
    - LH: ?
    - OP[byte1][byte2]: Write value byte2 to I/O port byte1
    - T: Nothing
    - XR: Return 26 bytes starting at location ff07
    - XM[26 bytes]: Write 26 bytes to location starting at ff07
    - ZP[word1][word2]: ff03=word1, ff05=word2
    - VR: Return id string

    NMI is related to the debug mode, not hooked up.

***************************************************************************/

#include "emu.h"
#include "model1io2.h"
#include "cpu/z80/tmpz84c015.h"
#include "machine/msm6253.h"
#include "machine/315_5338a.h"
#include "bus/rs232/rs232.h"
#include "screen.h"


//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(SEGA_MODEL1IO2, model1io2_device, "model1io2", "Sega Model 1 I/O Board (Advanced)")

//-------------------------------------------------
// mem_map - z80 memory map
//-------------------------------------------------

void model1io2_device::mem_map(address_map &map)
{
	map(0x0000, 0x7fff).rom();
	map(0x8000, 0x800f).rw("io", FUNC(sega_315_5338a_device::read), FUNC(sega_315_5338a_device::write));
	map(0x8040, 0x8040).portr("board");
	map(0x8080, 0x8080).portr("dsw1");
	map(0x8100, 0x810f).rw(FUNC(model1io2_device::fpga_r), FUNC(model1io2_device::fpga_w));
//  map(0x8180, 0x8183).nopr(); // displayed as 4 byte values in the diagnostic screen
//  map(0x81a0, 0x81af).nopw(); // the (reserved) test in the diagnostic screen sets these
	map(0x8200, 0x8203).mirror(0x04).rw("adc", FUNC(msm6253_device::d0_r), FUNC(msm6253_device::address_w));
//  map(0x8400, 0x8400) // jumps here when debug mode is set and board button 0 is not active on reset
	map(0xe000, 0xefff).ram(); // backup ram
	map(0xf000, 0xffff).ram();
}

//-------------------------------------------------
//  input_ports - device-specific input ports
//-------------------------------------------------

static INPUT_PORTS_START( model1io2 )
	PORT_START("board")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("Board 0") // on reset: port 4 does diagnostic menu (in debug mode: prevents jump to 8400)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("Board 1") // on reset: port 4 does parameter display
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("Board 2") // button 2 and 3 adjust the baud rate in debug
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("Board 3") // mode when active at reset (9600, 4800, 2400, 1200)
	PORT_DIPNAME(0x10, 0x10, "ROM_EMU") // JP4
	PORT_DIPSETTING(   0x00, DEF_STR(On))
	PORT_DIPSETTING(   0x10, DEF_STR(Off))
	PORT_DIPNAME(0x20, 0x20, "MODE") // JP3
	PORT_DIPSETTING(   0x00, DEF_STR(On))
	PORT_DIPSETTING(   0x20, DEF_STR(Off))
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_CUSTOM) PORT_READ_LINE_DEVICE_MEMBER("eeprom", eeprom_serial_93cxx_device, do_read)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNUSED) // eeprom nc
INPUT_PORTS_END

ioport_constructor model1io2_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(model1io2);
}

//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

ROM_START( model1io2 )
	ROM_REGION(0x10000, "iocpu", 0)

	// Wing War (taken from R360 version, is it the same for the regular version?)
	ROM_SYSTEM_BIOS(0, "epr16891", "EPR-16891")
	ROMX_LOAD("epr-16891.6", 0x00000, 0x10000, CRC(a33f84d1) SHA1(3079397c7241c1a6f494fa310faff0989dfa04a0), ROM_BIOS(0))

	// NetMerc
	ROM_SYSTEM_BIOS(1, "epr18021", "EPR-18021")
	ROMX_LOAD("epr-18021.6", 0x00000, 0x10000, CRC(5551837e) SHA1(bf5b9aad99c0f8f5e262e0855796f39119d11a97), ROM_BIOS(1))

	// Virtua Cop
	ROM_SYSTEM_BIOS(2, "epr17181", "EPR-17181")
	ROMX_LOAD("epr-17181.6", 0x00000, 0x10000, CRC(1add2b82) SHA1(81892251d466f630a96af25bde652c20e47d7ede), ROM_BIOS(2))
ROM_END

const tiny_rom_entry *model1io2_device::device_rom_region() const
{
	return ROM_NAME(model1io2);
}

//-------------------------------------------------
// device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_START( model1io2_device::device_add_mconfig )
	MCFG_DEVICE_ADD("iocpu", TMPZ84C015, 19.6608_MHz_XTAL / 2) // TMPZ84C015AF-12
	MCFG_DEVICE_PROGRAM_MAP(mem_map)

	// SIO channel a baud rate adjusted by dsw1 1+2: 38400, 19200, 9600, 4800
	MCFG_TMPZ84C015_ZC2_CB(WRITELINE("iocpu", tmpz84c015_device, rxca_w))
	MCFG_DEVCB_CHAIN_OUTPUT(WRITELINE("iocpu", tmpz84c015_device, txca_w))
	MCFG_TMPZ84C015_ZC3_CB(WRITELINE("iocpu", tmpz84c015_device, rxcb_w))
	MCFG_DEVCB_CHAIN_OUTPUT(WRITELINE("iocpu", tmpz84c015_device, txcb_w))

	MCFG_TMPZ84C015_OUT_TXDA_CB(WRITELINE("cn7", rs232_port_device, write_txd))
	MCFG_TMPZ84C015_OUT_RTSA_CB(WRITELINE("cn7", rs232_port_device, write_rts))
	MCFG_TMPZ84C015_OUT_DTRA_CB(WRITELINE("cn7", rs232_port_device, write_dtr))

	MCFG_TMPZ84C015_OUT_TXDB_CB(WRITELINE("cn8", rs232_port_device, write_txd))
	MCFG_TMPZ84C015_OUT_RTSB_CB(WRITELINE("cn8", rs232_port_device, write_rts))
	MCFG_TMPZ84C015_OUT_DTRB_CB(WRITELINE("cn8", rs232_port_device, write_dtr))

	MCFG_TMPZ84C015_IN_PA_CB(IOPORT("dsw2"))
	MCFG_TMPZ84C015_IN_PB_CB(IOPORT("dsw3"))

	MCFG_DEVICE_ADD("cn7", RS232_PORT, default_rs232_devices, nullptr)
	MCFG_RS232_RXD_HANDLER(WRITELINE("iocpu", tmpz84c015_device, rxa_w))
	MCFG_RS232_CTS_HANDLER(WRITELINE("iocpu", tmpz84c015_device, ctsa_w))
	MCFG_RS232_DCD_HANDLER(WRITELINE("iocpu", tmpz84c015_device, dcda_w))

	MCFG_DEVICE_ADD("cn8", RS232_PORT, default_rs232_devices, nullptr)
	MCFG_RS232_RXD_HANDLER(WRITELINE("iocpu", tmpz84c015_device, rxb_w))
	MCFG_RS232_CTS_HANDLER(WRITELINE("iocpu", tmpz84c015_device, ctsb_w))
	MCFG_RS232_DCD_HANDLER(WRITELINE("iocpu", tmpz84c015_device, dcdb_w))

	MCFG_DEVICE_ADD("io", SEGA_315_5338A, 32_MHz_XTAL)
	MCFG_315_5338A_READ_CB(READ8(*this, model1io2_device, io_r))
	MCFG_315_5338A_WRITE_CB(WRITE8(*this, model1io2_device, io_w))
	MCFG_315_5338A_IN_PA_CB(READ8(*this, model1io2_device, io_pa_r))
	MCFG_315_5338A_IN_PB_CB(READ8(*this, model1io2_device, io_pb_r))
	MCFG_315_5338A_IN_PC_CB(READ8(*this, model1io2_device, io_pc_r))
	MCFG_315_5338A_OUT_PD_CB(WRITE8(*this, model1io2_device, io_pd_w))
	MCFG_315_5338A_IN_PE_CB(READ8(*this, model1io2_device, io_pe_r))
	MCFG_315_5338A_OUT_PE_CB(WRITE8(*this, model1io2_device, io_pe_w))
	MCFG_315_5338A_OUT_PF_CB(WRITE8(*this, model1io2_device, io_pf_w))
	MCFG_315_5338A_OUT_PG_CB(WRITE8(*this, model1io2_device, io_pg_w))

	MCFG_DEVICE_ADD("eeprom", EEPROM_SERIAL_93C46_16BIT) // 93C45

	MCFG_DEVICE_ADD("watchdog", MB3773, 0)

	MCFG_DEVICE_ADD("adc", MSM6253, 32_MHz_XTAL / 16 / 4)
	MCFG_MSM6253_IN0_ANALOG_READ(model1io2_device, analog0_r)
	MCFG_MSM6253_IN1_ANALOG_READ(model1io2_device, analog1_r)
	MCFG_MSM6253_IN2_ANALOG_READ(model1io2_device, analog2_r)
	MCFG_MSM6253_IN3_ANALOG_READ(model1io2_device, analog3_r)

	// diagnostic lcd display
	MCFG_SCREEN_ADD("screen", LCD)
	MCFG_SCREEN_REFRESH_RATE(50)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) // not accurate
	MCFG_SCREEN_SIZE(6*20+1, 19)
	MCFG_SCREEN_VISIBLE_AREA(0, 6*20, 0, 19-1)
	MCFG_SCREEN_UPDATE_DEVICE("lcd", hd44780_device, screen_update)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD("palette", 3)
	MCFG_PALETTE_INIT_OWNER(model1io2_device, lcd)

	MCFG_HD44780_ADD("lcd")
	MCFG_HD44780_LCD_SIZE(2, 20)
	MCFG_HD44780_PIXEL_UPDATE_CB(model1io2_device, lcd_pixel_update)
MACHINE_CONFIG_END


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  model1io2_device - constructor
//-------------------------------------------------

model1io2_device::model1io2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, SEGA_MODEL1IO2, tag, owner, clock),
	m_eeprom(*this, "eeprom"),
	m_watchdog(*this, "watchdog"),
	m_lcd(*this, "lcd"),
	m_led_comm_err(*this, "led_comm_err"),
	m_lightgun_ports(*this, {finder_base::DUMMY_TAG, finder_base::DUMMY_TAG, finder_base::DUMMY_TAG, finder_base::DUMMY_TAG}),
	m_read_cb(*this), m_write_cb(*this),
	m_in_cb{ {*this}, {*this}, {*this} },
	m_drive_read_cb(*this), m_drive_write_cb(*this),
	m_an_cb{ {*this}, {*this}, {*this}, {*this}, {*this}, {*this}, {*this}, {*this} },
	m_output_cb(*this),
	m_secondary_controls(false),
	m_lcd_data(0),
	m_fpga_counter(0)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void model1io2_device::device_start()
{
	// resolve callbacks
	m_led_comm_err.resolve();

	m_read_cb.resolve_safe(0xff);
	m_write_cb.resolve_safe();

	for (unsigned i = 0; i < 3; i++)
		m_in_cb[i].resolve_safe(0xff);

	m_drive_read_cb.resolve_safe(0xff);
	m_drive_write_cb.resolve_safe();

	for (unsigned i = 0; i < 8; i++)
		m_an_cb[i].resolve_safe(0xff);

	m_output_cb.resolve_safe();

	// register for save states
	save_item(NAME(m_secondary_controls));
	save_item(NAME(m_lcd_data));
	save_item(NAME(m_fpga_counter));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void model1io2_device::device_reset()
{
	m_fpga_counter = 0;
}


//**************************************************************************
//  DIAGNOSTIC LCD
//**************************************************************************

PALETTE_INIT_MEMBER( model1io2_device, lcd )
{
	palette.set_pen_color(0, rgb_t(138, 146, 148)); // background
	palette.set_pen_color(1, rgb_t( 92,  83,  88)); // lcd pixel on
	palette.set_pen_color(2, rgb_t(131, 136, 139)); // lcd pixel off
}

HD44780_PIXEL_UPDATE( model1io2_device::lcd_pixel_update )
{
	// char size is 5x8
	if (x > 4 || y > 7)
		return;

	if (line < 2 && pos < 20)
		bitmap.pix16(1 + y + line*8 + line, 1 + pos*6 + x) = state ? 1 : 2;
}


//**************************************************************************
//  INTERFACE
//**************************************************************************

READ8_MEMBER( model1io2_device::io_r )
{
	return m_read_cb(offset);
}

WRITE8_MEMBER( model1io2_device::io_w )
{
	m_write_cb(offset, data, 0xff);
}

READ8_MEMBER( model1io2_device::io_pa_r )
{
	return m_in_cb[0](0);
}

READ8_MEMBER( model1io2_device::io_pb_r )
{
	return m_in_cb[1](0);
}

READ8_MEMBER( model1io2_device::io_pc_r )
{
	return m_in_cb[2](0);
}

WRITE8_MEMBER( model1io2_device::io_pd_w )
{
	m_output_cb(data);
}

READ8_MEMBER( model1io2_device::io_pe_r )
{
	// cn6
	return m_drive_read_cb(0);
}

WRITE8_MEMBER( model1io2_device::io_pe_w )
{
	// cn6
	m_lcd_data = data;
	m_drive_write_cb(data);
}

WRITE8_MEMBER( model1io2_device::io_pf_w )
{
	// 7-------  eeprom pe
	// -6------  eeprom di
	// --5-----  eeprom clk
	// ---4----  eeprom cs
	// ----3---  cn6 enabled
	// -----2--  cn6 lcd e
	// ------1-  cn6 lcd rw
	// -------0  cn6 lcd rs

	m_eeprom->clk_write(BIT(data, 5) ? ASSERT_LINE : CLEAR_LINE);
	m_eeprom->di_write(BIT(data, 6));
	m_eeprom->cs_write(BIT(data, 4) ? ASSERT_LINE : CLEAR_LINE);

	if (BIT(data, 3) == 0 && BIT(data, 2) == 1 && BIT(data, 1) == 0)
		m_lcd->write(space, BIT(data, 0), m_lcd_data);
}

WRITE8_MEMBER( model1io2_device::io_pg_w )
{
	// 7-------  watchdog
	// -6------  control panel switch
	// --5-----  comm_err led
	// ---43210  test points (0 = interrupt, 1 = sio a)

	m_watchdog->write_line_ck(BIT(data, 7));
	m_secondary_controls = BIT(data, 6);
	m_led_comm_err = BIT(~data, 5);
}

ioport_value model1io2_device::analog0_r()
{
	return m_secondary_controls ? m_an_cb[4](0) : m_an_cb[0](0);
}

ioport_value model1io2_device::analog1_r()
{
	return m_secondary_controls ? m_an_cb[5](0) : m_an_cb[1](0);
}

ioport_value model1io2_device::analog2_r()
{
	return m_secondary_controls ? m_an_cb[6](0) : m_an_cb[2](0);
}

ioport_value model1io2_device::analog3_r()
{
	return m_secondary_controls ? m_an_cb[7](0) : m_an_cb[3](0);
}


//**************************************************************************
//  FGPA (Virtua Cop)
//**************************************************************************

READ8_MEMBER( model1io2_device::fpga_r )
{
	// fpga upload not finished yet?
	if (m_fpga_counter < 0x1400)
		return 0x80;

	// lightgun coordinates
	if (offset < 8)
		return m_lightgun_ports[offset >> 1].read_safe(0) >> (8 * (offset & 1));

	// lightgun off-screen detect
	else if (offset == 8)
	{
		// 5 percent border size
		const float BORDER_SIZE = 0.05f;

		// calculate width depending on min/max port value
		const int BORDER_P1X = (m_lightgun_ports[1]->field(0x3ff)->maxval() - m_lightgun_ports[1]->field(0x3ff)->minval()) * BORDER_SIZE;
		const int BORDER_P1Y = (m_lightgun_ports[0]->field(0x3ff)->maxval() - m_lightgun_ports[0]->field(0x3ff)->minval()) * BORDER_SIZE;
		const int BORDER_P2X = (m_lightgun_ports[3]->field(0x3ff)->maxval() - m_lightgun_ports[3]->field(0x3ff)->minval()) * BORDER_SIZE;
		const int BORDER_P2Y = (m_lightgun_ports[2]->field(0x3ff)->maxval() - m_lightgun_ports[2]->field(0x3ff)->minval()) * BORDER_SIZE;

		uint8_t data = 0xfc;

		const uint16_t P1X = m_lightgun_ports[1].read_safe(0);
		const uint16_t P1Y = m_lightgun_ports[0].read_safe(0);
		const uint16_t P2X = m_lightgun_ports[3].read_safe(0);
		const uint16_t P2Y = m_lightgun_ports[2].read_safe(0);

		// border hit test for player 1 and 2
		if (P1X <= (m_lightgun_ports[1]->field(0x3ff)->minval() + BORDER_P1X)) data |= 1;
		if (P1X >= (m_lightgun_ports[1]->field(0x3ff)->maxval() - BORDER_P1X)) data |= 1;
		if (P1Y <= (m_lightgun_ports[0]->field(0x3ff)->minval() + BORDER_P1Y)) data |= 1;
		if (P1Y >= (m_lightgun_ports[0]->field(0x3ff)->maxval() - BORDER_P1Y)) data |= 1;
		if (P2X <= (m_lightgun_ports[3]->field(0x3ff)->minval() + BORDER_P2X)) data |= 2;
		if (P2X >= (m_lightgun_ports[3]->field(0x3ff)->maxval() - BORDER_P2X)) data |= 2;
		if (P2Y <= (m_lightgun_ports[2]->field(0x3ff)->minval() + BORDER_P2Y)) data |= 2;
		if (P2Y >= (m_lightgun_ports[2]->field(0x3ff)->maxval() - BORDER_P2Y)) data |= 2;

		return data;
	}

	return 0xff;
}

WRITE8_MEMBER( model1io2_device::fpga_w )
{
	// fpga data uploaded here (vcop)
	m_fpga_counter++;
}
