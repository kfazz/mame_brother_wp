// license:BSD-3-Clause
// copyright-holders:Miodrag Milanovic
/***************************************************************************

        Mannesmann Kienzle System 9002 Terminal

        2017-08-17 Skeleton driver.

        Chips used:
            Siemens SAB8085A-P
            NEC D8251AFC * 2
            NEC D4016C-3 * 4 + 2
            ST M2764A-4F1 * 4
            HD6845P

Seems a chargen is missing.
If write to the 6845 is enabled, MAME freezes after 1 or 2 seconds.

****************************************************************************/

#include "emu.h"
#include "cpu/i8085/i8085.h"
#include "video/mc6845.h"
#include "emupal.h"
#include "screen.h"
#include "machine/clock.h"
#include "machine/i8251.h"
#include "bus/rs232/rs232.h"

class sys9002_state : public driver_device
{
public:
	sys9002_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_p_videoram(*this, "videoram")
		, m_palette(*this, "palette")
	{ }

	void sys9002(machine_config &config);

private:
	MC6845_UPDATE_ROW(crtc_update_row);

	void sys9002_io(address_map &map);
	void sys9002_mem(address_map &map);

	required_device<cpu_device> m_maincpu;
	required_shared_ptr<uint8_t> m_p_videoram;
	required_device<palette_device> m_palette;
};


void sys9002_state::sys9002_mem(address_map &map)
{
	map.unmap_value_high();
	map(0x0000, 0x7fff).rom(); // 4 * 4K ROM
	map(0x8000, 0x9fff).ram(); // 4 * 2k RAM
	map(0xa000, 0xa7ff).ram().share("videoram"); // 2k RAM
	map(0xc000, 0xc07f).ram(); // ??
}

void sys9002_state::sys9002_io(address_map &map)
{
	map.unmap_value_high();
	map.global_mask(0xff);
	//AM_RANGE(0x04, 0x04) AM_DEVREADWRITE("crtc", mc6845_device, status_r, address_w)  // left commented out as mame freezes after about 2 seconds
	//AM_RANGE(0x05, 0x05) AM_DEVREADWRITE("crtc", mc6845_device, register_r, register_w)
	map(0x08, 0x09).rw("uart1", FUNC(i8251_device::read), FUNC(i8251_device::write));
	map(0x11, 0x11).nopr();  // continuous read
	map(0x1c, 0x1d).rw("uart2", FUNC(i8251_device::read), FUNC(i8251_device::write));
}

/* Input ports */
static INPUT_PORTS_START( sys9002 )
INPUT_PORTS_END

MC6845_UPDATE_ROW( sys9002_state::crtc_update_row )
{
	const rgb_t *pens = m_palette->palette()->entry_list_raw();
	uint8_t chr,gfx;
	uint16_t mem,x;
	uint32_t *p = &bitmap.pix32(y);

	for (x = 0; x < x_count; x++)
	{
		mem = (ma + x) & 0x7ff;
		chr = m_p_videoram[mem];

		/* get pattern of pixels for that character scanline */
		gfx = chr; //gfx = m_p_chargen[(chr<<4) | ra] ^ ((x == cursor_x) ? 0xff : 0);

		/* Display a scanline of a character (8 pixels) */
		*p++ = pens[BIT(gfx, 7)];
		*p++ = pens[BIT(gfx, 6)];
		*p++ = pens[BIT(gfx, 5)];
		*p++ = pens[BIT(gfx, 4)];
		*p++ = pens[BIT(gfx, 3)];
		*p++ = pens[BIT(gfx, 2)];
		*p++ = pens[BIT(gfx, 1)];
		*p++ = pens[BIT(gfx, 0)];
	}
}


static DEVICE_INPUT_DEFAULTS_START( uart1 )
	DEVICE_INPUT_DEFAULTS( "RS232_RXBAUD", 0xff, RS232_BAUD_9600 )
	DEVICE_INPUT_DEFAULTS( "RS232_TXBAUD", 0xff, RS232_BAUD_9600 )
	DEVICE_INPUT_DEFAULTS( "RS232_STARTBITS", 0xff, RS232_STARTBITS_1 )
	DEVICE_INPUT_DEFAULTS( "RS232_DATABITS", 0xff, RS232_DATABITS_7 )
	DEVICE_INPUT_DEFAULTS( "RS232_PARITY", 0xff, RS232_PARITY_ODD )
	DEVICE_INPUT_DEFAULTS( "RS232_STOPBITS", 0xff, RS232_STOPBITS_1 )
DEVICE_INPUT_DEFAULTS_END

static DEVICE_INPUT_DEFAULTS_START( uart2 )
	DEVICE_INPUT_DEFAULTS( "RS232_RXBAUD", 0xff, RS232_BAUD_9600 )
	DEVICE_INPUT_DEFAULTS( "RS232_TXBAUD", 0xff, RS232_BAUD_9600 )
	DEVICE_INPUT_DEFAULTS( "RS232_STARTBITS", 0xff, RS232_STARTBITS_1 )
	DEVICE_INPUT_DEFAULTS( "RS232_DATABITS", 0xff, RS232_DATABITS_8 )
	DEVICE_INPUT_DEFAULTS( "RS232_PARITY", 0xff, RS232_PARITY_EVEN )
	DEVICE_INPUT_DEFAULTS( "RS232_STOPBITS", 0xff, RS232_STOPBITS_1 )
DEVICE_INPUT_DEFAULTS_END

MACHINE_CONFIG_START(sys9002_state::sys9002)
	/* basic machine hardware */
	MCFG_DEVICE_ADD("maincpu",I8085A, XTAL(2'000'000)) // XTAL not visible on images
	MCFG_DEVICE_PROGRAM_MAP(sys9002_mem)
	MCFG_DEVICE_IO_MAP(sys9002_io)

	/* video hardware */
	MCFG_SCREEN_ADD_MONOCHROME("screen", RASTER, rgb_t::green())
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) // not correct
	MCFG_SCREEN_UPDATE_DEVICE("crtc", mc6845_device, screen_update)
	MCFG_SCREEN_SIZE(32*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 32*8-1, 2*8, 30*8-1)
	//MCFG_DEVICE_ADD("gfxdecode", GFXDECODE, "palette", gfx_mx2178)
	MCFG_PALETTE_ADD_MONOCHROME("palette")

	/* Devices */
	MCFG_MC6845_ADD("crtc", MC6845, "screen", XTAL(2'000'000)) // clk unknown
	MCFG_MC6845_SHOW_BORDER_AREA(false)
	MCFG_MC6845_CHAR_WIDTH(8)
	MCFG_MC6845_UPDATE_ROW_CB(sys9002_state, crtc_update_row)

	clock_device &uart_clock(CLOCK(config, "uart_clock", 614400));
	uart_clock.signal_handler().set("uart1", FUNC(i8251_device::write_txc));
	uart_clock.signal_handler().append("uart1", FUNC(i8251_device::write_rxc));
	uart_clock.signal_handler().append("uart2", FUNC(i8251_device::write_txc));
	uart_clock.signal_handler().append("uart2", FUNC(i8251_device::write_rxc));

	i8251_device &uart1(I8251(config, "uart1", 0)); // 7 bits even parity, x64
	uart1.txd_handler().set("rs232a", FUNC(rs232_port_device::write_txd));
	uart1.dtr_handler().set("rs232a", FUNC(rs232_port_device::write_dtr));
	uart1.rts_handler().set("rs232a", FUNC(rs232_port_device::write_rts));

	MCFG_DEVICE_ADD("rs232a", RS232_PORT, default_rs232_devices, nullptr)
	MCFG_RS232_RXD_HANDLER(WRITELINE("uart1", i8251_device, write_rxd))
	MCFG_RS232_DSR_HANDLER(WRITELINE("uart1", i8251_device, write_dsr))
	MCFG_RS232_CTS_HANDLER(WRITELINE("uart1", i8251_device, write_cts))
	MCFG_SLOT_OPTION_DEVICE_INPUT_DEFAULTS("terminal", uart1)

	i8251_device &uart2(I8251(config, "uart2", 0)); // enabled for transmit only, 8 bits odd parity, x64
	uart2.txd_handler().set("rs232b", FUNC(rs232_port_device::write_txd));
	uart2.dtr_handler().set("rs232b", FUNC(rs232_port_device::write_dtr));
	uart2.rts_handler().set("rs232b", FUNC(rs232_port_device::write_rts));

	MCFG_DEVICE_ADD("rs232b", RS232_PORT, default_rs232_devices, "terminal")
	MCFG_RS232_RXD_HANDLER(WRITELINE("uart2", i8251_device, write_rxd))
	MCFG_RS232_DSR_HANDLER(WRITELINE("uart2", i8251_device, write_dsr))
	MCFG_RS232_CTS_HANDLER(WRITELINE("uart2", i8251_device, write_cts))
	MCFG_SLOT_OPTION_DEVICE_INPUT_DEFAULTS("terminal", uart2)
MACHINE_CONFIG_END

/* ROM definition */
ROM_START( sys9002 )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD( "55-040.bin", 0x0000, 0x2000, CRC(781eaca9) SHA1(1bdae2bcc43deaef2eb1d6ec302fbadbb779fd48))
	ROM_LOAD( "55-041.bin", 0x2000, 0x2000, CRC(0f89fe81) SHA1(2dc8de7dabaf11a150cfd34460c5b47612cf5e61))
	ROM_LOAD( "55-042.bin", 0x4000, 0x2000, CRC(e6fbc837) SHA1(fc11f6a6927709552bedf06b9eb0dc66e9a81264))
	ROM_LOAD( "55-048.bin", 0x6000, 0x2000, CRC(879ef945) SHA1(a54fc01ac26a3cd05f6d1e1139d6d99198556575))
ROM_END

/* Driver */

//    YEAR  NAME     PARENT  COMPAT  MACHINE  INPUT    CLASS          INIT        COMPANY               FULLNAME                FLAGS
COMP( 198?, sys9002, 0,      0,      sys9002, sys9002, sys9002_state, empty_init, "Mannesmann Kienzle", "System 9002 Terminal", MACHINE_IS_SKELETON )
