// license: BSD-3-Clause
// copyright-holders: Dirk Best
/***************************************************************************

    Casio CZ-101

    Digital Synthesizer

    TODO:
    - To get the display to show something, our uPD7810 core needs to
      output the full port value even if the port is set as input (port a).
    - Currently seems to hang while processing the serial ports (midi). Our
      uPD7810 core is lacking the externally clocked serial mode.

***************************************************************************/

#include "emu.h"
#include "cpu/upd7810/upd7811.h"
#include "machine/clock.h"
#include "video/hd44780.h"
#include "emupal.h"
#include "screen.h"

#include "cz101.lh"


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class cz101_state : public driver_device
{
public:
	cz101_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_hd44780(*this, "hd44780"),
		m_keys(*this, "kc%u", 0),
		m_port_b(0), m_port_c(0)
	{ }

	void cz101(machine_config &config);

	DECLARE_PALETTE_INIT(cz101);
	HD44780_PIXEL_UPDATE(lcd_pixel_update);

protected:
	virtual void machine_start() override;
	virtual void machine_reset() override;

private:
	required_device<upd7810_device> m_maincpu;
	required_device<hd44780_device> m_hd44780;
	required_ioport_array<16> m_keys;

	void maincpu_map(address_map &map);

	DECLARE_READ8_MEMBER(port_a_r);
	DECLARE_WRITE8_MEMBER(port_a_w);
	DECLARE_WRITE8_MEMBER(port_b_w);
	DECLARE_READ8_MEMBER(port_c_r);
	DECLARE_WRITE8_MEMBER(port_c_w);

	DECLARE_WRITE8_MEMBER(led_1_w);
	DECLARE_WRITE8_MEMBER(led_2_w);
	DECLARE_WRITE8_MEMBER(led_3_w);
	DECLARE_WRITE8_MEMBER(led_4_w);
	DECLARE_READ8_MEMBER(keys_r);

	uint8_t m_port_b;
	uint8_t m_port_c;
};


//**************************************************************************
//  ADDRESS MAPS
//**************************************************************************

void cz101_state::maincpu_map(address_map &map)
{
	map(0x0000, 0x7fff).rom().region("program", 0);
	map(0x8000, 0x8fff).ram();
	map(0x9000, 0x97ff).noprw(); // rampack
	map(0x9800, 0x9fff).w(FUNC(cz101_state::led_4_w));
	map(0xa000, 0xa7ff).w(FUNC(cz101_state::led_3_w));
	map(0xa800, 0xafff).w(FUNC(cz101_state::led_2_w));
	map(0xb000, 0xb7ff).w(FUNC(cz101_state::led_1_w));
	map(0xb800, 0xbfff).r(FUNC(cz101_state::keys_r));
}


//**************************************************************************
//  INPUT PORT DEFINITIONS
//**************************************************************************

static INPUT_PORTS_START( cz101 )
	PORT_START("kc0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C2")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C#2")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D2")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D#2")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("E2")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F2")
	PORT_BIT(0xc0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("kc1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F#2")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G2")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G#2")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A2")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A#2")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("B2")
	PORT_BIT(0xc0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("kc2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C3")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C#3")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D3")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D#3")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("E3")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F3")
	PORT_BIT(0xc0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("kc3")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F#3")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G3")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G#3")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A3")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A#3")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("B3")
	PORT_BIT(0xc0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("kc4")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C4")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C#4")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D4")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D#4")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("E4")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F4")
	PORT_BIT(0xc0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("kc5")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F#4")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G4")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G#4")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A4")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A#4")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("B4")
	PORT_BIT(0xc0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("kc6")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C5")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C#5")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D5")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D#5")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("E5")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F5")
	PORT_BIT(0xc0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("kc7")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F#5")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G5")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G#5")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A5")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A#5")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("B5")
	PORT_BIT(0xc0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("kc8")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C6")
	PORT_BIT(0xfe, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("kc9")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("PORTMNT ON/OFF")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("PORTMT TIME")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("VIBRATO ON/OFF")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("BEND RANGE")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("PRESET")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("INTERNAL")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("CARTRIDGE")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("COMP/RECALL")

	PORT_START("kc10")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("SOLO")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("TONE MIX")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("TRANSPOSE")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("WRITE")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("MIDI")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("PROTECT")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("SELECT")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("P")

	PORT_START("kc11")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("TONE 1")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("TONE 2")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("TONE 3")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("TONE 4")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("TONE 5")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("TONE 6")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("TONE 7")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("TONE 8")

	PORT_START("kc12")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("SAVE")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("LOAD")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("CURSOR \xe2\x97\x81") // ◁
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("CURSOR \xe2\x96\xb7") // ▷
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("ENV \xe2\x96\xbd") // ▽
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("ENV \xe2\x96\xb3") // △
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("ENV SUST.")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("ENV END")

	PORT_START("kc13")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("PACK DETECT")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("VIBRAT")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("DC01 WAVE")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("DC01 ENV")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("DCW1 KEY")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("DCW1 ENV")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("DCA1 KEY")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("DCA1 ENV")

	PORT_START("kc14")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("INITIALIZE")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("OCTAVE")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("DCO2 WAVE")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("DCO2 ENV")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("DCW2 KEY")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("DCW2 ENV")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("DCA2 KEY")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("DCA2 ENV")

	PORT_START("kc15")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("DETUNE")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("LINE SELECT")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("RING")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("NOISE")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("TUNE \xe2\x96\xbd") // ▽
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("TUNE \xe2\x96\xb3") // △
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("APO ON/OFF")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNUSED)
INPUT_PORTS_END


//**************************************************************************
//  MACHINE EMULATION
//**************************************************************************

PALETTE_INIT_MEMBER( cz101_state, cz101 )
{
	palette.set_pen_color(0, rgb_t(138, 146, 148)); // background
	palette.set_pen_color(1, rgb_t( 92,  83,  88)); // lcd pixel on
	palette.set_pen_color(2, rgb_t(131, 136, 139)); // lcd pixel off
}

HD44780_PIXEL_UPDATE( cz101_state::lcd_pixel_update )
{
	// char size is 5x8
	if (x > 4 || y > 7)
		return;

	if (line < 2 && pos < 16)
		bitmap.pix16(1 + y + line*8 + line, 1 + pos*6 + x) = state ? 1 : 2;
}

WRITE8_MEMBER( cz101_state::led_4_w )
{
	output().set_value("led_0", BIT(data, 7) ? 0 : 1);
	output().set_value("led_1", BIT(data, 6) ? 0 : 1);
	output().set_value("led_2", BIT(data, 5) ? 0 : 1);
	output().set_value("led_3", BIT(data, 4) ? 0 : 1);
	output().set_value("led_4", BIT(data, 3) ? 0 : 1);
	output().set_value("led_5", BIT(data, 2) ? 0 : 1);
	output().set_value("led_6", BIT(data, 1) ? 0 : 1);
	output().set_value("led_7", BIT(data, 0) ? 0 : 1);
}

WRITE8_MEMBER( cz101_state::led_3_w )
{
	output().set_value("led_8", BIT(data, 7) ? 0 : 1);
	output().set_value("led_9", BIT(data, 6) ? 0 : 1);
	output().set_value("led_10", BIT(data, 5) ? 0 : 1);
	output().set_value("led_11", BIT(data, 4) ? 0 : 1);
	output().set_value("led_12", BIT(data, 3) ? 0 : 1);
	output().set_value("led_13", BIT(data, 2) ? 0 : 1);
	output().set_value("led_14", BIT(data, 1) ? 0 : 1);
	output().set_value("led_15", BIT(data, 0) ? 0 : 1);
}

WRITE8_MEMBER( cz101_state::led_2_w )
{
	output().set_value("led_16", BIT(data, 7) ? 0 : 1);
	output().set_value("led_17", BIT(data, 6) ? 0 : 1);
	output().set_value("led_18", BIT(data, 5) ? 0 : 1);
	output().set_value("led_19", BIT(data, 4) ? 0 : 1);
	output().set_value("led_20", BIT(data, 3) ? 0 : 1);
	output().set_value("led_21", BIT(data, 2) ? 0 : 1);
	output().set_value("led_22", BIT(data, 1) ? 0 : 1);
	output().set_value("led_23", BIT(data, 0) ? 0 : 1);
}

WRITE8_MEMBER( cz101_state::led_1_w )
{
	output().set_value("led_24", BIT(data, 7) ? 0 : 1);
	output().set_value("led_25", BIT(data, 6) ? 0 : 1);
	output().set_value("led_26", BIT(data, 5) ? 0 : 1);
	output().set_value("led_27", BIT(data, 4) ? 0 : 1);
	output().set_value("led_28", BIT(data, 3) ? 0 : 1);
	output().set_value("led_29", BIT(data, 2) ? 0 : 1);
	output().set_value("led_30", BIT(data, 1) ? 0 : 1);
	output().set_value("led_31", BIT(data, 0) ? 0 : 1);
}

READ8_MEMBER( cz101_state::keys_r )
{
	return m_keys[m_port_b & 0x0f]->read();
}

// 76543210  lcd data bus

READ8_MEMBER( cz101_state::port_a_r )
{
	if ((BIT(m_port_c, 7) == 1) && (BIT(m_port_c, 6) == 1))
		return m_hd44780->read(space, BIT(m_port_c, 5));

	return 0xff;
}

WRITE8_MEMBER( cz101_state::port_a_w )
{
	if ((BIT(m_port_c, 7) == 1) && (BIT(m_port_c, 6) == 0))
		m_hd44780->write(space, BIT(m_port_c, 5), data);
}

// 7-------  nmi output
// -6------  music lsi write enable
// --5-----  music lsi chip select
// ---4----  music lsi irq input
// ----3210  key select output

WRITE8_MEMBER( cz101_state::port_b_w )
{
	if (1)
		logerror("port_b_w: %02x\n", data);

	m_port_b = data;
}

// 7-------  lcd e
// -6------  lcd rw
// --5-----  lcd rs
// ---4----  not used
// ----3---  power down detection output
// -----2--  midi clock
// ------1-  midi input
// -------0  midi output

WRITE8_MEMBER( cz101_state::port_c_w )
{
	if (1)
		logerror("port_c_w: %02x\n", data);

	m_port_c = data;
}

void cz101_state::machine_start()
{
	// register for save states
	save_item(NAME(m_port_b));
	save_item(NAME(m_port_c));
}

void cz101_state::machine_reset()
{
}


//**************************************************************************
//  MACHINE DEFINTIONS
//**************************************************************************

MACHINE_CONFIG_START( cz101_state::cz101 )
	MCFG_DEVICE_ADD("maincpu", UPD7810, 10_MHz_XTAL) // actually 7811, but internal ROM disabled
	MCFG_DEVICE_PROGRAM_MAP(maincpu_map)
	MCFG_UPD7810_PORTA_READ_CB(READ8(*this, cz101_state, port_a_r))
	MCFG_UPD7810_PORTA_WRITE_CB(WRITE8(*this, cz101_state, port_a_w))
	MCFG_UPD7810_PORTB_WRITE_CB(WRITE8(*this, cz101_state, port_b_w))
	MCFG_UPD7810_PORTC_WRITE_CB(WRITE8(*this, cz101_state, port_c_w))

	MCFG_CLOCK_ADD("midi_clock", 2_MHz_XTAL)
//  MCFG_CLOCK_SIGNAL_HANDLER(WRITELINE("maincpu", upd7810_device, sck_w)) not supported yet

	// video hardware
	MCFG_SCREEN_ADD("screen", LCD)
	MCFG_SCREEN_REFRESH_RATE(50)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) /* not accurate */
	MCFG_SCREEN_SIZE(6*16+1, 19)
	MCFG_SCREEN_VISIBLE_AREA(0, 6*16, 0, 19-1)
	MCFG_SCREEN_UPDATE_DEVICE("hd44780", hd44780_device, screen_update)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD("palette", 3)
	MCFG_PALETTE_INIT_OWNER(cz101_state, cz101)

	MCFG_HD44780_ADD("hd44780")
	MCFG_HD44780_LCD_SIZE(2, 16)
	MCFG_HD44780_PIXEL_UPDATE_CB(cz101_state, lcd_pixel_update)

	MCFG_DEFAULT_LAYOUT(layout_cz101)
MACHINE_CONFIG_END


//**************************************************************************
//  ROM DEFINITIONS
//**************************************************************************

ROM_START( cz101 )
	ROM_REGION(0x1000, "maincpu", 0)
	ROM_LOAD("upd7811.bin", 0x0000, 0x1000, CRC(597ac04a) SHA1(96451a764296eaa22aaad3cba121226dcba865f4))

	ROM_REGION(0x8000, "program", 0)
	ROM_LOAD("5f3_s40.bin", 0x0000, 0x8000, CRC(c417bc57) SHA1(2aa5bfb76dc0a56797cf5dd547197816cedfa370))
ROM_END


//**************************************************************************
//  SYSTEM DRIVERS
//**************************************************************************

//    YEAR  NAME   PARENT  COMPAT  MACHINE  INPUT  CLASS        INIT        COMPANY  FULLNAME  FLAGS
CONS( 1984, cz101, 0,      0,      cz101,   cz101, cz101_state, empty_init, "Casio", "CZ-101", MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
