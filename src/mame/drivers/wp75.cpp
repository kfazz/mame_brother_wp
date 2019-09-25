// license:BSD-3-Clause
// copyright-holders:Ken Fazzone
/***************************************************************************

        TIM-011

        04/09/2010 Skeleton driver.

****************************************************************************/

#include "emu.h"
#include "cpu/z180/z180.h"
#include "imagedev/floppy.h"
#include "machine/upd765.h"
#include "emupal.h"
#include "screen.h"

#define FDC9266_TAG "fdc"

class wp75_state : public driver_device
{
public:
	wp75_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		//, m_videoram(*this, "vram")
		, m_fdc(*this, FDC9266_TAG)
		, m_floppy0(*this, FDC9266_TAG ":0:35dd")
		, m_y(*this, "Y%u", 0) //keyboard columns
		//, m_screen(*this, "screen")
		//, m_palette(*this, "palette")
	{ }

	void init_wp70();
	void init_wp75();

	void wp75(machine_config &config);
	void wp70(machine_config &config);
	void wp2450ds(machine_config &config);
	void wp5500ds(machine_config &config);


private:
	virtual void machine_reset() override;
	virtual void video_start() override;
	uint32_t screen_update_wp75(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	DECLARE_WRITE8_MEMBER(vpl_w);
	DECLARE_WRITE8_MEMBER(vph_w);
	DECLARE_WRITE8_MEMBER(vram_w);
	DECLARE_READ8_MEMBER(vram_r);
	DECLARE_WRITE8_MEMBER(vctl_w);
	DECLARE_WRITE8_MEMBER(vcrl_w);
	DECLARE_WRITE8_MEMBER(vcrh_w);
	DECLARE_WRITE8_MEMBER(bank_w);
	INTERRUPT_GEN_MEMBER(wp75_timer_interrupt);
	DECLARE_READ8_MEMBER(portb_r); //limit switches
	DECLARE_READ8_MEMBER(wp70_sys_r); //port c
	DECLARE_READ8_MEMBER(sys_r); //port c
	DECLARE_WRITE8_MEMBER(irq1_clear_w);

	DECLARE_READ8_MEMBER(unk_r);
	DECLARE_WRITE8_MEMBER(unk_w);
	//fake floppy state for now
	DECLARE_READ8_MEMBER(fdc_r);
	DECLARE_WRITE8_MEMBER(fdc_w);

	DECLARE_WRITE8_MEMBER(kb_w);
	DECLARE_READ8_MEMBER(kb_r);
	DECLARE_WRITE8_MEMBER(ca_w);
	DECLARE_WRITE8_MEMBER(wh_w);
	DECLARE_WRITE8_MEMBER(lf_w);

	DECLARE_WRITE8_MEMBER(window_w);
	DECLARE_READ8_MEMBER(window_r);
	DECLARE_READ8_MEMBER(rom_wind_r);

	DECLARE_WRITE8_MEMBER(fdd_w);
	DECLARE_READ8_MEMBER(fdd_r);

	DECLARE_WRITE8_MEMBER(pg_w);
	DECLARE_WRITE8_MEMBER(pk_w);

	DECLARE_WRITE_LINE_MEMBER(drq_w);

	uint8_t ga_irq_mask;

	uint16_t v_ptr; //addr vram io window points at
	uint8_t vl; //addr vram io window points at
	uint8_t vh; //addr vram io window points at

	bool ca_idx = false; //true when carriage is left homed or csol is true
	bool rsol = false; //ribbon solenoid
	bool csol = false; //correction solenoid

	//carriage homing routine appears to go 40 steps left then decides 'check printer'
	uint16_t carriage_position = 40; //120 steps per inch

	//used to determing whether carriage is going left or right
	uint8_t ca_prev = 0x00; //previous ca latch contents
	uint8_t ca_now; //current ca latch contents


//	b7	6	54	32	1	0
//	crsr?	c.blnk  cr.sz   vr incr. disp.	reverse
	
	uint8_t v_ctl;

	uint8_t kb_matrix;

	required_device<cpu_device> m_maincpu;
	required_device<upd765a_device> m_fdc;
	required_device<floppy_image_device> m_floppy0;
	required_ioport_array<9> m_y;
	//required_device<screen_device> m_screen;
	//required_device<palette_device> m_palette;

	void wp70_io(address_map &map); //has ioport dict rom bank control
	void wp70_mem(address_map &map);//has dict rom banked at 0x40000

	void wp75_io(address_map &map);
	void wp75_mem(address_map &map);
	void wp5500ds_mem(address_map &map);

};

//**************************************************************************
//  INPUT PORTS
//**************************************************************************
/*
A1 1
A9 9
A10 0

A13 Q
A14 W
A15 E
A16 R
A17 T
A18 Y
A19 U
a20 I
a21 O
A22 P
A23 A
A24 S
A25 D
A26 F
A27 G
A28 H
A29 J
A30 K
A31 L
A32 Z
A33 X
A34 C
A35 V
A36 B
A37 N
A38 M*/
/*
  Keyboard matrix
        X7    X6    X5    X4    X3    X2    X1    X0
     +-----+-----+-----+-----+-----+-----+-----+-----+
  Y0 |A22  | A11 |A10  | A21 |     | AA5 | A38 | F4  |
     +-----+-----+-----+-----+-----+-----+-----+-----+
  Y1 | A20 | A9  | A29 | A19 |     | A37 | A36 | F14 |
     +-----+-----+-----+-----+-----+-----+-----+-----+
  Y2 | A18 | A8  | A7  | A28 |     | A35 | A27 | F5  |
     +-----+-----+-----+-----+-----+-----+-----+-----+
  Y3 | A16 | A5  | A6  | A17 |     | A26 | A34 | F10 |
     +-----+-----+-----+-----+-----+-----+-----+-----+
  Y4 | A14 | A4  | A3  | A15 |     | A33 | A25 | F6  |
     +-----+-----+-----+-----+-----+-----+-----+-----+
  Y5 | A0  | A12 | A31 | A30 | AA4 | AA6 | A39 |     |
     +-----+-----+-----+-----+-----+-----+-----+-----+
  Y6 | A13 | A1  | A2  | A32 |     | A24 | A23 | F8  |
     +-----+-----+-----+-----+-----+-----+-----+-----+
  Y7 | F9  | F1  | F3  | F7  | F16 | F18 | F17 | F11 |
     +-----+-----+-----+-----+-----+-----+-----+-----+
  Y8 | AA2 | F2  | AA3 | AA1 | F15 | F13 | F12 |     |
     +-----+-----+-----+-----+-----+-----+-----+-----+
*/

static INPUT_PORTS_START( wp75 )
	PORT_START("Y0")
/*A22*/ PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_P) PORT_CHAR('p') PORT_CHAR('P') /*P Code:Print*/
/*A11*/	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS) PORT_CHAR('_') PORT_CHAR('%')
/*A10*/	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_0) PORT_CHAR(0)
/*A21*/	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_O) PORT_CHAR('O') PORT_CHAR('o')
/*N/A*/ PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNUSED)
/*AA5*/	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_COMMA)
/*A38*/	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_M) PORT_CHAR('m') PORT_CHAR('M')
/*F4*/	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_END)  /*MENU*/

	PORT_START("Y1")
/*a20*/	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_I) PORT_CHAR('i') PORT_CHAR('I')
/*a9*/	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_9) PORT_CHAR('9')
/*a29*/	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_J) PORT_CHAR('j') PORT_CHAR('J')
/*A19*/	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_U) PORT_CHAR('u') PORT_CHAR('U')
/*N/A*/ PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNUSED)
/*A37*/	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_N) PORT_CHAR('n') PORT_CHAR('N')
/*A36*/	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_B) PORT_CHAR('b') PORT_CHAR('B')
/*f14*/	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_RIGHT) PORT_CHAR(UCHAR_MAMEKEY(RIGHT))

	PORT_START("Y2")
/*a18*/	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_Y) PORT_CHAR('y') PORT_CHAR('Y')
/*A8*/	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_8) PORT_CHAR('8')
/*A7*/	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_7) PORT_CHAR('7')
/*A28*/	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_H) PORT_CHAR('h') PORT_CHAR('H')
/*N/A*/ PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNUSED)
/*A35*/	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_V) PORT_CHAR('v') PORT_CHAR('V')
/*A27*/	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_G) PORT_CHAR('g') PORT_CHAR('G')
/*F5*/	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_PGDN)

	PORT_START("Y3")
/*A16*/	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_R) PORT_CHAR('r') PORT_CHAR('R')
/*A5*/	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_5) PORT_CHAR('5')
/*A6*/	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_6) PORT_CHAR('6')
/*A17*/	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_T) PORT_CHAR('t') PORT_CHAR('T')
/*N/A*/ PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNUSED)
/*a26*/	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_F) PORT_CHAR('f') PORT_CHAR('F')
/*A34*/	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_C) PORT_CHAR('c') PORT_CHAR('C')
/*F10*/	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_UP) PORT_CHAR(UCHAR_MAMEKEY(UP))

	PORT_START("Y4")
/*A14*/	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_W) PORT_CHAR('w') PORT_CHAR('W')
/*A4*/	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_4) PORT_CHAR('4')
/*A3*/	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_3) PORT_CHAR('3')
/*A15*/	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_B) PORT_CHAR('e') PORT_CHAR('E')
/*N/A*/ PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNUSED)
/*A33*/	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_T) PORT_CHAR('x') PORT_CHAR('X')
/*A25*/	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_6) PORT_CHAR('d') PORT_CHAR('D')
/*F6*/	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_TAB) PORT_CHAR(UCHAR_MAMEKEY(TAB))

	PORT_START("Y5")
	/*Degree symbol, +_ == and ^*/
/*A0*/	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_TILDE) PORT_CHAR('=') PORT_CHAR('^')
/*A12*/	//PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_PLUS) PORT_CHAR('+') /*FIXME*/
/*A31*/	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_L) PORT_CHAR('l') PORT_CHAR('L')
/*A30*/	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_K) PORT_CHAR('k') PORT_CHAR('K')

/*FIXME CHECK THESE*/
/*AA4*/	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_QUOTE) PORT_CHAR('"') PORT_CHAR('\'')
/*AA6*/	//PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_PERIOD) PORT_CHAR('.')
/*A39*/	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_COMMA) PORT_CHAR(',')
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("Y6")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_QUOTE) PORT_CHAR('@') PORT_CHAR('`')
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_P) PORT_CHAR('p') PORT_CHAR('P')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_C) PORT_CHAR('c') PORT_CHAR('C')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_D) PORT_CHAR('d') PORT_CHAR('D')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_E) PORT_CHAR('e') PORT_CHAR('E')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_4) PORT_CHAR('4') PORT_CHAR('$')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_F3) PORT_CHAR(UCHAR_MAMEKEY(F3))

	PORT_START("Y7")
/*F9*/	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_ENTER) PORT_CHAR(UCHAR_MAMEKEY(ENTER))
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS) PORT_CHAR('-') PORT_CHAR('=')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSLASH) PORT_CHAR('\\') PORT_CHAR('|')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_X) PORT_CHAR('x')  PORT_CHAR('X')
/**/	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSPACE)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_DEL) /*Cancel /P Down*/
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_3) PORT_CHAR('3') PORT_CHAR('#')
/*F11*/	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_LSHIFT) PORT_CHAR(UCHAR_MAMEKEY(LSHIFT))

	PORT_START("Y8")
/*N/A*/	
/*F12*/	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_LEFT) PORT_CHAR(UCHAR_MAMEKEY(LEFT))
/*F13*/	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_DOWN) PORT_CHAR(UCHAR_MAMEKEY(DOWN))
/*F15*/	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_LCONTROL) PORT_CHAR(UCHAR_MAMEKEY(LCONTROL))

	/* 1/4    1/2   umlaut  and `  */
/*AA1*/	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_OPENBRACE) PORT_CHAR('[') PORT_CHAR('{')
/*AA3*/	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_L) PORT_CHAR('l') PORT_CHAR('L')
/*F2*/	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_F12) PORT_CHAR(UCHAR_MAMEKEY(F12))  /*File*/
/*AA2*/	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_CODE(KEYCODE_CLOSEBRACE) PORT_CHAR('`') PORT_CHAR('|') PORT_CHAR('[') PORT_CHAR(']')


INPUT_PORTS_END

void wp75_state::wp70_mem(address_map &map)
{

	map.unmap_value_high();
	map(0x0000, 0x1FFF).rom();
	map(0x2000, 0x5FFF).ram().rw(FUNC(wp75_state::window_r), FUNC(wp75_state::window_w)).share("window");
	map(0x6000,0x3FFFF).rom();

//	map(0x40000, 0x5FFFF).rom(); // is the main rom shadowed here accessible anywhere???

	map(0x40000, 0x5FFFF).bankr("rom1"); //dictionary & bank program

	map(0x60000,0x61FFF).mirror(0x10000).ram();
	map(0x62000,0x65FFF).ram(); // <== window points here
	map(0x66000,0x6FFFF).ram();

	//this window points at the rom that is shadowed by ram from 0x2000-0x5FFF
	map(0x72000, 0x75FFF).r(FUNC(wp75_state::rom_wind_r)).share("romwindow");
}


void wp75_state::wp75_mem(address_map &map)
{
	/*This is almost certainly 80% wrong*/
	map.unmap_value_high();
	map(0x0000, 0x7FFFF).rom();
	map(0x2000, 0x5FFF).ram().rw(FUNC(wp75_state::window_r), FUNC(wp75_state::window_w)).share("window");
	map(0x60000,0x61FFF).mirror(0x10000).ram();
	map(0x62000,0x65FFF).ram(); // <== window points here
	map(0x66000,0x6FFFF).ram();
	//this window points at the rom that is shadowed by ram from 0x2000-0x5FFF
	map(0x72000, 0x75FFF).r(FUNC(wp75_state::rom_wind_r)).share("romwindow");
}


void wp75_state::wp5500ds_mem(address_map &map)
{
	map.unmap_value_high();
	map(0x0000, 0x7FFFF).mirror(0x80000).rom();
	map(0x2000,  0x5FFF).mirror(0x80000).ram().rw(FUNC(wp75_state::window_r), FUNC(wp75_state::window_w)).share("window");
	map(0x60000,0x61FFF).mirror(0x81000).ram();
	map(0x62000,0x65FFF).mirror(0x80000).ram(); // <== window points here
	map(0x66000,0x6FFFF).mirror(0x80000).ram();

	//this window points at the rom that is shadowed by ram from 0x2000-0x5FFF
	map(0x72000, 0x75FFF).mirror(0x80000).r(FUNC(wp75_state::rom_wind_r)).share("romwindow");
	map(0xF8000, 0xFFFFF).rom().region("vram", 0); // I really wish this were vram on the 2450ds.. will test hardware soon
}

void wp75_state::wp70_io(address_map &map)
{
	map.unmap_value_high();
	map(0x0000, 0x003f).ram(); /* Z180 internal registers */

	map(0x70, 0x70).mirror(0xff00).w(FUNC(wp75_state::vpl_w));
	map(0x71, 0x71).mirror(0xff00).w(FUNC(wp75_state::vph_w));
	map(0x72, 0x72).mirror(0xff00).w(FUNC(wp75_state::vram_w));
	map(0x73, 0x73).mirror(0xff00).r(FUNC(wp75_state::vram_r));
	map(0x74, 0x74).mirror(0xff00).w(FUNC(wp75_state::vctl_w));
	map(0x75, 0x75).mirror(0xff00).w(FUNC(wp75_state::vcrl_w));
	map(0x76, 0x76).mirror(0xff00).w(FUNC(wp75_state::vcrh_w));

//	map(0x78, 0x7b).mirror(0xff00).m(m_fdc, FUNC(upd765a_device::map)).umask16(0x0Ff);
//	map(0x7c, 0x7F).mirror(0xff00).m(m_fdc, FUNC(upd765a_device::map)).umask16(0x0Ff);

//	map(0x78, 0x7b).mirror(0xff00).r(FUNC(wp75_state::fdc_r));
//	map(0x78, 0x7b).mirror(0xff00).w(FUNC(wp75_state::fdc_w));

	map(0x90, 0x90).mirror(0xff00).r(FUNC(wp75_state::unk_r));
	map(0x90, 0x90).mirror(0xff00).w(FUNC(wp75_state::unk_w));


	map(0xB8,0xB8).mirror(0xff00).r(FUNC(wp75_state::kb_r));
	map(0xB8,0xB8).mirror(0xff00).w(FUNC(wp75_state::kb_w));

	map(0xC0,0xC0).mirror(0xff00).w(FUNC(wp75_state::ca_w));
	map(0xC8,0xC8).mirror(0xff00).w(FUNC(wp75_state::wh_w));
	map(0xD0,0xD0).mirror(0xff00).w(FUNC(wp75_state::lf_w));

	map(0xD8,0xD8).mirror(0xff00).w(FUNC(wp75_state::pg_w)); //solenoids and dc motor control
	map(0xF0,0xF0).mirror(0xff00).w(FUNC(wp75_state::pk_w)); //buzzer and ?

	map(0xB0,0xB0).mirror(0xff00).r(FUNC(wp75_state::wp70_sys_r)); //spec strap
//	map(0xE0,0xE0).mirror(0xff00).w(FUNC(wp75_state::bank_w)); //rom1 bank switch control
	map(0xA8,0xA8).mirror(0xff00).r(FUNC(wp75_state::portb_r));
	map(0xF8,0xF8).mirror(0xff00).w(FUNC(wp75_state::irq1_clear_w));
}

void wp75_state::wp75_io(address_map &map)
{
	map.unmap_value_high();
	map(0x0000, 0x003f).ram(); /* Z180 internal registers */

	map(0x70, 0x70).mirror(0xff00).w(FUNC(wp75_state::vpl_w));
	map(0x71, 0x71).mirror(0xff00).w(FUNC(wp75_state::vph_w));
	map(0x72, 0x72).mirror(0xff00).w(FUNC(wp75_state::vram_w));
	map(0x73, 0x73).mirror(0xff00).r(FUNC(wp75_state::vram_r));
	map(0x74, 0x74).mirror(0xff00).w(FUNC(wp75_state::vctl_w));
	map(0x75, 0x75).mirror(0xff00).w(FUNC(wp75_state::vcrl_w));
	map(0x76, 0x76).mirror(0xff00).w(FUNC(wp75_state::vcrh_w));
//	map(0x80, 0xA0).mirror(0xff00).r(FUNC(wp75_state::unk_r));
//	map(0x80, 0xA0).mirror(0xff00).w(FUNC(wp75_state::unk_w));

//	map(0x78, 0x7b).mirror(0xff00).m(m_fdc, FUNC(upd765a_device::map)).umask16(0x0Ff);
//	map(0x7c, 0x7F).mirror(0xff00).m(m_fdc, FUNC(upd765a_device::map)).umask16(0x0Ff);

//	map(0x78, 0x7b).mirror(0xff00).r(FUNC(wp75_state::fdc_r));
//	map(0x78, 0x7b).mirror(0xff00).w(FUNC(wp75_state::fdc_w));

	map(0xB8,0xB8).mirror(0xff00).r(FUNC(wp75_state::kb_r));
	map(0xB8,0xB8).mirror(0xff00).w(FUNC(wp75_state::kb_w));

	map(0xC0,0xC0).mirror(0xff00).w(FUNC(wp75_state::ca_w));
	map(0xC8,0xC8).mirror(0xff00).w(FUNC(wp75_state::wh_w));
	map(0xD0,0xD0).mirror(0xff00).w(FUNC(wp75_state::lf_w));

	map(0xD8,0xD8).mirror(0xff00).w(FUNC(wp75_state::pg_w)); //solenoids and dc motor control
	map(0xF0,0xF0).mirror(0xff00).w(FUNC(wp75_state::pk_w)); //buzzer and ?

	map(0xB0,0xB0).mirror(0xff00).r(FUNC(wp75_state::sys_r)); //spec strap
	//map(0xE0,0xE0).mirror(0xff00).w(FUNC(wp75_state::bank_w)); //rom1 bank switch control
	map(0xA8,0xA8).mirror(0xff00).r(FUNC(wp75_state::portb_r));
	map(0xF8,0xF8).mirror(0xff00).w(FUNC(wp75_state::irq1_clear_w));

}

void wp75_state::machine_reset()
{
}

void wp75_state::video_start()
{
	v_ctl &= (1 << 2);

}

void wp75_state::init_wp70()
{
	//8 banks of 128k, but A16 is don't care
	membank("rom1")->configure_entries(0, 8, memregion("rom1")->base(), 0x20000);
	membank("rom1")->set_entry(0);
}


void wp75_state::init_wp75()
{
}

uint32_t wp75_state::screen_update_wp75(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	uint8_t *vram = memregion("vram")->base();

	uint32_t *scanline;
	int x, y;
	uint8_t pixels;
	static const uint32_t palette[2] = {0x282828, 0xffCC00 };

	for (y = 0; y < 240; y++)
	{
		scanline = &bitmap.pix32(y);
		for (x = 0; x < 91; x++)
		{
			pixels = vram[(y * 91) + x];

			*scanline++ = palette[(pixels>>7)&1];
			*scanline++ = palette[(pixels>>6)&1];
			*scanline++ = palette[(pixels>>5)&1];
			*scanline++ = palette[(pixels>>4)&1];
			*scanline++ = palette[(pixels>>3)&1];
			*scanline++ = palette[(pixels>>2)&1];
			*scanline++ = palette[(pixels>>1)&1];
			*scanline++ = palette[(pixels>>0)&1];
		}
	}
	return 0;
}


	WRITE8_MEMBER(wp75_state::bank_w)
{
	//FIXME is this correct?
	printf("Rom1 bank dat:%02x\n",data ); //, (data >>1) & 3);
	membank("rom1")->set_entry((data >> 1) & 3 ); //(data >> 1) & 3);
	//membank("rom1")->set_entry(data);

}

	WRITE8_MEMBER(wp75_state::vpl_w)
{
	vl = data & 0xff;
	v_ptr = (vh << 8) | (vl & 0xff);
//	printf("vpl_w :%02x   inc:%x \n", vl, (v_ctl & 0xC) >> 2);
}
	WRITE8_MEMBER(wp75_state::vph_w)
{
	vh = data & 0xFF;
	v_ptr = (vh << 8) | (vl & 0xff);
//	printf("vph_w :%02x   inc:%x\n",vh, (v_ctl & 0xC) >> 2);
}
	WRITE8_MEMBER(wp75_state::vram_w)
{
	uint8_t *vram = memregion("vram")->base();
	uint16_t index = (vh << 8) | (vl & 0xff);
//	printf("vram_w @:%02x :%02x inc %d\n",index, data, (v_ctl & 0xC) >> 2);

	if (index > 0x7FFF)
	{
		index = 0;
	}
	vram[index] = (data & 0xFF);
	if (index < 0x7FFF)
	{
		if ((v_ctl & 0xC) == 0) { //if vram addr auto increment
		v_ptr = index + 1;
		vl = (v_ptr & 0xFF);
		vh = (v_ptr >> 8);
		}
	}
	else {
		v_ptr = 0;
		vl = 0;
		vh = 0;
	}

}
	READ8_MEMBER(wp75_state::vram_r)
{
	uint8_t *vram = memregion("vram")->base();
	uint16_t index = (vh << 8) | (vl & 0xff);
	uint8_t data;
	if (index > 0x7FFF)
	{
		index = 0;
	}
	data = vram[index];
	if (index < 0x7FFF)
	{
		if ((v_ctl & 0xC) == 0) { //if vram addr auto increment
		v_ptr = index + 1;
		vl = (v_ptr & 0xFF);
		vh = (v_ptr >> 8);
		}
	}
	else {
		v_ptr = 0;
		vl = 0;
		vh = 0;
	}
	return data;
}
	WRITE8_MEMBER(wp75_state::vctl_w)
{
	v_ctl = data;
//	printf("vctl:%02x\n",data);
}
	WRITE8_MEMBER(wp75_state::vcrl_w)
{
//	printf("cursr addr l:%02x\n",data);
}

	WRITE8_MEMBER(wp75_state::vcrh_w)
{
//	printf("cursr addr h:%02x\n",data);
}


	READ8_MEMBER(wp75_state::wp70_sys_r)
{
	//11 = s5 70 or 66 line s4 0, s3,2,1,0 country code 
	uint8_t data = 0xc0; //??
	printf("sysr h:%02x\n",data);
	return data;
}
	READ8_MEMBER(wp75_state::sys_r)
{
	//11 = s5 70 or 66 line s4 0, s3,2,1,0 country code 
	uint8_t data = 0xcf; //0xc2
//	printf("sysr h:%02x\n",data);
	return data;
}

	WRITE8_MEMBER(wp75_state::pg_w)
{
	if (data & 4)
	{
	csol = false;
	}
	else {
	csol = true;
	}
	//printf("pg_w h:%02x\n",data);
}

	WRITE8_MEMBER(wp75_state::pk_w)
{
	if (data & 1)
	{
//	printf("pg_w BEEP h:%02x\n",data);
	}
	else {
	//printf("pg_w h:%02x\n",data);
	}
	
}


	READ8_MEMBER(wp75_state::portb_r)
{
	uint8_t data = 0xa;//e
	if(ca_idx)
		data = 0xe;//a

	//if (data != 0xa)
	//printf("portb h:%02x\n",data);
	return data;
}



        READ8_MEMBER(wp75_state::fdc_r)
{
        uint8_t data = 0;
	switch (offset)
	{
	case 0: data = 00; break;
	case 1: data = 00; break;
	case 2: data = 00; break;
	case 3: data = 00; break;
	}
//        printf("unk a:%02x h:%02x\n",offset,data);
        return data;
}

        WRITE8_MEMBER(wp75_state::fdc_w)
{
//        printf("fdc a:%02x h:%02x\n",offset,data);
}


	READ8_MEMBER(wp75_state::unk_r)
{
	uint8_t data = 0x00;
//	printf("unk a:%02x h:%02x\n",offset,data);
	return data;
}

	WRITE8_MEMBER(wp75_state::unk_w)
{
//	printf("unk a:%02x h:%02x\n",offset,data);
}

WRITE8_MEMBER(wp75_state::irq1_clear_w)
{
	ga_irq_mask = data;
	//printf("irq clr h:%02x\n",data);
	m_maincpu->set_input_line(INPUT_LINE_IRQ1, CLEAR_LINE);	
}


WRITE_LINE_MEMBER(wp75_state::drq_w)
{
	if (state)
	{
		m_maincpu->set_input_line(Z180_INPUT_LINE_DREQ0, ASSERT_LINE);	
	}
	else
	{
		m_maincpu->set_input_line(Z180_INPUT_LINE_DREQ0, CLEAR_LINE);	
	}
}


INTERRUPT_GEN_MEMBER(wp75_state::wp75_timer_interrupt)
{
	if (carriage_position <= 1 || csol)
	{
	ca_idx = true;
	}
	else {
	ca_idx = false;
	}

	m_maincpu->set_input_line(INPUT_LINE_IRQ1, HOLD_LINE);
}



	WRITE8_MEMBER(wp75_state::kb_w)
{
	kb_matrix = data;
//	printf("kb_w h:%02x\n",data);
}

	READ8_MEMBER(wp75_state::kb_r)
{

	uint8_t data = 0xFF;
	if (kb_matrix < 9)
	{
		data = m_y[kb_matrix]->read();
	}
	//printf("kb_r m:%x h:%02x\n", kb_matrix,data);
	return data;
}

	WRITE8_MEMBER(wp75_state::ca_w)
{
	int8_t dir = 0;
	ca_now = data;
	switch (data)
	{  // 33-> 99 -> CC ->66 going left
	  //  33 ->66 -> CC ->99 going right
	case 0x33:
	if (ca_prev == 0x66) dir = -1;
	if (ca_prev == 0x99) dir = 1;
	break;
	case 0x66:
	if (ca_prev == 0xCC) dir = -1;
	if (ca_prev == 0x33) dir = 1;
	break;
	case 0x99:
	if (ca_prev == 0x33) dir = -1;
	if (ca_prev == 0xCC) dir = 1;
	break;
	case 0xCC:
	if (ca_prev == 0x99) dir = -1;
	if (ca_prev == 0x66) dir = 1;
	break;
	}
	ca_prev = data;
	if (dir == 1) carriage_position++;
	if (dir == -1) carriage_position--;

//	if (data!=0xff)
//		printf("ca_w h:%02x  cpos: %d\n",data, carriage_position);
}

	WRITE8_MEMBER(wp75_state::wh_w)
{
// 	if(data != 0xff)
//	printf("wh_w h:%02x\n",data);
}

	WRITE8_MEMBER(wp75_state::lf_w)
{
//	if(data != 0xff)
//		printf("lf_w h:%02x\n",data);
}




	WRITE8_MEMBER(wp75_state::window_w)
{
	uint8_t *rom = memregion("maincpu")->base();	
	rom[0x60000 + offset] = data;
	//printf("window_w a:%02x h:%02x\n",offset ,data);
}
	READ8_MEMBER(wp75_state::window_r)
{
	uint8_t *rom = memregion("maincpu")->base();
	//printf("window_r a:%02x h:%02x\n",offset ,rom[0x60000 + offset]);
	return rom[0x60000 + offset];
}

	READ8_MEMBER(wp75_state::rom_wind_r)
{
	uint8_t *rom = memregion("maincpu")->base();
	//printf("romwind__r a:%02x h:%02x\n",offset ,rom[0x2000 + offset]);
	return rom[0x2000 + offset];// & 0x77FFF];
}



	WRITE8_MEMBER(wp75_state::fdd_w)
{
	printf("fdd_w p:%02x h:%02x\n",offset ,data);
}
	READ8_MEMBER(wp75_state::fdd_r)
{
	uint8_t data = 0xff;
	printf("fdd_r p:%02x h:%02x\n",offset ,data);
	return data;
}


static void wp75_floppies(device_slot_interface &device)
{
	device.option_add("35dd", FLOPPY_35_DD);
}

static const floppy_format_type wp75_floppy_formats[] = {
	FLOPPY_IMD_FORMAT,
	FLOPPY_MFI_FORMAT,
	FLOPPY_MFM_FORMAT,
	nullptr
};

	void wp75(machine_config &config);
	void wp70(machine_config &config);
	void wp2450ds(machine_config &config);
	void wp5500ds(machine_config &config);


MACHINE_CONFIG_START(wp75_state::wp70)
	/* basic machine hardware */
	MCFG_DEVICE_ADD("maincpu",Z180, XTAL(12'288'000) / 2) // location U17 HD64180
	MCFG_DEVICE_PROGRAM_MAP(wp70_mem)
	MCFG_DEVICE_IO_MAP(wp70_io)
//	MCFG_DEVICE_VBLANK_INT_DRIVER("screen", wp75_state, irq1_line_hold)
	MCFG_DEVICE_PERIODIC_INT_DRIVER(wp75_state, wp75_timer_interrupt,1000)

	// FDC9266 location U43
	UPD765A(config, m_fdc, XTAL(12'288'000), true, true);//not sure what this is clocked by 12.2Mhz cpu clock?
	//m_fdc->intrq_wr_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ2);
	//m_fdc->drq_wr_callback().set(FUNC(wp75_state::drq_w));
	m_fdc->drq_wr_callback().set_inputline(m_maincpu, Z180_INPUT_LINE_DREQ0);

	/* floppy drives */
	MCFG_FLOPPY_DRIVE_ADD(FDC9266_TAG ":0", wp75_floppies, "35dd", wp75_floppy_formats)

	/* video hardware */
	//screen_device &m_screen(SCREEN(config, "screen", SCREEN_TYPE_RASTER));
	//SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	//m_screen.set_refresh_hz(60); // Two interlaced fields at 60Hz => 30Hz frame rate
	//m_screen->set_vblank_time(ATTOSECONDS_IN_USEC(2500));
	//m_screen->set_size(819, 240);
	//m_screen->set_visarea(0, 819, 0, 240);
	//m_screen.set_physical_aspect(9, 5);
	//m_screen->set_palette(m_palette);

	//PALETTE(config, "palette", palette_device::MONOCHROME);

	// Video hardware
//	screen_device &screen(SCREEN(config, "screen", SCREEN_TYPE_RASTER));
//	screen.set_color(0xFFCC00);
//	screen.set_raw(XTAL(15'300'000), 819, 0, 819-1, 240, 0, 240-1);
//	screen.set_screen_update(FUNC(wp75_state::screen_update_wp75));
//	screen.set_palette("palette");
//	m_screen->set_refresh_hz(56.18);
//	m_screen->set_vblank_time(ATTOSECONDS_IN_USEC(3450));
//	m_screen->set_size(256, 256);
//	m_screen->set_visarea(0*8, 32*8-1, 2*8, 30*8-1);

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) /* not accurate */
	MCFG_SCREEN_SIZE(819, 240)
	MCFG_SCREEN_VISIBLE_AREA(0, 819-1, 0, 240-1)
	MCFG_SCREEN_UPDATE_DRIVER(wp75_state, screen_update_wp75)
	MCFG_SCREEN_PALETTE("palette")

MACHINE_CONFIG_END


MACHINE_CONFIG_START(wp75_state::wp75)
	/* basic machine hardware */
	MCFG_DEVICE_ADD("maincpu",Z180, XTAL(12'288'000) / 2) // location U17 HD64180
	MCFG_DEVICE_PROGRAM_MAP(wp75_mem)
	MCFG_DEVICE_IO_MAP(wp75_io)
//	MCFG_DEVICE_VBLANK_INT_DRIVER("screen", wp75_state, irq1_line_hold)
	MCFG_DEVICE_PERIODIC_INT_DRIVER(wp75_state, wp75_timer_interrupt,1000)

	// FDC9266 location U43
	UPD765A(config, m_fdc, XTAL(12'288'000), true, true);//not sure what this is clocked by 12.2Mhz cpu clock?
	//m_fdc->intrq_wr_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ2);
	//m_fdc->drq_wr_callback().set(FUNC(wp75_state::drq_w));
	m_fdc->drq_wr_callback().set_inputline(m_maincpu, Z180_INPUT_LINE_DREQ0);

	/* floppy drives */
	MCFG_FLOPPY_DRIVE_ADD(FDC9266_TAG ":0", wp75_floppies, "35dd", wp75_floppy_formats)

	/* video hardware */
	//screen_device &m_screen(SCREEN(config, "screen", SCREEN_TYPE_RASTER));
	//SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	//m_screen.set_refresh_hz(60); // Two interlaced fields at 60Hz => 30Hz frame rate
	//m_screen->set_vblank_time(ATTOSECONDS_IN_USEC(2500));
	//m_screen->set_size(819, 240);
	//m_screen->set_visarea(0, 819, 0, 240);
	//m_screen.set_physical_aspect(9, 5);
	//m_screen->set_palette(m_palette);

	PALETTE(config, "palette", palette_device::MONOCHROME);

	// Video hardware
//	screen_device &screen(SCREEN(config, "screen", SCREEN_TYPE_RASTER));
//	screen.set_color(0xFFCC00);
//	screen.set_raw(XTAL(15'300'000), 819, 0, 819-1, 240, 0, 240-1);
//	screen.set_screen_update(FUNC(wp75_state::screen_update_wp75));
//	screen.set_palette("palette");
//	m_screen->set_refresh_hz(56.18);
//	m_screen->set_vblank_time(ATTOSECONDS_IN_USEC(3450));
//	m_screen->set_size(256, 256);
//	m_screen->set_visarea(0*8, 32*8-1, 2*8, 30*8-1);

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) /* not accurate */
	MCFG_SCREEN_SIZE(819, 240)
	MCFG_SCREEN_VISIBLE_AREA(0, 819-1, 0, 240-1)
	MCFG_SCREEN_UPDATE_DRIVER(wp75_state, screen_update_wp75)
	MCFG_SCREEN_PALETTE("palette")

MACHINE_CONFIG_END

MACHINE_CONFIG_START(wp75_state::wp2450ds)
	/* basic machine hardware */
	MCFG_DEVICE_ADD("maincpu",Z180, XTAL(12'288'000) / 2) // location U17 HD64180
	MCFG_DEVICE_PROGRAM_MAP(wp75_mem)
	MCFG_DEVICE_IO_MAP(wp75_io)
	MCFG_DEVICE_PERIODIC_INT_DRIVER(wp75_state, wp75_timer_interrupt,1000)

	// FDC9266 location U43
	UPD765A(config, m_fdc, XTAL(12'288'000), true, true);//not sure what this is clocked by 12.2Mhz cpu clock?
	//m_fdc->intrq_wr_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ2);
	//m_fdc->drq_wr_callback().set(FUNC(wp75_state::drq_w));
	m_fdc->drq_wr_callback().set_inputline(m_maincpu, Z180_INPUT_LINE_DREQ0);

	/* floppy drives */
	MCFG_FLOPPY_DRIVE_ADD(FDC9266_TAG ":0", wp75_floppies, "35dd", wp75_floppy_formats)

	PALETTE(config, "palette", palette_device::MONOCHROME);

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) /* not accurate */
	MCFG_SCREEN_SIZE(819, 240)
	MCFG_SCREEN_VISIBLE_AREA(0, 819-1, 0, 240-1)
	MCFG_SCREEN_UPDATE_DRIVER(wp75_state, screen_update_wp75)
	MCFG_SCREEN_PALETTE("palette")

MACHINE_CONFIG_END

MACHINE_CONFIG_START(wp75_state::wp5500ds)
	/* basic machine hardware */
	MCFG_DEVICE_ADD("maincpu",Z180, XTAL(12'288'000) / 2) // location U17 HD64180
	MCFG_DEVICE_PROGRAM_MAP(wp5500ds_mem)
	MCFG_DEVICE_IO_MAP(wp75_io)
	MCFG_DEVICE_PERIODIC_INT_DRIVER(wp75_state, wp75_timer_interrupt,1000)

	// FDC9266 location U43
	UPD765A(config, m_fdc, XTAL(12'288'000), true, true);//not sure what this is clocked by 12.2Mhz cpu clock?
	//m_fdc->intrq_wr_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ2);
	//m_fdc->drq_wr_callback().set(FUNC(wp75_state::drq_w));
	m_fdc->drq_wr_callback().set_inputline(m_maincpu, Z180_INPUT_LINE_DREQ0);

	/* floppy drives */
	MCFG_FLOPPY_DRIVE_ADD(FDC9266_TAG ":0", wp75_floppies, "35dd", wp75_floppy_formats)

	PALETTE(config, "palette", palette_device::MONOCHROME);

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) /* not accurate */
	MCFG_SCREEN_SIZE(819, 240)
	MCFG_SCREEN_VISIBLE_AREA(0, 819-1, 0, 240-1)
	MCFG_SCREEN_UPDATE_DRIVER(wp75_state, screen_update_wp75)
	MCFG_SCREEN_PALETTE("palette")

MACHINE_CONFIG_END


/* ROM definition */
ROM_START( wp70 )	/*German WP-2b family derivative, 240kb floppy */
	ROM_REGION( 0x80000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD( "ua6386.bin", 0x0000, 0x80000, CRC(6d61e49d) SHA1(4f2ee2b479e8b630dad9abeb82aa86ac3ab63ef7))
	ROM_REGION( 0x80000, "rom1", ROMREGION_ERASEFF )
	ROM_LOAD( "UA2849-dict-ger.BIN", 0x0000, 0x80000, CRC(fa8712eb) SHA1(2d3454138c79e75604b30229c05ed8fb8e7d15fe))
	ROM_REGION( 0x8000, "vram", ROMREGION_ERASEFF )
ROM_END

ROM_START( wp75 )	/* 240kb floppy, 3 roms, not dumped yet */
	ROM_REGION( 0x80000, "maincpu", ROMREGION_ERASEFF ) //prog rom
	ROM_REGION( 0x80000, "rom1", ROMREGION_ERASEFF ) //dict rom
	ROM_REGION( 0x8000, "chargen", ROMREGION_ERASEFF ) //character generator
	ROM_REGION( 0x8000, "vram", ROMREGION_ERASEFF )
ROM_END

ROM_START( wp2450ds ) /*also wp2510ds*/
	ROM_REGION( 0x80000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD( "UA8504001.BIN", 0x0000, 0x80000, CRC(c515ab07) SHA1(2e99eb22e9e8ed613d1490f56b2a9296474db2e0))
	ROM_REGION( 0x8000, "vram", ROMREGION_ERASEFF )
ROM_END

ROM_START( wp5500ds )
	ROM_REGION( 0x80000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD( "UA8584001.BIN", 0x0000, 0x80000, CRC(99f29d66) SHA1(61947cd312b890cb3a8b6c1ab05f73833f36d6b2))
	ROM_REGION( 0x8000, "vram", ROMREGION_ERASEFF )
ROM_END


/* Driver */
		
/*    YEAR  NAME    PARENT  COMPAT  MACHINE INPUT   CLASS         INIT        COMPANY                    FULLNAME   FLAGS */

COMP( 1989, wp75,     0,      0,    wp75,     wp75, wp75_state, init_wp75, "Brother", "WP-75"    , MACHINE_NO_SOUND)
COMP( 1993, wp70,     wp75,      0,    wp70,     wp75, wp75_state, init_wp70, "Brother", "WP-70"    , MACHINE_NO_SOUND)
COMP( 1993, wp2450ds, wp75,      0,    wp2450ds, wp75, wp75_state, init_wp75, "Brother", "WP-2540DS", MACHINE_NO_SOUND)
COMP( 1993, wp5500ds, wp75,      0,    wp5500ds, wp75, wp75_state, init_wp75, "Brother", "WP-5500DS", MACHINE_NO_SOUND)


//COMP( 1989, wp70, 0,   wp75,      wp75,  wp75, wp75_state, init_wp75, "Brother", "WP-70", MACHINE_NOT_WORKING | MACHINE_NO_SOUND)
