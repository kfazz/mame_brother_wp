// license:BSD-3-Clause
// copyright-holders:Miodrag Milanovic
/***************************************************************************

        Callan Unistar Terminal

        2009-12-09 Skeleton driver.

        Chips used: i8275, AM9513, AM8085A-2, i8237, i8255, 2x 2651. XTAL 20MHz

****************************************************************************/

#include "emu.h"
#include "cpu/i8085/i8085.h"
#include "machine/am9513.h"
#include "machine/i8255.h"
#include "video/i8275.h"
#include "emupal.h"
#include "screen.h"


class unistar_state : public driver_device
{
public:
	unistar_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_p_chargen(*this, "chargen")
	{ }

	void unistar(machine_config &config);
private:
	DECLARE_PALETTE_INIT(unistar);
	I8275_DRAW_CHARACTER_MEMBER(draw_character);

	void unistar_io(address_map &map);
	void unistar_mem(address_map &map);

	virtual void machine_reset() override;
	required_device<cpu_device> m_maincpu;
	required_region_ptr<u8> m_p_chargen;
};


void unistar_state::unistar_mem(address_map &map)
{
	map.unmap_value_high();
	map(0x0000, 0x2fff).rom();
	map(0x8000, 0x8fff).ram();
}

void unistar_state::unistar_io(address_map &map)
{
	//ADDRESS_MAP_UNMAP_HIGH
	map.global_mask(0xff);
	map(0x84, 0x84).portr("CONFIG");
	map(0x8c, 0x8d).rw("stc", FUNC(am9513_device::read8), FUNC(am9513_device::write8));
	map(0x94, 0x97).rw("ppi", FUNC(i8255_device::read), FUNC(i8255_device::write));
	map(0x98, 0x99).rw("crtc", FUNC(i8275_device::read), FUNC(i8275_device::write));
	// ports used: 00,02,03(W),08(RW),09,0A,0B,0D,0F(W),80,81(R),82,83(W),84(R),8C,8D(W),94(R),97,98(W),99(RW)
	// if nonzero returned from port 94, it goes into test mode.
}

/* Input ports */
static INPUT_PORTS_START( unistar )
	PORT_START("CONFIG")
	PORT_DIPNAME(0x01, 0x01, "Screen Refresh Rate")
	PORT_DIPSETTING(0x01, "50 Hz")
	PORT_DIPSETTING(0x00, "60 Hz")
	PORT_BIT(0xfe, 0xfe, IPT_UNKNOWN)
INPUT_PORTS_END


void unistar_state::machine_reset()
{
}

PALETTE_INIT_MEMBER( unistar_state, unistar )
{
	palette.set_pen_color(0, 0, 0, 0 ); /* Black */
	palette.set_pen_color(1, 0, 255, 0 );   /* Full */
	palette.set_pen_color(2, 0, 128, 0 );   /* Dimmed */
}

I8275_DRAW_CHARACTER_MEMBER(unistar_state::draw_character)
{
}

/* F4 Character Displayer */
static const gfx_layout unistar_charlayout =
{
	8, 16,                  /* 8 x 16 characters */
	128,                    /* 128 characters */
	1,                  /* 1 bits per pixel */
	{ 0 },                  /* no bitplanes */
	/* x offsets */
	{ 0, 1, 2, 3, 4, 5, 6, 7 },
	/* y offsets */
	{  0*8,  1*8,  2*8,  3*8,  4*8,  5*8,  6*8,  7*8, 8*8,  9*8, 10*8, 11*8, 12*8, 13*8, 14*8, 15*8 },
	8*16                    /* every char takes 16 bytes */
};

static GFXDECODE_START( gfx_unistar )
	GFXDECODE_ENTRY( "chargen", 0x0000, unistar_charlayout, 0, 1 )
GFXDECODE_END

MACHINE_CONFIG_START(unistar_state::unistar)
	/* basic machine hardware */
	MCFG_DEVICE_ADD("maincpu", I8085A, 20_MHz_XTAL / 2)
	MCFG_DEVICE_PROGRAM_MAP(unistar_mem)
	MCFG_DEVICE_IO_MAP(unistar_io)

	AM9513(config, "stc", 8_MHz_XTAL).fout_cb().set("stc", FUNC(am9513_device::source1_w));

	MCFG_DEVICE_ADD("ppi", I8255A, 0)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(50)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) /* not accurate */
	MCFG_SCREEN_SIZE(640, 480)
	MCFG_SCREEN_VISIBLE_AREA(0, 640-1, 0, 480-1)
	MCFG_SCREEN_UPDATE_DEVICE("crtc", i8275_device, screen_update)

	MCFG_DEVICE_ADD("crtc", I8275, 20_MHz_XTAL / 8.5) // clock is probably wrong
	MCFG_I8275_CHARACTER_WIDTH(8)
	MCFG_I8275_DRAW_CHARACTER_CALLBACK_OWNER(unistar_state, draw_character)

	MCFG_DEVICE_ADD("gfxdecode", GFXDECODE, "palette", gfx_unistar)
	MCFG_PALETTE_ADD("palette", 3)
	MCFG_PALETTE_INIT_OWNER(unistar_state, unistar)
MACHINE_CONFIG_END

/* ROM definition */
ROM_START( unistar )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD( "280010c.u48", 0x0000, 0x1000, CRC(613ef521) SHA1(a77459e91617d2882778ab2dada74fcb5f44e949))
	ROM_LOAD( "280011c.u49", 0x1000, 0x1000, CRC(6cc5e704) SHA1(fb93645f51d5ad0635cbc8a9174c61f96799313d))
	ROM_LOAD( "280012c.u50", 0x2000, 0x1000, CRC(0b9ca5a5) SHA1(20bf4aeacda14ff7a3cf988c7c0bff6ec60406c7))

	ROM_REGION( 0x0800, "chargen", ROMREGION_ERASEFF )
	ROM_LOAD( "280014a.u1",  0x0000, 0x0800, CRC(a9e1b5b2) SHA1(6f5b597ee1417f1108ac5957b005a927acb5314a))
ROM_END

/* Driver */

//    YEAR  NAME     PARENT  COMPAT  MACHINE  INPUT    CLASS          INIT        COMPANY                FULLNAME                FLAGS
COMP( 198?, unistar, 0,      0,      unistar, unistar, unistar_state, empty_init, "Callan Data Systems", "Unistar 200 Terminal", MACHINE_IS_SKELETON )
