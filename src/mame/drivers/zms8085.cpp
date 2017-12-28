// license:BSD-3-Clause
// copyright-holders:
/***********************************************************************************************************************************

Skeleton driver for Zentec's 8085-based Zephyr/ZMS-series terminals.

This represents the second generation of display terminals by Zentec Corporation. The
first, which included the ZMS-50, ZMS-70 & ZMS-90, had an 8080 CPU and different video
timings.

U.S. Patent No. 4,203,107 looks like the best available description of the Zephyr video
hardware, since no schematics or manuals have been found.

************************************************************************************************************************************/

#include "emu.h"
#include "cpu/i8085/i8085.h"
#include "screen.h"

class zms8085_state : public driver_device
{
public:
	zms8085_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_screen(*this, "screen")
		, m_mainram(*this, "mainram")
		, m_p_chargen(*this, "chargen")
	{ }

	u32 screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

private:
	required_device<cpu_device> m_maincpu;
	required_device<screen_device> m_screen;
	required_shared_ptr<u8> m_mainram;
	required_region_ptr<u8> m_p_chargen;
};


u32 zms8085_state::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	return 0;
}


static ADDRESS_MAP_START( mem_map, AS_PROGRAM, 8, zms8085_state )
	AM_RANGE(0x0000, 0x0fff) AM_ROM AM_REGION("maincpu", 0)
	AM_RANGE(0x1000, 0x1fff) AM_RAM AM_SHARE("mainram")
ADDRESS_MAP_END

static ADDRESS_MAP_START( io_map, AS_PROGRAM, 8, zms8085_state )
	AM_RANGE(0x68, 0x68) AM_WRITENOP
ADDRESS_MAP_END

static INPUT_PORTS_START( zephyr )
INPUT_PORTS_END

static MACHINE_CONFIG_START( zephyr )
	MCFG_CPU_ADD("maincpu", I8085A, XTAL_15_582MHz / 2) // divider not verified
	MCFG_CPU_PROGRAM_MAP(mem_map)
	MCFG_CPU_IO_MAP(io_map)

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_RAW_PARAMS(XTAL_15_582MHz, 980, 0, 800, 265, 0, 250)
	MCFG_SCREEN_UPDATE_DRIVER(zms8085_state, screen_update)
MACHINE_CONFIG_END

/**************************************************************************************************************

Zentec Zephyr (Model 00-441-01).
Chips: COM2017, i8085A, 2x unreadable (40-pin AMI DIP), Beeper
Crystal: 15.582000

***************************************************************************************************************/

ROM_START( zephyr )
	ROM_REGION(0x10000, "maincpu", 0)
	ROM_LOAD( "23-006-32c.bin",  0x0000, 0x0800, CRC(0a3a5447) SHA1(a8c25730a1d7e5b9c86e0d504afc923e931f9025) )
	ROM_LOAD( "23-067-004b.bin", 0x0800, 0x0800, CRC(37741104) SHA1(52b9998e0a8d4949e0dc7c3349b3681e13345061) )
	ROM_LOAD( "23-067-03b.bin",  0x1000, 0x0800, CRC(29cfa003) SHA1(9de7a8402173a2c448e54ee433ba3050db7b70bb) ) // this doesn't seem to fit

	ROM_REGION(0x0800, "chargen", 0)
	ROM_LOAD( "23-066-02a.bin",  0x0000, 0x0800, CRC(d5650b6c) SHA1(e6333e59018d9904f12abb270db4ba28aeff1995) )
ROM_END

COMP( 1979, zephyr, 0, 0, zephyr, zephyr, zms8085_state, 0, "Zentec", "Zephyr (00-441-01)", MACHINE_IS_SKELETON )
