// license:BSD-3-Clause
// copyright-holders:David Haywood
/*

System Board Y2

'YATA-2 ASIC : 32-bit RISC processor @ 266 MHz'

This SoC looks suspiciously like the ones used for some Dreamcast derivatives, suggesting that this too
could be a DC / Naomi based platform, but with added encryption etc.

The System Board Y2 was released by SI Electronics, LTD. in 2009, The hardware was developed after Kaga
Electronics had acquired SI Electronics from Sega Sammy in 2008.  SI Electronics was also responsible
for the Atomiswave manufacturing, again suggesting this could be DC based.

The rest of the specs are quite close to DC / Naomi too.

--

ROMs are contained on a small sub-board

*/

#include "emu.h"
#include "emupal.h"
#include "screen.h"
#include "speaker.h"

class system_board_y2_state : public driver_device
{
public:
	system_board_y2_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
	{ }

	virtual void video_start() override;
	uint32_t screen_update_system_board_y2(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void system_board_y2(machine_config &config);
};

void system_board_y2_state::video_start()
{
}

uint32_t system_board_y2_state::screen_update_system_board_y2(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	return 0;
}

static INPUT_PORTS_START( system_board_y2 )
INPUT_PORTS_END


MACHINE_CONFIG_START(system_board_y2_state::system_board_y2)
	/*
	MCFG_CPU_ADD("maincpu", SH4LE, 266666666)
	MCFG_SH4_MD0(1)
	MCFG_SH4_MD1(0)
	MCFG_SH4_MD2(1)
	MCFG_SH4_MD3(0)
	MCFG_SH4_MD4(0)
	MCFG_SH4_MD5(1)
	MCFG_SH4_MD6(0)
	MCFG_SH4_MD7(1)
	MCFG_SH4_MD8(0)
	MCFG_SH4_CLOCK(CPU_CLOCK)
	*/

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(640, 480)
	MCFG_SCREEN_VISIBLE_AREA(0, 640-1, 0, 480-1)
	MCFG_SCREEN_UPDATE_DRIVER(system_board_y2_state, screen_update_system_board_y2)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD("palette", 0x1000)

	SPEAKER(config, "mono").front_center();
MACHINE_CONFIG_END

ROM_START( kof2002um )
	ROM_REGION( 0x8000000, "boot", 0 ) // presumably the boot code (encrypted?)
	ROM_LOAD( "s29gl01gp11fcr2.u103", 0x0000000, 0x8000000, CRC(722cbad1) SHA1(0292be12255ee4bd586166a3f5cd108c5453295b) )

	ROM_REGION( 0x42000000, "nand_u101", 0 ) // presumably accessed like a filesystem (encrypted)
	ROM_LOAD( "nand08gw3b2cn6.u101", 0x00000000, 0x42000000, CRC(ddeebb49) SHA1(6907205a0e0b69e2b37528f71647c70b4dd9e0f2) )
	ROM_REGION( 0x42000000, "nand_u102", 0 )
	ROM_LOAD( "nand08gw3b2cn6.u102", 0x00000000, 0x42000000, CRC(ac2dc586) SHA1(5168b4c0c6343b6c040a206da04fa7cdbc3b35b9) )
ROM_END

/* The title screen shows "The King of Fighters - Road to Revenge" (Chinese / English) while the speech on the title screen announcer says "The King of Fighters 2002 Unlimited Match"
   There is a PS2 version with the Unlimited Match title screen, but unless it's used for a different region the arcade doesn't show that title, only announces it. */
GAME( 2009, kof2002um,  0,    system_board_y2, system_board_y2,  system_board_y2_state, empty_init, ROT0, "SNK Playmore / New Channel", "The King of Fighters - Road to Revenge / The King of Fighters 2002 Unlimited Match", MACHINE_IS_SKELETON )
