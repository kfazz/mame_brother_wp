// license:BSD-3-Clause
// copyright-holders:Pierpaolo Prazzoli
/********************************************************************

 Eolith 16 bits hardware

 Supported Games:
 - KlonDike+      (c) 1999 Eolith

 driver by Pierpaolo Prazzoli

**********************************************************************/

#include "emu.h"
#include "includes/eolith.h"

#include "cpu/e132xs/e132xs.h"
#include "machine/eepromser.h"
#include "sound/okim6295.h"

#include "emupal.h"
#include "speaker.h"


class eolith16_state : public eolith_state
{
public:
	eolith16_state(const machine_config &mconfig, device_type type, const char *tag)
		: eolith_state(mconfig, type, tag)
		, m_special_io(*this, "SPECIAL")
		, m_vram(*this, "vram", 16)
		, m_vrambank(*this, "vrambank")
	{
	}

	void eolith16(machine_config &config);

	void init_eolith16();

protected:
	virtual void video_start() override;

private:
	required_ioport m_special_io;
	required_shared_ptr<uint8_t> m_vram;
	required_memory_bank m_vrambank;

	DECLARE_WRITE16_MEMBER(eeprom_w);
	DECLARE_READ16_MEMBER(eolith16_custom_r);

	DECLARE_PALETTE_INIT(eolith16);

	uint32_t screen_update_eolith16(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void eolith16_map(address_map &map);
};



WRITE16_MEMBER(eolith16_state::eeprom_w)
{
	m_vrambank->set_entry(((data & 0x80) >> 7) ^ 1);
	machine().bookkeeping().coin_counter_w(0, data & 1);

	m_eepromoutport->write(data, 0xff);

	//data & 0x100 and data & 0x004 always set
}

READ16_MEMBER(eolith16_state::eolith16_custom_r)
{
	speedup_read();
	return m_special_io->read();
}

void eolith16_state::eolith16_map(address_map &map)
{
	map(0x00000000, 0x001fffff).ram();
	map(0x50000000, 0x5000ffff).bankrw("vrambank").share("vram");
	map(0x90000000, 0x9000002f).nopw(); //?
	map(0xff000000, 0xff1fffff).rom().region("maindata", 0);
	map(0xffe40001, 0xffe40001).rw("oki", FUNC(okim6295_device::read), FUNC(okim6295_device::write));
	map(0xffe80000, 0xffe80001).w(FUNC(eolith16_state::eeprom_w));
	map(0xffea0000, 0xffea0001).r(FUNC(eolith16_state::eolith16_custom_r));
	map(0xffea0002, 0xffea0003).portr("SYSTEM");
	map(0xffec0000, 0xffec0001).nopr(); // not used?
	map(0xffec0002, 0xffec0003).portr("INPUTS");
	map(0xfff80000, 0xffffffff).rom().region("maincpu", 0);
}

static INPUT_PORTS_START( eolith16 )
	PORT_START("SPECIAL")
	PORT_BIT( 0x0010, IP_ACTIVE_HIGH, IPT_CUSTOM ) PORT_READ_LINE_DEVICE_MEMBER("eeprom", eeprom_serial_93cxx_device, do_read)
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_CUSTOM ) PORT_CUSTOM_MEMBER(DEVICE_SELF, eolith16_state, eolith_speedup_getvblank, nullptr)
	PORT_BIT( 0xff6f, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("SYSTEM")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_SERVICE_NO_TOGGLE( 0x0040, IP_ACTIVE_LOW )
	PORT_BIT( 0xff80, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("INPUTS")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_PLAYER(1)
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_PLAYER(1)
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_PLAYER(1)
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(1)
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(1)
	PORT_BIT( 0xffe0, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START( "EEPROMOUT" )
	PORT_BIT( 0x00000010, IP_ACTIVE_HIGH, IPT_OUTPUT ) PORT_WRITE_LINE_DEVICE_MEMBER("eeprom", eeprom_serial_93cxx_device, cs_write)
	PORT_BIT( 0x00000020, IP_ACTIVE_HIGH, IPT_OUTPUT ) PORT_WRITE_LINE_DEVICE_MEMBER("eeprom", eeprom_serial_93cxx_device, clk_write)
	PORT_BIT( 0x00000040, IP_ACTIVE_HIGH, IPT_OUTPUT ) PORT_WRITE_LINE_DEVICE_MEMBER("eeprom", eeprom_serial_93cxx_device, di_write)
INPUT_PORTS_END

void eolith16_state::video_start()
{
	m_vrambank->configure_entries(0, 2, memshare("vram")->ptr(), 0x10000);
	m_vrambank->set_entry(0);
}

uint32_t eolith16_state::screen_update_eolith16(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	for (int y = 0; y < 204; y++)
	{
		for (int x = 0; x < 320; x++)
		{
			bitmap.pix16(y, x) = m_vram[(y * 320) + x] & 0xff;
		}
	}
	return 0;
}


// setup a custom palette because pixels use 8 bits per color
PALETTE_INIT_MEMBER(eolith16_state,eolith16)
{
	for (int c = 0; c < 256; c++)
	{
		int bit0,bit1,bit2,r,g,b;
		bit0 = (c >> 0) & 0x01;
		bit1 = (c >> 1) & 0x01;
		bit2 = (c >> 2) & 0x01;
		r = 0x21 * bit0 + 0x47 * bit1 + 0x97 * bit2;
		bit0 = (c >> 3) & 0x01;
		bit1 = (c >> 4) & 0x01;
		bit2 = (c >> 5) & 0x01;
		g = 0x21 * bit0 + 0x47 * bit1 + 0x97 * bit2;
		bit0 = (c >> 6) & 0x01;
		bit1 = (c >> 7) & 0x01;
		b = 0x55 * bit0 + 0xaa * bit1;

		palette.set_pen_color(c,rgb_t(r,g,b));
	}
}


MACHINE_CONFIG_START(eolith16_state::eolith16)
	MCFG_DEVICE_ADD("maincpu", E116T, XTAL(60'000'000))        /* no internal multiplier */
	MCFG_DEVICE_PROGRAM_MAP(eolith16_map)
	MCFG_TIMER_DRIVER_ADD_SCANLINE("scantimer", eolith16_state, eolith_speedup, "screen", 0, 1)

	MCFG_DEVICE_ADD("eeprom", EEPROM_SERIAL_93C66_8BIT)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500))
	MCFG_SCREEN_SIZE(512, 262)
	MCFG_SCREEN_VISIBLE_AREA(0, 319, 0, 199)
	MCFG_SCREEN_UPDATE_DRIVER(eolith16_state, screen_update_eolith16)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD("palette", 256)

	MCFG_PALETTE_INIT_OWNER(eolith16_state,eolith16)

	SPEAKER(config, "lspeaker").front_left();
	SPEAKER(config, "rspeaker").front_right();

	MCFG_DEVICE_ADD("oki", OKIM6295, XTAL(1'000'000), okim6295_device::PIN7_HIGH)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "lspeaker", 1.0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "rspeaker", 1.0)
MACHINE_CONFIG_END

/*

KlonDike+
Eolith, 1999

This game is like Freecell which comes with Windows XP etc

PCB Layout
----------

9812C
|-------------------------------------------|
|TDA1519A      KD.U28  PAL        RESET_SW  |
|     VOL  M6295   1MHz                60MHz|
|               EV0514-001                  |
|        14.31818MHz            E1-16T      |
|                                           |
|                          GM71C18160       |
|   TEST_SW                                 |
|J                                          |
|A                  KM6161002               |
|M  SERV_SW                                 |
|M                              KD.U5       |
|A                                          |
|                               93C66       |
|                                           |
|                                           |
|                                     TL7705|
|                                           |
|    *    *    *    *    *    KD.U31        |
|                                           |
|-------------------------------------------|
Notes:
      E1-16T - Main CPU, HyperStone E1-16T, clock input 60.000MHz (TQFP100)
      M6294  - Oki M6295 sound chip, clock 1.000MHz, sample rate = 1000000 / 132 (QFP44)
      93C66  - Atmel 93C66 4096bit Serial EEPROM (DIP8)
   KM6161002 - Samsung Electronics KM6161002BJ-10 64k x16 High Speed CMOS Static RAM (SOJ44)
  GM71C18160 - LG Semiconductor GM71C18160CJ6 1M x16 DRAM (SOJ42)
  EV0514-001 - Custom Eolith IC (QFP100)
      VSync  - 60Hz
      HSync  - 15.64kHz
          *  - Empty DIP42 sockets
       ROMs  -
               KD.U28 - TMS27C040 EPROM, M6295 samples (DIP32)
               KD.U5  - TMS27C040 EPROM, Main Program (DIP32)
               KD.U31 - ST M27C160 EPROM, Graphics Data (DIP42)
*/

ROM_START( klondkp )
	ROM_REGION16_BE( 0x80000, "maincpu", 0 ) /* E1-16T program code */
	ROM_LOAD( "kd.u5",  0x000000, 0x080000, CRC(591f0c73) SHA1(a9f338204c77a724fa6a6e08d78ca89bd5191aba) )

	ROM_REGION16_BE( 0x200000, "maindata", 0 ) /* gfx data */
	ROM_LOAD16_WORD_SWAP( "kd.u31", 0x000000, 0x200000, CRC(e5dd12b5) SHA1(0a0cd75cbcdccce3575e5a58ba09c88452e1a5ee) )

	ROM_REGION( 0x80000, "oki", 0 ) /* oki samples */
	ROM_LOAD( "kd.u28", 0x000000, 0x080000, CRC(c12112a1) SHA1(729bbaca6db933a730099a4a560a10ed99cae1c3) )
ROM_END

void eolith16_state::init_eolith16()
{
	init_speedup();
}

GAME( 1999, klondkp, 0, eolith16, eolith16, eolith16_state, init_eolith16, ROT0, "Eolith", "KlonDike+", MACHINE_SUPPORTS_SAVE )
