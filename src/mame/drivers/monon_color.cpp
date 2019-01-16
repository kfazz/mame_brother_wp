// license:BSD-3-Clause
// copyright-holders:David Haywood
/***************************************************************************
   Monon Color - Chinese handheld

   see
   https://gbatemp.net/threads/the-monon-color-a-new-video-game-console-from-china.467788/
   https://twitter.com/Splatoon2weird/status/1072182093206052864

   uses AX208 CPU (custom 8051 @ 96Mhz with single cycle instructions, extended '16 bit' opcodes + integrated video, jpeg decoder etc.)
   https://docplayer.net/52724058-Ax208-product-specification.html

***************************************************************************/

#include "emu.h"
#include "cpu/mcs51/axc51-core.h"
#include "emupal.h"
#include "screen.h"
#include "speaker.h"
#include "bus/generic/slot.h"
#include "bus/generic/carts.h"

class monon_color_state : public driver_device
{
public:
	monon_color_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_cart(*this, "cartslot"),
		m_maincpu(*this, "maincpu"),
		m_screen(*this, "screen"),
		m_palette(*this, "palette"),
		m_mainram(*this, "mainram"),
		m_otherram(*this, "otherram")
	{ }

	void monon_color(machine_config &config);
protected:
	// driver_device overrides
	virtual void machine_start() override;
	virtual void machine_reset() override;

	virtual void video_start() override;

	// screen updates
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
private:
	required_device<generic_slot_device> m_cart;
	required_device<cpu_device> m_maincpu;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
	required_shared_ptr<uint8_t> m_mainram;
	required_shared_ptr<uint8_t> m_otherram;

	DECLARE_DEVICE_IMAGE_LOAD_MEMBER(monon_color_cart);

	void monon_color_map(address_map &map);
};

void monon_color_state::machine_start()
{
	uint8_t* flash = memregion("flash")->base();
	uint8_t* maincpu = &m_mainram[0];

	memcpy(maincpu, flash+0x200, 0x1e00); // 0x4000-0x5dff fixed code?

	// there are a whole bunch of blocks that map at 0x5e00 (boot code jumps straight to 0x5e00)

	memcpy(maincpu+0x1e00, flash+0x2000, 0x1000); // clears RAM, sets up stack etc. but then jumps to 0x9xxx where we have nothing (probably the correct initial block tho)
//  memcpy(maincpu+0x1e00, flash+0x4200, 0x1000); // just set register + a jump (to function that writes to UART)
//  memcpy(maincpu+0x1e00, flash+0x4c00, 0x1000);
//  memcpy(maincpu+0x1e00, flash+0x5600, 0x1000);
//  memcpy(maincpu+0x1e00, flash+0x6000, 0x1000); // ends up reting with nothing on the stack
//  memcpy(maincpu+0x1e00, flash+0x6a00, 0x1000);
//  memcpy(maincpu+0x1e00, flash+0x7e00, 0x1000);
//  memcpy(maincpu+0x1e00, flash+0x8800, 0x1000);
//  memcpy(maincpu+0x1e00, flash+0x9200, 0x1000);

	/*  block starting at e000 in flash is not code? (or encrypted?)
	    no code to map at 0x9000 in address space (possible BIOS?)
	    no code in flash ROM past the first 64kb(?) which is basically the same on all games, must be some kind of script interpreter? J2ME maybe?

	    there are 4 different 'versions' of the code in the dumped ROMs, where the code is the same the roms match up until 0x50000 after which the game specific data starts

	    by game number:

	    101,102,103,104,105          (1st revision)
	    106,107                      (2nd revision)
	    201                          (3rd revision)
	    202,203,204,205,301,303,304  (4th revision)
	*/
}



void monon_color_state::machine_reset()
{
	m_maincpu->set_state_int(MCS51_PC, 0x4000);
}

void monon_color_state::video_start()
{
}


uint32_t monon_color_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	return 0;
}

static INPUT_PORTS_START( monon_color )
INPUT_PORTS_END

void monon_color_state::monon_color_map(address_map &map)
{
	map(0x4000, 0x6fff).ram().share("mainram");
	map(0x9000, 0x9fff).ram().share("otherram"); // lots of jumps to here, is there some kind of BIOS? none of the code appears to map here?
}

MACHINE_CONFIG_START(monon_color_state::monon_color)

	/* basic machine hardware */
	MCFG_DEVICE_ADD("maincpu", AX208, 96000000) // (8051 / MCS51 derived) incomplete core!
	MCFG_DEVICE_PROGRAM_MAP(monon_color_map)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500))
	MCFG_SCREEN_UPDATE_DRIVER(monon_color_state, screen_update)
	MCFG_SCREEN_SIZE(32*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 32*8-1, 2*8, 30*8-1)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_PALETTE_ADD("palette", 256)

	/* sound hardware */
	SPEAKER(config, "mono").front_center();

	MCFG_GENERIC_CARTSLOT_ADD("cartslot", generic_plain_slot, "monon_color_cart")
	MCFG_GENERIC_EXTENSIONS("bin")
	MCFG_GENERIC_WIDTH(GENERIC_ROM8_WIDTH)
	MCFG_GENERIC_LOAD(monon_color_state, monon_color_cart)
	MCFG_GENERIC_MANDATORY

	/* software lists */
	MCFG_SOFTWARE_LIST_ADD("cart_list","monon_color")
MACHINE_CONFIG_END

DEVICE_IMAGE_LOAD_MEMBER( monon_color_state, monon_color_cart )
{
	uint32_t size = m_cart->common_get_size("rom");
	std::vector<uint8_t> temp;
	temp.resize(size);
	m_cart->rom_alloc(size, GENERIC_ROM8_WIDTH, ENDIANNESS_LITTLE);
	m_cart->common_load_rom(&temp[0], size, "rom");

	memcpy(memregion("flash")->base(), &temp[0], size);

	return image_init_result::PASS;
}

ROM_START( mononcol )
	ROM_REGION( 0x1000000, "flash", ROMREGION_ERASE00 )
ROM_END

CONS( 2011, mononcol,    0,          0,  monon_color,  monon_color,    monon_color_state, empty_init,    "M&D",   "Monon Color", MACHINE_IS_SKELETON )
