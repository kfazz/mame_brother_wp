// license:BSD-3-Clause
// copyright-holders:R. Belmont, Sven Schnelle
/***************************************************************************

  HP98544 high-resolution monochrome board

  VRAM at 0x200000, ROM and registers at 0x560000

***************************************************************************/

#include "emu.h"
#include "hp98544.h"
#include "screen.h"

#define HP98544_SCREEN_NAME   "98544_screen"
#define HP98544_ROM_REGION    "98544_rom"

ROM_START( hp98544 )
	ROM_REGION( 0x2000, HP98544_ROM_REGION, 0 )
	ROM_LOAD( "98544_1818-1999.bin", 0x000000, 0x002000, CRC(8c7d6480) SHA1(d2bcfd39452c38bc652df39f84c7041cfdf6bd51) )
ROM_END

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(HPDIO_98544, dio16_98544_device, "dio98544", "HP98544 high-res monochrome DIO video card")


//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_START(dio16_98544_device::device_add_mconfig)
	MCFG_SCREEN_ADD(HP98544_SCREEN_NAME, RASTER)
	MCFG_SCREEN_UPDATE_DEVICE(DEVICE_SELF, dio16_98544_device, screen_update)
	MCFG_SCREEN_SIZE(1024,1024)
	MCFG_SCREEN_VISIBLE_AREA(0, 1024-1, 0, 768-1)
	MCFG_SCREEN_REFRESH_RATE(70)

	MCFG_DEVICE_ADD("topcat", TOPCAT, XTAL(35904000))
	MCFG_TOPCAT_FB_WIDTH(1024)
	MCFG_TOPCAT_FB_HEIGHT(768)
	MCFG_TOPCAT_PLANEMASK(1)

MACHINE_CONFIG_END

//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

const tiny_rom_entry *dio16_98544_device::device_rom_region() const
{
	return ROM_NAME( hp98544 );
}

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  dio16_98544_device - constructor
//-------------------------------------------------

dio16_98544_device::dio16_98544_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	dio16_98544_device(mconfig, HPDIO_98544, tag, owner, clock)
{
}

dio16_98544_device::dio16_98544_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	device_dio16_card_interface(mconfig, *this),
	device_memory_interface(mconfig, *this),
	m_topcat(*this, "topcat"),
	m_space_config("vram", ENDIANNESS_BIG, 8, 20, 0, address_map_constructor(FUNC(dio16_98544_device::map), this)),
	m_rom(*this, HP98544_ROM_REGION),
	m_vram(*this, "vram")
{
}

void dio16_98544_device::map(address_map& map)
{
	map(0, 0xfffff).ram().share("vram");
}

device_memory_interface::space_config_vector dio16_98544_device::memory_space_config() const
{
	return space_config_vector{ std::make_pair(0, &m_space_config) };
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void dio16_98544_device::device_start()
{
	dio().install_memory(
			0x200000, 0x2fffff,
			read16_delegate(FUNC(topcat_device::vram_r), static_cast<topcat_device*>(m_topcat)),
			write16_delegate(FUNC(topcat_device::vram_w), static_cast<topcat_device*>(m_topcat)));
	dio().install_memory(
			0x560000, 0x563fff,
			read16_delegate(FUNC(dio16_98544_device::rom_r), this),
			write16_delegate(FUNC(dio16_98544_device::rom_w), this));
	dio().install_memory(
			0x564000, 0x567fff,
			read16_delegate(FUNC(topcat_device::ctrl_r), static_cast<topcat_device*>(m_topcat)),
			write16_delegate(FUNC(topcat_device::ctrl_w), static_cast<topcat_device*>(m_topcat)));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void dio16_98544_device::device_reset()
{
}

READ16_MEMBER(dio16_98544_device::rom_r)
{
	return 0xff00 | m_rom[offset];
}

// the video chip registers live here, so these writes are valid
WRITE16_MEMBER(dio16_98544_device::rom_w)
{
}

uint32_t dio16_98544_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	int startx, starty, endx, endy;
	m_topcat->get_cursor_pos(startx, starty, endx, endy);

	for (int y = 0; y < m_v_pix; y++) {
		uint32_t *scanline = &bitmap.pix32(y);
		for (int x = 0; x < 1024; x++) {
			uint8_t tmp = m_vram[y * m_h_pix + x];
			if (y >= starty && y <= endy && x >= startx && x <= endx)
				tmp |= 0xff;
			*scanline++ = tmp ? rgb_t(255,255,255) : rgb_t(0, 0, 0);
		}
	}
	return 0;
}

