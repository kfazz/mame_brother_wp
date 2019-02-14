// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    TeleNova Compis (Ultra) High Resolution Graphics adapter emulation

**********************************************************************/

#include "emu.h"
#include "hrg.h"
#include "screen.h"


//**************************************************************************
//  MACROS / CONSTANTS
//**************************************************************************

#define UPD7220_TAG     "upd7220"
#define SCREEN_TAG      "screen"



//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(COMPIS_HRG,  compis_hrg_device,  "compis_hrg",  "Compis HRG")
DEFINE_DEVICE_TYPE(COMPIS_UHRG, compis_uhrg_device, "compis_uhrg", "Compis UHRG")


//-------------------------------------------------
//  ADDRESS_MAP( upd7220_map )
//-------------------------------------------------

void compis_hrg_device::hrg_map(address_map &map)
{
	map.global_mask(0x7fff);
	map(0x00000, 0x7fff).ram().share("video_ram");
}

void compis_uhrg_device::uhrg_map(address_map &map)
{
	map.global_mask(0x1ffff);
	map(0x00000, 0x1ffff).ram().share("video_ram");
}


//-------------------------------------------------
//  UPD7220_DISPLAY_PIXELS_MEMBER( display_pixels )
//-------------------------------------------------

UPD7220_DISPLAY_PIXELS_MEMBER( compis_hrg_device::display_pixels )
{
	uint16_t i,gfx = m_video_ram[(address & 0x7fff) >> 1];
	const pen_t *pen = m_palette->pens();

	for(i=0; i<16; i++)
		bitmap.pix32(y, x + i) = pen[BIT(gfx, i)];
}


//-------------------------------------------------
//  UPD7220_DISPLAY_PIXELS_MEMBER( display_pixels )
//-------------------------------------------------

UPD7220_DISPLAY_PIXELS_MEMBER( compis_uhrg_device::display_pixels )
{
	uint16_t i,gfx = m_video_ram[(address & 0x1ffff) >> 1];
	const pen_t *pen = m_palette->pens();

	for(i=0; i<16; i++)
		bitmap.pix32(y, x + i) = pen[BIT(gfx, i)];
}


//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_START(compis_hrg_device::device_add_mconfig)
	MCFG_SCREEN_ADD_MONOCHROME(SCREEN_TAG, RASTER, rgb_t::green())
	MCFG_SCREEN_VIDEO_ATTRIBUTES(VIDEO_UPDATE_BEFORE_VBLANK)
	MCFG_SCREEN_REFRESH_RATE(50)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) // not accurate
	MCFG_SCREEN_SIZE(640, 400)
	MCFG_SCREEN_VISIBLE_AREA(0, 640-1, 0, 400-1)
	MCFG_SCREEN_UPDATE_DEVICE(UPD7220_TAG, upd7220_device, screen_update)

	UPD7220(config, m_crtc, 2252500); // unknown clock
	m_crtc->set_addrmap(0, &compis_hrg_device::hrg_map);
	m_crtc->set_display_pixels_callback(FUNC(compis_hrg_device::display_pixels), this);
	m_crtc->set_screen(SCREEN_TAG);

	PALETTE(config, m_palette, palette_device::MONOCHROME);
MACHINE_CONFIG_END


MACHINE_CONFIG_START(compis_uhrg_device::device_add_mconfig)
	MCFG_SCREEN_ADD_MONOCHROME(SCREEN_TAG, RASTER, rgb_t::green())
	MCFG_SCREEN_VIDEO_ATTRIBUTES(VIDEO_UPDATE_BEFORE_VBLANK)
	MCFG_SCREEN_REFRESH_RATE(50)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) // not accurate
	MCFG_SCREEN_SIZE(1280, 800)
	MCFG_SCREEN_VISIBLE_AREA(0, 1280-1, 0, 800-1)
	MCFG_SCREEN_UPDATE_DEVICE(UPD7220_TAG, upd7220_device, screen_update)

	UPD7220(config, m_crtc, 2252500*2); // unknown clock
	m_crtc->set_addrmap(0, &compis_uhrg_device::uhrg_map);
	m_crtc->set_display_pixels_callback(FUNC(compis_uhrg_device::display_pixels), this);
	m_crtc->set_screen(SCREEN_TAG);

	PALETTE(config, m_palette, palette_device::MONOCHROME);
MACHINE_CONFIG_END


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  compis_hrg_device - constructor
//-------------------------------------------------

compis_hrg_device::compis_hrg_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	device_compis_graphics_card_interface(mconfig, *this),
	m_crtc(*this, UPD7220_TAG),
	m_palette(*this, "palette"),
	m_video_ram(*this, "video_ram")
{
}

compis_hrg_device::compis_hrg_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	compis_hrg_device(mconfig, COMPIS_HRG, tag, owner, clock)
{
}

compis_uhrg_device::compis_uhrg_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	compis_hrg_device(mconfig, COMPIS_UHRG, tag, owner, clock)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void compis_hrg_device::device_start()
{
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void compis_hrg_device::device_reset()
{
}


//-------------------------------------------------
//  pcs6_6_r -
//-------------------------------------------------

uint8_t compis_hrg_device::pcs6_6_r(address_space &space, offs_t offset)
{
	uint8_t data = 0xff;

	if (offset < 2)
		data = m_crtc->read(space, offset & 0x01);
	else
		// monochrome only, hblank? vblank?
		if(offset == 2)
		{
			switch(m_unk_video)
			{
				case 0x04:
					m_unk_video = 0x44;
					break;
				case 0x44:
					m_unk_video = 0x64;
					break;
				default:
					m_unk_video = 0x04;
					break;
			}
			data = m_unk_video;
		}
	else
		data = 0;

	//logerror("%s PCS 6:6 read %04x : %02x\n", machine().describe_context(), offset, data);

	return data;
}


//-------------------------------------------------
//  pcs6_6_w -
//-------------------------------------------------

void compis_hrg_device::pcs6_6_w(address_space &space, offs_t offset, uint8_t data)
{
	//logerror("%s PCS 6:6 write %04x : %02x\n", machine().describe_context(), offset, data);

	// 0x336 is likely the color plane register
	if (offset < 2) m_crtc->write(space, offset & 0x01, data);
}
