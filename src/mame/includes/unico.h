// license:BSD-3-Clause
// copyright-holders:Luca Elia
#ifndef MAME_INCLUDES_UNICO_H
#define MAME_INCLUDES_UNICO_H

#pragma once

#include "sound/okim6295.h"
#include "machine/eepromser.h"
#include "emupal.h"
#include "screen.h"

class unico_state : public driver_device
{
public:
	unico_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_palette(*this, "palette"),
		m_leds(*this, "led%u", 0U),
		m_vram(*this, "vram", 0),
		m_scroll(*this, "scroll", 0),
		m_spriteram(*this, "spriteram", 0),
		m_maincpu(*this, "maincpu"),
		m_oki(*this, "oki"),
		m_gfxdecode(*this, "gfxdecode")
	{ }

	void burglarx(machine_config &config);

protected:
	DECLARE_PALETTE_DECODER(unico_R6G6B6X);
	DECLARE_READ16_MEMBER(vram_r);
	DECLARE_WRITE16_MEMBER(vram_w);
	DECLARE_READ16_MEMBER(scroll_r);
	DECLARE_WRITE16_MEMBER(scroll_w);
	DECLARE_READ16_MEMBER(spriteram_r);
	DECLARE_WRITE16_MEMBER(spriteram_w);

	DECLARE_WRITE8_MEMBER(burglarx_okibank_w);
	TILE_GET_INFO_MEMBER(get_tile_info);
	virtual void machine_start() override;
	virtual void video_start() override;
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void draw_sprites(screen_device &screen, bitmap_ind16 &bitmap,const rectangle &cliprect);

	void burglarx_map(address_map &map);

	required_device<palette_device> m_palette;
	output_finder<2> m_leds;

private:
	required_shared_ptr<uint16_t> m_vram;
	required_shared_ptr<uint16_t> m_scroll;
	tilemap_t *m_tilemap[3];
	int m_sprites_scrolldx;
	int m_sprites_scrolldy;
	required_shared_ptr<uint16_t> m_spriteram;

	required_device<cpu_device> m_maincpu;
	optional_device<okim6295_device> m_oki;
	required_device<gfxdecode_device> m_gfxdecode;
};

class zeropnt_state : public unico_state
{
public:
	zeropnt_state(const machine_config &mconfig, device_type type, const char *tag) :
		unico_state(mconfig, type, tag),
		m_okibank(*this, "okibank"),
		m_screen(*this, "screen"),
		m_gun_axes(*this, { "Y0", "X0", "Y1", "X1" })
	{ }

	void zeropnt(machine_config &config);

protected:
	virtual void machine_start() override;

	DECLARE_WRITE8_MEMBER(zeropnt_okibank_leds_w);
	DECLARE_READ16_MEMBER(gunx_0_msb_r);
	DECLARE_READ16_MEMBER(guny_0_msb_r);
	DECLARE_READ16_MEMBER(gunx_1_msb_r);
	DECLARE_READ16_MEMBER(guny_1_msb_r);

	required_memory_bank m_okibank;

	void zeropnt_map(address_map &map);
	void zeropnt_oki_map(address_map &map);

private:
	enum { Y0, X0, Y1, X1 }; // gun axis indices

	required_device<screen_device> m_screen;
	required_ioport_array<4> m_gun_axes;
};

class zeropnt2_state : public zeropnt_state
{
public:
	zeropnt2_state(const machine_config &mconfig, device_type type, const char *tag) :
		zeropnt_state(mconfig, type, tag),
		m_eeprom(*this, "eeprom")
	{ }

	void zeropnt2(machine_config &config);

protected:
	virtual void machine_start() override;

	DECLARE_READ32_MEMBER(zeropnt2_gunx_0_msb_r);
	DECLARE_READ32_MEMBER(zeropnt2_guny_0_msb_r);
	DECLARE_READ32_MEMBER(zeropnt2_gunx_1_msb_r);
	DECLARE_READ32_MEMBER(zeropnt2_guny_1_msb_r);
	DECLARE_WRITE8_MEMBER(zeropnt2_okibank);
	DECLARE_WRITE8_MEMBER(leds_w);

	DECLARE_WRITE32_MEMBER(eeprom_w);

	void zeropnt2_map(address_map &map);

private:
	required_device<eeprom_serial_93cxx_device> m_eeprom;
};

#endif // MAME_INCLUDES_UNICO_H
