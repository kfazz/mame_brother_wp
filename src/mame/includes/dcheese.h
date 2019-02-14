// license:BSD-3-Clause
// copyright-holders:Aaron Giles

/***************************************************************************

    HAR MadMax hardware

**************************************************************************/
#ifndef MAME_INCLUDES_DCHEESE_H
#define MAME_INCLUDES_DCHEESE_H

#pragma once

#include "machine/gen_latch.h"
#include "sound/bsmt2000.h"
#include "emupal.h"
#include "screen.h"

class dcheese_state : public driver_device
{
public:
	dcheese_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_palrom(*this, "palrom"),
		m_gfxrom(*this, "gfx"),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_screen(*this, "screen"),
		m_bsmt(*this, "bsmt"),
		m_soundlatch(*this, "soundlatch")
	{ }

	void fredmem(machine_config &config);
	void dcheese(machine_config &config);

	DECLARE_CUSTOM_INPUT_MEMBER(sound_latch_state_r);

protected:
	enum
	{
		TIMER_BLITTER_SCANLINE,
		TIMER_SIGNAL_IRQ
	};

	virtual void machine_start() override;
	virtual void video_start() override;

	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	required_region_ptr<uint16_t> m_palrom;
	required_region_ptr<uint8_t> m_gfxrom;

	/* video-related */
	uint16_t   m_blitter_color[2];
	uint16_t   m_blitter_xparam[16];
	uint16_t   m_blitter_yparam[16];
	uint16_t   m_blitter_vidparam[32];

	std::unique_ptr<bitmap_ind16> m_dstbitmap;
	emu_timer *m_blitter_timer;
	emu_timer *m_signal_irq_timer;

	/* misc */
	uint8_t    m_irq_state[5];
	uint8_t    m_sound_control;
	uint8_t    m_sound_msb_latch;

	/* devices */
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	required_device<screen_device> m_screen;
	required_device<bsmt2000_device> m_bsmt;
	required_device<generic_latch_8_device> m_soundlatch;

	DECLARE_WRITE16_MEMBER(eeprom_control_w);
	DECLARE_READ8_MEMBER(sound_status_r);
	DECLARE_WRITE8_MEMBER(sound_control_w);
	DECLARE_WRITE8_MEMBER(bsmt_data_w);
	DECLARE_WRITE16_MEMBER(blitter_color_w);
	DECLARE_WRITE16_MEMBER(blitter_xparam_w);
	DECLARE_WRITE16_MEMBER(blitter_yparam_w);
	DECLARE_WRITE16_MEMBER(blitter_vidparam_w);
	DECLARE_WRITE16_MEMBER(blitter_unknown_w);
	DECLARE_READ16_MEMBER(blitter_vidparam_r);
	void dcheese_palette(palette_device &palette) const;
	uint32_t screen_update_dcheese(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	INTERRUPT_GEN_MEMBER(dcheese_vblank);
	void dcheese_signal_irq(int which);
	void update_irq_state();
	IRQ_CALLBACK_MEMBER(irq_callback);
	void update_scanline_irq();
	void do_clear();
	void do_blit();

	void main_cpu_map(address_map &map);
	void sound_cpu_map(address_map &map);
};

#endif // MAME_INCLUDES_DCHEESE_H
