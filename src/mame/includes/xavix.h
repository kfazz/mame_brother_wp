// license:BSD-3-Clause
// copyright-holders:David Haywood
#ifndef MAME_INCLUDES_XAVIX_H
#define MAME_INCLUDES_XAVIX_H

#include "cpu/m6502/xavix.h"
#include "cpu/m6502/xavix2000.h"
#include "machine/timer.h"
#include "emupal.h"
#include "screen.h"
#include "speaker.h"
#include "machine/bankdev.h"
#include "machine/i2cmem.h"


class xavix_state : public driver_device
{
public:
	xavix_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_screen(*this, "screen"),
		m_mainram(*this, "mainram"),
		m_fragment_sprite(*this, "fragment_sprite"),
		m_rom_dma_src(*this,"rom_dma_src"),
		m_rom_dma_dst(*this,"rom_dma_dst"),
		m_rom_dma_len(*this,"rom_dma_len"),
		m_palram_sh(*this, "palram_sh"),
		m_palram_l(*this, "palram_l"),
		m_bmp_palram_sh(*this, "bmp_palram_sh"),
		m_bmp_palram_l(*this, "bmp_palram_l"),
		m_bmp_base(*this, "bmp_base"),
		m_colmix_sh(*this, "colmix_sh"),
		m_colmix_l(*this, "colmix_l"),
		m_colmix_ctrl(*this, "colmix_ctrl"),
		m_posirq_x(*this, "posirq_x"),
		m_posirq_y(*this, "posirq_y"),
		m_segment_regs(*this, "segment_regs"),
		m_palette(*this, "palette"),
		m_in0(*this, "IN0"),
		m_in1(*this, "IN1"),
		m_region(*this, "REGION"),
		m_gfxdecode(*this, "gfxdecode"),
		m_lowbus(*this, "lowbus"),
		m_i2cmem(*this, "i2cmem")
	{ }

	void xavix(machine_config &config);
	void xavixp(machine_config &config);
	void xavix2000(machine_config &config);

	void xavix_i2c(machine_config &config);
	void xavix2000_i2c(machine_config &config);

	void init_xavix();

	DECLARE_CUSTOM_INPUT_MEMBER(rad_rh_in1_08_r);

private:
	// screen updates
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	void xavix_map(address_map &map);

	void xavix_lowbus_map(address_map &map);
	void xavix_extbus_map(address_map &map);
void superxavix_lowbus_map(address_map &map);

	INTERRUPT_GEN_MEMBER(interrupt);
	TIMER_DEVICE_CALLBACK_MEMBER(scanline_cb);

	// driver_device overrides
	virtual void machine_start() override;
	virtual void machine_reset() override;

	virtual void video_start() override;

	DECLARE_READ8_MEMBER(main_r);
	DECLARE_WRITE8_MEMBER(main_w);
	DECLARE_READ8_MEMBER(main2_r);
	DECLARE_WRITE8_MEMBER(main2_w);
	DECLARE_READ8_MEMBER(main3_r);
	DECLARE_WRITE8_MEMBER(main3_w);

	DECLARE_READ8_MEMBER(extbus_r) { return m_rgn[(offset) & (m_rgnlen - 1)]; }
	DECLARE_WRITE8_MEMBER(extbus_w) { logerror("write to external bus %06x %02x\n", offset, data); }

	DECLARE_WRITE8_MEMBER(extintrf_7900_w);
	DECLARE_WRITE8_MEMBER(extintrf_7901_w);
	DECLARE_WRITE8_MEMBER(extintrf_7902_w);

	DECLARE_WRITE8_MEMBER(xavix_7a80_w);
	DECLARE_WRITE8_MEMBER(adc_7b00_w);
	DECLARE_READ8_MEMBER(adc_7b80_r);
	DECLARE_WRITE8_MEMBER(adc_7b80_w);
	DECLARE_READ8_MEMBER(adc_7b81_r);
	DECLARE_WRITE8_MEMBER(adc_7b81_w);

	DECLARE_WRITE8_MEMBER(slotreg_7810_w);

	DECLARE_WRITE8_MEMBER(rom_dmatrg_w);

	DECLARE_WRITE8_MEMBER(rom_dmasrc_w);

	DECLARE_WRITE8_MEMBER(rom_dmadst_w);
	DECLARE_WRITE8_MEMBER(rom_dmalen_w);
	DECLARE_READ8_MEMBER(rom_dmastat_r);

	DECLARE_WRITE8_MEMBER(spritefragment_dma_params_1_w);
	DECLARE_WRITE8_MEMBER(spritefragment_dma_params_2_w);
	DECLARE_WRITE8_MEMBER(spritefragment_dma_trg_w);
	DECLARE_READ8_MEMBER(spritefragment_dma_status_r);

	DECLARE_READ8_MEMBER(io0_data_r);
	DECLARE_READ8_MEMBER(io1_data_r);
	DECLARE_READ8_MEMBER(io0_direction_r);
	DECLARE_READ8_MEMBER(io1_direction_r);

	DECLARE_WRITE8_MEMBER(io0_data_w);
	DECLARE_WRITE8_MEMBER(io1_data_w);
	DECLARE_WRITE8_MEMBER(io0_direction_w);
	DECLARE_WRITE8_MEMBER(io1_direction_w);

	DECLARE_WRITE8_MEMBER(vector_enable_w);
	DECLARE_WRITE8_MEMBER(nmi_vector_lo_w);
	DECLARE_WRITE8_MEMBER(nmi_vector_hi_w);
	DECLARE_WRITE8_MEMBER(irq_vector_lo_w);
	DECLARE_WRITE8_MEMBER(irq_vector_hi_w);

	DECLARE_READ8_MEMBER(irq_source_r);
	DECLARE_WRITE8_MEMBER(irq_source_w);

	DECLARE_READ8_MEMBER(arena_start_r);
	DECLARE_WRITE8_MEMBER(arena_start_w);
	DECLARE_READ8_MEMBER(arena_end_r);
	DECLARE_WRITE8_MEMBER(arena_end_w);
	DECLARE_READ8_MEMBER(arena_control_r);
	DECLARE_WRITE8_MEMBER(arena_control_w);

	DECLARE_READ8_MEMBER(colmix_6ff0_r);
	DECLARE_WRITE8_MEMBER(colmix_6ff0_w);

	DECLARE_WRITE8_MEMBER(colmix_6ff1_w);
	DECLARE_WRITE8_MEMBER(colmix_6ff2_w);

	DECLARE_READ8_MEMBER(dispctrl_6ff8_r);
	DECLARE_WRITE8_MEMBER(dispctrl_6ff8_w);

	DECLARE_READ8_MEMBER(sound_75f0_r);
	DECLARE_WRITE8_MEMBER(sound_75f0_w);

	DECLARE_READ8_MEMBER(sound_75f1_r);
	DECLARE_WRITE8_MEMBER(sound_75f1_w);

	DECLARE_READ8_MEMBER(sound_75f4_r);
	DECLARE_READ8_MEMBER(sound_75f5_r);
	DECLARE_READ8_MEMBER(sound_75f6_r);
	DECLARE_WRITE8_MEMBER(sound_75f6_w);

	DECLARE_WRITE8_MEMBER(sound_75f7_w);

	DECLARE_READ8_MEMBER(sound_75f8_r);
	DECLARE_WRITE8_MEMBER(sound_75f8_w);

	DECLARE_READ8_MEMBER(sound_75f9_r);
	DECLARE_WRITE8_MEMBER(sound_75f9_w);

	DECLARE_READ8_MEMBER(sound_75fa_r);
	DECLARE_WRITE8_MEMBER(sound_75fa_w);
	DECLARE_READ8_MEMBER(sound_75fb_r);
	DECLARE_WRITE8_MEMBER(sound_75fb_w);
	DECLARE_READ8_MEMBER(sound_75fc_r);
	DECLARE_WRITE8_MEMBER(sound_75fc_w);
	DECLARE_READ8_MEMBER(sound_75fd_r);
	DECLARE_WRITE8_MEMBER(sound_75fd_w);

	DECLARE_WRITE8_MEMBER(sound_75fe_w);
	DECLARE_WRITE8_MEMBER(sound_75ff_w);

	DECLARE_WRITE8_MEMBER(timer_control_w);
	DECLARE_READ8_MEMBER(timer_baseval_r);
	DECLARE_WRITE8_MEMBER(timer_baseval_w);
	DECLARE_WRITE8_MEMBER(timer_freq_w);

	DECLARE_WRITE8_MEMBER(tmap1_regs_w);
	DECLARE_WRITE8_MEMBER(tmap2_regs_w);
	DECLARE_READ8_MEMBER(tmap1_regs_r);
	DECLARE_READ8_MEMBER(tmap2_regs_r);

	DECLARE_WRITE8_MEMBER(spriteregs_w);

	DECLARE_READ8_MEMBER(pal_ntsc_r);

	DECLARE_READ8_MEMBER(xavix_memoryemu_txarray_r);
	DECLARE_WRITE8_MEMBER(xavix_memoryemu_txarray_w);
	uint8_t m_txarray[3];

	DECLARE_READ8_MEMBER(mult_r);
	DECLARE_WRITE8_MEMBER(mult_w);
	DECLARE_READ8_MEMBER(mult_param_r);
	DECLARE_WRITE8_MEMBER(mult_param_w);

	required_device<xavix_device> m_maincpu;
	required_device<screen_device> m_screen;

	uint8_t m_vectorenable;
	uint8_t m_nmi_vector_lo_data;
	uint8_t m_nmi_vector_hi_data;
	uint8_t m_irq_vector_lo_data;
	uint8_t m_irq_vector_hi_data;

	uint8_t m_multparams[3];
	uint8_t m_multresults[2];

	uint8_t m_spritefragment_dmaparam1[2];
	uint8_t m_spritefragment_dmaparam2[2];

	uint8_t m_tmap1_regs[8];
	uint8_t m_tmap2_regs[8];

	int m_arena_start;
	int m_arena_end;
	uint8_t m_arena_control;

	uint8_t m_6ff0;
	uint8_t m_video_ctrl;

	uint8_t m_io0_data;
	uint8_t m_io1_data;
	uint8_t m_io0_direction;
	uint8_t m_io1_direction;

	uint8_t m_soundregs[0x10];

	uint8_t m_timer_baseval;

	int16_t get_vectors(int which, int half);

	// raster IRQ
	TIMER_CALLBACK_MEMBER(interrupt_gen);
	emu_timer *m_interrupt_timer;
	DECLARE_WRITE8_MEMBER(dispctrl_posirq_x_w);
	DECLARE_WRITE8_MEMBER(dispctrl_posirq_y_w);

	required_shared_ptr<uint8_t> m_mainram;
	required_shared_ptr<uint8_t> m_fragment_sprite;
	required_shared_ptr<uint8_t> m_rom_dma_src;
	required_shared_ptr<uint8_t> m_rom_dma_dst;
	required_shared_ptr<uint8_t> m_rom_dma_len;

	required_shared_ptr<uint8_t> m_palram_sh;
	required_shared_ptr<uint8_t> m_palram_l;

	optional_shared_ptr<uint8_t> m_bmp_palram_sh;
	optional_shared_ptr<uint8_t> m_bmp_palram_l;
	optional_shared_ptr<uint8_t> m_bmp_base;

	required_shared_ptr<uint8_t> m_colmix_sh;
	required_shared_ptr<uint8_t> m_colmix_l;
	required_shared_ptr<uint8_t> m_colmix_ctrl;

	required_shared_ptr<uint8_t> m_posirq_x;
	required_shared_ptr<uint8_t> m_posirq_y;

	required_shared_ptr<uint8_t> m_segment_regs;

	required_device<palette_device> m_palette;

	required_ioport m_in0;
	required_ioport m_in1;
	required_ioport m_region;

	required_device<gfxdecode_device> m_gfxdecode;

	void handle_palette(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, uint8_t* ramsh, uint8_t* raml, int size, int basecol);
	double hue2rgb(double p, double q, double t);
	void draw_tile_line(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int tile, int bpp, int xpos, int ypos, int drawheight, int drawwidth, int flipx, int flipy, int pal, int zval, int line);
	void draw_tilemap(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int which);
	void draw_tilemap_line(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int which, int line);
	void draw_sprites(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void draw_sprites_line(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect, int line);

	bitmap_ind16 m_zbuffer;

	uint8_t m_spritereg;

	int m_rgnlen;
	uint8_t* m_rgn;

	// variables used by rendering
	int m_tmp_dataaddress;
	int m_tmp_databit;
	uint8_t m_bit;

	void set_data_address(int address, int bit);
	uint8_t get_next_bit();
	uint8_t get_next_byte();

	int get_current_address_byte();
	required_device<address_map_bank_device> m_lowbus;
	optional_device<i2cmem_device> m_i2cmem;
};

#endif // MAME_INCLUDES_XAVIX_H
