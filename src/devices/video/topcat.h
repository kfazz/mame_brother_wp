// license:BSD-3-Clause
// copyright-holders:Sven Schnelle
#ifndef MAME_VIDEO_TOPCAT_H
#define MAME_VIDEO_TOPCAT_H

#pragma once

#define MCFG_TOPCAT_FB_WIDTH(_pixels) \
	downcast<topcat_device &>(*device).set_fb_width(_pixels);

#define MCFG_TOPCAT_FB_HEIGHT(_pixels) \
	downcast<topcat_device &>(*device).set_fb_height(_pixels);

#define MCFG_TOPCAT_PLANEMASK(_mask) \
	downcast<topcat_device &>(*device).set_planemask(_mask);

class topcat_device : public device_t
{
public:
	topcat_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	void set_fb_width(int _pixels) { m_fb_width = _pixels; }
	void set_fb_height(int _pixels) { m_fb_height = _pixels; }
	void set_planemask(int _mask) { m_plane_mask = _mask; }
	void get_cursor_pos(int &startx, int &starty, int &endx, int &endy);

	DECLARE_READ16_MEMBER(vram_r);
	DECLARE_WRITE16_MEMBER(vram_w);
	DECLARE_READ16_MEMBER(ctrl_r);
	DECLARE_WRITE16_MEMBER(ctrl_w);

	void topcat_mem(address_map &map);

protected:
	topcat_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_start() override;
	virtual void device_reset() override;

	TIMER_CALLBACK_MEMBER(cursor_callback);

private:

	typedef enum {
		TOPCAT_REPLACE_RULE_CLEAR, /* 0 */
		TOPCAT_REPLACE_RULE_SRC_AND_DST,
		TOPCAT_REPLACE_RULE_SRC_AND_NOT_DST,
		TOPCAT_REPLACE_RULE_SRC,
		TOPCAT_REPLACE_RULE_NOT_SRC_AND_DST,
		TOPCAT_REPLACE_RULE_NOP,
		TOPCAT_REPLACE_RULE_SRC_XOR_DST,
		TOPCAT_REPLACE_RULE_SRC_OR_DST,
		TOPCAT_REPLACE_RULE_NOT_SRC_AND_NOT_DST,
		TOPCAT_REPLACE_RULE_NOT_SRC_XOR_DST,
		TOPCAT_REPLACE_RULE_NOT_DST,
		TOPCAT_REPLACE_RULE_SRC_OR_NOT_DST,
		TOPCAT_REPLACE_RULE_NOT_SRC,
		TOPCAT_REPLACE_RULE_NOT_SRC_OR_DST,
		TOPCAT_REPLACE_RULE_NOT_SRC_OR_NOT_DST,
		TOPCAT_REPLACE_RULE_SET,
	} replacement_rule_t;

	enum topcat_reg {
		TOPCAT_REG_VBLANK=0x20,
		TOPCAT_REG_WMOVE_ACTIVE=0x22,
		TOPCAT_REG_VERT_RETRACE_INTRQ=0x24,
		TOPCAT_REG_WMOVE_INTRQ=0x26,
		TOPCAT_REG_DISPLAY_PLANE_ENABLE=0x40,
		TOPCAT_REG_WRITE_ENABLE_PLANE=0x44,
		TOPCAT_REG_READ_ENABLE_PLANE=0x46,
		TOPCAT_REG_FB_WRITE_ENABLE=0x48,
		TOPCAT_REG_START_WMOVE=0x4e,
		TOPCAT_REG_ENABLE_BLINK_PLANES=0x50,
		TOPCAT_REG_ENABLE_ALT_FRAME=0x54,
		TOPCAT_REG_CURSOR_PLANE_ENABLE=0x56,
		TOPCAT_REG_PIXEL_REPLACE_RULE=0x75,
		TOPCAT_REG_MOVE_REPLACE_RULE=0x77,
		TOPCAT_REG_SOURCE_X_PIXEL=0x79,
		TOPCAT_REG_SOURCE_Y_PIXEL=0x7b,
		TOPCAT_REG_DST_X_PIXEL=0x7d,
		TOPCAT_REG_DST_Y_PIXEL=0x7f,
		TOPCAT_REG_BLOCK_MOVER_PIXEL_WIDTH=0x81,
		TOPCAT_REG_BLOCK_MOVER_PIXEL_HEIGHT=0x83,
		TOPCAT_REG_CURSOR_X_POS=0x85,
		TOPCAT_REG_CURSOR_Y_POS=0x87,
		TOPCAT_REG_CURSOR_WIDTH=0x89,
	};

	void window_move();
	void execute_rule(bool src, replacement_rule_t rule, bool &dst);

	void update_cursor(int x, int y, uint16_t ctrl, uint8_t width);

	void modify_vram(int x, int y, bool state) {
		if (state)
			m_vram[y * m_fb_width + x] |= m_plane_mask;
		else
			m_vram[y * m_fb_width + x] &= ~m_plane_mask;
	}

	void modify_vram_offset(int offset, bool state) {
		if (state)
			m_vram[offset] |= m_plane_mask;
		else
			m_vram[offset] &= ~m_plane_mask;
	}

	bool get_vram_pixel(int x, int y) {
		return m_vram[y * m_fb_width + x] & m_plane_mask;
	}

	uint8_t m_vblank;
	uint8_t m_wmove_active;
	uint8_t m_vert_retrace_intrq;
	uint8_t m_wmove_intrq;
	uint8_t m_display_enable_planes;
	bool m_write_enable_plane;
	bool m_read_enable_plane;
	uint16_t m_fb_write_enable;
	uint16_t m_enable_blink_planes;
	uint16_t m_enable_alt_frame;
	uint16_t m_cursor_plane_enable;
	uint16_t m_move_replacement_rule;
	uint16_t m_pixel_replacement_rule;
	uint16_t m_source_x_pixel;
	uint16_t m_source_y_pixel;
	uint16_t m_dst_x_pixel;
	uint16_t m_dst_y_pixel;
	uint16_t m_block_mover_pixel_width;
	uint16_t m_block_mover_pixel_height;

	emu_timer *m_cursor_timer;
	bool m_cursor_state;
	uint16_t m_cursor_x_pos;
	uint16_t m_cursor_y_pos;
	uint16_t m_cursor_width;

	int m_fb_width;
	int m_fb_height;
	uint8_t m_plane_mask;

	bool m_read_enable;
	bool m_write_enable;
	bool m_fb_enable;

	required_shared_ptr<uint8_t> m_vram;
};

DECLARE_DEVICE_TYPE(TOPCAT, topcat_device)
#endif // MAME_VIDEO_TOPCAT_H
