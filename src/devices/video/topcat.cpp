// license:BSD-3-Clause
// copyright-holders:Sven Schnelle

#include "emu.h"
#include "topcat.h"

//#define VERBOSE 1
#include "logmacro.h"

DEFINE_DEVICE_TYPE(TOPCAT, topcat_device, "topcat", "HP Topcat ASIC")

topcat_device::topcat_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	m_cursor_timer(nullptr),
	m_vram(*this, "^vram")
{
}

topcat_device::topcat_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	topcat_device(mconfig, TOPCAT, tag, owner, clock)
{
}

void topcat_device::device_start()
{
	m_cursor_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(topcat_device::cursor_callback), this));
	m_cursor_timer->adjust(attotime::from_hz(3));

	save_item(NAME(m_vblank));
	save_item(NAME(m_wmove_active));
	save_item(NAME(m_vert_retrace_intrq));
	save_item(NAME(m_wmove_intrq));
	save_item(NAME(m_display_enable_planes));
	save_item(NAME(m_write_enable_plane));
	save_item(NAME(m_read_enable_plane));
	save_item(NAME(m_fb_write_enable));
	save_item(NAME(m_enable_blink_planes));
	save_item(NAME(m_enable_alt_frame));
	save_item(NAME(m_cursor_plane_enable));
	save_item(NAME(m_move_replacement_rule));
	save_item(NAME(m_pixel_replacement_rule));
	save_item(NAME(m_source_x_pixel));
	save_item(NAME(m_source_y_pixel));
	save_item(NAME(m_dst_x_pixel));
	save_item(NAME(m_dst_y_pixel));
	save_item(NAME(m_block_mover_pixel_width));
	save_item(NAME(m_block_mover_pixel_height));
	save_item(NAME(m_fb_width));
	save_item(NAME(m_fb_height));
	save_item(NAME(m_read_enable));
	save_item(NAME(m_write_enable));
	save_item(NAME(m_fb_enable));
}

//-------------------------------------------------
//  memory_space_config - return a description of
//  any address spaces owned by this device
//-------------------------------------------------

void topcat_device::device_reset()
{
	m_pixel_replacement_rule = TOPCAT_REPLACE_RULE_SRC;
}

READ16_MEMBER(topcat_device::vram_r)
{
	uint16_t ret = 0;

	if (mem_mask & m_plane_mask)
		ret |= m_vram[offset*2+1] ? m_plane_mask : 0;

	if (mem_mask & m_plane_mask << 8)
		ret |= m_vram[offset*2] ? m_plane_mask << 8 : 0;

	return ret;
}

WRITE16_MEMBER(topcat_device::vram_w)
{
	if (mem_mask & m_plane_mask)
		modify_vram_offset(offset * 2 + 1, (data & m_plane_mask));

	if (mem_mask & m_plane_mask << 8)
		modify_vram_offset(offset * 2, (data & m_plane_mask << 8));
}

void topcat_device::get_cursor_pos(int &startx, int &starty, int &endx, int &endy)
{
	if (m_cursor_state && ((m_cursor_plane_enable >> 8) & m_plane_mask)) {
		startx = m_cursor_x_pos;
		starty = m_cursor_y_pos;
		endx = m_cursor_x_pos + m_cursor_width;
		endy = m_cursor_y_pos;
	} else {
		startx = 0;
		starty = 0;
		endx = 0;
		endy = 0;
	}
}

TIMER_CALLBACK_MEMBER(topcat_device::cursor_callback)
{
	m_cursor_timer->adjust(attotime::from_hz(5));
	m_cursor_state ^= true;
}

void topcat_device::update_cursor(int x, int y, uint16_t ctrl, uint8_t width)
{
	m_cursor_x_pos = (std::min)(x, m_fb_width - m_cursor_width);
	m_cursor_y_pos = (std::max)((std::min)(y, m_fb_height), 2);
	m_cursor_plane_enable = ctrl;
	m_cursor_width = width;
}

void topcat_device::execute_rule(bool src, replacement_rule_t rule, bool &dst)
{
	switch (rule & 0x0f) {
	case TOPCAT_REPLACE_RULE_CLEAR:
		dst = false;
		break;
	case TOPCAT_REPLACE_RULE_SRC_AND_DST:
		dst &= src;
		break;
	case TOPCAT_REPLACE_RULE_SRC_AND_NOT_DST:
		dst = !dst & src;
		break;
	case TOPCAT_REPLACE_RULE_SRC:
		dst = src;
		break;
	case TOPCAT_REPLACE_RULE_NOT_SRC_AND_DST:
		dst &= !src;
		break;
	case TOPCAT_REPLACE_RULE_NOP:
		break;
	case TOPCAT_REPLACE_RULE_SRC_XOR_DST:
		dst ^= src;
		break;
	case TOPCAT_REPLACE_RULE_SRC_OR_DST:
		dst |= src;
		break;
	case TOPCAT_REPLACE_RULE_NOT_SRC_AND_NOT_DST:
		dst = !dst & !src;
		break;
	case TOPCAT_REPLACE_RULE_NOT_SRC_XOR_DST:
		dst ^= !src;
		break;
	case TOPCAT_REPLACE_RULE_NOT_DST:
		dst ^= true;
		break;
	case TOPCAT_REPLACE_RULE_SRC_OR_NOT_DST:
		dst = src | !dst;
		break;
	case TOPCAT_REPLACE_RULE_NOT_SRC:
		dst = !src;
		break;
	case TOPCAT_REPLACE_RULE_NOT_SRC_OR_DST:
		dst |= !src;
		break;
	case TOPCAT_REPLACE_RULE_NOT_SRC_OR_NOT_DST:
		dst = !src | !dst;
		break;
	case TOPCAT_REPLACE_RULE_SET:
		dst = true;
		break;

	}
}

void topcat_device::window_move()
{
	if (!((m_fb_write_enable >> 8) & m_plane_mask))
		return;

	LOG("WINDOWMOVE: %3ux%3u -> %3ux%3u / %3ux%3u rule %x\n",
		m_source_x_pixel,
		m_source_y_pixel,
		m_dst_x_pixel,
		m_dst_y_pixel,
		m_block_mover_pixel_width,
		m_block_mover_pixel_height,
		m_move_replacement_rule);

	int line, endline, lineincr;
	if (m_dst_y_pixel > m_source_y_pixel) {
		/* move down */
		line = m_block_mover_pixel_height-1;
		endline = -1;
		lineincr = -1;
	} else {
		/* move up */
		line = 0;
		endline = m_block_mover_pixel_height;
		lineincr = 1;
	}

	int startcolumn, endcolumn, columnincr;
	if (m_dst_x_pixel > m_source_x_pixel) {
		/* move right */
		startcolumn = m_block_mover_pixel_width-1;
		endcolumn = -1;
		columnincr = -1;
	} else {
		/* move left */
		startcolumn = 0;
		endcolumn = m_block_mover_pixel_width;
		columnincr = 1;

	}

	for ( ; line != endline; line += lineincr) {
		for (int column = startcolumn; column != endcolumn; column += columnincr) {
			bool const src = get_vram_pixel(m_source_x_pixel + column, m_source_y_pixel + line);
			bool dst = get_vram_pixel(m_dst_x_pixel + column, m_dst_y_pixel + line);
			execute_rule(src, (replacement_rule_t)(m_move_replacement_rule & 0x0f), dst);
			modify_vram(m_dst_x_pixel + column, m_dst_y_pixel + line, dst);
		}
	}
}

READ16_MEMBER(topcat_device::ctrl_r)
{
	if (!m_read_enable)
		return 0;

	uint16_t ret = 0xffff;
	switch (offset) {
	case TOPCAT_REG_VBLANK:
		ret = m_vblank;
		break;
	case TOPCAT_REG_WMOVE_ACTIVE:
		ret = m_wmove_active;
		break;
	case TOPCAT_REG_VERT_RETRACE_INTRQ:
		ret = m_vert_retrace_intrq;
		break;
	case TOPCAT_REG_WMOVE_INTRQ:
		ret = m_wmove_intrq;
		break;
	case TOPCAT_REG_DISPLAY_PLANE_ENABLE:
		ret = m_display_enable_planes;
		break;
	case TOPCAT_REG_WRITE_ENABLE_PLANE:
		ret = m_write_enable_plane;
		break;
	case TOPCAT_REG_READ_ENABLE_PLANE:
		ret = m_read_enable_plane;
		break;
	case TOPCAT_REG_FB_WRITE_ENABLE:
		ret = m_fb_write_enable;
		break;
	case TOPCAT_REG_START_WMOVE:
		ret = 0;
		break;
	case TOPCAT_REG_ENABLE_BLINK_PLANES:
		ret = m_enable_blink_planes;
		break;
	case TOPCAT_REG_ENABLE_ALT_FRAME:
		ret = m_enable_alt_frame;
		break;
	case TOPCAT_REG_CURSOR_PLANE_ENABLE:
		ret = m_cursor_plane_enable;
		break;
	case TOPCAT_REG_PIXEL_REPLACE_RULE:
		ret = m_pixel_replacement_rule;
		break;
	case TOPCAT_REG_MOVE_REPLACE_RULE:
		ret = m_move_replacement_rule;
		break;
	case TOPCAT_REG_SOURCE_X_PIXEL:
		ret = m_source_x_pixel;
		break;
	case TOPCAT_REG_SOURCE_Y_PIXEL:
		ret = m_source_y_pixel;
		break;
	case TOPCAT_REG_DST_X_PIXEL:
		ret = m_dst_x_pixel;
		break;
	case TOPCAT_REG_DST_Y_PIXEL:
		ret = m_dst_y_pixel;
		break;
	case TOPCAT_REG_BLOCK_MOVER_PIXEL_WIDTH:
		ret = m_block_mover_pixel_width;
		break;
	case TOPCAT_REG_BLOCK_MOVER_PIXEL_HEIGHT:
		ret = m_block_mover_pixel_height;
		break;
	default:
		logerror("unknown register read %02x\n", offset);
		return space.unmap();
	}
	return ret;
}

WRITE16_MEMBER(topcat_device::ctrl_w)
{
	data &= mem_mask;

	if (offset == TOPCAT_REG_WRITE_ENABLE_PLANE) {
		m_write_enable = (data >> 8) & m_plane_mask;
		return;
	}

	if (offset == TOPCAT_REG_READ_ENABLE_PLANE) {
		m_read_enable = (data >> 8) & m_plane_mask;
		return;
	}

	if (!m_write_enable)
		return;

	switch (offset) {
	case TOPCAT_REG_VBLANK:
		m_vblank = data & 0xff;
		break;
	case TOPCAT_REG_WMOVE_ACTIVE:
		break;
	case TOPCAT_REG_VERT_RETRACE_INTRQ:
		m_vert_retrace_intrq = data;
		break;
	case TOPCAT_REG_WMOVE_INTRQ:
		m_wmove_intrq = data;
		break;
	case TOPCAT_REG_DISPLAY_PLANE_ENABLE:
		m_display_enable_planes = data;
		break;
	case TOPCAT_REG_FB_WRITE_ENABLE:
		m_fb_write_enable = data;
		break;

	case TOPCAT_REG_START_WMOVE:
		window_move();
		break;
	case TOPCAT_REG_ENABLE_BLINK_PLANES:
		m_enable_blink_planes = data;
		break;
	case TOPCAT_REG_ENABLE_ALT_FRAME:
		m_enable_alt_frame = data;
		break;
	case TOPCAT_REG_PIXEL_REPLACE_RULE:
		m_pixel_replacement_rule = data;
		break;
	case TOPCAT_REG_MOVE_REPLACE_RULE:
		m_move_replacement_rule = data;
		break;
	case TOPCAT_REG_SOURCE_X_PIXEL:
		m_source_x_pixel = data;
		break;
	case TOPCAT_REG_SOURCE_Y_PIXEL:
		m_source_y_pixel = data;
		break;
	case TOPCAT_REG_DST_X_PIXEL:
		m_dst_x_pixel = data;
		break;
	case TOPCAT_REG_DST_Y_PIXEL:
		m_dst_y_pixel = data;
		break;
	case TOPCAT_REG_BLOCK_MOVER_PIXEL_WIDTH:
		m_block_mover_pixel_width = data;
		break;
	case TOPCAT_REG_BLOCK_MOVER_PIXEL_HEIGHT:
		m_block_mover_pixel_height = data;
		break;
	case TOPCAT_REG_CURSOR_PLANE_ENABLE:
		update_cursor(m_cursor_x_pos, m_cursor_y_pos, data, m_cursor_width);
		break;
	case TOPCAT_REG_CURSOR_X_POS:
		update_cursor(data, m_cursor_y_pos, m_cursor_plane_enable, m_cursor_width);
		break;
	case TOPCAT_REG_CURSOR_Y_POS:
		update_cursor(m_cursor_x_pos, data, m_cursor_plane_enable, m_cursor_width);
		break;
	case TOPCAT_REG_CURSOR_WIDTH:
		update_cursor(m_cursor_x_pos, m_cursor_y_pos, m_cursor_plane_enable, data);
		break;
	default:
		logerror("unknown register: %02X = %04x\n", offset, data, mem_mask);
		break;
	}
}
