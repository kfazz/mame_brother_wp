// license:BSD-3-Clause
// copyright-holders:Carl

#include "emu.h"
#include "pcd.h"

#include "cpu/mcs48/mcs48.h"
#include "cpu/mcs51/mcs51.h"
#include "screen.h"


DEFINE_DEVICE_TYPE(PCD_VIDEO, pcd_video_device, "pcd_video", "Siemens PC-D Video")
DEFINE_DEVICE_TYPE(PCX_VIDEO, pcx_video_device, "pcx_video", "Siemens PC-X Video")

pcdx_video_device::pcdx_video_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	device_gfx_interface(mconfig, *this, nullptr, "palette"),
	m_maincpu(*this, ":maincpu"),
	m_mcu(*this, "graphics"),
	m_crtc(*this, "crtc"),
	m_pic2(*this, ":pic2")
{
}

pcd_video_device::pcd_video_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	pcdx_video_device(mconfig, PCD_VIDEO, tag, owner, clock),
	m_mouse_btn(*this, "MOUSE"),
	m_mouse_x(*this, "MOUSEX"),
	m_mouse_y(*this, "MOUSEY"),
	m_vram(32*1024),
	m_charram(8*1024)
{
}

pcx_video_device::pcx_video_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	pcdx_video_device(mconfig, PCX_VIDEO, tag, owner, clock),
	device_serial_interface(mconfig, *this),
	m_vram(4*1024),
	m_charrom(*this, "char"),
	m_txd_handler(*this)
{
}

ROM_START( pcd_video )
	ROM_REGION(0x400, "graphics", 0)
	ROM_LOAD("s36361-d321-v1.bin", 0x000, 0x400, CRC(69baeb2a) SHA1(98b9cd0f38c51b4988a3aed0efcf004bedd115ff))
ROM_END

const tiny_rom_entry *pcd_video_device::device_rom_region() const
{
	return ROM_NAME( pcd_video );
}

ROM_START( pcx_video )
	ROM_REGION(0x2000, "char", 0)
	ROM_LOAD("d12-graka.bin", 0x0000, 0x2000, CRC(e4933c16) SHA1(932ae1f0cd2b029b7f5fc3d2d1679e70b25c0828))

	ROM_REGION(0x6000, "graphics", 0)
	ROM_LOAD("d40-graka.bin", 0x0000, 0x2000, CRC(dce48252) SHA1(0d9a575b2d001168a36864d7bd5db1c3aca5fb8d))
	ROM_LOAD("d39-graka.bin", 0x4000, 0x2000, CRC(02920e25) SHA1(145a6648d75c1dc4788f9bc7790281ef7e8f8426))
ROM_END

const tiny_rom_entry *pcx_video_device::device_rom_region() const
{
	return ROM_NAME( pcx_video );
}

static const gfx_layout pcd_charlayout =
{
	8, 14,                   /* 8 x 14 characters */
	512,                    /* 512 characters */
	1,                  /* 1 bits per pixel */
	{ 0 },                  /* no bitplanes */
	/* x offsets */
	{ 0, 1, 2, 3, 4, 5, 6, 7 },
	/* y offsets */
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8, 8*8, 9*8, 10*8, 11*8, 12*8, 13*8, 14*8 },
	8*16
};

static GFXDECODE_START( gfx_pcx )
	GFXDECODE_DEVICE( "char", 0x0000, pcd_charlayout, 0, 1 )
GFXDECODE_END

static INPUT_PORTS_START( pcd_mouse )
	PORT_START("MOUSE")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_NAME("Right Mouse Button")   PORT_CODE(MOUSECODE_BUTTON1)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_BUTTON2) PORT_NAME("Left Mouse Button")    PORT_CODE(MOUSECODE_BUTTON2)

	PORT_START("MOUSEX")
	PORT_BIT( 0xfff, 0x000, IPT_MOUSE_X ) PORT_SENSITIVITY(50) PORT_KEYDELTA(0)

	PORT_START("MOUSEY")
	PORT_BIT( 0xfff, 0x000, IPT_MOUSE_Y ) PORT_SENSITIVITY(50) PORT_KEYDELTA(0)
INPUT_PORTS_END

ioport_constructor pcd_video_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(pcd_mouse);
}

MACHINE_CONFIG_START(pcd_video_device::device_add_mconfig)
	MCFG_DEVICE_ADD("graphics", I8741, XTAL(16'000'000)/2)
	MCFG_MCS48_PORT_P1_IN_CB(READ8(*this, pcd_video_device, p1_r))
	MCFG_MCS48_PORT_P2_OUT_CB(WRITE8(*this, pcd_video_device, p2_w))
	MCFG_MCS48_PORT_T1_IN_CB(READLINE(*this, pcd_video_device, t1_r))

	// video hardware
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_SIZE(640, 350)
	MCFG_SCREEN_VISIBLE_AREA(0, 639, 0, 349)
	MCFG_SCREEN_REFRESH_RATE(50)
	MCFG_SCREEN_UPDATE_DEVICE("crtc", scn2674_device, screen_update)

	MCFG_PALETTE_ADD("palette", 3)
	MCFG_PALETTE_INIT_OWNER(pcdx_video_device, pcdx)

	MCFG_DEVICE_ADD("crtc", SCN2674, 0)
	MCFG_SCN2674_CHARACTER_WIDTH(16)
	MCFG_SCN2674_DRAW_CHARACTER_CALLBACK_OWNER(pcd_video_device, display_pixels)
	MCFG_VIDEO_SET_SCREEN("screen")
	MCFG_TIMER_DRIVER_ADD_PERIODIC("mouse_timer", pcd_video_device, mouse_timer, attotime::from_hz(15000)) // guess
MACHINE_CONFIG_END

void pcx_video_device::pcx_vid_map(address_map &map)
{
	map(0x0000, 0x5fff).rom().region("graphics", 0);
}

void pcx_video_device::pcx_vid_io(address_map &map)
{
	map(0x8000, 0x8007).rw("crtc", FUNC(scn2674_device::read), FUNC(scn2674_device::write));
	map(0x8008, 0x8008).r(FUNC(pcx_video_device::unk_r));
	map(0xa000, 0xa001).rw(FUNC(pcx_video_device::vram_latch_r), FUNC(pcx_video_device::vram_latch_w));
	map(0xa002, 0xa003).rw(FUNC(pcx_video_device::term_mcu_r), FUNC(pcx_video_device::term_mcu_w));
	map(0xc000, 0xc7ff).ram();
}

void pcx_video_device::pcx_vram(address_map &map)
{
	map(0x0000, 0x07ff).rw(FUNC(pcx_video_device::vram_r), FUNC(pcx_video_device::vram_w));
}

MACHINE_CONFIG_START(pcx_video_device::device_add_mconfig)
	MCFG_DEVICE_ADD("graphics", I8031, XTAL(24'000'000)/2)
	MCFG_DEVICE_PROGRAM_MAP(pcx_vid_map)
	MCFG_DEVICE_IO_MAP(pcx_vid_io)
	MCFG_MCS51_PORT_P1_OUT_CB(WRITE8(*this, pcx_video_device, p1_w))
	MCFG_MCS51_SERIAL_TX_CB(WRITE8(*this, pcx_video_device, tx_callback))
	MCFG_MCS51_SERIAL_RX_CB(READ8(*this, pcx_video_device, rx_callback))

	// video hardware
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_SIZE(640, 350)
	MCFG_SCREEN_VISIBLE_AREA(0, 639, 0, 349)
	MCFG_SCREEN_REFRESH_RATE(50)
	MCFG_SCREEN_UPDATE_DEVICE("crtc", scn2674_device, screen_update)

	MCFG_PALETTE_ADD_MONOCHROME("palette")

	MCFG_DEVICE_ADD("crtc", SCN2674, 0)
	MCFG_SCN2674_INTR_CALLBACK(INPUTLINE("graphics", MCS51_INT0_LINE))
	MCFG_SCN2674_CHARACTER_WIDTH(16)
	MCFG_SCN2674_DRAW_CHARACTER_CALLBACK_OWNER(pcx_video_device, display_pixels)
	MCFG_VIDEO_SET_SCREEN("screen")
	MCFG_DEVICE_ADDRESS_MAP(0, pcx_vram)
MACHINE_CONFIG_END


SCN2674_DRAW_CHARACTER_MEMBER(pcd_video_device::display_pixels)
{
	address <<= 1;
	if(lg)
	{
		uint16_t data = m_vram[address + 1] | (m_vram[address] << 8);
		if(m_p2 & 8)
			data = ~data;
		for(int i = 0; i < 16; i++)
		{
			bitmap.pix32(y, x++) = palette().pen(BIT(data, 15) ? 1 : 0);
			data <<= 1;
		}
	}
	else
	{
		uint8_t data, attr;
		int bgnd = 0, fgnd = 1;
		data = m_charram[m_vram[address] * 16 + linecount];
		attr = m_vram[address + 1];
		if(cursor && blink)
			data = 0xff;
		if(ul && (attr & 0x20))
			data = 0xff;

		if(attr & 0x10)
			data = ~data;
		if(m_p2 & 8)
		{
			fgnd = 0;
			bgnd = (attr & 8) ? 2 : 1;
		}
		else if(attr & 8)
			bgnd = 2;
		for(int i = 0; i < 8; i++)
		{
			rgb_t pix = palette().pen(BIT(data, 7) ? fgnd : bgnd);
			bitmap.pix32(y, x++) = pix;
			bitmap.pix32(y, x++) = pix;
			data <<= 1;
		}
	}
}

SCN2674_DRAW_CHARACTER_MEMBER(pcx_video_device::display_pixels)
{
	uint8_t data;
	address <<= 1;
	data = m_charrom[m_vram[address] * 16 + linecount + (m_vram[address + 1] & 0x20 ? 4096 : 0)];
	if(cursor && blink)
		data = 0xff;
	if(m_p1 & 0x20)
		data = ~data;
	for(int i = 0; i < 8; i++)
	{
		rgb_t pix = palette().pen(BIT(data, 7) ? 1 : 0);
		bitmap.pix32(y, x++) = pix;
		bitmap.pix32(y, x++) = pix;
		data <<= 1;
	}
}

PALETTE_INIT_MEMBER(pcdx_video_device, pcdx)
{
	palette.set_pen_color(0, rgb_t::black());
	palette.set_pen_color(1, rgb_t::white());
	palette.set_pen_color(2, rgb_t(128, 128, 128));
}

TIMER_DEVICE_CALLBACK_MEMBER(pcd_video_device::mouse_timer)
{
	m_t1 = (m_t1 == CLEAR_LINE) ? ASSERT_LINE : CLEAR_LINE;
	if(m_t1)
	{
		switch(m_mouse.phase)
		{
		case 0:
			m_mouse.xa = m_mouse.x > m_mouse.prev_x ? CLEAR_LINE : ASSERT_LINE;
			m_mouse.xb = m_mouse.x < m_mouse.prev_x ? CLEAR_LINE : ASSERT_LINE;
			m_mouse.ya = m_mouse.y > m_mouse.prev_y ? CLEAR_LINE : ASSERT_LINE;
			m_mouse.yb = m_mouse.y < m_mouse.prev_y ? CLEAR_LINE : ASSERT_LINE;
			break;
		case 1:
			m_mouse.xa = m_mouse.xb = m_mouse.x != m_mouse.prev_x ? CLEAR_LINE : ASSERT_LINE;
			m_mouse.ya = m_mouse.yb = m_mouse.y != m_mouse.prev_y ? CLEAR_LINE : ASSERT_LINE;
			break;
		case 2:
			m_mouse.xa = m_mouse.x < m_mouse.prev_x ? CLEAR_LINE : ASSERT_LINE;
			m_mouse.xb = m_mouse.x > m_mouse.prev_x ? CLEAR_LINE : ASSERT_LINE;
			m_mouse.ya = m_mouse.y < m_mouse.prev_y ? CLEAR_LINE : ASSERT_LINE;
			m_mouse.yb = m_mouse.y > m_mouse.prev_y ? CLEAR_LINE : ASSERT_LINE;
			break;
		case 3:
			m_mouse.xa = m_mouse.xb = ASSERT_LINE;
			m_mouse.ya = m_mouse.yb = ASSERT_LINE;
			m_mouse.prev_x = m_mouse.x;
			m_mouse.prev_y = m_mouse.y;
			m_mouse.x = m_mouse_x->read();
			m_mouse.y = m_mouse_y->read();
			break;
		}

		m_mouse.phase = (m_mouse.phase + 1) & 3;
	}
}

WRITE8_MEMBER(pcd_video_device::vram_w)
{
	if(m_vram_sw)
		m_vram[offset] = data;
	else if(!(offset & 1))
	{
		offset >>= 1;
		m_charram[offset & 0x1fff] = data;
		gfx(0)->mark_dirty(offset/16);
	}
}

READ8_MEMBER(pcd_video_device::vram_r)
{
	return m_vram[offset];
}

WRITE8_MEMBER(pcd_video_device::vram_sw_w)
{
	m_vram_sw = data & 1;
}

READ_LINE_MEMBER(pcd_video_device::t1_r)
{
	return m_t1;
}

READ8_MEMBER(pcd_video_device::p1_r)
{
	uint8_t data = (m_mouse_btn->read() & 0x30) | 0x80; // char ram/rom jumper?
	data |= (m_mouse.xa != CLEAR_LINE ? 0 : 1);
	data |= (m_mouse.xb != CLEAR_LINE ? 0 : 2);
	data |= (m_mouse.ya != CLEAR_LINE ? 0 : 4);
	data |= (m_mouse.yb != CLEAR_LINE ? 0 : 8);

	return data;
}

WRITE8_MEMBER(pcd_video_device::p2_w)
{
	m_p2 = data;
	m_pic2->ir7_w((data & 0x80) ? CLEAR_LINE : ASSERT_LINE);
}

READ8_MEMBER(pcx_video_device::term_r)
{
	switch(offset)
	{
		case 0:
			m_pic2->ir0_w(CLEAR_LINE);
			m_term_stat &= ~2;
			return m_term_key;
		case 1:
			m_pic2->ir0_w(CLEAR_LINE);
			return m_term_stat >> 1;
	}
	return 0xff;
}

WRITE8_MEMBER(pcx_video_device::term_w)
{
	if(!offset)
	{
		m_mcu->set_input_line(MCS51_INT1_LINE, ASSERT_LINE);
		m_term_char = data;
		m_term_stat |= 4;
	}
}

READ8_MEMBER(pcx_video_device::term_mcu_r)
{
	switch(offset)
	{
		case 0:
			m_mcu->set_input_line(MCS51_INT1_LINE, CLEAR_LINE);
			m_pic2->ir0_w(ASSERT_LINE);
			m_term_stat &= ~4;
			return m_term_char;
		case 1:
			return m_term_stat;
	}
	return 0;
}

WRITE8_MEMBER(pcx_video_device::term_mcu_w)
{
	if(!offset)
	{
		m_term_key = data;
		m_pic2->ir0_w(ASSERT_LINE);
		m_term_stat |= 2;
	}
}

WRITE8_MEMBER(pcx_video_device::vram_w)
{
	offset <<= 1;
	m_vram[offset] = m_vram_latch_w[0];
	m_vram[offset+1] = m_vram_latch_w[1];
}

READ8_MEMBER(pcx_video_device::vram_r)
{
	offset <<= 1;
	m_vram_latch_r[0] = m_vram[offset];
	m_vram_latch_r[1] = m_vram[offset+1];
	return m_vram[offset];
}

WRITE8_MEMBER(pcx_video_device::vram_latch_w)
{
	m_vram_latch_w[offset] = data;
}

READ8_MEMBER(pcx_video_device::vram_latch_r)
{
	return m_vram_latch_r[offset];
}

READ8_MEMBER(pcdx_video_device::detect_r)
{
	return 0;
}

WRITE8_MEMBER(pcdx_video_device::detect_w)
{
}

READ8_MEMBER(pcx_video_device::unk_r)
{
	return 0x80;
}

WRITE8_MEMBER(pcx_video_device::p1_w)
{
	m_p1 = data;
}

void pcd_video_device::device_start()
{
	m_maincpu->space(AS_IO).install_readwrite_handler(0xfb00, 0xfb01, read8_delegate(FUNC(pcdx_video_device::detect_r), this), write8_delegate(FUNC(pcdx_video_device::detect_w), this), 0xff00);
	m_maincpu->space(AS_PROGRAM).install_readwrite_handler(0xf0000, 0xf7fff, read8_delegate(FUNC(pcd_video_device::vram_r), this), write8_delegate(FUNC(pcd_video_device::vram_w), this), 0xffff);
	set_gfx(0, std::make_unique<gfx_element>(&palette(), pcd_charlayout, &m_charram[0], 0, 1, 0));
}

void pcd_video_device::device_reset()
{
	m_mouse.phase = 0;
	m_mouse.xa = m_mouse.xb = ASSERT_LINE;
	m_mouse.ya = m_mouse.yb = ASSERT_LINE;
	m_mouse.x = m_mouse.y = 0;
	m_mouse.prev_x = m_mouse.prev_y = 0;
	m_vram_sw = 1;
	m_t1 = CLEAR_LINE;
}

void pcd_video_device::map(address_map &map)
{
	map(0x00, 0x0f).w("crtc", FUNC(scn2674_device::write)).umask16(0x00ff);
	map(0x00, 0x0f).r("crtc", FUNC(scn2674_device::read)).umask16(0xff00);
	map(0x20, 0x20).w(FUNC(pcd_video_device::vram_sw_w));
	map(0x30, 0x33).rw("graphics", FUNC(i8741_device::upi41_master_r), FUNC(i8741_device::upi41_master_w)).umask16(0x00ff);
}

void pcx_video_device::device_start()
{
	m_maincpu->space(AS_IO).install_readwrite_handler(0xfb00, 0xfb01, read8_delegate(FUNC(pcdx_video_device::detect_r), this), write8_delegate(FUNC(pcdx_video_device::detect_w), this), 0x00ff);
	m_txd_handler.resolve_safe();

	set_data_frame(1, 8, PARITY_NONE, STOP_BITS_1);
	set_rate(600*2);  // FIXME: fix the keyboard when the mc2661 baud rate calc is fixed

	decode_gfx(gfx_pcx);
}

void pcx_video_device::device_reset()
{
	m_term_key = 0;
	m_term_stat = 0;
	transmit_register_reset();
	receive_register_reset();

	m_txd_handler(1);
}

void pcx_video_device::map(address_map &map)
{
	map(0x0, 0xf).rw(FUNC(pcx_video_device::term_r), FUNC(pcx_video_device::term_w));
}

READ8_MEMBER(pcx_video_device::rx_callback)
{
	return get_received_char();
}

WRITE8_MEMBER(pcx_video_device::tx_callback)
{
	transmit_register_setup(data);
}

void pcx_video_device::tra_callback()
{
	m_txd_handler(transmit_register_get_data_bit());
}

void pcx_video_device::rcv_complete()
{
	receive_register_extract();
	m_mcu->set_input_line(MCS51_RX_LINE, ASSERT_LINE);
	m_mcu->set_input_line(MCS51_RX_LINE, CLEAR_LINE);
}
