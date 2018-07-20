// license:BSD-3-Clause
// copyright-holders:David Haywood

#include "emupal.h"

class xyonix_state : public driver_device
{
public:
	xyonix_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_gfxdecode(*this, "gfxdecode"),
		m_vidram(*this, "vidram") { }

	required_device<cpu_device> m_maincpu;
	required_device<gfxdecode_device> m_gfxdecode;

	required_shared_ptr<uint8_t> m_vidram;

	tilemap_t *m_tilemap;

	int m_e0_data;
	int m_credits;
	int m_coins;
	int m_prev_coin;
	bool m_nmi_mask;

	DECLARE_WRITE_LINE_MEMBER(nmiclk_w);
	DECLARE_WRITE8_MEMBER(irqack_w);
	DECLARE_WRITE8_MEMBER(nmiack_w);
	DECLARE_READ8_MEMBER(io_r);
	DECLARE_WRITE8_MEMBER(io_w);
	DECLARE_WRITE8_MEMBER(vidram_w);

	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;
	TILE_GET_INFO_MEMBER(get_tile_info);
	DECLARE_PALETTE_INIT(xyonix);
	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	void handle_coins(int coin);
	void xyonix(machine_config &config);
	void main_map(address_map &map);
	void port_map(address_map &map);
};
