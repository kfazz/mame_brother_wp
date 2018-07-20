// license:BSD-3-Clause
// copyright-holders:Roberto Fresca

#include "machine/watchdog.h"
#include "emupal.h"

class ampoker2_state : public driver_device
{
public:
	ampoker2_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_videoram(*this, "videoram"),
		m_maincpu(*this, "maincpu"),
		m_watchdog(*this, "watchdog"),
		m_gfxdecode(*this, "gfxdecode"),
		m_lamps(*this, "lamp%u", 0U)
	{ }

	void sigma2k(machine_config &config);
	void ampoker2(machine_config &config);

	DECLARE_WRITE8_MEMBER(ampoker2_port30_w);
	DECLARE_WRITE8_MEMBER(ampoker2_port31_w);
	DECLARE_WRITE8_MEMBER(ampoker2_port32_w);
	DECLARE_WRITE8_MEMBER(ampoker2_port33_w);
	DECLARE_WRITE8_MEMBER(ampoker2_port34_w);
	DECLARE_WRITE8_MEMBER(ampoker2_port35_w);
	DECLARE_WRITE8_MEMBER(ampoker2_port36_w);
	DECLARE_WRITE8_MEMBER(ampoker2_watchdog_reset_w);
	DECLARE_WRITE8_MEMBER(ampoker2_videoram_w);
	void init_rabbitpk();
	void init_piccolop();
	TILE_GET_INFO_MEMBER(get_bg_tile_info);
	TILE_GET_INFO_MEMBER(s2k_get_bg_tile_info);
	DECLARE_PALETTE_INIT(ampoker2);
	DECLARE_VIDEO_START(sigma2k);

protected:
	virtual void video_start() override;
	virtual void machine_start() override;

private:
	void ampoker2_io_map(address_map &map);
	void ampoker2_map(address_map &map);

	required_shared_ptr<uint8_t> m_videoram;
	tilemap_t *m_bg_tilemap;
	uint32_t screen_update_ampoker2(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	required_device<cpu_device> m_maincpu;
	required_device<watchdog_timer_device> m_watchdog;
	required_device<gfxdecode_device> m_gfxdecode;
	output_finder<10> m_lamps;
};
