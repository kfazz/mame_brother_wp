// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria, Pierpaolo Prazzoli
// thanks-to:HIGHWAYMAN
/*
    Wink    -   (c) 1985 Midcoin

    TODO:
    - better interrupts?
    - finish sound
    - fix protection properly
    - better handling of nvram? it loses the default values
    - I need a better comparison screenshot to be sure about the colors.
*/

#include "emu.h"
#include "cpu/z80/z80.h"
#include "machine/gen_latch.h"
#include "machine/nvram.h"
#include "sound/ay8910.h"
#include "screen.h"
#include "speaker.h"


class wink_state : public driver_device
{
public:
	wink_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_audiocpu(*this, "audiocpu"),
		m_gfxdecode(*this, "gfxdecode"),
		m_videoram(*this, "videoram") { }

	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	required_device<gfxdecode_device> m_gfxdecode;

	required_shared_ptr<uint8_t> m_videoram;

	tilemap_t *m_bg_tilemap;
	uint8_t m_sound_flag;
	uint8_t m_tile_bank;

	DECLARE_WRITE8_MEMBER(bgram_w);
	DECLARE_WRITE8_MEMBER(player_mux_w);
	DECLARE_WRITE8_MEMBER(tile_banking_w);
	DECLARE_WRITE8_MEMBER(wink_coin_counter_w);
	DECLARE_READ8_MEMBER(analog_port_r);
	DECLARE_READ8_MEMBER(player_inputs_r);
	DECLARE_WRITE8_MEMBER(sound_irq_w);
	DECLARE_READ8_MEMBER(prot_r);
	DECLARE_WRITE8_MEMBER(prot_w);
	DECLARE_READ8_MEMBER(sound_r);

	TILE_GET_INFO_MEMBER(get_bg_tile_info);

	DECLARE_DRIVER_INIT(wink);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void video_start() override;

	uint32_t screen_update_wink(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	INTERRUPT_GEN_MEMBER(wink_sound);
	void wink(machine_config &config);
	void wink_io(address_map &map);
	void wink_map(address_map &map);
	void wink_sound_io(address_map &map);
	void wink_sound_map(address_map &map);
};


TILE_GET_INFO_MEMBER(wink_state::get_bg_tile_info)
{
	uint8_t *videoram = m_videoram;
	int code = videoram[tile_index];
	code |= 0x200 * m_tile_bank;

	// the 2 parts of the screen use different tile banking
	if(tile_index < 0x360)
	{
		code |= 0x100;
	}

	SET_TILE_INFO_MEMBER(0, code, 0, 0);
}

void wink_state::video_start()
{
	m_bg_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(FUNC(wink_state::get_bg_tile_info),this), TILEMAP_SCAN_ROWS, 8, 8, 32, 32);
}

uint32_t wink_state::screen_update_wink(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	m_bg_tilemap->draw(screen, bitmap, cliprect, 0, 0);
	return 0;
}

WRITE8_MEMBER(wink_state::bgram_w)
{
	uint8_t *videoram = m_videoram;
	videoram[offset] = data;
	m_bg_tilemap->mark_tile_dirty(offset);
}

WRITE8_MEMBER(wink_state::player_mux_w)
{
	//player_mux = data & 1;
	//no mux / cocktail mode in the real pcb? strange...
}

WRITE8_MEMBER(wink_state::tile_banking_w)
{
	m_tile_bank = data & 1;
	m_bg_tilemap->mark_all_dirty();
}

WRITE8_MEMBER(wink_state::wink_coin_counter_w)
{
	machine().bookkeeping().coin_counter_w(offset,data & 1);
}

READ8_MEMBER(wink_state::analog_port_r)
{
	return ioport(/* player_mux ? "DIAL2" : */ "DIAL1")->read();
}

READ8_MEMBER(wink_state::player_inputs_r)
{
	return ioport(/* player_mux ? "INPUTS2" : */ "INPUTS1")->read();
}

WRITE8_MEMBER(wink_state::sound_irq_w)
{
	m_audiocpu->set_input_line(0, HOLD_LINE);
	//sync with sound cpu (but it still loses some soundlatches...)
	//machine().scheduler().synchronize();
}

void wink_state::wink_map(address_map &map)
{
	map(0x0000, 0x7fff).rom();
	map(0x8000, 0x87ff).ram();
	map(0x9000, 0x97ff).ram().share("nvram");
	map(0xa000, 0xa3ff).ram().w(this, FUNC(wink_state::bgram_w)).share("videoram");
}


READ8_MEMBER(wink_state::prot_r)
{
	//take a0-a7 and do some math using the variable created from the upper address-lines,
	//put the result onto the databus.

/*
math

take 2 bytes:

byte1 = a8,a9,a10,a11,a12,a13,a14,a15
byte2 = a0,a2,a4,a6,a1,a3,a5,a7

add the 2 bytes together so that if the final value overflows past 255 it wraps to 0
the 8bit result is placed on the databus.

*/
	return 0x20; //hack to pass the jump calculated using this value
}

WRITE8_MEMBER(wink_state::prot_w)
{
	//take a9-a15 and stuff them in a variable for later use.
}

void wink_state::wink_io(address_map &map)
{
	map.global_mask(0xff);
	map(0x00, 0x1f).ram().w("palette", FUNC(palette_device::write8)).share("palette"); //0x10-0x1f is likely to be something else
//  AM_RANGE(0x20, 0x20) AM_WRITENOP                //??? seems unused..
	map(0x21, 0x21).w(this, FUNC(wink_state::player_mux_w));     //??? no mux on the pcb.
	map(0x22, 0x22).w(this, FUNC(wink_state::tile_banking_w));
//  AM_RANGE(0x23, 0x23) AM_WRITENOP                //?
//  AM_RANGE(0x24, 0x24) AM_WRITENOP                //cab Knocker like in q-bert!
	map(0x25, 0x27).w(this, FUNC(wink_state::wink_coin_counter_w));
	map(0x40, 0x40).w("soundlatch", FUNC(generic_latch_8_device::write));
	map(0x60, 0x60).w(this, FUNC(wink_state::sound_irq_w));
	map(0x80, 0x80).r(this, FUNC(wink_state::analog_port_r));
	map(0xa0, 0xa0).r(this, FUNC(wink_state::player_inputs_r));
	map(0xa4, 0xa4).portr("DSW1");   //dipswitch bank2
	map(0xa8, 0xa8).portr("DSW2");   //dipswitch bank1
//  AM_RANGE(0xac, 0xac) AM_WRITENOP            //protection - loads video xor unit (written only once at startup)
	map(0xb0, 0xb0).portr("DSW3");   //unused inputs
	map(0xb4, 0xb4).portr("DSW4");   //dipswitch bank3
	map(0xc0, 0xdf).w(this, FUNC(wink_state::prot_w));       //load load protection-buffer from upper address bus
	map(0xc3, 0xc3).nopr();             //watchdog?
	map(0xe0, 0xff).r(this, FUNC(wink_state::prot_r));        //load math unit from buffer & lower address-bus
}

void wink_state::wink_sound_map(address_map &map)
{
	map(0x0000, 0x1fff).rom();
	map(0x4000, 0x43ff).ram();
	map(0x8000, 0x8000).r("soundlatch", FUNC(generic_latch_8_device::read));
}

void wink_state::wink_sound_io(address_map &map)
{
	map.global_mask(0xff);
	map(0x00, 0x00).rw("aysnd", FUNC(ay8910_device::data_r), FUNC(ay8910_device::data_w));
	map(0x80, 0x80).w("aysnd", FUNC(ay8910_device::address_w));
}

static INPUT_PORTS_START( wink )
	PORT_START("DIAL1")
	PORT_BIT( 0xff, 0x00, IPT_DIAL ) PORT_SENSITIVITY(50) PORT_KEYDELTA(3) PORT_REVERSE

	PORT_START("INPUTS1")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_BUTTON2 )    // right curve
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_BUTTON1 )    // left curve
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_START1 )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_COIN3 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_BUTTON3 )    // slam

	PORT_START("DSW1")
	PORT_DIPNAME( 0x01, 0x01, "1" )
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x02, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x04, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x04, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x08, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x10, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x20, 0x20, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x20, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )

	PORT_START("DSW2")
	PORT_DIPNAME( 0x01, 0x01, "2" )
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x02, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x04, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x04, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x08, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x10, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x20, 0x20, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x20, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )

	PORT_START("DSW3")
	PORT_DIPNAME( 0x01, 0x01, "3" )
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x02, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x04, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x04, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x08, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x10, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x10, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x20, 0x20, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x20, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )

	PORT_START("DSW4")
	PORT_DIPNAME( 0x01, 0x01, "Summary" )
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x02, "4" )
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x04, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x04, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x08, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_SERVICE( 0x10, IP_ACTIVE_LOW )
	PORT_DIPNAME( 0x20, 0x20, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x20, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x40, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
INPUT_PORTS_END

static const gfx_layout charlayout =
{
	8,8,    /* 8*8 characters */
	RGN_FRAC(1,3),
	3,
	{ RGN_FRAC(0,3), RGN_FRAC(1,3), RGN_FRAC(2,3) },
	{ 0, 1, 2, 3, 4, 5, 6, 7 },
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8 },
	8*8 /* every char takes 8 consecutive bytes */
};

static GFXDECODE_START( wink )
	GFXDECODE_ENTRY( "gfx1", 0, charlayout, 0, 4 )
GFXDECODE_END

READ8_MEMBER(wink_state::sound_r)
{
	return m_sound_flag;
}

//AY portA is fed by an input clock at 15625 Hz
INTERRUPT_GEN_MEMBER(wink_state::wink_sound)
{
	m_sound_flag ^= 0x80;
}

void wink_state::machine_start()
{
	save_item(NAME(m_sound_flag));
	save_item(NAME(m_tile_bank));
}

void wink_state::machine_reset()
{
	m_sound_flag = 0;
}

MACHINE_CONFIG_START(wink_state::wink)
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", Z80, 12000000 / 4)
	MCFG_CPU_PROGRAM_MAP(wink_map)
	MCFG_CPU_IO_MAP(wink_io)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", wink_state,  nmi_line_pulse)

	MCFG_CPU_ADD("audiocpu", Z80, 12000000 / 8)
	MCFG_CPU_PROGRAM_MAP(wink_sound_map)
	MCFG_CPU_IO_MAP(wink_sound_io)
	MCFG_CPU_PERIODIC_INT_DRIVER(wink_state, wink_sound,  15625)

	MCFG_NVRAM_ADD_1FILL("nvram")

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(32*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 32*8-1, 0*8, 32*8-1)
	MCFG_SCREEN_UPDATE_DRIVER(wink_state, screen_update_wink)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", wink)
	MCFG_PALETTE_ADD("palette", 16)
	MCFG_PALETTE_FORMAT(xxxxBBBBRRRRGGGG)


	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_GENERIC_LATCH_8_ADD("soundlatch")

	MCFG_SOUND_ADD("aysnd", AY8912, 12000000 / 8)
	MCFG_AY8910_PORT_A_READ_CB(READ8(wink_state, sound_r))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)
MACHINE_CONFIG_END

/***************************************************************************

  Game driver(s)

***************************************************************************/

ROM_START( wink )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "midcoin-wink00.rom", 0x0000, 0x4000, CRC(044f82d6) SHA1(4269333578c4fb14891b937c683aa5b105a193e7) )
	ROM_LOAD( "midcoin-wink01.rom", 0x4000, 0x4000, CRC(acb0a392) SHA1(428c24845a27b8021823a4a930071b3b47108f01) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "midcoin-wink05.rom", 0x0000, 0x2000, CRC(c6c9d9cf) SHA1(99984905282c2310058d1ce93aec68d8a920b2c0) )

	ROM_REGION( 0x6000, "gfx1", 0 )
	ROM_LOAD( "midcoin-wink02.rom", 0x0000, 0x2000, CRC(d1cd9d06) SHA1(3b3ce61a0516cc94663f6d3aff3fea46aceb771f) )
	ROM_LOAD( "midcoin-wink03.rom", 0x2000, 0x2000, CRC(2346f50c) SHA1(a8535fcde0e9782ea61ad18443186fd5a6ebdc7d) )
	ROM_LOAD( "midcoin-wink04.rom", 0x4000, 0x2000, CRC(06dd229b) SHA1(9057cf10e9ec4119297c2d40b26f0ce0c1d7b86a) )
ROM_END

ROM_START( winka )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_LOAD( "wink0.bin",    0x0000, 0x4000, CRC(554d86e5) SHA1(bf2de874a62d9137f79063d6ca1906b1ed0c87e6) )
	ROM_LOAD( "wink1.bin",    0x4000, 0x4000, CRC(9d8ad539) SHA1(77246df8195f7e3f3b06edc08d344801bf62e1ba) )

	ROM_REGION( 0x10000, "audiocpu", 0 )
	ROM_LOAD( "wink5.bin",    0x0000, 0x2000, CRC(c6c9d9cf) SHA1(99984905282c2310058d1ce93aec68d8a920b2c0) )

	ROM_REGION( 0x6000, "gfx1", 0 )
	ROM_LOAD( "wink2.bin",    0x0000, 0x2000, CRC(d1cd9d06) SHA1(3b3ce61a0516cc94663f6d3aff3fea46aceb771f) )
	ROM_LOAD( "wink3.bin",    0x2000, 0x2000, CRC(2346f50c) SHA1(a8535fcde0e9782ea61ad18443186fd5a6ebdc7d) )
	ROM_LOAD( "wink4.bin",    0x4000, 0x2000, CRC(06dd229b) SHA1(9057cf10e9ec4119297c2d40b26f0ce0c1d7b86a) )
ROM_END

DRIVER_INIT_MEMBER(wink_state,wink)
{
	uint32_t i;
	uint8_t *ROM = memregion("maincpu")->base();
	std::vector<uint8_t> buffer(0x8000);

	// protection module reverse engineered by HIGHWAYMAN

	memcpy(&buffer[0],ROM,0x8000);

	for (i = 0x0000; i <= 0x1fff; i++)
		ROM[i] = buffer[bitswap<16>(i,15,14,13, 11,12, 7, 9, 8,10, 6, 4, 5, 1, 2, 3, 0)];

	for (i = 0x2000; i <= 0x3fff; i++)
		ROM[i] = buffer[bitswap<16>(i,15,14,13, 10, 7,12, 9, 8,11, 6, 3, 1, 5, 2, 4, 0)];

	for (i = 0x4000; i <= 0x5fff; i++)
		ROM[i] = buffer[bitswap<16>(i,15,14,13,  7,10,11, 9, 8,12, 6, 1, 3, 4, 2, 5, 0)];

	for (i = 0x6000; i <= 0x7fff; i++)
		ROM[i] = buffer[bitswap<16>(i,15,14,13, 11,12, 7, 9, 8,10, 6, 4, 5, 1, 2, 3, 0)];

	for (i = 0; i < 0x8000; i++)
		ROM[i] += bitswap<8>(i & 0xff, 7,5,3,1,6,4,2,0);
}

GAME( 1985, wink,  0,    wink, wink, wink_state, wink, ROT0, "Midcoin", "Wink (set 1)", MACHINE_IMPERFECT_SOUND | MACHINE_UNEMULATED_PROTECTION | MACHINE_SUPPORTS_SAVE )
GAME( 1985, winka, wink, wink, wink, wink_state, wink, ROT0, "Midcoin", "Wink (set 2)", MACHINE_IMPERFECT_SOUND | MACHINE_UNEMULATED_PROTECTION | MACHINE_SUPPORTS_SAVE )
