// license:BSD-3-Clause
// copyright-holders:Mariusz Wojcieszek
/****************************************************************************

    Go! Go! Connie chan Jaka Jaka Janken

    Driver by Mariusz Wojcieszek

    EC9601

    Hudson Chip
    CPU  :Hu6280
    Video:Hu6202,Hu6260,Hu6270

    OSC  :21.47727MHz
    Other:XILINX XC7336-15,OKI M6295


****************************************************************************/

#include "emu.h"
#include "machine/pcecommn.h"

#include "video/huc6270.h"
#include "video/huc6260.h"
#include "video/huc6202.h"
#include "sound/c6280.h"
#include "sound/okim6295.h"
#include "machine/msm6242.h"
#include "screen.h"
#include "speaker.h"


class ggconnie_state : public pce_common_state
{
public:
	ggconnie_state(const machine_config &mconfig, device_type type, const char *tag)
		: pce_common_state(mconfig, type, tag)
		, m_rtc(*this, "rtc")
		, m_oki(*this, "oki")
	{ }

	void ggconnie(machine_config &config);

private:
	DECLARE_WRITE8_MEMBER(lamp_w);
	DECLARE_WRITE8_MEMBER(output_w);
	DECLARE_WRITE8_MEMBER(oki_bank_w);
	void sgx_io(address_map &map);
	void sgx_mem(address_map &map);
	required_device <msm6242_device> m_rtc;
	required_device <okim6295_device> m_oki;
};

WRITE8_MEMBER(ggconnie_state::lamp_w)
{
	output().set_value("lamp", !BIT(data,0));
}

WRITE8_MEMBER(ggconnie_state::output_w)
{
	// written in "Output Test" in test mode
}

/* TODO: banking not understood (is the ROM dumped correctly btw?) */
WRITE8_MEMBER(ggconnie_state::oki_bank_w)
{
	m_oki->set_rom_bank(data != 0);
}


void ggconnie_state::sgx_mem(address_map &map)
{
	map(0x000000, 0x17ffff).rom();
	map(0x180000, 0x1edfff).noprw();
	map(0x1ee800, 0x1effff).noprw();
	map(0x1f0000, 0x1f5fff).ram();
	map(0x1f7000, 0x1f7000).portr("SWA");
	map(0x1f7100, 0x1f7100).portr("SWB");
	map(0x1f7200, 0x1f7200).portr("SWC");
	map(0x1f7700, 0x1f7700).portr("IN1");
	map(0x1f7800, 0x1f7800).w(FUNC(ggconnie_state::output_w));
	map(0x1fe000, 0x1fe007).rw("huc6270_0", FUNC(huc6270_device::read), FUNC(huc6270_device::write)).mirror(0x03E0);
	map(0x1fe008, 0x1fe00f).rw("huc6202", FUNC(huc6202_device::read), FUNC(huc6202_device::write)).mirror(0x03E0);
	map(0x1fe010, 0x1fe017).rw("huc6270_1", FUNC(huc6270_device::read), FUNC(huc6270_device::write)).mirror(0x03E0);
	map(0x1fe400, 0x1fe7ff).rw(m_huc6260, FUNC(huc6260_device::read), FUNC(huc6260_device::write));
	map(0x1fe800, 0x1febff).rw("c6280", FUNC(c6280_device::c6280_r), FUNC(c6280_device::c6280_w));
	map(0x1fec00, 0x1fefff).rw(m_maincpu, FUNC(h6280_device::timer_r), FUNC(h6280_device::timer_w));
	map(0x1f7300, 0x1f7300).rw(m_oki, FUNC(okim6295_device::read), FUNC(okim6295_device::write));
	map(0x1f7400, 0x1f74ff).w(FUNC(ggconnie_state::oki_bank_w));
	map(0x1f7500, 0x1f750f).rw(m_rtc, FUNC(msm6242_device::read), FUNC(msm6242_device::write));
	map(0x1ff000, 0x1ff000).portr("IN0").w(FUNC(ggconnie_state::lamp_w));
	map(0x1ff400, 0x1ff7ff).rw(m_maincpu, FUNC(h6280_device::irq_status_r), FUNC(h6280_device::irq_status_w));
}

void ggconnie_state::sgx_io(address_map &map)
{
	map(0x00, 0x03).rw("huc6202", FUNC(huc6202_device::io_read), FUNC(huc6202_device::io_write));
}


static INPUT_PORTS_START(ggconnie)
	PORT_START("IN0")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_BUTTON6 ) PORT_NAME( "Medal" )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_COIN1 ) /* 100 Yen */
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_COIN2 ) /* 10 Yen */
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_START1 ) /* run */
	PORT_SERVICE_NO_TOGGLE(0x10, IP_ACTIVE_LOW)  PORT_DIPLOCATION("SWC:8")
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_NAME("Hopper")
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("IN1")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_START )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_BUTTON1 )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_BUTTON2 )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_BUTTON3 )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("SWA")
	PORT_DIPNAME(0x03, 0x03, "Coin Set")  PORT_DIPLOCATION("SWA:1,2")
	PORT_DIPSETTING(0x03, DEF_STR(1C_1C) )
	PORT_DIPSETTING(0x02, "2")
	PORT_DIPSETTING(0x01, "1")
	PORT_DIPSETTING(0x00, "0")
	PORT_DIPNAME(0x1c, 0x1c, "100 Yen -> Coin" )  PORT_DIPLOCATION("SWA:3,4,5")
	PORT_DIPSETTING(0x08, "0 Coin")
	PORT_DIPSETTING(0x0c, "5 Coin")
	PORT_DIPSETTING(0x10, "6 Coin")
	PORT_DIPSETTING(0x14, "7 Coin")
	PORT_DIPSETTING(0x18, "8 Coin")
	PORT_DIPSETTING(0x1c, "10 Coin")
	PORT_DIPSETTING(0x00, "11 Coin")
	PORT_DIPSETTING(0x04, "12 Coin")
	PORT_DIPUNUSED_DIPLOC( 0x20, 0x00, "SWA:6" )
	PORT_DIPUNUSED_DIPLOC( 0x40, 0x00, "SWA:7" )
	PORT_DIPUNUSED_DIPLOC( 0x80, 0x00, "SWA:8" )

	PORT_START("SWB")
	PORT_DIPNAME(0x07, 0x07, "Payout")  PORT_DIPLOCATION("SWB:1,2,3")
	PORT_DIPSETTING(0x00, "85%")
	PORT_DIPSETTING(0x01, "90%")
	PORT_DIPSETTING(0x02, "55%")
	PORT_DIPSETTING(0x03, "60%")
	PORT_DIPSETTING(0x04, "65%")
	PORT_DIPSETTING(0x05, "70%")
	PORT_DIPSETTING(0x06, "75%")
	PORT_DIPSETTING(0x07, "80%")
	PORT_DIPNAME(0x18, 0x18, DEF_STR(Difficulty))  PORT_DIPLOCATION("SWB:4,5")
	PORT_DIPSETTING(0x00, DEF_STR(Easy))
	PORT_DIPSETTING(0x08, DEF_STR(Very_Hard))
	PORT_DIPSETTING(0x10, DEF_STR(Hard))
	PORT_DIPSETTING(0x18, DEF_STR(Normal))
	PORT_DIPNAME(0x20, 0x20, "Payout Info")  PORT_DIPLOCATION("SWB:6")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x20, DEF_STR(On))
	PORT_DIPNAME(0xc0, 0xc0, "Rate")  PORT_DIPLOCATION("SWB:7,8")
	PORT_DIPSETTING(0x00, "Few" )
	PORT_DIPSETTING(0x40, "Most" )
	PORT_DIPSETTING(0x80, "More" )
	PORT_DIPSETTING(0xc0, DEF_STR(Normal))

	PORT_START("SWC")
	PORT_DIPNAME(0x03, 0x03, "Demo Sound" )  PORT_DIPLOCATION("SWC:1,2")
	PORT_DIPSETTING(0x00, DEF_STR(Off) )
	PORT_DIPSETTING(0x01, "3/1" )
	PORT_DIPSETTING(0x02, "2/1" )
	PORT_DIPSETTING(0x03, "1/1" )
	PORT_DIPNAME(0x0c, 0x0c, "Start Time" )  PORT_DIPLOCATION("SWC:3,4")
	PORT_DIPSETTING(0x00, "4 sec" )
	PORT_DIPSETTING(0x04, "8 sec" )
	PORT_DIPSETTING(0x08, "6 sec" )
	PORT_DIPSETTING(0x0c, "5 sec" )
	PORT_DIPUNUSED_DIPLOC( 0x10, 0x00, "SWC:5" )
	PORT_DIPUNUSED_DIPLOC( 0x20, 0x00, "SWC:6" )
	PORT_DIPNAME(0x40, 0x00, "RAM Clear" )  PORT_DIPLOCATION("SWC:7")
	PORT_DIPSETTING(0x40, DEF_STR(Off) )
	PORT_DIPSETTING(0x00, DEF_STR(On) )
	PORT_DIPNAME(0x80, 0x80, DEF_STR(Service_Mode) )  PORT_DIPLOCATION("SWC:8")
	PORT_DIPSETTING(0x80, DEF_STR(Off) )
	PORT_DIPSETTING(0x00, DEF_STR(On) )
INPUT_PORTS_END

static INPUT_PORTS_START(smf)
	PORT_START("IN0")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_BUTTON7 ) PORT_NAME( "Medal" )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_COIN1 ) /* 100 Yen */
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_COIN2 ) /* 10 Yen */
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_START1 ) /* run */
	PORT_SERVICE_NO_TOGGLE(0x10, IP_ACTIVE_LOW)  PORT_DIPLOCATION("SWC:8")
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_BUTTON6 ) PORT_NAME("Hopper")
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("IN1")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_START )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_BUTTON5 )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_BUTTON4 )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_BUTTON3 )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON2 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON1 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_BUTTON8 ) PORT_NAME( "Payout" )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("SWA")
	PORT_DIPNAME(0x03, 0x03, "Coin Set")  PORT_DIPLOCATION("SWA:1,2")
	PORT_DIPSETTING(0x03, DEF_STR(1C_1C) )
	PORT_DIPSETTING(0x02, "2")
	PORT_DIPSETTING(0x01, "1")
	PORT_DIPSETTING(0x00, "0")
	PORT_DIPNAME(0x1c, 0x1c, "100 Yen -> Coin" )  PORT_DIPLOCATION("SWA:3,4,5")
	PORT_DIPSETTING(0x08, "0 Coin")
	PORT_DIPSETTING(0x0c, "5 Coin")
	PORT_DIPSETTING(0x10, "6 Coin")
	PORT_DIPSETTING(0x14, "7 Coin")
	PORT_DIPSETTING(0x18, "8 Coin")
	PORT_DIPSETTING(0x1c, "10 Coin")
	PORT_DIPSETTING(0x00, "11 Coin")
	PORT_DIPSETTING(0x04, "12 Coin")
	PORT_DIPNAME( 0x60, 0x40, "10 Yen Set" )   PORT_DIPLOCATION("SWA:6,7")
	PORT_DIPSETTING(    0x60, DEF_STR(1C_1C)  )
	PORT_DIPSETTING(    0x40, DEF_STR(2C_1C) )
	PORT_DIPSETTING(    0x20, DEF_STR(3C_1C) )
	PORT_DIPSETTING(    0x00, DEF_STR(Off) )
	PORT_DIPUNUSED_DIPLOC( 0x80, 0x00, "SWA:8" )

	PORT_START("SWB")
	PORT_DIPNAME(0x07, 0x07, "Payout")  PORT_DIPLOCATION("SWB:1,2,3")
	PORT_DIPSETTING(0x00, "85%")
	PORT_DIPSETTING(0x01, "90%")
	PORT_DIPSETTING(0x02, "55%")
	PORT_DIPSETTING(0x03, "60%")
	PORT_DIPSETTING(0x04, "65%")
	PORT_DIPSETTING(0x05, "70%")
	PORT_DIPSETTING(0x06, "75%")
	PORT_DIPSETTING(0x07, "80%")
	PORT_DIPUNUSED_DIPLOC( 0x08, 0x00, "SWB:4" )
	PORT_DIPUNUSED_DIPLOC( 0x10, 0x00, "SWB:5" )
	PORT_DIPNAME(0x20, 0x20, "Payout Info")  PORT_DIPLOCATION("SWB:6")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x20, DEF_STR(On))
	PORT_DIPNAME(0xc0, 0xc0, "Rate")  PORT_DIPLOCATION("SWB:7,8")
	PORT_DIPSETTING(0x00, "Few" )
	PORT_DIPSETTING(0x40, "Most" )
	PORT_DIPSETTING(0x80, "More" )
	PORT_DIPSETTING(0xc0, DEF_STR(Normal))

	PORT_START("SWC")
	PORT_DIPNAME(0x03, 0x03, "Demo Sound" )  PORT_DIPLOCATION("SWC:1,2")
	PORT_DIPSETTING(0x00, DEF_STR(Off) )
	PORT_DIPSETTING(0x01, "1/3" )
	PORT_DIPSETTING(0x02, "1/2" )
	PORT_DIPSETTING(0x03, "1/1" )
	PORT_DIPUNUSED_DIPLOC( 0x04, 0x00, "SWC:3" )
	PORT_DIPUNUSED_DIPLOC( 0x08, 0x00, "SWC:4" )
	PORT_DIPUNUSED_DIPLOC( 0x10, 0x00, "SWC:5" )
	PORT_DIPUNUSED_DIPLOC( 0x20, 0x00, "SWC:6" )
	PORT_DIPNAME(0x40, 0x00, "RAM Clear" )  PORT_DIPLOCATION("SWC:7")
	PORT_DIPSETTING(0x40, DEF_STR(Off) )
	PORT_DIPSETTING(0x00, DEF_STR(On) )
	PORT_DIPNAME(0x80, 0x80, DEF_STR(Service_Mode) )  PORT_DIPLOCATION("SWC:8")
	PORT_DIPSETTING(0x80, DEF_STR(Off) )
	PORT_DIPSETTING(0x00, DEF_STR(On) )
INPUT_PORTS_END


MACHINE_CONFIG_START(ggconnie_state::ggconnie)
	/* basic machine hardware */
	MCFG_DEVICE_ADD("maincpu", H6280, PCE_MAIN_CLOCK/3)
	MCFG_DEVICE_PROGRAM_MAP(sgx_mem)
	MCFG_DEVICE_IO_MAP(sgx_io)

	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_RAW_PARAMS(PCE_MAIN_CLOCK/3, huc6260_device::WPF, 64, 64 + 1024 + 64, huc6260_device::LPF, 18, 18 + 242)
	MCFG_SCREEN_UPDATE_DRIVER( ggconnie_state, screen_update )
	MCFG_SCREEN_PALETTE("huc6260")

	MCFG_DEVICE_ADD("huc6260", HUC6260, PCE_MAIN_CLOCK/3)
	MCFG_HUC6260_NEXT_PIXEL_DATA_CB(READ16("huc6202", huc6202_device, next_pixel))
	MCFG_HUC6260_TIME_TIL_NEXT_EVENT_CB(READ16("huc6202", huc6202_device, time_until_next_event))
	MCFG_HUC6260_VSYNC_CHANGED_CB(WRITELINE("huc6202", huc6202_device, vsync_changed))
	MCFG_HUC6260_HSYNC_CHANGED_CB(WRITELINE("huc6202", huc6202_device, hsync_changed))
	MCFG_DEVICE_ADD( "huc6270_0", HUC6270, 0 )
	MCFG_HUC6270_VRAM_SIZE(0x10000)
	MCFG_HUC6270_IRQ_CHANGED_CB(INPUTLINE("maincpu", 0))
	MCFG_DEVICE_ADD( "huc6270_1", HUC6270, 0 )
	MCFG_HUC6270_VRAM_SIZE(0x10000)
	MCFG_HUC6270_IRQ_CHANGED_CB(INPUTLINE("maincpu", 0))
	MCFG_DEVICE_ADD( "huc6202", HUC6202, 0 )
	MCFG_HUC6202_NEXT_PIXEL_0_CB(READ16("huc6270_0", huc6270_device, next_pixel))
	MCFG_HUC6202_TIME_TIL_NEXT_EVENT_0_CB(READ16("huc6270_0", huc6270_device, time_until_next_event))
	MCFG_HUC6202_VSYNC_CHANGED_0_CB(WRITELINE("huc6270_0", huc6270_device, vsync_changed))
	MCFG_HUC6202_HSYNC_CHANGED_0_CB(WRITELINE("huc6270_0", huc6270_device, hsync_changed))
	MCFG_HUC6202_READ_0_CB(READ8("huc6270_0", huc6270_device, read))
	MCFG_HUC6202_WRITE_0_CB(WRITE8("huc6270_0", huc6270_device, write))
	MCFG_HUC6202_NEXT_PIXEL_1_CB(READ16("huc6270_1", huc6270_device, next_pixel))
	MCFG_HUC6202_TIME_TIL_NEXT_EVENT_1_CB(READ16("huc6270_1", huc6270_device, time_until_next_event))
	MCFG_HUC6202_VSYNC_CHANGED_1_CB(WRITELINE("huc6270_1", huc6270_device, vsync_changed))
	MCFG_HUC6202_HSYNC_CHANGED_1_CB(WRITELINE("huc6270_1", huc6270_device, hsync_changed))
	MCFG_HUC6202_READ_1_CB(READ8("huc6270_1", huc6270_device, read))
	MCFG_HUC6202_WRITE_1_CB(WRITE8("huc6270_1", huc6270_device, write))

	MCFG_DEVICE_ADD("rtc", MSM6242, XTAL(32'768))

	SPEAKER(config, "lspeaker").front_left();
	SPEAKER(config, "rspeaker").front_right();
	MCFG_DEVICE_ADD("c6280", C6280, PCE_MAIN_CLOCK/6)
	MCFG_C6280_CPU("maincpu")
	MCFG_SOUND_ROUTE(0, "lspeaker", 1.00)
	MCFG_SOUND_ROUTE(1, "rspeaker", 1.00)

	MCFG_DEVICE_ADD("oki", OKIM6295, PCE_MAIN_CLOCK/12, okim6295_device::PIN7_HIGH) /* unknown clock / pin 7 */
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "lspeaker", 1.00)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "rspeaker", 1.00)
MACHINE_CONFIG_END

ROM_START(ggconnie)
	ROM_REGION( 0x180000, "maincpu", 0 )
	ROM_LOAD( "prg0_u3_ver.2.bin", 0x00000, 0x80000, CRC(5e104855) SHA1(3ab2b1ec1fc3aefbb57d9b2ba272e75b34b69383) )
	ROM_LOAD( "prg1_u4.bin", 0x80000, 0x80000, CRC(513f0b18) SHA1(44c61dc1a06bb4c8b4840ea6a372f92114888490) )
	// u5 not populated

	ROM_REGION( 0x80000, "oki", 0 )
	ROM_LOAD( "adpcm_u31.bin", 0x00000, 0x80000, CRC(de514c2b) SHA1(da73aa825d73646f556f6d4dbb46f43acf7c3357) )
ROM_END

// TODO: runs too slow (seems to run ok removing the /3 divider), OKI banking, hopper, lamps
ROM_START(smf)
	ROM_REGION( 0x180000, "maincpu", 0 )
	ROM_LOAD( "smf03.u3", 0x000000, 0x80000, CRC(2435ff3d) SHA1(4de1c5c2ed4ce2be5f3bb3fd31e176c8e24c7155) ) // 27c040
	ROM_LOAD( "smf05.u4", 0x080000, 0x80000, CRC(7c477ae1) SHA1(ecdc1bf7052121f4ce3ef222c0f51d72057a3a2b) ) // 27c040
	ROM_LOAD( "smf04.u5", 0x100000, 0x80000, CRC(8adc8ff6) SHA1(06bca0bf09bb6094700d2c3d4fc8aa7246e4b3f7) ) // 27c040

	ROM_REGION( 0x80000, "oki", 0 )
	ROM_LOAD( "smf01.u31", 0x00000, 0x80000, CRC(141ff32a) SHA1(7a7ef623c5dd5fdfa7364c2b75136fe81aea3b43) ) // 27c040

	ROM_REGION( 0x600, "plds", 0 ) // protected
	ROM_LOAD( "gal16v8b.u6", 0x000, 0x117, NO_DUMP )
	ROM_LOAD( "gal16v8b.u7", 0x200, 0x117, NO_DUMP )
	ROM_LOAD( "gal16v8b.u8", 0x400, 0x117, NO_DUMP )
ROM_END

GAME( 1996, ggconnie, 0, ggconnie, ggconnie, ggconnie_state, init_pce_common, ROT0, "Eighting", "Go! Go! Connie chan Jaka Jaka Janken", MACHINE_IMPERFECT_SOUND | MACHINE_NOT_WORKING )
GAME( 1997, smf,      0, ggconnie, smf,      ggconnie_state, init_pce_common, ROT0, "Eighting", "Super Medal Fighters (Japan 970228)", MACHINE_IMPERFECT_SOUND | MACHINE_NOT_WORKING )
