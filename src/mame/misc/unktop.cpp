// license:BSD-3-Clause
// copyright-holders:

/*
Unknown PCB with no marking apart from L.C. and L.S. ('lato componenti' and 'lato saldature'),
which may point to it being manufactured in Italy.
It was sold as Top-01 Fashion Games, which Google doesn't seem to have relevant hits for.

Main components are:

HD63B03XP CPU
4.000 MHz XTAL
NEC D7759C ADPCM speech synthesizer

Maybe just the sound PCB for something else?
*/

#include "emu.h"

#include "cpu/m6800/m6801.h"
#include "sound/upd7759.h"

#include "speaker.h"


namespace {

class unktop_state : public driver_device
{
public:
	unktop_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu")
	{
	}

	void unktop(machine_config &config);

private:
	required_device<hd6303x_cpu_device> m_maincpu;

	void program_map(address_map &map);
};


void unktop_state::program_map(address_map &map)
{
	map(0x0000, 0x001f).m(m_maincpu, FUNC(hd6303x_cpu_device::hd6301x_io));
	map(0x0040, 0x00ff).ram(); // internal RAM
	//map(0x6000, 0x6000).w();
	//map(0x6800, 0x6800).w();
	//map(0x7800, 0x7800).r();
	map(0xe000, 0xffff).rom().region("maincpu", 0);
}


static INPUT_PORTS_START( unktop )
INPUT_PORTS_END


void unktop_state::unktop(machine_config &config)
{
	HD6303X(config, m_maincpu, 4_MHz_XTAL); // HD63B03XP
	m_maincpu->set_addrmap(AS_PROGRAM, &unktop_state::program_map);
	m_maincpu->in_p1_cb().set_log("P1 read");
	m_maincpu->out_p1_cb().set_log("P1 write");
	m_maincpu->in_p2_cb().set_log("P2 read");
	m_maincpu->out_p2_cb().set_log("P2 write");
	m_maincpu->in_p3_cb().set_log("P3 read");
	m_maincpu->out_p3_cb().set_log("P3 write");
	m_maincpu->in_p4_cb().set_log("P4 read");
	m_maincpu->out_p4_cb().set_log("P4 write");
	m_maincpu->in_p5_cb().set_log("P5 read");
	m_maincpu->out_p5_cb().set_log("P5 write");
	m_maincpu->in_p6_cb().set_log("P6 read");
	m_maincpu->out_p6_cb().set_log("P6 write");
	m_maincpu->out_p7_cb().set_log("P7 write");
	m_maincpu->out_sc2_cb().set_log("SC2 write");
	m_maincpu->out_ser_tx_cb().set_log("serial TX");

	SPEAKER(config, "mono").front_center();

	UPD7759(config, "upd").add_route(ALL_OUTPUTS, "mono", 0.50);
}


ROM_START( unktop ) // all labels handwritten
	ROM_REGION(0x2000, "maincpu", 0)
	ROM_LOAD( "top_iii", 0x0000, 0x2000, CRC(d79a697e) SHA1(c04691383d4eb7781f7d8fdd927848cc2e629152) ) // 0xxxxxxxxxxxx = 0xFF

	ROM_REGION( 0x20000, "upd", 0 )
	ROM_LOAD( "top_it", 0x00000, 0x20000, CRC(388b129b) SHA1(e2fe5c0249f3a5682f93fef30c1d113f51608582) )
ROM_END

} // anonymous namespace


GAME( 19??, unktop, 0, unktop, unktop, unktop_state, empty_init, ROT0, "<unknown>", "unknown Top game", MACHINE_IS_SKELETON )
