// license:BSD-3-Clause
// copyright-holders:
/*
infos provided by Sean Riddle

Milton Bradley Milton

Chips labeled

SC87008P
7834043001
R2H8033
28 DIP = Motorola MC6805P2

GI
783-4043-002
8033CBA
28 DIP = GI SP0250

4043-003
DDS 8027
16 DIP = TMC0430 8K GROM

4043-004
DDS 8022
16 DIP = TMC0430 8K GROM

20 buttons: 1, 2, 3, Go, Score, Reset, Red A-G, Yellow A-G
Speaker
2 LEDs (pulse with speech)

schematic in patent 4326710 except MC6805 clocked from SP0250 3.12MHz and GROM clocked by 3.12MHz/8=390KHz

MC6805 pinout

1  VSS     ground
2  /INT    tied high
3  VCC     +5
4  Extal   SP0250 pin 9 3.12MHz clock
5  Xtal    ground
6  NUM     ground
7  Timer   VCC
8  PC0     button matrix
9  PC1     button matrix
10 PC2     button matrix
11 PC3     button matrix
12 PB0     button matrix
13 PB1     button matrix+GROM M
14 PB2     button matrix
15 PB3     button matrix+GROM M0
16 PB4     button matrix
17 PB5     SP0250 DPRES
18 PB6     SP0250 DREQ
19 PB7     GROM STROBE
20 PA0     SP0250 and GROM D0
21 PA1     SP0250 and GROM D1
22 PA2     SP0250 and GROM D2
23 PA3     SP0250 and GROM D3
24 PA4     SP0250 and GROM D4
25 PA5     SP0250 and GROM D5
26 PA6     SP0250 and GROM D6
27 PA7     SP0250 and GROM D7
28 /RESET  cap and diode


button matrix
      PB0      PB1     PB2    PB3       PB4
PC0  red G    red C     2    Reset    Yellow D
PC1  red F    red B     3   Yellow A  Yellow E
PC2  red E    red A    GO   Yellow B  Yellow F
PC3  red D      1     Score Yellow C  Yellow G

button layout
red
  D E
C     F
 B   G
   A

yellow
   A
 G   B
F     C
  E D
*/

#include "emu.h"
#include "cpu/m6805/m6805.h"
#include "machine/tmc0430.h"
#include "sound/sp0250.h"
#include "screen.h"
#include "speaker.h"

class milton_state : public driver_device
{
public:
	milton_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu")
	{ }

	void milton(machine_config &config);

private:
	required_device<cpu_device> m_maincpu;

	void prg_map(address_map &map);
};

void milton_state::prg_map(address_map &map)
{
	map(0x800, 0xfff).rom().region("maincpu", 0); // Internal ROM
}

static INPUT_PORTS_START( milton )
INPUT_PORTS_END

MACHINE_CONFIG_START(milton_state::milton)

	/* basic machine hardware */
	MCFG_DEVICE_ADD("maincpu", M6805, 3120000) // MC6805P2, needs a CPU core
	MCFG_DEVICE_PROGRAM_MAP(prg_map)

	MCFG_DEVICE_ADD("grom3", TMC0430, 3120000 / 8)
	downcast<tmc0430_device &>(*device).set_region_and_ident("groms", 0x0000, 0);

	MCFG_DEVICE_ADD("grom4", TMC0430, 3120000 / 8)
	downcast<tmc0430_device &>(*device).set_region_and_ident("groms", 0x2000, 1);

	SPEAKER(config, "speaker").front_center();
	MCFG_DEVICE_ADD("sp0250", SP0250, 3120000)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 1.0)
MACHINE_CONFIG_END

/***************************************************************************

  Game drivers

***************************************************************************/

ROM_START( milton )
	ROM_REGION(0x800, "maincpu", 0)
	ROM_LOAD("milton.bin", 0x000, 0x800, CRC(acb261fd) SHA1(6efd836578c580031d9835191afd20cdd192125d) )

	ROM_REGION(0x4000, "groms", 0)
	ROM_LOAD("miltongrom3.bin", 0x0000, 0x1800, CRC(d95df757) SHA1(6723480866f6393d310e304ef3b61e3a319a7beb) )
	ROM_LOAD("miltongrom4.bin", 0x2000, 0x1800, CRC(9ac929f7) SHA1(1a27d56fc49eb4e58ea3b5c58d7fbedc5a751592) )
ROM_END

CONS( 1980, milton, 0, 0, milton, milton, milton_state, empty_init, "Milton Bradley", "Electronic Milton",  MACHINE_IS_SKELETON )
