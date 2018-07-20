// license:BSD-3-Clause
// copyright-holders:David Haywood, Angelo Salese
/***************************************************************************

    Preliminary driver for XaviX TV PNP console and childs (Let's! Play TV Classic)

    CPU is an M6502 derivative with added opcodes for far-call handling

    Notes from http://www.videogameconsolelibrary.com/pg00-xavix.htm#page=reviews (thanks Guru!)
    (** this isn't entirely accurate, XaviX Tennis appears to be Super Xavix, see other notes in driver)

    XaviXPORT arrived on the scene with 3 game titles (XaviX Tennis, XaviX Bowling and XaviX Baseball) using their
    original XaviX Multiprocessor.  This proprietary chip is reported to contain an 8-bit high speed central processing
    unit (6502) at 21 MHz, picture processor, sound processor, DMA controller, 1K bytes high speed RAM, universal timer,
    AD/Converter and I/O device control.  Each cartridge comes with a wireless peripheral to be used with the game (Baseball Bat,
    Tennis Racquet, etc.) that requires "AA" batteries.  The XaviXPORT system retailed for $79.99 USD with the cartridges
    retailing for $49.99 USD.

    The following year at CES 2005, SSD COMPANY LIMITED introduced two new XaviXPORT titles (XaviX Golf and XaviX Bass Fishing) each
    containing the upgraded "Super XaviX".  This new chip is said to sport a 16-bit high central processing unit (65816) at 43 MHz.
    SSD COMPANY LIMITED is already working on their next chip called "XaviX II" that is said to be a 32-bit RISC processor
    with 3D capabilities.

    Notes:

    To access service mode in Monster Truck hold Horn and Nitro on startup

    There are multiple revisions of the CPU hardware, the SSD 2000 / SSD 2002 chips definitely add more opcodes
    (thanks to Sean Riddle for this table)

    preliminary list of XaviX software based on various sources (some likely still missing)

     year       name                                                                                            PCB ID      ROM width       TSOP pads   ROM size        SEEPROM             die markings            extra components / notes

    2011        anpan-man kazoku de ikunou mat DX/JoyPalette/Japan                                              -           -               -           -               -                   -                       -
    2009        anpan-man pyon-pyon ikunou mat/JoyPalette/Japan                                                 -           -               -           -               -                   -                       -
    2008        kyuukyoku! kinniku grand slam! SASUKE kanzen seiha/EPOCH/Japan                                  -           -               -           -               -                   -                       -
    2007        Tokyo Friend park? perfect! mezase! grand slam!/EPOCH/Japan                                     -           -               -           -               -                   -                       -
    2006    1   Let's TV Play series "Kamen Rider Kabuto" /EPOCH/Japan                                          -           -               -           -               -                   -                       -
            2   Let's TV Play series "Bo-kenger" /EPOCH/Japan                                                   -           -               -           -               -                   -                       -
            3   Challenge Ai-chan! Exciting Ping-pong /TAKARATOMY/Japan                                         -           -               -           -               -                   -                       -
            4   Sasuke & Sportsman Tournament /BANDAI/Japan                                                     -           -               -           -               -                   -                       -
            5   Hyper resque, I am a resque team /BANDAI/Japan                                                  -           -               -           -               -                   -                       -
            6   Let's TV Play series "Ultraman" /BANDAI/Japan                                                   -           -               -           -               -                   -                       -
            7   Let's TV Play Classic series "Namco Nostalgia 1" /BANDAI/Japan                                  CGSJ        x8              48          1M              24LC04                                      dumped non destructively
            8   Let's TV Play Classic series "Namco Nostalgia 2" /BANDAI/Japan                                  CGSJ        x8              48          1M              24LC04              SSD 98 PL7351-181       dumped
            9   Let's TV Play Classic series "Taito Nostalgia 1" /BANDAI/Japan                                  CGSJ        x8              48          2M              24LC04                                      flash, dumped non destructively
            10  Let's TV Play Classic series "Taito Nostalgia 2" /BANDAI/Japan                                  CGSJ        x8              48          2M              24LC04                                      flash, dumped non destructively
            11  Let's play and study! Doraemon Hiragana book /BANDAI/Japan                                      -           -               -           -               -                   -                       -
            12  Scan card! Exciting Stage Soccer. /EPOCH/Japan                                                  -           -               -           -               -                   -                       -
            13  Hello Kitty Super TV computer /EPOCH/Japan                                                      -           -               -           -               -                   -                       -
            14  Doraemon Super TV computer /EPOCH/Japan                                                         -           -               -           -               -                   -                       -
    2005    1   Let's TV Play series "Dragon Ball Z" /BANDAI/Japan                                              -           -               -           -               -                   -                       -
            2   Let's TV Play series "Purikyua" /BANDAI/Japan                                                   -           -               -           -               -                   -                       -
            3   Idaten Jump /TOMY/Japan                                                                         -           -               -           -               -                   -                       -
            4   Tokyo Friend Park 2 Special /EPOCH/Japan                                                        -           -               -           -               -                   -                       -
            5   Masked Rider HIBIKI /BANDAI/Japan                                                               -           -               -           -               -                   -                       -
            6   Magic Ranger Battle /BANDAI/Japan                                                               -           -               -           -               -                   -                       -
            7   Accessory cartridge for Super TV computer "ECC Junior"/EPOCH/Japan                              -           -               -           -               -                   -                       -
            8   Wild Adventure Mini Golf Game /Hasbro/USA                                                       MGFA        x8              48          4M              24C04               SSD 98 PL7351-181       dumped
            9   MX DIRT REBEL Game /Hasbro/USA                                                                  MTXA        x8              48          8M              24C04               SSD 2000 NEC 85605-621  dumped
            10  Dokodemo Doraemon Japan Travel Game DX /EPOCH/Japan                                             -           -               -           -               -                   -                       -
            11  Tomas Plarail /TOMY/Japan                                                                       -           -               -           -               -                   -                       -
            12  Thomas TV Personal Computer /EPOCH/Japan                                                        -           -               -           -               -                   -                       -
            13  STAR WARS Light Saber Battle /TOMY/Japan                                                        -           -               -           -               -                   -                       -
            14  Jala Jaland /atlus/Japan                                                                        -           -               -           -               -                   -                       -
            15  Star Wars Lightsaber Battle Game /Hasbro/USA                                                    SWSA        x8              48          8M              24C02               SSD 2000 NEC 85605-621  dumped
            16  Gururin World /EPOCH/Japan                                                                      -           -               -           -               -                   -                       -
            17  Toinohgi Onmyo-daisenki /BANDAI/Japan                                                           -           -               -           -               -                   -                       -
    2004    1   Accessory cartridge for Super TV computer "Double mouse party"/EPOCH/Japan                      -           -               -           -               -                   -                       -
            2   Printer for TV computer /EPOCH/Japan                                                            -           -               -           -               -                   -                       -
            3   Virtual punching battle of "One Piece" /BANDAI/Japan                                            -           -               -           -               -                   -                       -
            4   Accessory cartridge for Super TV computer "Doraemon"/EPOCH/Japan                                -           -               -           -               -                   -                       -
            5   Accessory cartridge for Super TV computer "Hamutaro"/EPOCH/Japan                                -           -               -           -               -                   -                       -
            6   Super TV computer /EPOCH/Japan                                                                  -           -               -           -               -                   -                       -
            7   Super Dash ball /EPOCH/Japan                                                                    -           -               -           -               -                   -                       -
            8   Exciting sports Tennis X Fitness /EPOCH/Japan                                                   -           -               -           -               -                   -                       -
            9   Accessory memory mascot for TV mail Pc mail cot 2 characters (Putchi, Petchi) /EPOCH/Japan      -           -               -           -               -                   -                       -
            10  Accessory memory mascot for TV mail Pc mail cot 2 characters (Charuru, Kurau) /EPOCH/Japan      -           -               -           -               -                   -                       -
            11  The Lord of the Rings Warrior of Middle Earth /Hasbro/USA                                       LORA        x8              48          8M              24C02               SSD 2000 NEC 85605-621  dumped
            12  Beyblade Arcade Challenge 5-in-1 /Hasbro/USA                                                    -           -               -           -               -                   -                       -
            13  All star Festival Quize /EPOCH/Japan                                                            -           -               -           -               -                   -                       -
            14  e-kara mix /TAKARA/Japan                                                                        -           -               -           -               -                   -                       -
            15  Jumping Popira /TAKARA/Japan                                                                    -           -               -           -               -                   -                       -
            16  Tour around Japan. I'm a Prarail motorman /TOMY/Japan                                           -           -               -           -               -                   -                       -
            17  TV mail PC "Mercot /EPOCH/Japan                                                                 -           -               -           -               -                   -                       -
            18  Play TV Monster Truck /RADICA/USA                                                               74026       x8              48          4M              none                SSD 98 PL7351-181       dumped
            19  Play TV Madden Football /RADICA/USA                                                             74021       x8              48          4M              none                SSD 98 PL7351-181       dumped
            20  Play TV SSX Snowboarder (and Snowboarder white?) /RADICA/USA                                    74023                       none                                                                    have
            21  Disney Princess "Kira-Kira magical lesson" /TOMY/Japan                                          -           -               -           -               -                   -                       -
            22  Mermaid Melody "pichi-pichi Pitch" e-pitch microcomputer pure starter set /TAKARA/Japan         -           -               -           -               -                   -                       -
            23  Hello Kitty TV computer /EPOCH/Japan                                                            -           -               -           -               -                   -                       -
            24  Gan-Gan Revoultion /TAKARA/Japan                                                                -           -               -           -               -                   -                       -
    2003    1   Tokyo Friend Park II /EPOCH/Japan                                                               -           -               -           -               -                   -                       -
            2   TV mah-jongg /EPOCH/Japan                                                                       -           -               -           -               -                   -                       -
            3   e-kara Web /TAKARA/Japan                                                                        -           -               -           -               -                   -                       -
            4   Doraemon TV computer /EPOCH/Japan                                                               -           -               -           -               -                   -                       -
            5   Exciting stadium DX, Hansin Tigers version /EPOCH/Japan                                         -           -               -           -               -                   -                       -
            6   Dragon Quest /SQUARE ENIX/Japan                                                                 -           -               -           8M              -                   SSD 2000?               dumped
            7   Croquette! Win a medal! /EPOCH/Japan                                                            -           -               -           -               -                   -                       -
            8   Taiko Popira /TAKARA/Japan                                                                      -           -               -           -               -                   -                       -
            9   Together Minimoni, Dancing' Stage! plus /EPOCH/Japan                                            -           -               -           -               -                   -                       -
            10  Evio /TOMY/Japan                                                                                -           -               -           -               -                   -                       -
            11  Together Minimoni,Jumping Party! /EPOCH/Japan                                                   -           -               -           -               -                   -                       -
            12  Hamutaro TV computer /EPOCH/Japan                                                               -           -               -           -               -                   -                       -
            13  Jara-Ja Land /TAKARA/Japan                                                                      -           -               -           -               -                   -                       -
            14  Tomika, Draiving by Car navigation system /TOMY/Japan                                           -           -               -           -               -                   -                       -
            15  PLAY TV Rescue Heroes /RADICA/USA                                                               73036       x8              48          2M              none                SSD 98 PL7351-181       dumped
            16  PLAY TV Huntin' 2 /RADICA/USA                                                                   73030       x8              none                        none                SSD 98 PL7351-181       have
            17  Let's play Ping-pong. Exciting pingpong2 /EPOCH/Japan                                           -           -               -           -               -                   -                       -
            18  Cartridge for Slot machine TV "King of wild animal" /TAKARA/Japan                               -           -               -           -               -                   -                       -
            19  ChyoroQ "Burning up Racer /TAKARA/Japan                                                         -           -               -           -               -                   -                       -
            20  Super shot! Exciting golf /EPOCH/Japan                                                          -           -               -           -               -                   -                       -
            21  PichiPichi Pitchi /TAKARA/Japan                                                                 -           -               -           -               -                   -                       -
            22  Dual Station /TAKARA/Japan                                                                      -           -               -           -               -                   -                       -
            23  Gei-Geki GoGo! Shooting /TAKARA/Japan                                                           -           -               -           -               -                   -                       -
            24  Let's fish a big one. Exciting fishing! /EPOCH/Japan                                            -           -               -           -               -                   -                       -
            25  Champion Pinball /TOMY/Japan                                                                    -           -               -           -               -                   -                       -
            26  Excite Fishing DX                                                                               EF2J        x8              48          4M              24C08               SSD 98 PL7351-181       dumped
    2002    1   Accessory cartridge for Slot machine "Gin-gin maru TV" /TAKARA/Japan                            -           -               -           -               -                   -                       -
            2   Wildest computer robot "Daigander" (Korean version) /TAKARA/Korea                               -           -               -           -               -                   -                       -
            3   Hamutaro's circus /EPOCH/Japan                                                                  -           -               -           -               -                   -                       -
            4   Doraemon ,computer megaphone /EPOCH/Japan                                                       -           -               -           -               -                   -                       -
            5   Strike! Exciting bowling /EPOCH/Japan                                                           -           -               -           -               -                   -                       -
            6   e-kara /Hasbro/Spain                                                                            -           -               -           -               -                   -                       -
            7   Starter set for e-kara H.S," Morning sisters" /TAKARA/Japan                                     -           -               -           -               -                   -                       -
            8   e-kara H.S.(headphones set) /TAKARA/Japan                                                       -           -               -           -               -                   -                       -
            9   Accessory cartridge for Slot machine TV," Aladdin TV" /TAKARA/Japan                             -           -               -           -               -                   -                       -
            10  Accessory cartridge for Slot machine TV "Businessman Kintaro/TAKARA/Japan                       -           -               -           -               -                   -                       -
            11  Poko-poko Hammers /TAKARA/Japan                                                                 -           -               -           -               -                   -                       -
            12  e-kara N Angel blue special set /TAKARA/Japan                                                   -           -               -           -               -                   -                       -
            13  Together Minimoni,Dancing Stage! /EPOCH/Japan                                                   -           -               -           -               -                   -                       -
            14  King of shooting /TOMY/Japan                                                                    -           -               -           -               -                   -                       -
            15  Knock them out! Exciting boxing /EPOCH/Japan                                                    -           -               -           -               -                   -                       -
            16  Popira2 /TAKARA/Japan                                                                           -           -               -           -               -                   -                       -
            17  Zuba-Zuba Blade /TAKARA/Japan                                                                   -           -               -           -               -                   -                       -
            18  Starter set for e-kara N "Morning sisters" /TAKARA/Japan                                        -           -               -           -               -                   -                       -
            19  e-kara /Hasbro/England                                                                          -           -               -           -               -                   -                       -
            20  e-kara /Takara USA/USA                                                                          -           -               -           -               -                   -                       -
            21  e-kara PLAY TV Soccer /RADICA/USA                                                               76088500    x8              none                        none                SSD 98 PA7351-107       (aka Radica PlayTV Soccer? if so, have)
            22  PLAY TV Jr. Construction /RADICA/USA                                                            -           -               -           -               -                   -                       -
            23  PLAY TV Boxing /RADICA/Japan                                                                    72039       x8              48          2M              none                SSD 98 PA7351-107       dumped
            24  PLAY TV Baseball 2 /RADICA/USA                                                                  72042       x8              48          2M              none                SSD 98 PL7351-181       dumped
            25  Barbie Dance Party /RADICA/USA,EU                                                               -           -               -           -               -                   -                       -
            26  Compete! Exciting stadium DX /EPOCH/Japan                                                       -           -               -           -               -                   -                       -
            27  e-kara N /EPOCH/Japan                                                                           -           -               -           -               -                   -                       -
            28  Who's the ace? Excite Tennis /EPOCH/Japan                                                       -           -               -           -               -                   -                       -
            29  Wildest computer robot, "Daigander" /TAKARA/Japan                                               -           -               -           -               -                   -                       -
            30  Cartridge for Slot machine TV "King of wild animal Jr." /TAKARA/Japan                           -           -               -           -               -                   -                       -
            31  Gachinko Contest! Slot machine TV /DCT/Japan                                                    -           -               -           -               -                   -                       -
            32  Beyblade Ultimate shooter /TAKARA/Japan                                                         -           -               -           -               -                   -                       -
    2001    1   Ping-pong(Chinese version) /Tenpon/China                                                        -           -               -           -               -                   -                       -
            2   TV hockey /TOMY/Japan                                                                           -           -               -           -               -                   -                       -
            3   e-kara Morning sisters /TAKARA/Japan                                                            -           -               -           -               -                   -                       -
            4   e-kara Duet microphone plus /TAKARA/Japan                                                       -           -               -           -               -                   -                       -
            5   e-kara plus /TAKARA/Japan                                                                       -           -               -           -               -                   -                       -
            6   Hamutaro, Dancing', Running /EPOCH/Japan                                                        -           -               -           -               -                   -                       -
            7   Gin-gin Snowboarders /TAKARA/Japan                                                              -           -               -           -               -                   -                       -
            8   Shoot! Exciting striker /EPOCH/Japan                                                            -           -               -           -               -                   -                       -
            9   e-kara US version /TAKARA USA, Hasbro/USA,EU                                                    71076       x8              none        1M                                  SSD 98 PA7351-107       this one or #20 above?  dumped
            10  Popira Korea version /SONOKONG/Korea                                                            -           -               -           -               -                   -                       -
            11  I singer: e-kara Korean version /SONOKONG/Korea                                                 -           -               -           -               -                   -                       -
            12  Ms.Comett, Lovely baton /TAKARA/Japan<                                                          -           -               -           -               -                   -                       -
            13  Dance Dance revolution family mat /KONAMI,KONAMI Sports/Japan                                   -           -               -           -               -                   -                       -
            14  PLAY TV Card Night /RADICA/USA                                                                  71063       x8              40          1M              none                SSD 98 PA7351-107       dumped
            15  PLAY TV Bass Fishin' /RADICA/USA                                                                71008       x8              40          1M              none                SSD 98 PA7351-107       dumped
            16  PLAY TV Snowboarder (blue) /RADICA/USA                                                          71023       x8              40          1M              none                SSD 98 PL7351-181       dumped
            17  Bistro Kids /SEGA Toys/Japan                                                                    -           -               -           -               -                   -                       -
            18  Let's construct the town! /TAKARA/Japan                                                         -           -               -           -               -                   -                       -
            19  Let's fish black bass! Exciting Fishing /EPOCH/Japan                                            -           -               -           -               -                   -                       -
            20  Baseball Korean version /SONOKONG/Korea                                                         -           -               -           -               -                   -                       -
            21  Ping-pong Korean version /SONOKONG/Korea                                                        -           -               -           -               -                   -                       -
            22  e-kara Hello Kitty /TAKARA/Japan                                                                -           -               -           -               -                   -                       -
            23  Special box "Morning sisters" /TAKARA/Japan                                                     -           -               -           -               -                   -                       -
            24  Gan-Gan Adventure /TAKARA/Japan                                                                 -           -               -           -               -                   -                       -
            25  e-kara wireless unit /TAKARA/Japan                                                              -           -               -           -               -                   -                       -
            26  Webdiver Gradion /TAKARA/Japan                                                                  -           -               -           -               -                   -                       -
            27  black bass tsurouze! Excite Fishing/EPOCH/Japan                                                 -           -               -           -               -                   -                       -
            28  Hamu-chan's Daishuugou dance surunoda! hasirunoda!/EPOCH/Japan                                  -           -               -           -               -                   -                       -
    2000    1   Popira /TAKARA/Japan                                                                            -           -               -           -               -                   -                       -
            2   e-kara Duet microphone /TAKARA/Japan                                                            -           -               -           -               -                   -                       -
            3   e-kara /TAKARA/Japan                                                                            -           -               -           -               -                   -                       -
            4   Let's play ping-pong. Exciting ping-pong /EPOCH/Japan                                           -           -               -           -               -                   -                       -
            5   PLAY TV Huntin' Buckmasters /RADICA/USA                                                         8074        x8              none                        none                SSD 98 PA7351-107       have
            6   PLAY TV Ping Pong /RADICA/USA,HK,EU                                                             8028        x8              48          1M              none                SSD 97 PA7270-107       dumped
            7   PLAY TV OPUS /RADICA/USA,EU                                                                     -           -               -           -               -                   -                       -
            8   PLAY TV Baseball 2 /EPOCH/Japan, HK                                                             -           -               -           -               -                   -                       -
            9   Let's hit a homerun! Exciting baseball /RADICA/USA,EU                                           8017        x8              none                        none                SSD 98 PA7351-107       (aka Radica PlayTV Baseball, if so, have)
    1999    1   ABC Jungle Fun Hippo /Vteck/HK, USA, France                                                     -           -               -           -               -                   -                       -
    Unknown 1   PLAY TV Football /RADICA/USA                                                                    74021       x8              48          4M              none                SSD 98 PL7351-181       dumped
                XaviXTennis                                                                                     SGM6446     x16             48          8M              24C08               SSD 2002 NEC 85054-611  dumped
                XaviXBowling                                                                                    SGM644C     x16             48                                                                      not dumped


    TODO: put into above table (XaviXPORT cartridges)

    XaviX Tennis and XaviX Baseball are the simplest: just the CPU, x16 ROM and 24C08 SEEPROM.  Bowling and Boxing also have 4 IR LEDs and a 32x32 sensor.
    XaviX Fishing has ROM, 24C08 and a 24-pin daughterboard with a Nordic nRF24E1 2.4GHz 8051-based SoC.
    All the rest of the carts include an S35390 I2C clock chip with crystal and battery backup, and have two x16 ROM chips, using a 5-pin single-gate inverter to create complementary /OE signals.
    J-MAT, Fitness Exercise, Fitness Challenge and Bike Concept have a 24CS64 SEEPROM.  Bike Concept also has two 74HC14s.
    Fitness Dance has an Atmel H93864C (maybe SEEPROM?) a Microchip DSPIC 33FJ12GP202 and two JRC 2740 dual op amps.
    Music and Circuit has a 24CS64, two UTC324 quad op amps, a 74HC14, a 74HCT04, and an 8-pin SOIC labeled 61545, which is likely an M61545 dual electronic volume control.

***************************************************************************/

#include "emu.h"
#include "includes/xavix.h"

// not confirmed for all games
#define MAIN_CLOCK XTAL(21'477'272)

READ8_MEMBER(xavix_state::main_r)
{
	if (offset & 0x8000)
	{
		return  m_rgn[(offset) & (m_rgnlen - 1)];
	}
	else
	{
		return m_lowbus->read8(space, offset&0x7fff);
	}
}

WRITE8_MEMBER(xavix_state::main_w)
{
	if (offset & 0x8000)
	{
		logerror("write to ROM area?\n");
	}
	else
	{
		m_lowbus->write8(space, offset & 0x7fff, data);
	}
}

/* rad_madf has callf #$8f3f21 in various places, and expects to jump to code in ROM, it is unclear how things map in this case, as presumably
   the CPU 0 page memory and stack are still at 0 but ROM must be in the 3xxx range (game hasn't got far enough to call this yet to help either)

   the maximum romsize appears to be 0x800000 so presumably the high bit being set has some additional meaning

   for now treat it as a swapped arrangement vs. the reads from the lower range, except where page 0 ram would map, it's also possible that
   vram etc. is completely unavailable if executing from these addresses, there isn't much evidence at the moment

   note, many DMA operations and tile bank redirects etc. have the high bit set too, so that could be significant if it defines how it accesses
   memory in those cases too

*/
READ8_MEMBER(xavix_state::main2_r)
{
	if (offset & 0x8000)
	{
		return m_lowbus->read8(space, offset&0x7fff);
	}
	else
	{
		if (offset>0x200)
			return  m_rgn[(offset) & (m_rgnlen - 1)];
		else
			return m_lowbus->read8(space, offset&0x7fff);
	}
}

WRITE8_MEMBER(xavix_state::main2_w)
{
	if (offset & 0x8000)
	{
		m_lowbus->write8(space, offset & 0x7fff, data);
	}
	else
	{
		if (offset>0x200)
			logerror("write to ROM area?\n");
		else
			m_lowbus->write8(space, offset & 0x7fff, data);
	}
}

// DATA reads from 0x8000-0xffff are banked by byte 0xff of 'ram' (this is handled in the CPU core)

void xavix_state::xavix_map(address_map &map)
{
	map(0x000000, 0x7fffff).rw(FUNC(xavix_state::main_r), FUNC(xavix_state::main_w));
	map(0x800000, 0xffffff).rw(FUNC(xavix_state::main2_r), FUNC(xavix_state::main2_w));
}

void xavix_state::xavix_lowbus_map(address_map &map)
{
	map(0x0000, 0x01ff).ram();
	map(0x0200, 0x3fff).ram().share("mainram");

	// this might not be a real area, the tilemap base register gets set to 0x40 in monster truck service mode, and expects a fixed layout.
	// As that would point at this address maybe said layout is being read from here, or maybe it's just a magic tilemap register value that doesn't read address space at all.
	map(0x4000, 0x41ff).r(FUNC(xavix_state::xavix_4000_r));

	// 6xxx ranges are the video hardware
	// appears to be 256 sprites
	map(0x6000, 0x60ff).ram().share("spr_attr0");
	map(0x6100, 0x61ff).ram().share("spr_attr1");
	map(0x6200, 0x62ff).ram().share("spr_ypos"); // cleared to 0x80 by both games, maybe enable registers?
	map(0x6300, 0x63ff).ram().share("spr_xpos");
	map(0x6400, 0x64ff).ram(); // 6400 range gets populated in some cases, but it seems to be more like work ram, data doesn't matter and must be ignored?
	map(0x6500, 0x65ff).ram().share("spr_addr_lo");
	map(0x6600, 0x66ff).ram().share("spr_addr_md");
	map(0x6700, 0x67ff).ram().share("spr_addr_hi");
	map(0x6800, 0x68ff).ram().share("palram1"); // written with 6900
	map(0x6900, 0x69ff).ram().share("palram2"); // startup (taitons1)
	map(0x6a00, 0x6a1f).ram().share("spr_attra"); // test mode, pass flag 0x20

	map(0x6fc0, 0x6fc0).w(FUNC(xavix_state::xavix_6fc0_w)); // startup (maybe this is a mirror of tmap1_regs_w)

	map(0x6fc8, 0x6fcf).w(FUNC(xavix_state::tmap1_regs_w)); // video registers

	map(0x6fd0, 0x6fd7).rw(FUNC(xavix_state::tmap2_regs_r), FUNC(xavix_state::tmap2_regs_w));
	map(0x6fd8, 0x6fd8).w(FUNC(xavix_state::xavix_6fd8_w)); // startup (mirror of tmap2_regs_w?)

	map(0x6fe0, 0x6fe0).rw(FUNC(xavix_state::vid_dma_trigger_r), FUNC(xavix_state::vid_dma_trigger_w)); // after writing to 6fe1/6fe2 and 6fe5/6fe6 rad_mtrk writes 0x43/0x44 here then polls on 0x40   (see function call at c273) write values are hardcoded, similar code at 18401
	map(0x6fe1, 0x6fe2).w(FUNC(xavix_state::vid_dma_params_1_w));
	map(0x6fe5, 0x6fe6).w(FUNC(xavix_state::vid_dma_params_2_w));

	// function in rad_mtrk at 0184b7 uses this
	map(0x6fe8, 0x6fe8).rw(FUNC(xavix_state::xavix_6fe8_r), FUNC(xavix_state::xavix_6fe8_w)); // r/w tested
	map(0x6fe9, 0x6fe9).rw(FUNC(xavix_state::xavix_6fe9_r), FUNC(xavix_state::xavix_6fe9_w)); // r/w tested
	map(0x6fea, 0x6fea).w(FUNC(xavix_state::xavix_6fea_w));

	map(0x6ff0, 0x6ff0).rw(FUNC(xavix_state::xavix_6ff0_r), FUNC(xavix_state::xavix_6ff0_w)); // r/w tested
	map(0x6ff1, 0x6ff1).w(FUNC(xavix_state::xavix_6ff1_w)); // startup - cleared in interrupt 0
	map(0x6ff2, 0x6ff2).w(FUNC(xavix_state::xavix_6ff2_w)); // set to 07 after clearing above things in interrupt 0

	map(0x6ff8, 0x6ff8).rw(FUNC(xavix_state::xavix_6ff8_r), FUNC(xavix_state::xavix_6ff8_w)); // always seems to be a read/store or read/modify/store
	map(0x6ff9, 0x6ff9).r(FUNC(xavix_state::pal_ntsc_r));
	map(0x6ffa, 0x6ffa).w(FUNC(xavix_state::xavix_6ffa_w));
	map(0x6ffb, 0x6ffb).w(FUNC(xavix_state::xavix_6ffb_w)); // increases / decreases when you jump in snowboard, maybe raster irq pos or part of a clipping window?

	// 7xxx ranges system controller?
	map(0x75f0, 0x75f0).rw(FUNC(xavix_state::xavix_75f0_r), FUNC(xavix_state::xavix_75f0_w)); // r/w tested read/written 8 times in a row
	map(0x75f1, 0x75f1).rw(FUNC(xavix_state::xavix_75f1_r), FUNC(xavix_state::xavix_75f1_w)); // r/w tested read/written 8 times in a row
	map(0x75f3, 0x75f3).ram();
	map(0x75f4, 0x75f4).r(FUNC(xavix_state::xavix_75f4_r)); // related to 75f0 (read after writing there - rad_mtrk)
	map(0x75f5, 0x75f5).r(FUNC(xavix_state::xavix_75f5_r)); // related to 75f1 (read after writing there - rad_mtrk)

	// taitons1 after 75f7/75f8
	map(0x75f6, 0x75f6).rw(FUNC(xavix_state::xavix_75f6_r), FUNC(xavix_state::xavix_75f6_w)); // r/w tested
	// taitons1 written as a pair
	map(0x75f7, 0x75f7).w(FUNC(xavix_state::xavix_75f7_w));
	map(0x75f8, 0x75f8).rw(FUNC(xavix_state::xavix_75f8_r), FUNC(xavix_state::xavix_75f8_w)); // r/w tested
	// taitons1 written after 75f6, then read
	map(0x75f9, 0x75f9).rw(FUNC(xavix_state::xavix_75f9_r), FUNC(xavix_state::xavix_75f9_w));
	// at another time
	map(0x75fa, 0x75fa).rw(FUNC(xavix_state::xavix_75fa_r), FUNC(xavix_state::xavix_75fa_w)); // r/w tested
	map(0x75fb, 0x75fb).rw(FUNC(xavix_state::xavix_75fb_r), FUNC(xavix_state::xavix_75fb_w)); // r/w tested
	map(0x75fc, 0x75fc).rw(FUNC(xavix_state::xavix_75fc_r), FUNC(xavix_state::xavix_75fc_w)); // r/w tested
	map(0x75fd, 0x75fd).rw(FUNC(xavix_state::xavix_75fd_r), FUNC(xavix_state::xavix_75fd_w)); // r/w tested
	map(0x75fe, 0x75fe).w(FUNC(xavix_state::xavix_75fe_w));
	// taitons1 written other 75xx operations
	map(0x75ff, 0x75ff).w(FUNC(xavix_state::xavix_75ff_w));

	map(0x7810, 0x7810).w(FUNC(xavix_state::xavix_7810_w)); // startup

	map(0x7900, 0x7900).w(FUNC(xavix_state::xavix_7900_w));
	map(0x7901, 0x7901).w(FUNC(xavix_state::xavix_7901_w));
	map(0x7902, 0x7902).w(FUNC(xavix_state::xavix_7902_w));

	// DMA trigger for below (written after the others) waits on status of bit 1 in a loop
	map(0x7980, 0x7980).rw(FUNC(xavix_state::dma_trigger_r), FUNC(xavix_state::dma_trigger_w));
	// DMA source
	map(0x7981, 0x7981).w(FUNC(xavix_state::rom_dmasrc_lo_w));
	map(0x7982, 0x7982).w(FUNC(xavix_state::rom_dmasrc_md_w));
	map(0x7983, 0x7983).w(FUNC(xavix_state::rom_dmasrc_hi_w));
	// DMA dest
	map(0x7984, 0x7984).w(FUNC(xavix_state::rom_dmadst_lo_w));
	map(0x7985, 0x7985).w(FUNC(xavix_state::rom_dmadst_hi_w));
	// DMA length
	map(0x7986, 0x7986).w(FUNC(xavix_state::rom_dmalen_lo_w));
	map(0x7987, 0x7987).w(FUNC(xavix_state::rom_dmalen_hi_w));

	// GPIO stuff
	map(0x7a00, 0x7a00).rw(FUNC(xavix_state::xavix_io_0_r),FUNC(xavix_state::xavix_7a00_w));
	map(0x7a01, 0x7a01).rw(FUNC(xavix_state::xavix_io_1_r),FUNC(xavix_state::xavix_7a01_w)); // startup (taitons1)
	map(0x7a02, 0x7a02).rw(FUNC(xavix_state::xavix_7a02_r),FUNC(xavix_state::xavix_7a02_w)); // startup, gets set to 20, 7a00 is then also written with 20
	map(0x7a03, 0x7a03).rw(FUNC(xavix_state::xavix_7a03_r),FUNC(xavix_state::xavix_7a03_w)); // startup (gets set to 84 which is the same as the bits checked on 7a01, possible port direction register?)

	map(0x7a80, 0x7a80).w(FUNC(xavix_state::xavix_7a80_w)); // still IO? ADC related?

	map(0x7b00, 0x7b00).w(FUNC(xavix_state::xavix_7b00_w)); // rad_snow (not often)

	map(0x7b80, 0x7b80).rw(FUNC(xavix_state::xavix_7b80_r), FUNC(xavix_state::xavix_7b80_w)); // rad_snow (not often)
	map(0x7b81, 0x7b81).w(FUNC(xavix_state::xavix_7b81_w)); // written (often, m_trck, analog related?)

	map(0x7c00, 0x7c00).w(FUNC(xavix_state::xavix_7c00_w));
	map(0x7c01, 0x7c01).rw(FUNC(xavix_state::xavix_7c01_r), FUNC(xavix_state::xavix_7c01_w)); // r/w tested
	map(0x7c02, 0x7c02).w(FUNC(xavix_state::xavix_7c02_w));

	// this is a multiplication chip
	map(0x7ff2, 0x7ff4).w(FUNC(xavix_state::mult_param_w));
	map(0x7ff5, 0x7ff6).rw(FUNC(xavix_state::mult_r), FUNC(xavix_state::mult_w));

	// maybe irq enable, written after below
	map(0x7ff9, 0x7ff9).w(FUNC(xavix_state::irq_enable_w)); // interrupt related, but probalby not a simple 'enable' otherwise interrupts happen before we're ready for them.
	// an IRQ vector (nmi?)
	map(0x7ffa, 0x7ffa).w(FUNC(xavix_state::irq_vector0_lo_w));
	map(0x7ffb, 0x7ffb).w(FUNC(xavix_state::irq_vector0_hi_w));

	map(0x7ffc, 0x7ffc).rw(FUNC(xavix_state::irq_source_r), FUNC(xavix_state::irq_source_w));

	// an IRQ vector (irq?)
	map(0x7ffe, 0x7ffe).w(FUNC(xavix_state::irq_vector1_lo_w));
	map(0x7fff, 0x7fff).w(FUNC(xavix_state::irq_vector1_hi_w));
}

static INPUT_PORTS_START( xavix )
	PORT_START("IN0")
	PORT_DIPNAME( 0x01, 0x00, "IN0" )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x01, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x02, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x04, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x08, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x10, DEF_STR( On ) )
	PORT_DIPNAME( 0x20, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x20, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x40, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x80, DEF_STR( On ) )

	PORT_START("IN1")
	PORT_DIPNAME( 0x01, 0x00, "IN1" )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x01, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x02, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x04, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x08, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x10, DEF_STR( On ) )
	PORT_DIPNAME( 0x20, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x20, DEF_STR( On ) )
	PORT_DIPNAME( 0x40, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x40, DEF_STR( On ) )
	PORT_DIPNAME( 0x80, 0x00, DEF_STR( Unknown ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x80, DEF_STR( On ) )

	PORT_START("REGION") // PAL/NTSC flag
	PORT_BIT( 0xff, IP_ACTIVE_HIGH, IPT_CUSTOM )
INPUT_PORTS_END

static INPUT_PORTS_START( xavixp )
	PORT_INCLUDE(xavix)

	PORT_MODIFY("REGION") // PAL/NTSC flag
	PORT_BIT( 0xff, IP_ACTIVE_LOW, IPT_CUSTOM )
INPUT_PORTS_END

/* Test mode lists the following

  LED (on power button?)
  Throttle Low
  Throttle High
  Reverse
  NO2
  Steering Left (4 positions)
  Steering Right (4 positions)
  Horn

*/

static INPUT_PORTS_START( rad_mtrk )
	PORT_INCLUDE(xavix)

	PORT_MODIFY("IN0")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_BUTTON5 ) PORT_NAME("Nitro")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_BUTTON4 ) PORT_NAME("Throttle High")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_BUTTON3 ) PORT_NAME("Throttle Low")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_BUTTON2 ) PORT_NAME("Reverse / Back")

	PORT_MODIFY("IN1")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_BUTTON1 ) PORT_NAME("Horn")
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_SERVICE1 ) // some kind of 'power off' (fades screen to black, jumps to an infinite loop) maybe low battery condition or just the power button?
INPUT_PORTS_END

static INPUT_PORTS_START( rad_mtrkp )
	PORT_INCLUDE(rad_mtrk)

	PORT_MODIFY("REGION") // PAL/NTSC flag
	PORT_BIT( 0xff, IP_ACTIVE_LOW, IPT_CUSTOM )
INPUT_PORTS_END

static INPUT_PORTS_START( rad_crdn )
	PORT_INCLUDE(xavix)

	PORT_MODIFY("IN0")
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON1 ) // can press this to get to a game select screen
INPUT_PORTS_END

static INPUT_PORTS_START( rad_crdnp )
	PORT_INCLUDE(rad_crdn)

	PORT_MODIFY("REGION") // PAL/NTSC flag
	PORT_BIT( 0xff, IP_ACTIVE_LOW, IPT_CUSTOM )
INPUT_PORTS_END

static INPUT_PORTS_START( rad_box )
	PORT_INCLUDE(xavix)

	PORT_MODIFY("IN0")
	// 6 types of punch and some navigation controls?
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_BUTTON1 ) PORT_NAME("Left Jan")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_BUTTON2 ) PORT_NAME("Left Hook")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_BUTTON3 ) PORT_NAME("Left Uppercut")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_BUTTON4 ) PORT_NAME("Left Jab")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_BUTTON5 ) PORT_NAME("Left Hook")
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_BUTTON6 ) PORT_NAME("Left Uppercut")
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT )
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT )

	PORT_MODIFY("IN1")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_BUTTON7 )  PORT_NAME("Block")
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNKNOWN ) // needs to be high to pass warning screen?
INPUT_PORTS_END

static INPUT_PORTS_START( rad_boxp )
	PORT_INCLUDE(rad_box)

	PORT_MODIFY("REGION") // PAL/NTSC flag
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_CUSTOM )
INPUT_PORTS_END


static INPUT_PORTS_START( rad_snow )
	PORT_INCLUDE(xavix)

	PORT_MODIFY("IN0")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_BUTTON1 ) PORT_NAME("Go") // is this a button, or 'up' ?

	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN )
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT )
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT )
INPUT_PORTS_END

static INPUT_PORTS_START( rad_snowp )
	PORT_INCLUDE(rad_snow)

	PORT_MODIFY("REGION") // PAL/NTSC flag
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_CUSTOM )
INPUT_PORTS_END

static INPUT_PORTS_START( namcons2 )
	PORT_INCLUDE(xavix)

	PORT_MODIFY("IN0")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_BUTTON1 )
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_BUTTON2 )
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_BUTTON3 )
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_BUTTON4 )
INPUT_PORTS_END

/* correct, 4bpp gfxs */
static const gfx_layout charlayout =
{
	8,8,
	RGN_FRAC(1,1),
	4,
	{ STEP4(0,1) },
	{ 1*4,0*4,3*4,2*4,5*4,4*4,7*4,6*4 },
	{ STEP8(0,4*8) },
	8*8*4
};

static const gfx_layout char16layout =
{
	16,16,
	RGN_FRAC(1,1),
	4,
	{ STEP4(0,1) },
	{ 1*4,0*4,3*4,2*4,5*4,4*4,7*4,6*4, 9*4,8*4,11*4,10*4,13*4,12*4,15*4,14*4   },
	{ STEP16(0,4*16) },
	16*16*4
};

static const gfx_layout charlayout8bpp =
{
	8,8,
	RGN_FRAC(1,1),
	8,
	{ STEP8(0,1) },
	{ STEP8(0,8) },
	{ STEP8(0,8*8) },
	8*8*8
};

static const gfx_layout char16layout8bpp =
{
	16,16,
	RGN_FRAC(1,1),
	8,
	{ STEP8(0,1) },
	{ STEP16(0,8) },
	{ STEP16(0,16*8) },
	16*16*8
};

static GFXDECODE_START( gfx_xavix )
	GFXDECODE_ENTRY( "bios", 0, charlayout,   0, 16 )
	GFXDECODE_ENTRY( "bios", 0, char16layout, 0, 16 )
	GFXDECODE_ENTRY( "bios", 0, charlayout8bpp, 0, 1 )
	GFXDECODE_ENTRY( "bios", 0, char16layout8bpp, 0, 1 )
GFXDECODE_END


MACHINE_CONFIG_START(xavix_state::xavix)

	/* basic machine hardware */
	MCFG_DEVICE_ADD("maincpu",XAVIX,MAIN_CLOCK)
	MCFG_DEVICE_PROGRAM_MAP(xavix_map)
	MCFG_M6502_DISABLE_CACHE()
	MCFG_DEVICE_VBLANK_INT_DRIVER("screen", xavix_state,  interrupt)
	MCFG_XAVIX_VECTOR_CALLBACK(xavix_state, get_vectors)

	MCFG_TIMER_DRIVER_ADD_SCANLINE("scantimer", xavix_state, scanline_cb, "screen", 0, 1)

	MCFG_DEVICE_ADD("lowbus", ADDRESS_MAP_BANK, 0)
	MCFG_DEVICE_PROGRAM_MAP(xavix_lowbus_map)
	MCFG_ADDRESS_MAP_BANK_ENDIANNESS(ENDIANNESS_LITTLE)
	MCFG_ADDRESS_MAP_BANK_DATA_WIDTH(8)
	MCFG_ADDRESS_MAP_BANK_ADDR_WIDTH(24)
	MCFG_ADDRESS_MAP_BANK_STRIDE(0x8000)


	/* video hardware */
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500))
	MCFG_SCREEN_UPDATE_DRIVER(xavix_state, screen_update)
	MCFG_SCREEN_SIZE(32*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(1*8, 31*8-1, 0*8, 28*8-1)
	MCFG_SCREEN_PALETTE("palette")

	MCFG_DEVICE_ADD("gfxdecode", GFXDECODE, "palette", gfx_xavix)

	MCFG_PALETTE_ADD("palette", 256)

	/* sound hardware */
	SPEAKER(config, "mono").front_center();
	// sound is PCM
MACHINE_CONFIG_END


MACHINE_CONFIG_START(xavix_state::xavixp)
	xavix(config);

	MCFG_SCREEN_MODIFY("screen")
	MCFG_SCREEN_REFRESH_RATE(50)
MACHINE_CONFIG_END

MACHINE_CONFIG_START(xavix_state::xavix2000)
	xavix(config);

	MCFG_DEVICE_REMOVE("maincpu")

	MCFG_DEVICE_ADD("maincpu",XAVIX2000,MAIN_CLOCK)
	MCFG_DEVICE_PROGRAM_MAP(xavix_map)
	MCFG_M6502_DISABLE_CACHE()
	MCFG_DEVICE_VBLANK_INT_DRIVER("screen", xavix_state,  interrupt)
	MCFG_XAVIX_VECTOR_CALLBACK(xavix_state, get_vectors)

MACHINE_CONFIG_END


void xavix_state::init_xavix()
{
	m_rgnlen = memregion("bios")->bytes();
	m_rgn = memregion("bios")->base();
}

void xavix_state::init_taitons1()
{
	init_xavix();
	m_alt_addressing = 1;
}

void xavix_state::init_rad_box()
{
	init_xavix();
	m_alt_addressing = 2;
}

/***************************************************************************

  Game driver(s)

***************************************************************************/

ROM_START( taitons1 )
	ROM_REGION( 0x200000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "taitonostalgia1.u3", 0x000000, 0x200000, CRC(25bd8c67) SHA1(a109cd2da6aa4596e3ca3abd1afce2d0001a473f) )
ROM_END

ROM_START( taitons2 )
	ROM_REGION( 0x200000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "taitonostalgia2.bin", 0x000000, 0x200000, CRC(d7dbd93d) SHA1(ad96f80d317e7fd64682a1fe406c5ee9dd5eabf9) )
ROM_END

ROM_START( namcons1 )
	ROM_REGION( 0x100000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "namconostalgia1.bin", 0x000000, 0x100000, CRC(9bcccccd) SHA1(cf8fe6de76fbd23974f999299db6f558f79c8f22) )
ROM_END

ROM_START( namcons2 )
	ROM_REGION( 0x100000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "nostalgia.bin", 0x000000, 0x100000, CRC(03f7f755) SHA1(bdf1b10ab0104ed580951b0c428c4e93e7373afe) )
ROM_END

ROM_START( rad_box )
	ROM_REGION(0x200000, "bios", ROMREGION_ERASE00)
	ROM_LOAD("boxing.bin", 0x000000, 0x200000, CRC(5cd40714) SHA1(165260228c029a9502ca0598c84c24fd9bdeaebe) )
ROM_END

ROM_START( rad_boxp )
	ROM_REGION(0x200000, "bios", ROMREGION_ERASE00)
	ROM_LOAD("boxing.bin", 0x000000, 0x200000, CRC(5cd40714) SHA1(165260228c029a9502ca0598c84c24fd9bdeaebe) )
ROM_END

ROM_START( rad_bass )
	ROM_REGION(0x100000, "bios", ROMREGION_ERASE00)
	ROM_LOAD("bassfishin.bin", 0x000000, 0x100000, CRC(b54eb1c5) SHA1(084faa9349369f2b8846950765f9c8f758db3e9e) )
ROM_END

ROM_START( rad_bassp )
	ROM_REGION(0x100000, "bios", ROMREGION_ERASE00)
	ROM_LOAD("bassfishin.bin", 0x000000, 0x100000, CRC(b54eb1c5) SHA1(084faa9349369f2b8846950765f9c8f758db3e9e) )
ROM_END

ROM_START( rad_snow )
	ROM_REGION(0x100000, "bios", ROMREGION_ERASE00)
	ROM_LOAD("snoblu.bin", 0x000000, 0x100000, CRC(593e40b3) SHA1(03483ac39eddd7746470fb60018e704382b0da59) )
ROM_END

ROM_START( rad_snowp )
	ROM_REGION(0x100000, "bios", ROMREGION_ERASE00)
	ROM_LOAD("snoblu.bin", 0x000000, 0x100000, CRC(593e40b3) SHA1(03483ac39eddd7746470fb60018e704382b0da59) )
ROM_END


ROM_START( rad_ping )
	ROM_REGION( 0x100000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "pingpong.bin", 0x000000, 0x100000, CRC(629f7f47) SHA1(2bb19fd202f1e6c319d2f7d18adbfed8a7669235) )
ROM_END

ROM_START( rad_crdn )
	ROM_REGION( 0x100000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "cardnight.bin", 0x000000, 0x100000, CRC(d19eba08) SHA1(cedb9fe785f2a559f518a1d8ecf80d500ddc63c7) )
ROM_END

ROM_START( rad_crdnp )
	ROM_REGION( 0x100000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "cardnight.bin", 0x000000, 0x100000, CRC(d19eba08) SHA1(cedb9fe785f2a559f518a1d8ecf80d500ddc63c7) )
ROM_END

ROM_START( rad_bb2 )
	ROM_REGION( 0x200000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "baseball2.bin", 0x000000, 0x200000, CRC(bdbf6202) SHA1(18d5cc2d77cbb734629a7a5b6e0f419d21beedbd) )
ROM_END

ROM_START( rad_mtrk )
	ROM_REGION( 0x400000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "monstertruck.bin", 0x000000, 0x400000, CRC(dccda0a7) SHA1(7953cf29643672f8367639555b797c20bb533eab) )
ROM_END

ROM_START( rad_mtrkp ) // rom was dumped from NTSC unit, assuming to be the same
	ROM_REGION( 0x400000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "monstertruck.bin", 0x000000, 0x400000, CRC(dccda0a7) SHA1(7953cf29643672f8367639555b797c20bb533eab) )
ROM_END


ROM_START( rad_madf )
	ROM_REGION(0x400000, "bios", ROMREGION_ERASE00)
	ROM_LOAD("madden.bin", 0x000000, 0x400000, CRC(e972fdcf) SHA1(52001316254880755da959c3441d232fd2c72c7a) )
ROM_END

ROM_START( rad_fb )
	ROM_REGION(0x400000, "bios", ROMREGION_ERASE00)
	ROM_LOAD("rfootball.bin", 0x000000, 0x400000, CRC(025e0cb4) SHA1(60ce363de236d5119d078e346ad5d2ae50dbc7e1) )
ROM_END

ROM_START( epo_efdx )
	ROM_REGION(0x400000, "bios", ROMREGION_ERASE00)
	ROM_LOAD("excitefishing.bin", 0x000000, 0x400000, CRC(9c85b261) SHA1(6a363faed2ec89c5176e46554a98ca1e20132579) )
ROM_END

ROM_START( rad_rh )
	ROM_REGION(0x200000, "bios", ROMREGION_ERASE00)
	ROM_LOAD("rescueheroes.bin", 0x000000, 0x200000, CRC(38c391a7) SHA1(120334d4ce89d98438c2a35bf7e53af5096cc878) )
ROM_END

/*
    There's more code in here than in the 'eka_strt' set, but all it seems to do is display an 'insert cartridge' message,
    however eka_strt also seems a complete program in it's own right, it's unclear if it can see any of the data from this
    ROM.

    (TODO: turn the cartridges into a software list once the mapping is properly understood)
*/

ROM_START( eka_base )
	ROM_REGION( 0x100000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "ekara.bin", 0x000000, 0x100000, CRC(9b27c4a2) SHA1(d75dda7434933135d2f7e353840a9384e9a0d586) )
ROM_END

ROM_START( eka_strt )
	ROM_REGION( 0x080000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "ekarastartcart.bin", 0x000000, 0x080000, CRC(8c12c0c2) SHA1(8cc1b098894af25a4bfccada884125b66f5fe8b2) )
ROM_END

ROM_START( eka_vol1 )
	ROM_REGION( 0x100000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "ekaravol1.bin", 0x000000, 0x100000, CRC(29df4aea) SHA1(b95835aaf8630b61b47e5da0968cd4a1dd3bc517) )
ROM_END

ROM_START( eka_vol2 )
	ROM_REGION( 0x100000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "ekaravol2.bin", 0x000000, 0x100000, CRC(6c66772e) SHA1(e1e719df1e51caaafd9b3af187059334f7abbba3) )
ROM_END

ROM_START( has_wamg )
	ROM_REGION( 0x400000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "minigolf.bin", 0x000000, 0x400000, CRC(35cee2ad) SHA1(c7344e8ba336bc329638485ea571cd731ebf7649) )
ROM_END

/* Standalone TV Games */

CONS( 2006, taitons1,  0,          0,  xavix,  xavix,    xavix_state, init_taitons1, "Bandai / SSD Company LTD / Taito", "Let's! TV Play Classic - Taito Nostalgia 1", MACHINE_IS_SKELETON )

CONS( 2006, taitons2,  0,          0,  xavix,  namcons2, xavix_state, init_xavix,    "Bandai / SSD Company LTD / Taito", "Let's! TV Play Classic - Taito Nostalgia 2", MACHINE_IS_SKELETON )

CONS( 2006, namcons1,  0,          0,  xavix,  namcons2, xavix_state, init_taitons1, "Bandai / SSD Company LTD / Namco", "Let's! TV Play Classic - Namco Nostalgia 1", MACHINE_IS_SKELETON )

CONS( 2006, namcons2,  0,          0,  xavix,  namcons2, xavix_state, init_taitons1, "Bandai / SSD Company LTD / Namco", "Let's! TV Play Classic - Namco Nostalgia 2", MACHINE_IS_SKELETON )

CONS( 2000, rad_ping,  0,          0,  xavix,  xavix,    xavix_state, init_xavix,    "Radica / SSD Company LTD / Simmer Technology", "Play TV Ping Pong", MACHINE_IS_SKELETON ) // "Simmer Technology" is also known as "Hummer Technology Co., Ltd"

CONS( 2003, rad_mtrk,  0,          0,  xavix,  rad_mtrk, xavix_state, init_xavix,    "Radica / SSD Company LTD",                     "Play TV Monster Truck (NTSC)", MACHINE_IS_SKELETON )
CONS( 2003, rad_mtrkp, rad_mtrk,   0,  xavixp, rad_mtrkp,xavix_state, init_xavix,    "Radica / SSD Company LTD",                     "ConnecTV Monster Truck (PAL)", MACHINE_IS_SKELETON )

CONS( 200?, rad_box,   0,          0,  xavix,  rad_box,  xavix_state, init_rad_box,  "Radica / SSD Company LTD",                     "Play TV Boxing (NTSC)", MACHINE_IS_SKELETON)
CONS( 200?, rad_boxp,  rad_box,    0,  xavixp, rad_boxp, xavix_state, init_rad_box,  "Radica / SSD Company LTD",                     "ConnecTV Boxing (PAL)", MACHINE_IS_SKELETON)

CONS( 200?, rad_crdn,  0,          0,  xavix,  rad_crdn, xavix_state, init_rad_box,  "Radica / SSD Company LTD",                     "Play TV Card Night (NTSC)", MACHINE_IS_SKELETON)
CONS( 200?, rad_crdnp, rad_crdn,   0,  xavixp, rad_crdnp,xavix_state, init_rad_box,  "Radica / SSD Company LTD",                     "ConnecTV Card Night (PAL)", MACHINE_IS_SKELETON)

CONS( 2002, rad_bb2,   0,          0,  xavix,  xavix,    xavix_state, init_xavix,    "Radica / SSD Company LTD",                     "Play TV Baseball 2", MACHINE_IS_SKELETON ) // contains string "Radica RBB2 V1.0"

CONS( 2001, rad_bass,  0,          0,  xavix,  xavix,    xavix_state, init_rad_box,  "Radica / SSD Company LTD",                     "Play TV Bass Fishin' (NTSC)", MACHINE_IS_SKELETON)
CONS( 2001, rad_bassp, rad_bass,   0,  xavixp, xavixp,   xavix_state, init_rad_box,  "Radica / SSD Company LTD",                     "ConnecTV Bass Fishin' (PAL)", MACHINE_IS_SKELETON)

// there is another 'Snowboarder' with a white coloured board, it appears to be a newer game closer to 'SSX Snowboarder' but without the SSX license.
CONS( 2001, rad_snow,  0,          0,  xavix,  rad_snow, xavix_state, init_rad_box,  "Radica / SSD Company LTD",                     "Play TV Snowboarder (Blue) (NTSC)", MACHINE_IS_SKELETON)
CONS( 2001, rad_snowp, rad_snow,   0,  xavixp, rad_snowp,xavix_state, init_rad_box,  "Radica / SSD Company LTD",                     "ConnecTV Snowboarder (Blue) (PAL)", MACHINE_IS_SKELETON)

CONS( 2003, rad_madf,  0,          0,  xavix,  xavix,  xavix_state, init_taitons1,  "Radica / SSD Company LTD",                     "EA Sports Madden Football (NTSC)", MACHINE_IS_SKELETON) // no Play TV branding, USA only release?

CONS( 200?, rad_fb,    0,          0,  xavix,  xavix,  xavix_state, init_taitons1,  "Radica / SSD Company LTD",                     "Play TV Football (NTSC)", MACHINE_IS_SKELETON) // USA only release? doesn't change logo for PAL

CONS( 200?, rad_rh,    0,          0,  xavix,  xavix,  xavix_state, init_taitons1,  "Radioa / Fisher-Price / SSD Company LTD",      "Play TV Rescue Heroes", MACHINE_IS_SKELETON)

CONS( 200?, epo_efdx,  0,          0,  xavix,  xavix,  xavix_state, init_taitons1,  "Epoch / SSD Company LTD",                      "Excite Fishing DX (Japan)", MACHINE_IS_SKELETON)

CONS( 200?, has_wamg,  0,          0,  xavix,  xavix,  xavix_state, init_rad_box,   "Hasbro / Milton Bradley / SSD Company LTD",    "TV Wild Adventure Mini Golf", MACHINE_IS_SKELETON)

CONS( 200?, eka_base,  0,          0,  xavix,  xavix,  xavix_state, init_xavix,     "Takara / Hasbro / SSD Company LTD",                     "e-kara (US?)", MACHINE_IS_SKELETON)

CONS( 200?, eka_strt,  0,          0,   xavix,  xavix,  xavix_state, init_xavix,    "Takara / Hasbro / SSD Company LTD",                     "e-kara Starter (US?)", MACHINE_IS_SKELETON)

CONS( 200?, eka_vol1,  0,          0,   xavix,  xavix,  xavix_state, init_xavix,    "Takara / Hasbro / SSD Company LTD",                     "e-kara Volume 1 (US?)", MACHINE_IS_SKELETON) // insert calls it 'HIT MIX Vol 1'

CONS( 200?, eka_vol2,  0,          0,   xavix,  xavix,  xavix_state, init_xavix,    "Takara / Hasbro / SSD Company LTD",                     "e-kara Volume 2 (US?)", MACHINE_IS_SKELETON) // insert calls it 'HIT MIX Vol 2'

/* The 'XaviXPORT' isn't a real console, more of a TV adapter, all the actual hardware (CPU including video hw, sound hw) is in the cartridges and controllers
   and can vary between games, see notes at top of driver.
*/

ROM_START( xavtenni )
	ROM_REGION( 0x800000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "xavixtennis.bin", 0x000000, 0x800000, CRC(23a1d918) SHA1(2241c59e8ea8328013e55952ebf9060ea0a4675b) )
ROM_END

/* Tiger games have extended opcodes too */


ROM_START( ttv_sw )
	ROM_REGION( 0x800000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "jedi.bin", 0x000000, 0x800000, CRC(51cae5fd) SHA1(1ed8d556f31b4182259ca8c766d60c824d8d9744) )
ROM_END

ROM_START( ttv_lotr )
	ROM_REGION( 0x800000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "lotr.bin", 0x000000, 0x800000, CRC(a034ecd5) SHA1(264a9d4327af0a075841ad6129db67d82cf741f1) )
ROM_END

ROM_START( ttv_mx )
	ROM_REGION( 0x800000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "mxdirtrebel.bin", 0x000000, 0x800000, CRC(e64bf1a1) SHA1(137f97d7d857697a13e0c8984509994dc7bc5fc5) )
ROM_END

ROM_START( drgqst )
	ROM_REGION( 0x800000, "bios", ROMREGION_ERASE00 )
	ROM_LOAD( "dragonquest.bin", 0x000000, 0x800000, CRC(3d24413f) SHA1(1677e81cedcf349de7bf091a232dc82c6424efba) )
ROM_END


CONS( 2004, xavtenni, 0, 0, xavix2000, xavix, xavix_state, init_xavix, "SSD Company LTD",         "XaviX Tennis (XaviXPORT)", MACHINE_IS_SKELETON )

CONS( 2005, ttv_sw,   0, 0, xavix2000, xavix, xavix_state, init_xavix, "Tiger / SSD Company LTD", "Star Wars Saga Edition - Lightsaber Battle Game", MACHINE_IS_SKELETON )
CONS( 2005, ttv_lotr, 0, 0, xavix2000, xavix, xavix_state, init_xavix, "Tiger / SSD Company LTD", "Lord Of The Rings - Warrior of Middle-Earth", MACHINE_IS_SKELETON )
CONS( 2005, ttv_mx,   0, 0, xavix2000, xavix, xavix_state, init_xavix, "Tiger / SSD Company LTD", "MX Dirt Rebel", MACHINE_IS_SKELETON )
CONS( 2003, drgqst,   0, 0, xavix2000, xavix, xavix_state, init_xavix, "Square Enix / SSD Company LTD", "Kenshin Dragon Quest: Yomigaerishi Densetsu no Ken", MACHINE_IS_SKELETON )
