// license:GPL-2.0+
// copyright-holders: Felipe Sanches
/***************************************************************************

    Ultratec Minicom IV
    http://www.ultratec.com/ttys/non-printing/minicom.php

  Driver by Felipe Sanches

There messages are displayed in a 20 digit 14 segment VFD.

  ***********
 * *   *   * *
 *  *  *  *  *
 *   * * *   *
  ***** *****
 *   * * *   *
 *  *  *  *  *
 * *   *   * *
  ***********  *

Digits seem to be selected by a mux fed by a counter that is incremented by pulses in P1.2
There may be a bit connected to the counter reset signal...

Segment data is sent to each 14seg digit by first writing half of the data to port P0 and
 toggling T0 and then the other half of data is written to P0 again but toggling T1 afterwards.

  Changelog:

   2014 JUL 22 [Felipe Sanches]:
   * Fixing alignment of 14-seg display multiplexation

   2014 JUL 19 [Felipe Sanches]:
   * Got the display working except for a few glitches

   2014 JUL 16 [Felipe Sanches]:
   * Initial driver skeleton
*/

#define LOG_IO_PORTS 0
#define PRINTER_ATTACHED 1

#include "emu.h"
#include "cpu/mcs51/mcs51.h"
#include "minicom.lh"

class minicom_state : public driver_device
{
public:
	minicom_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_digits(*this, "digit%u", 0U)
	{ }

	DECLARE_WRITE8_MEMBER(i87c52_p0_w);
	DECLARE_WRITE8_MEMBER(i87c52_p1_w);
	DECLARE_WRITE8_MEMBER(i87c52_p2_w);
	DECLARE_WRITE8_MEMBER(i87c52_p3_w);
	DECLARE_READ8_MEMBER(i87c52_p1_r);
	DECLARE_READ8_MEMBER(i87c52_p2_r);
	void init_minicom();
	void minicom(machine_config &config);
private:
	uint8_t m_p[4];
	uint16_t m_display_data;
	int m_digit_index;
	virtual void machine_start() override;
	virtual void machine_reset() override;
	required_device<cpu_device> m_maincpu;
	output_finder<20> m_digits;
};

void minicom_state::machine_start()
{
	m_digits.resolve();
	// zerofill
	memset(m_p, 0, 4);
	m_digit_index = 0;
	m_display_data = 0;

	// register for savestates
	save_item(NAME(m_p));
	save_item(NAME(m_digit_index));
	save_item(NAME(m_display_data));
}

void minicom_state::machine_reset()
{
	m_digit_index = 19;
	m_display_data = 0;

	for (int i=0; i<20; i++)
		m_digits[i] = 0;
}

READ8_MEMBER(minicom_state::i87c52_p1_r)
{
	//P1.3 seems to be an indicator of whether or not we have a printer device attached.
	// at address 0xABF the code checks this flag in order to decide which string to display:
	// "MINIPRINT IS RESET" or "MINICOM IS RESET"
	return PRINTER_ATTACHED << 3;
}

READ8_MEMBER(minicom_state::i87c52_p2_r)
{
//          return 0; //para a palestra no Garoa... :-)
	return 1; //to skip the "NO POWER" warning. I'm not sure why.
}

#if LOG_IO_PORTS
static void printbits(uint8_t v) {
	int i;
	for(i = 7; i >= 0; i--) putchar('0' + ((v >> i) & 1));
}
#endif

#define FALLING_EDGE(old_data, new_data, bit) (BIT(old_data ^ new_data, bit) && !BIT(new_data, bit))
#define RISING_EDGE(old_data, new_data, bit) (BIT(old_data ^ new_data, bit) && BIT(new_data, bit))

#define P1_UNKNOWN_BITS (0xFF & ~(1 << 2))
#define P2_UNKNOWN_BITS 0xFF
#define P3_UNKNOWN_BITS (0xFF & ~((1 << 4)|(1 << 5)))
WRITE8_MEMBER(minicom_state::i87c52_p0_w)
{
	if (data != m_p[0])
	{
		m_p[0]=data;

		//Bit P0.1 is the serial-input of a 20-bit shift register (made of a couple of chained UCN5810AF chips)
		//We are emulating the display based on the assumption that the firmware will multiplex it by defining one digit at a given time
		//It would be better (in terms of being closer to the actual hardware) to emulate the 20 bit shift register and update all digits
		//for which a bit is TTL high. It seems to me that in the real hardware that would result in dimmer brightness in the display and it
		//does not seem trivial to me to implement this using our current layout system. I'm leaving this note to whoever finds it exciting
				//to explore these possibilities (perhaps myself in the future?).
		if (BIT(data,1)){
			m_digit_index = 0;
		}
	}
}

WRITE8_MEMBER(minicom_state::i87c52_p1_w)
{
	if (data != m_p[1])
	{
#if LOG_IO_PORTS
		uint8_t changed = m_p[1] ^ data;
		if (changed ^ P1_UNKNOWN_BITS)
		{
			printf("Write to P1: %02X changed: (        ) (", data);
			printbits(changed);
			printf(") (        ) (        )\n");
		}
#endif
		if (FALLING_EDGE(m_p[1], data, 2))
		{
			m_digit_index--;
			if (m_digit_index<0) m_digit_index = 19;
		}
		m_p[1] = data;
	}
}

WRITE8_MEMBER(minicom_state::i87c52_p2_w)
{
	if (data != m_p[2])
	{
#if LOG_IO_PORTS
		uint8_t changed = m_p[2] ^ data;
		if (changed ^ P2_UNKNOWN_BITS)
		{
			printf("Write to P2: %02X changed: (        ) (        ) (", data);
			printbits(changed);
			printf(") (        )\n");
		}
#endif
		m_p[2] = data;
	}
}

WRITE8_MEMBER(minicom_state::i87c52_p3_w)
{
	if (data != m_p[3])
	{
		uint8_t changed = m_p[3] ^ data;
#if LOG_IO_PORTS
		if (changed ^ P3_UNKNOWN_BITS)
		{
			printf("Write to P3: %02X changed: (        ) (        ) (        ) (", data);
			printbits(changed);
			printf(")\n");
		}
#endif

		if (FALLING_EDGE(m_p[3], data, 4)) //P3.4 = T0
		{
			m_display_data &= 0xFF00;
			m_display_data |= m_p[0];
		}

		if (FALLING_EDGE(m_p[3], data, 5)) //P3.5 = T1
		{
			m_display_data &= 0xFF;
			m_display_data |= (m_p[0] << 8);
		}

		if (BIT(changed,4) || BIT(changed,5))
		{
			m_digits[m_digit_index] = bitswap<16>(m_display_data,  9,  1,  3, 11, 12,  4,  2, 10, 14, 6,  7, 5,  0, 15,  13, 8) & 0x3FFF;
		}
		m_p[3] = data;
	}
}

void minicom_state::init_minicom()
{
}

MACHINE_CONFIG_START(minicom_state::minicom)
	/* basic machine hardware */
	MCFG_DEVICE_ADD("maincpu", I87C52, XTAL(10'000'000)) /*FIX-ME: verify the correct clock frequency */
	MCFG_MCS51_PORT_P0_OUT_CB(WRITE8(*this, minicom_state, i87c52_p0_w))
	MCFG_MCS51_PORT_P1_IN_CB(READ8(*this, minicom_state, i87c52_p1_r))
	MCFG_MCS51_PORT_P1_OUT_CB(WRITE8(*this, minicom_state, i87c52_p1_w))
	MCFG_MCS51_PORT_P2_IN_CB(READ8(*this, minicom_state, i87c52_p2_r))
	MCFG_MCS51_PORT_P2_OUT_CB(WRITE8(*this, minicom_state, i87c52_p2_w))
	MCFG_MCS51_PORT_P3_OUT_CB(WRITE8(*this, minicom_state, i87c52_p3_w))

	/* video hardware */
	/* fluorescent 14-segment display forming a row of 20 characters */
	MCFG_DEFAULT_LAYOUT(layout_minicom)

/* TODO: Map the keyboard rows/cols inputs (43-key, 4-row keyboard) */

/* TODO: Treat the modem as a sound device. That may be an interesting challenge... :-) */
MACHINE_CONFIG_END

ROM_START( minicom )
	ROM_REGION( 0x2000, "maincpu", 0 )
	ROM_LOAD( "ultratec_minicom_iv.rom",  0x0000, 0x2000, CRC(22881366) SHA1(fc3faea5ecc1476e5bcb7999638f3150d06c9a81) )
ROM_END

ROM_START( mcom4_02 )
	ROM_REGION( 0x2000, "maincpu", 0 )
	ROM_LOAD( "ultratec_minicom_iv_20020419.rom",  0x0000, 0x2000, CRC(99b6cc35) SHA1(32577005bf02042f893c8880f8ce5b3d8a5f55f9) )
ROM_END

//    YEAR  NAME      PARENT  COMPAT  MACHINE  INPUT  CLASS          INIT          COMPANY     FULLNAME                   FLAGS
COMP( 1997, minicom,  0,      0,      minicom, 0,     minicom_state, init_minicom, "Ultratec", "Minicom IV (1997-08-11)", MACHINE_NOT_WORKING | MACHINE_IMPERFECT_GRAPHICS | MACHINE_NO_SOUND ) // fw release data: 11th Aug 1997
COMP( 2002, mcom4_02, 0,      0,      minicom, 0,     minicom_state, init_minicom, "Ultratec", "Minicom IV (2002-04-19)", MACHINE_NOT_WORKING | MACHINE_IMPERFECT_GRAPHICS | MACHINE_NO_SOUND ) // fw release data: 19th Apr 2002
