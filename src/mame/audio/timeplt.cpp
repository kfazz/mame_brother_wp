// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria
/***************************************************************************

    This code is used by the following module:

    timeplt.c
    pooyan.c
    rallyx.c (for locomotn)
    tutankhm.c
    rocnrope.c

***************************************************************************/

#include "emu.h"
#include "audio/timeplt.h"

#include "machine/gen_latch.h"
#include "speaker.h"


DEFINE_DEVICE_TYPE(TIMEPLT_AUDIO, timeplt_audio_device, "timplt_audio", "Time Pilot Audio")
DEFINE_DEVICE_TYPE(LOCOMOTN_AUDIO, locomotn_audio_device, "locomotn_audio", "Loco-Motion Audio")

timeplt_audio_device::timeplt_audio_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: timeplt_audio_device(mconfig, TIMEPLT_AUDIO, tag, owner, clock)
{
}

locomotn_audio_device::locomotn_audio_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: timeplt_audio_device(mconfig, LOCOMOTN_AUDIO, tag, owner, clock)
{
}

timeplt_audio_device::timeplt_audio_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, m_soundcpu(*this, "tpsound")
	, m_soundlatch(*this, "soundlatch")
	, m_filter_0(*this, "filter.0.%u", 0)
	, m_filter_1(*this, "filter.1.%u", 0)
	, m_last_irq_state(0)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void timeplt_audio_device::device_start()
{
	m_last_irq_state = 0;
	save_item(NAME(m_last_irq_state));
}


/*************************************
 *
 *  Sound timer
 *
 *************************************/

/* The timer clock which feeds the upper 4 bits of                      */
/* AY-3-8910 port A is based on the same clock                          */
/* feeding the sound CPU Z80.  It is a divide by                        */
/* 5120, formed by a standard divide by 512,                            */
/* followed by a divide by 10 using a 4 bit                             */
/* bi-quinary count sequence. (See LS90 data sheet                      */
/* for an example).                                                     */
/*                                                                      */
/* Bit 4 comes from the output of the divide by 1024                    */
/*       0, 1, 0, 1, 0, 1, 0, 1, 0, 1                                   */
/* Bit 5 comes from the QC output of the LS90 producing a sequence of   */
/*       0, 0, 1, 1, 0, 0, 1, 1, 1, 0                                   */
/* Bit 6 comes from the QD output of the LS90 producing a sequence of   */
/*       0, 0, 0, 0, 1, 0, 0, 0, 0, 1                                   */
/* Bit 7 comes from the QA output of the LS90 producing a sequence of   */
/*       0, 0, 0, 0, 0, 1, 1, 1, 1, 1                                   */

READ8_MEMBER( timeplt_audio_device::portB_r )
{
	static const int timeplt_timer[10] =
	{
		0x00, 0x10, 0x20, 0x30, 0x40, 0x90, 0xa0, 0xb0, 0xa0, 0xd0
	};

	return timeplt_timer[(m_soundcpu->total_cycles() / 512) % 10];
}



/*************************************
 *
 *  Filter controls
 *
 *************************************/

void timeplt_audio_device::set_filter(filter_rc_device &device, int data)
{
	int C = 0;

	if (data & 1)
		C += 220000;    /* 220000pF = 0.220uF */
	if (data & 2)
		C +=  47000;    /*  47000pF = 0.047uF */

	device.filter_rc_set_RC(filter_rc_device::LOWPASS, 1000, 5100, 0, CAP_P(C));
}


WRITE8_MEMBER( timeplt_audio_device::filter_w )
{
	set_filter(*m_filter_1[0], (offset >>  0) & 3);
	set_filter(*m_filter_1[1], (offset >>  2) & 3);
	set_filter(*m_filter_1[2], (offset >>  4) & 3);
	set_filter(*m_filter_0[0], (offset >>  6) & 3);
	set_filter(*m_filter_0[1], (offset >>  8) & 3);
	set_filter(*m_filter_0[2], (offset >> 10) & 3);
}



/*************************************
 *
 *  External interfaces
 *
 *************************************/

WRITE8_MEMBER(timeplt_audio_device::sound_data_w)
{
	m_soundlatch->write(space, 0, data);
}


WRITE_LINE_MEMBER(timeplt_audio_device::sh_irqtrigger_w)
{
	if (m_last_irq_state == 0 && state)
	{
		/* setting bit 0 low then high triggers IRQ on the sound CPU */
		m_soundcpu->set_input_line_and_vector(0, HOLD_LINE, 0xff);
	}

	m_last_irq_state = state;
}


WRITE_LINE_MEMBER(timeplt_audio_device::mute_w)
{
	// controls pin 6 (DC audio mute) of LA4460 amplifier
	machine().sound().system_mute(state);
}



/*************************************
 *
 *  Memory maps
 *
 *************************************/

void timeplt_audio_device::timeplt_sound_map(address_map &map)
{
	map(0x0000, 0x2fff).rom();
	map(0x3000, 0x33ff).mirror(0x0c00).ram();
	map(0x4000, 0x4000).mirror(0x0fff).rw("ay1", FUNC(ay8910_device::data_r), FUNC(ay8910_device::data_w));
	map(0x5000, 0x5000).mirror(0x0fff).w("ay1", FUNC(ay8910_device::address_w));
	map(0x6000, 0x6000).mirror(0x0fff).rw("ay2", FUNC(ay8910_device::data_r), FUNC(ay8910_device::data_w));
	map(0x7000, 0x7000).mirror(0x0fff).w("ay2", FUNC(ay8910_device::address_w));
	map(0x8000, 0xffff).w(FUNC(timeplt_audio_device::filter_w));
}


void locomotn_audio_device::locomotn_sound_map(address_map &map)
{
	map(0x0000, 0x1fff).rom();
	map(0x2000, 0x23ff).mirror(0x0c00).ram();
	map(0x3000, 0x3fff).w(FUNC(locomotn_audio_device::filter_w));
	map(0x4000, 0x4000).mirror(0x0fff).rw("ay1", FUNC(ay8910_device::data_r), FUNC(ay8910_device::data_w));
	map(0x5000, 0x5000).mirror(0x0fff).w("ay1", FUNC(ay8910_device::address_w));
	map(0x6000, 0x6000).mirror(0x0fff).rw("ay2", FUNC(ay8910_device::data_r), FUNC(ay8910_device::data_w));
	map(0x7000, 0x7000).mirror(0x0fff).w("ay2", FUNC(ay8910_device::address_w));
}


/*************************************
 *
 *  Machine drivers
 *
 *************************************/

MACHINE_CONFIG_START(timeplt_audio_device::device_add_mconfig)

	/* basic machine hardware */
	MCFG_DEVICE_ADD(m_soundcpu, Z80, DERIVED_CLOCK(1, 8))
	MCFG_DEVICE_PROGRAM_MAP(timeplt_sound_map)

	/* sound hardware */
	SPEAKER(config, "mono").front_center();

	MCFG_GENERIC_LATCH_8_ADD(m_soundlatch)

	MCFG_DEVICE_ADD("ay1", AY8910, DERIVED_CLOCK(1, 8))
	MCFG_AY8910_PORT_A_READ_CB(READ8(m_soundlatch, generic_latch_8_device, read))
	MCFG_AY8910_PORT_B_READ_CB(READ8(*this, timeplt_audio_device, portB_r))
	MCFG_SOUND_ROUTE(0, "filter.0.0", 0.60)
	MCFG_SOUND_ROUTE(1, "filter.0.1", 0.60)
	MCFG_SOUND_ROUTE(2, "filter.0.2", 0.60)

	MCFG_DEVICE_ADD("ay2", AY8910, DERIVED_CLOCK(1, 8))
	MCFG_SOUND_ROUTE(0, "filter.1.0", 0.60)
	MCFG_SOUND_ROUTE(1, "filter.1.1", 0.60)
	MCFG_SOUND_ROUTE(2, "filter.1.2", 0.60)

	for (required_device<filter_rc_device> &filter : m_filter_0)
		FILTER_RC(config, filter).add_route(ALL_OUTPUTS, "mono", 1.0);

	for (required_device<filter_rc_device> &filter : m_filter_1)
		FILTER_RC(config, filter).add_route(ALL_OUTPUTS, "mono", 1.0);
MACHINE_CONFIG_END


MACHINE_CONFIG_START(locomotn_audio_device::device_add_mconfig)
	timeplt_audio_device::device_add_mconfig(config);

	/* basic machine hardware */
	MCFG_DEVICE_MODIFY("tpsound")
	MCFG_DEVICE_PROGRAM_MAP(locomotn_sound_map)
MACHINE_CONFIG_END

//-------------------------------------------------
//  sound_stream_update - handle a stream update
//-------------------------------------------------

void timeplt_audio_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
	// should never get here
	fatalerror("sound_stream_update called; not applicable to legacy sound devices\n");
}
