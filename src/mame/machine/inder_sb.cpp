// license:BSD-3-Clause
// copyright-holders:David Haywood
/* Inder / Dinamic Sound Board */


#include "emu.h"
#include "machine/inder_sb.h"

#include "sound/volt_reg.h"
#include "speaker.h"


DEFINE_DEVICE_TYPE(INDER_AUDIO, inder_sb_device, "indersb", "Inder 4xDAC Sound Board")


inder_sb_device::inder_sb_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, INDER_AUDIO, tag, owner, clock)
	, device_mixer_interface(mconfig, *this, 2)
	, m_audiocpu(*this, "audiocpu")
	, m_ctc(*this, "ctc")
	, m_audiocpu_rom(*this, "audiocpu")
	, m_sounddata_bank(*this, "snddata")
{
}



// hacks for test purposes, these are installed over the program rom so we know when irqs are actually taken
READ8_MEMBER(inder_sb_device::vec_bankswitch_r)
{
	if (!machine().side_effects_disabled())
		m_sounddata_bank->set_entry(m_soundbank[(offset & 6) >> 1] & 7);
	return m_audiocpu_rom[offset + 0x0020];
}



READ16_MEMBER(inder_sb_device::megaphx_0x050002_r)
{
	machine().scheduler().synchronize();
//  int pc = machine().device("maincpu")->safe_pc();
	int ret = m_soundback;
	//m_soundback = 0;
	//logerror("(%06x) megaphx_0x050002_r (from z80?) %04x\n", pc, mem_mask);
	return ret;
}

WRITE16_MEMBER(inder_sb_device::megaphx_0x050000_w)
{
//  int pc = machine().device("maincpu")->safe_pc();
	machine().scheduler().synchronize();

	//logerror("(%06x) megaphx_0x050000_w (to z80?) %04x %04x\n", pc, data, mem_mask);
	m_soundsent = 0xff;
	m_sounddata = data;

}




static const z80_daisy_config daisy_chain[] =
{
	{ "ctc" },
	{ nullptr }
};


void inder_sb_device::sound_map(address_map &map)
{
	map(0x0000, 0x1fff).rom();
	map(0x0020, 0x0020).select(0x0006).r(this, FUNC(inder_sb_device::vec_bankswitch_r));
	map(0x4000, 0x7fff).ram();
	map(0x8000, 0xffff).bankr("snddata");
}

READ8_MEMBER(inder_sb_device::megaphx_sound_cmd_r)
{
	machine().scheduler().synchronize();
	return m_sounddata;
}

READ8_MEMBER(inder_sb_device::megaphx_sound_sent_r)
{
	machine().scheduler().synchronize();
	int ret = m_soundsent;
	m_soundsent = 0;
	return ret;
}

WRITE8_MEMBER(inder_sb_device::megaphx_sound_to_68k_w)
{
//  int pc = machine().device("audiocpu")->safe_pc();
	machine().scheduler().synchronize();
	//logerror("(%04x) megaphx_sound_to_68k_w (to 68k?) %02x\n", pc, data);

	m_soundback = data;
}

WRITE8_MEMBER(inder_sb_device::dac0_rombank_write)
{
	m_soundbank[0] = data;

//  printf("dac0_rombank_write %02x", data);
}

WRITE8_MEMBER(inder_sb_device::dac1_rombank_write)
{
	m_soundbank[1] = data;
//  printf("dac1_rombank_write %02x", data);

}

WRITE8_MEMBER(inder_sb_device::dac2_rombank_write)
{
	m_soundbank[2] = data;
//  printf("dac2_rombank_write %02x", data);
}

WRITE8_MEMBER(inder_sb_device::dac3_rombank_write)
{
	m_soundbank[3] = data;
//  printf("dac3_rombank_write %02x", data);

}


void inder_sb_device::sound_io(address_map &map)
{
	map.global_mask(0xff);
	map(0x00, 0x00).w("dac0", FUNC(dac_byte_interface::write));
	map(0x01, 0x01).w("dac0vol", FUNC(dac_byte_interface::write));
	map(0x02, 0x02).w("dac1", FUNC(dac_byte_interface::write));
	map(0x03, 0x03).w("dac1vol", FUNC(dac_byte_interface::write));
	map(0x04, 0x04).w("dac2", FUNC(dac_byte_interface::write));
	map(0x05, 0x05).w("dac2vol", FUNC(dac_byte_interface::write));
	map(0x06, 0x06).w("dac3", FUNC(dac_byte_interface::write));
	map(0x07, 0x07).w("dac3vol", FUNC(dac_byte_interface::write));

	// not 100% sure how rom banking works.. but each channel can specify a different bank for the 0x8000 range.  Maybe the bank happens when the interrupt triggers so each channel reads the correct data? (so we'd need to put the actual functions in the CTC callbacks)
	map(0x10, 0x10).w(this, FUNC(inder_sb_device::dac0_rombank_write));
	map(0x11, 0x11).w(this, FUNC(inder_sb_device::dac1_rombank_write));
	map(0x12, 0x12).w(this, FUNC(inder_sb_device::dac2_rombank_write));
	map(0x13, 0x13).w(this, FUNC(inder_sb_device::dac3_rombank_write));




	map(0x20, 0x23).rw("ctc", FUNC(z80ctc_device::read), FUNC(z80ctc_device::write));

	map(0x30, 0x30).rw(this, FUNC(inder_sb_device::megaphx_sound_cmd_r), FUNC(inder_sb_device::megaphx_sound_to_68k_w));
	map(0x31, 0x31).r(this, FUNC(inder_sb_device::megaphx_sound_sent_r));
}


MACHINE_CONFIG_START(inder_sb_device::device_add_mconfig)
	MCFG_CPU_ADD("audiocpu", Z80, 8000000) // unk freq
	MCFG_Z80_DAISY_CHAIN(daisy_chain)
	MCFG_CPU_PROGRAM_MAP(sound_map)
	MCFG_CPU_IO_MAP(sound_io)

	MCFG_DEVICE_ADD("ctc", Z80CTC, 4000000) // unk freq
	// runs in IM2 , vector set to 0x20 , values there are 0xCC, 0x02, 0xE6, 0x02, 0x09, 0x03, 0x23, 0x03  (so 02cc, 02e6, 0309, 0323, all of which are valid irq handlers)
	MCFG_Z80CTC_INTR_CB(INPUTLINE("audiocpu", 0))

	MCFG_SPEAKER_STANDARD_MONO("speaker")
	MCFG_SOUND_ADD("dac0", DAC_8BIT_R2R_TWOS_COMPLEMENT, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.5) // unknown DAC
	MCFG_SOUND_ADD("dac1", DAC_8BIT_R2R_TWOS_COMPLEMENT, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.5) // unknown DAC
	MCFG_SOUND_ADD("dac2", DAC_8BIT_R2R_TWOS_COMPLEMENT, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.5) // unknown DAC
	MCFG_SOUND_ADD("dac3", DAC_8BIT_R2R_TWOS_COMPLEMENT, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.5) // unknown DAC
	MCFG_SOUND_ADD("dac0vol", DAC_8BIT_R2R, 0) MCFG_SOUND_ROUTE_EX(0, "dac0", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "dac0", -1.0, DAC_VREF_NEG_INPUT) // unknown DAC
	MCFG_SOUND_ADD("dac1vol", DAC_8BIT_R2R, 0) MCFG_SOUND_ROUTE_EX(0, "dac1", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "dac1", -1.0, DAC_VREF_NEG_INPUT) // unknown DAC
	MCFG_SOUND_ADD("dac2vol", DAC_8BIT_R2R, 0) MCFG_SOUND_ROUTE_EX(0, "dac2", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "dac2", -1.0, DAC_VREF_NEG_INPUT) // unknown DAC
	MCFG_SOUND_ADD("dac3vol", DAC_8BIT_R2R, 0) MCFG_SOUND_ROUTE_EX(0, "dac3", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "dac3", -1.0, DAC_VREF_NEG_INPUT) // unknown DAC
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac0vol", 1.0, DAC_VREF_POS_INPUT)
	MCFG_SOUND_ROUTE_EX(0, "dac1vol", 1.0, DAC_VREF_POS_INPUT)
	MCFG_SOUND_ROUTE_EX(0, "dac2vol", 1.0, DAC_VREF_POS_INPUT)
	MCFG_SOUND_ROUTE_EX(0, "dac3vol", 1.0, DAC_VREF_POS_INPUT)
MACHINE_CONFIG_END


void inder_sb_device::device_start()
{
	m_sounddata_bank->configure_entries(0, 8, memregion("user2")->base(), 0x8000);
	m_sounddata_bank->set_entry(0);

	save_item(NAME(m_soundbank));
	save_item(NAME(m_soundsent));
	save_item(NAME(m_sounddata));
	save_item(NAME(m_soundback));
}

void inder_sb_device::device_reset()
{
}
