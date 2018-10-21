// license:BSD-3-Clause
// copyright-holders:AJR
/***************************************************************************

    Super Star (Recreativos Franco)

    Skeleton driver for 8085-based pinball hardware.

***************************************************************************/

#include "emu.h"
#include "cpu/i8085/i8085.h"
#include "cpu/mcs48/mcs48.h"
#include "machine/i8212.h"
#include "sound/ay8910.h"
#include "speaker.h"

class supstarf_state : public driver_device
{
public:
	supstarf_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
			m_maincpu(*this, "maincpu"),
			m_soundcpu(*this, "soundcpu"),
			m_psg(*this, {"psg1", "psg2"}),
			m_soundlatch(*this, {"soundlatch1", "soundlatch2"})
	{
	}

	void supstarf(machine_config &config);

private:
	DECLARE_READ8_MEMBER(psg_latch_r);
	DECLARE_WRITE8_MEMBER(psg_latch_w);
	DECLARE_WRITE8_MEMBER(port1_w);
	DECLARE_WRITE8_MEMBER(port2_w);
	DECLARE_READ_LINE_MEMBER(contacts_r);
	DECLARE_WRITE_LINE_MEMBER(displays_w);
	DECLARE_WRITE8_MEMBER(driver_clk_w);
	DECLARE_READ_LINE_MEMBER(phase_detect_r);
	DECLARE_WRITE8_MEMBER(lights_a_w);
	DECLARE_WRITE8_MEMBER(lights_b_w);

	void main_io_map(address_map &map);
	void main_map(address_map &map);
	void sound_io_map(address_map &map);
	void sound_map(address_map &map);

	virtual void machine_start() override;

	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_soundcpu;
	required_device_array<ay8910_device, 2> m_psg;
	required_device_array<i8212_device, 2> m_soundlatch;

	u8 m_port1_data;
	bool m_pcs[2];
	bool m_latch_select;
};

void supstarf_state::main_map(address_map &map)
{
	map(0x0000, 0x3fff).rom();
	map(0x8000, 0x8000).r("soundlatch1", FUNC(i8212_device::read)).w("soundlatch2", FUNC(i8212_device::strobe));
	map(0xc000, 0xc7ff).ram(); // 5517 (2Kx8) at IC11
}

void supstarf_state::main_io_map(address_map &map)
{
	map.global_mask(0xff);
	map(0x00, 0xff).w(FUNC(supstarf_state::driver_clk_w));
}

void supstarf_state::sound_map(address_map &map)
{
	map(0x000, 0xfff).rom(); // external EPROM
}

void supstarf_state::sound_io_map(address_map &map)
{
	map(0x00, 0xff).rw(FUNC(supstarf_state::psg_latch_r), FUNC(supstarf_state::psg_latch_w));
}

READ8_MEMBER(supstarf_state::psg_latch_r)
{
	u8 result = 0xff; // AR3 +5v pullup

	for (int d = 0; d < 2; d++)
	{
		if (m_pcs[d])
		{
			m_psg[d]->address_w(space, 0, offset);
			result &= m_psg[d]->data_r(space, 0);
		}
	}

	if (m_latch_select)
		result &= m_soundlatch[1]->read(space, 0);

	return result;
}

WRITE8_MEMBER(supstarf_state::psg_latch_w)
{
	for (int d = 0; d < 2; d++)
	{
		if (m_pcs[d])
		{
			m_psg[d]->address_w(space, 0, offset);
			m_psg[d]->data_w(space, 0, data);
		}
	}

	if (m_latch_select)
		m_soundlatch[0]->strobe(space, 0, data);
}

WRITE8_MEMBER(supstarf_state::port1_w)
{
	m_port1_data = data;
}

WRITE8_MEMBER(supstarf_state::port2_w)
{
	m_maincpu->set_input_line(INPUT_LINE_RESET, BIT(data, 4) ? CLEAR_LINE : ASSERT_LINE);
	if (!BIT(data, 4))
	{
		m_soundlatch[0]->reset();
		m_soundlatch[1]->reset();
		m_psg[0]->reset();
		m_psg[1]->reset();
	}

	m_pcs[0] = !BIT(data, 6);
	m_pcs[1] = !BIT(data, 5);
	m_latch_select = !BIT(data, 7);
}

READ_LINE_MEMBER(supstarf_state::contacts_r)
{
	return 1;
}

WRITE_LINE_MEMBER(supstarf_state::displays_w)
{
}

WRITE8_MEMBER(supstarf_state::driver_clk_w)
{
}

READ_LINE_MEMBER(supstarf_state::phase_detect_r)
{
	return 0;
}

WRITE8_MEMBER(supstarf_state::lights_a_w)
{
}

WRITE8_MEMBER(supstarf_state::lights_b_w)
{
}

void supstarf_state::machine_start()
{
	m_pcs[0] = m_pcs[1] = 0;
	m_latch_select = 0;
	m_port1_data = 0xff;

	save_item(NAME(m_pcs));
	save_item(NAME(m_latch_select));
	save_item(NAME(m_port1_data));
}

MACHINE_CONFIG_START(supstarf_state::supstarf)
	MCFG_DEVICE_ADD("maincpu", I8085A, XTAL(5'068'800))
	MCFG_DEVICE_PROGRAM_MAP(main_map)
	MCFG_DEVICE_IO_MAP(main_io_map)
	MCFG_I8085A_SID(READLINE(*this, supstarf_state, contacts_r))
	MCFG_I8085A_SOD(WRITELINE(*this, supstarf_state, displays_w))

	MCFG_DEVICE_ADD("soundcpu", I8035, XTAL(5'068'800) / 2) // from 8085 pin 37 (CLK OUT)
	MCFG_DEVICE_PROGRAM_MAP(sound_map)
	MCFG_DEVICE_IO_MAP(sound_io_map)
	MCFG_MCS48_PORT_P1_OUT_CB(WRITE8(*this, supstarf_state, port1_w))
	MCFG_MCS48_PORT_P2_OUT_CB(WRITE8(*this, supstarf_state, port2_w))
	MCFG_MCS48_PORT_T1_IN_CB(READLINE(*this, supstarf_state, phase_detect_r))

	I8212(config, m_soundlatch[0], 0);
	m_soundlatch[0]->md_rd_callback().set_constant(0);
	m_soundlatch[0]->int_wr_callback().set_inputline("maincpu", I8085_RST55_LINE);

	I8212(config, m_soundlatch[1], 0);
	m_soundlatch[1]->md_rd_callback().set_constant(0);
	m_soundlatch[1]->int_wr_callback().set_inputline("soundcpu", MCS48_INPUT_IRQ);
	//MCFG_DEVCB_CHAIN_OUTPUT(INPUTLINE("maincpu", I8085_READY_LINE))

	SPEAKER(config, "mono").front_center();

	MCFG_DEVICE_ADD("psg1", AY8910, XTAL(5'068'800) / 6) // from 8035 pin 1 (T0)
	MCFG_AY8910_PORT_A_WRITE_CB(WRITE8(*this, supstarf_state, lights_a_w))
	MCFG_AY8910_PORT_B_WRITE_CB(WRITE8(*this, supstarf_state, lights_b_w))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.50)

	MCFG_DEVICE_ADD("psg2", AY8910, XTAL(5'068'800) / 6) // from 8035 pin 1 (T0)
	MCFG_AY8910_PORT_A_READ_CB(IOPORT("JO"))
	MCFG_AY8910_PORT_B_READ_CB(IOPORT("I1"))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.50)
MACHINE_CONFIG_END

static INPUT_PORTS_START(supstarf)
	PORT_START("I1")
	PORT_DIPUNKNOWN_DIPLOC(0x01, 0x01, "I1:1")
	PORT_DIPUNKNOWN_DIPLOC(0x02, 0x02, "I1:2")
	PORT_DIPUNKNOWN_DIPLOC(0x04, 0x04, "I1:3")
	PORT_DIPUNKNOWN_DIPLOC(0x08, 0x08, "I1:4")
	PORT_DIPUNKNOWN_DIPLOC(0x10, 0x10, "I1:5")
	PORT_DIPUNKNOWN_DIPLOC(0x20, 0x20, "I1:6")
	PORT_DIPUNKNOWN_DIPLOC(0x40, 0x40, "I1:7")
	PORT_DIPUNKNOWN_DIPLOC(0x80, 0x80, "I1:8")

	PORT_START("JM")
	PORT_BIT(0x80, 0x80, IPT_UNKNOWN)
	PORT_BIT(0x40, 0x40, IPT_UNKNOWN)
	PORT_BIT(0x20, 0x20, IPT_UNKNOWN)
	PORT_BIT(0x10, 0x10, IPT_UNKNOWN)
	PORT_BIT(0x08, 0x08, IPT_UNKNOWN)
	PORT_BIT(0x04, 0x04, IPT_UNKNOWN)
	PORT_BIT(0x02, 0x02, IPT_UNKNOWN)
	PORT_BIT(0x01, 0x01, IPT_UNKNOWN)

	PORT_START("JN")
	PORT_BIT(0x80, 0x80, IPT_UNKNOWN)
	PORT_BIT(0x40, 0x40, IPT_UNKNOWN)
	PORT_BIT(0x20, 0x20, IPT_UNKNOWN)
	PORT_BIT(0x10, 0x10, IPT_UNKNOWN)
	PORT_BIT(0x08, 0x08, IPT_UNKNOWN)
	PORT_BIT(0x04, 0x04, IPT_UNKNOWN)
	PORT_BIT(0x02, 0x02, IPT_UNKNOWN)
	PORT_BIT(0x01, 0x01, IPT_UNUSED)

	PORT_START("JO")
	PORT_BIT(0x80, 0x80, IPT_UNKNOWN)
	PORT_BIT(0x40, 0x40, IPT_UNKNOWN)
	PORT_BIT(0x20, 0x20, IPT_UNKNOWN)
	PORT_BIT(0x10, 0x10, IPT_UNKNOWN)
	PORT_BIT(0x0f, 0x0f, IPT_UNUSED)
INPUT_PORTS_END

ROM_START(supstarf)
	ROM_REGION(0x4000, "maincpu", 0)
	ROM_LOAD("27c128.ic19", 0x0000, 0x4000, CRC(9a440461) SHA1(e2f8dcf95084f755d3a34d77ba2649602687a610))
	// IC14 for second program ROM is unpopulated

	ROM_REGION(0x1000, "soundcpu", 0)
	ROM_LOAD("2532.ic4", 0x0000, 0x1000, CRC(b6ef3c7a) SHA1(aabb6f8569685fc3a917a7bb5ebfcc4b20086b15) BAD_DUMP) // D6 stuck high and probably totally garbage
ROM_END

GAME( 1986, supstarf, 0, supstarf, supstarf, supstarf_state, empty_init, ROT0, "Recreativos Franco", "Super Star (Recreativos Franco)", MACHINE_IS_SKELETON_MECHANICAL )
