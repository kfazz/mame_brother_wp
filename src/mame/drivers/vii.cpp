// license:BSD-3-Clause
// copyright-holders:Ryan Holtz, Robbbert
/******************************************************************************

    Short Description:

        Systems which run on the SPG243 SoC

    Status:

        Mostly working

    To-Do:

        Audio (SPG243)

    Known u'nSP-Based Systems:

         D - SPG240 - Radica Skateboarder (Sunplus QL8041C die)
        ND - SPG243 - Some form of Leapfrog "edutainment" system
        ND - SPG243 - Star Wars: Clone Wars
        ND - SPG243 - Toy Story
        ND - SPG243 - Animal Art Studio
        ND - SPG243 - Finding Nemo
         D - SPG243 - The Batman
         D - SPG243 - Wall-E
         D - SPG243 - KenSingTon / Siatronics / Jungle Soft Vii
 Partial D - SPG200 - VTech V.Smile
        ND - unknown - Zone 40
         D - SPG243 - Zone 60
         D - SPG243 - Wireless 60
        ND - unknown - Wireless Air 60
        ND - Likely many more


Similar Systems: ( from http://en.wkikpedia.org/wiki/V.Smile )
- V.Smile by VTech, a system designed for children under the age of 10
- V.Smile Pocket (2 versions)
- V.Smile Cyber Pocket
- V.Smile PC Pal
- V-Motion Active Learning System
- Leapster
- V.Smile Baby Infant Development System
- V.Flash

also on this hardware

    name                        PCB ID      ROM width   TSOP pads   ROM size        SEEPROM         die markings
    Radica Play TV Football 2   L7278       x16         48          not dumped      no              Sunplus
    Dream Life                  ?           x16         48          not dumped      no              Sunplus

Detailed list of bugs:

- all systems:
  Various inaccuracies in samples/envelopes.
- vsmile:
  Games loop the first legal screen rather than continuing.
- walle:
  Voice sample on the title screen is continually retriggered by the game code.
  Title screen lacks New Game / Continue Game menu options.
- rad_skat:
  Palette issues on the High Score screen.
- vii:
  Music does not loop.
  When loading a cart from file manager, sometimes MAME will crash.
  The "MOTOR" option in the diagnostic menu does nothing when selected.
  The "SPEECH IC" option in the diagnostic menu does nothing when selected.
  On 'vii_vc1' & 'vii_vc2' cart, the left-right keys are transposed with the up-down keys.
    This is not a bug per se, as the games are played with the controller physically rotated 90 degrees.
- zone60/wirels60:
  All Games: Music does not loop.
  Basketball: MAME fatalerrors when starting the game due to jumping to invalid code.


*******************************************************************************/

#include "emu.h"

#include "cpu/unsp/unsp.h"
#include "machine/bankdev.h"
#include "machine/i2cmem.h"
#include "machine/nvram.h"
#include "machine/spg2xx.h"

#include "bus/generic/slot.h"
#include "bus/generic/carts.h"

#include "emupal.h"
#include "screen.h"
#include "softlist.h"
#include "speaker.h"

class spg2xx_game_state : public driver_device
{
public:
	spg2xx_game_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_screen(*this, "screen")
#if SPG2XX_VISUAL_AUDIO_DEBUG
		, m_debug_screen(*this, "debug_screen")
#endif
		, m_spg(*this, "spg")
		, m_bank(*this, "cartbank")
		, m_io_p1(*this, "P1")
		, m_io_p2(*this, "P2")
		, m_io_p3(*this, "P3")
		, m_io_motionx(*this, "MOTIONX")
		, m_io_motiony(*this, "MOTIONY")
		, m_io_motionz(*this, "MOTIONZ")
		, m_i2cmem(*this, "i2cmem")
		, m_nvram(*this, "nvram")
	{ }

	void spg2xx_base(machine_config &config);
	void spg2xx_basep(machine_config &config);
	void jakks(machine_config &config);
	void walle(machine_config &config);
	void wireless60(machine_config &config);
	void rad_skat(machine_config &config);
	void rad_skatp(machine_config &config);
	void rad_crik(machine_config &config);
	void non_spg_base(machine_config &config);

	void init_rad_crik();

protected:
	void switch_bank(uint32_t bank);

	virtual void machine_start() override;

	DECLARE_WRITE8_MEMBER(eeprom_w);
	DECLARE_READ8_MEMBER(eeprom_r);

	DECLARE_READ16_MEMBER(jakks_porta_r);
	DECLARE_WRITE16_MEMBER(wireless60_porta_w);
	DECLARE_WRITE16_MEMBER(wireless60_portb_w);
	DECLARE_READ16_MEMBER(wireless60_porta_r);

	required_device<cpu_device> m_maincpu;
	required_device<screen_device> m_screen;
#if SPG2XX_VISUAL_AUDIO_DEBUG
	required_device<screen_device> m_debug_screen;
#endif
	required_device<spg2xx_device> m_spg;
	optional_memory_bank m_bank;

protected:
	DECLARE_WRITE_LINE_MEMBER(poll_controls);

	DECLARE_READ16_MEMBER(walle_portc_r);
	DECLARE_WRITE16_MEMBER(walle_portc_w);

	virtual void machine_reset() override;

	virtual void mem_map(address_map &map);

	uint32_t m_current_bank;

	std::unique_ptr<uint8_t[]> m_serial_eeprom;
	uint8_t m_controller_input[8];
	uint8_t m_w60_controller_input;
	uint16_t m_w60_porta_data;

	uint16_t m_walle_portc_data;

	inline void verboselog(int n_level, const char *s_fmt, ...) ATTR_PRINTF(3, 4);

	required_ioport m_io_p1;
	optional_ioport m_io_p2;
	optional_ioport m_io_p3;
	optional_ioport m_io_motionx;
	optional_ioport m_io_motiony;
	optional_ioport m_io_motionz;
	optional_device<i2cmem_device> m_i2cmem;
	optional_device<nvram_device> m_nvram;

	// temp hack
	DECLARE_READ16_MEMBER(rad_crik_hack_r);
};

class vsmile_state : public spg2xx_game_state
{
public:
	vsmile_state(const machine_config &mconfig, device_type type, const char *tag)
		: spg2xx_game_state(mconfig, type, tag)
		, m_cart(*this, "cartslot")
		, m_bankdev(*this, "bank")
		, m_system_region(*this, "maincpu")
	{ }

	void vsmile(machine_config &config);
	void vsmilep(machine_config &config);

private:
	virtual void machine_start() override;
	virtual void machine_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	static const device_timer_id TIMER_PAD = 0;

	virtual void mem_map(address_map &map) override;
	void banked_map(address_map &map);

	DECLARE_DEVICE_IMAGE_LOAD_MEMBER(cart);

	DECLARE_READ16_MEMBER(portb_r);
	DECLARE_READ16_MEMBER(portc_r);
	DECLARE_WRITE16_MEMBER(portb_w);
	DECLARE_WRITE16_MEMBER(portc_w);

	DECLARE_WRITE8_MEMBER(chip_sel_w);

	DECLARE_WRITE8_MEMBER(uart_tx);

	DECLARE_READ16_MEMBER(bank0_r);
	DECLARE_READ16_MEMBER(bank1_r);
	DECLARE_READ16_MEMBER(bank2_r);
	DECLARE_READ16_MEMBER(bank3_r);
	optional_device<generic_slot_device> m_cart;
	required_device<address_map_bank_device> m_bankdev;
	memory_region *m_cart_region;
	required_memory_region m_system_region;

	emu_timer *m_pad_timer;
	uint8_t m_pad_counter;

	uint16_t m_portb_data;
	uint16_t m_portc_data;
};

class spg2xx_cart_state : public spg2xx_game_state
{
public:
	spg2xx_cart_state(const machine_config &mconfig, device_type type, const char *tag)
		: spg2xx_game_state(mconfig, type, tag)
		, m_cart(*this, "cartslot")
	{ }

	void vii(machine_config &config);

private:
	DECLARE_WRITE16_MEMBER(vii_portb_w);

	DECLARE_DEVICE_IMAGE_LOAD_MEMBER(vii_cart);

	virtual void machine_start() override;
	virtual void machine_reset() override;

	optional_device<generic_slot_device> m_cart;
	memory_region *m_cart_region;
};

#define VERBOSE_LEVEL   (4)

#define ENABLE_VERBOSE_LOG (1)

inline void spg2xx_game_state::verboselog(int n_level, const char *s_fmt, ...)
{
#if ENABLE_VERBOSE_LOG
	if (VERBOSE_LEVEL >= n_level)
	{
		va_list v;
		char buf[32768];
		va_start(v, s_fmt);
		vsprintf(buf, s_fmt, v);
		logerror("%s", buf);
		va_end(v);
	}
#endif
}

/*************************
*    Machine Hardware    *
*************************/

void spg2xx_game_state::switch_bank(uint32_t bank)
{
	if (bank != m_current_bank)
	{
		m_current_bank = bank;
		m_bank->set_entry(bank);
	}
}

WRITE8_MEMBER(spg2xx_game_state::eeprom_w)
{
	m_serial_eeprom[offset & 0x3ff] = data;
}

READ8_MEMBER(spg2xx_game_state::eeprom_r)
{
	return m_serial_eeprom[offset & 0x3ff];
}

WRITE16_MEMBER(spg2xx_game_state::wireless60_porta_w)
{
	m_w60_porta_data = data & 0xf00;
	switch (m_w60_porta_data & 0x300)
	{
	case 0x300:
		m_w60_controller_input = -1;
		break;

	case 0x200:
		m_w60_controller_input++;
		break;

	default:
		uint16_t temp1 = m_io_p1->read();
		uint16_t temp2 = m_io_p2->read();
		uint16_t temp3 = 1 << m_w60_controller_input;
		if (temp1 & temp3) m_w60_porta_data ^= 0x400;
		if (temp2 & temp3) m_w60_porta_data ^= 0x800;
		break;
	}
}

READ16_MEMBER(spg2xx_game_state::wireless60_porta_r)
{
	return m_w60_porta_data;
}

WRITE16_MEMBER(spg2xx_game_state::wireless60_portb_w)
{
	switch_bank(data & 7);
}

WRITE16_MEMBER(spg2xx_cart_state::vii_portb_w)
{
	if (data == 0x7c) machine().debug_break();
	switch_bank(((data & 0x80) >> 7) | ((data & 0x20) >> 4));
}

void vsmile_state::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	if (id == TIMER_PAD)
	{
		m_pad_counter++;
		if (m_pad_counter >= 100)
		{
			m_pad_counter = 0;
			//m_spg->uart_rx(0x55);
		}
	}
}

#define VSMILE_PORTB_CS1        0x0001
#define VSMILE_PORTB_CS2        0x0002
#define VSMILE_PORTB_CART       0x0004
#define VSMILE_PORTB_RESET      0x0008
#define VSMILE_PORTB_FRONT24    0x0010
#define VSMILE_PORTB_OFF        0x0020
#define VSMILE_PORTB_OFF_SW     0x0040
#define VSMILE_PORTB_ON_SW      0x0080

#define VSMILE_PORTC_VER        0x000f
#define VSMILE_PORTC_LOGO       0x0010
#define VSMILE_PORTC_TEST       0x0020
#define VSMILE_PORTC_AMP        0x0040
#define VSMILE_PORTC_SYSRESET   0x0080

READ16_MEMBER(vsmile_state::bank0_r)
{
	return ((uint16_t*)m_cart_region->base())[offset];
}

READ16_MEMBER(vsmile_state::bank1_r)
{
	return ((uint16_t*)m_cart_region->base())[offset + 0x100000];
}

READ16_MEMBER(vsmile_state::bank2_r)
{
	return ((uint16_t*)m_cart_region->base())[offset + 0x200000];
}

READ16_MEMBER(vsmile_state::bank3_r)
{
	return ((uint16_t*)m_system_region->base())[offset];
}

READ16_MEMBER(vsmile_state::portb_r)
{
	//const uint8_t inputs = m_io_p2->read();
	//const uint16_t input_bits = BIT(inputs, 0) ? VSMILE_PORTB_ON_SW : 0;
	//const uint16_t data = VSMILE_PORTB_ON_SW | VSMILE_PORTB_OFF_SW | (m_cart && m_cart->exists() ? VSMILE_PORTB_CART : 0);
	//logerror("V.Smile Port B read  %04x, mask %04x\n", data, mem_mask);
	//printf("V.Smile Port B read  %04x, mask %04x\n", data, mem_mask);
	return m_portb_data;// | data;
}

READ16_MEMBER(vsmile_state::portc_r)
{
	uint16_t data = 0x0004;
	if (m_portc_data & 0x0100)
		data |= 0x0400;
	if (m_portc_data & 0x0200)
		data |= 0x1000;
	//logerror("V.Smile Port C read  %04x, mask %04x\n", data, mem_mask);
	return (m_portc_data & ~0x000f) | data;
}

WRITE16_MEMBER(vsmile_state::portb_w)
{
	m_portb_data = data;//(m_portb_data &~ mem_mask) | (data & mem_mask);
	//logerror("V.Smile Port B write %04x, mask %04x\n", m_portb_data, mem_mask);
	//printf("V.Smile Port B write %04x, mask %04x\n", m_portb_data, mem_mask);
}

WRITE16_MEMBER(vsmile_state::portc_w)
{
	m_portc_data = data;//(m_portc_data &~ mem_mask) | (data & mem_mask);
	//logerror("V.Smile Port C write %04x, mask %04x\n", m_portc_data, mem_mask);
	//printf("V.Smile Port C write %04x, mask %04x\n", m_portc_data, mem_mask);
	//printf("%02x ", data >> 8);
}

WRITE8_MEMBER(vsmile_state::uart_tx)
{
	logerror("UART Tx: %02x\n", data);
}

WRITE8_MEMBER(vsmile_state::chip_sel_w)
{
	const uint16_t cart_offset = m_cart && m_cart->exists() ? 4 : 0;
	switch (data)
	{
		case 0:
			m_bankdev->set_bank(cart_offset);
			break;
		case 1:
			m_bankdev->set_bank(1 + cart_offset);
			break;
		case 2:
		case 3:
			m_bankdev->set_bank(2 + cart_offset);
			break;
	}
}

READ16_MEMBER(spg2xx_game_state::walle_portc_r)
{
	return m_i2cmem->read_sda();
}

WRITE16_MEMBER(spg2xx_game_state::walle_portc_w)
{
	m_walle_portc_data = data & mem_mask;
	if (BIT(mem_mask, 1))
		m_i2cmem->write_scl(BIT(data, 1));
	if (BIT(mem_mask, 0))
		m_i2cmem->write_sda(BIT(data, 0));
}

READ16_MEMBER(spg2xx_game_state::jakks_porta_r)
{
	const uint16_t temp = m_io_p1->read();
	uint16_t value = 0;
	value |= (temp & 0x0001) ? 0x8000 : 0;
	value |= (temp & 0x0002) ? 0x4000 : 0;
	value |= (temp & 0x0004) ? 0x2000 : 0;
	value |= (temp & 0x0008) ? 0x1000 : 0;
	value |= (temp & 0x0010) ? 0x0800 : 0;
	value |= (temp & 0x0020) ? 0x0400 : 0;
	value |= (temp & 0x0040) ? 0x0200 : 0;
	value |= (temp & 0x0080) ? 0x0100 : 0;
	return value;
}

void spg2xx_game_state::mem_map(address_map &map)
{
	map(0x000000, 0x3fffff).bankr("cartbank");
	map(0x000000, 0x003fff).m(m_spg, FUNC(spg2xx_device::map));
}

void vsmile_state::banked_map(address_map &map)
{
	map(0x0000000, 0x00fffff).rom().region("maincpu", 0);
	map(0x0100000, 0x01fffff).rom().region("maincpu", 0);
	map(0x0200000, 0x02fffff).rom().region("maincpu", 0);
	map(0x0300000, 0x03fffff).rom().region("maincpu", 0);

	map(0x0400000, 0x04fffff).rom().region("maincpu", 0);
	map(0x0500000, 0x05fffff).rom().region("maincpu", 0);
	map(0x0600000, 0x06fffff).rom().region("maincpu", 0);
	map(0x0700000, 0x07fffff).rom().region("maincpu", 0);

	map(0x0800000, 0x08fffff).rom().region("maincpu", 0);
	map(0x0900000, 0x09fffff).rom().region("maincpu", 0);
	map(0x0a00000, 0x0afffff).rom().region("maincpu", 0);
	map(0x0b00000, 0x0bfffff).rom().region("maincpu", 0);

	map(0x1000000, 0x13fffff).r(FUNC(vsmile_state::bank0_r));

	map(0x1400000, 0x15fffff).r(FUNC(vsmile_state::bank0_r));
	map(0x1600000, 0x17fffff).r(FUNC(vsmile_state::bank1_r));

	map(0x1800000, 0x18fffff).r(FUNC(vsmile_state::bank0_r));
	map(0x1900000, 0x19fffff).r(FUNC(vsmile_state::bank1_r));
	map(0x1a00000, 0x1afffff).nopr();
	map(0x1b00000, 0x1bfffff).r(FUNC(vsmile_state::bank3_r));
}

void vsmile_state::mem_map(address_map &map)
{
	map(0x000000, 0x3fffff).r(m_bankdev, FUNC(address_map_bank_device::read16));
	map(0x000000, 0x003fff).m(m_spg, FUNC(spg2xx_device::map));
}

static INPUT_PORTS_START( vii )
	PORT_START("P1")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP )    PORT_PLAYER(1) PORT_NAME("Joypad Up")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN )  PORT_PLAYER(1) PORT_NAME("Joypad Down")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT )  PORT_PLAYER(1) PORT_NAME("Joypad Left")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(1) PORT_NAME("Joypad Right")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_BUTTON1 )        PORT_PLAYER(1) PORT_NAME("Button A")
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_BUTTON2 )        PORT_PLAYER(1) PORT_NAME("Button B")
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_BUTTON3 )        PORT_PLAYER(1) PORT_NAME("Button C")
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_BUTTON4 )        PORT_PLAYER(1) PORT_NAME("Button D")

	PORT_START("MOTIONX")
	PORT_BIT( 0x3ff, 0x200, IPT_PADDLE ) PORT_MINMAX(0x000, 0x3ff) PORT_SENSITIVITY(50) PORT_KEYDELTA(50) PORT_NAME("Motion Control X")

	PORT_START("MOTIONY")
	PORT_BIT( 0x3ff, 0x200, IPT_PADDLE ) PORT_MINMAX(0x000, 0x3ff) PORT_SENSITIVITY(50) PORT_KEYDELTA(50) PORT_NAME("Motion Control Y") PORT_PLAYER(2)

	PORT_START("MOTIONZ")
	PORT_BIT( 0x3ff, 0x200, IPT_PADDLE ) PORT_MINMAX(0x000, 0x3ff) PORT_SENSITIVITY(50) PORT_KEYDELTA(50) PORT_NAME("Motion Control Z") PORT_PLAYER(3)
INPUT_PORTS_END

static INPUT_PORTS_START( batman )
	PORT_START("P1")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP )    PORT_PLAYER(1) PORT_NAME("Joypad Up")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN )  PORT_PLAYER(1) PORT_NAME("Joypad Down")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT )  PORT_PLAYER(1) PORT_NAME("Joypad Left")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(1) PORT_NAME("Joypad Right")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_BUTTON1 )        PORT_PLAYER(1) PORT_NAME("A Button")
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_BUTTON2 )        PORT_PLAYER(1) PORT_NAME("Menu")
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_BUTTON3 )        PORT_PLAYER(1) PORT_NAME("B Button")
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_BUTTON4 )        PORT_PLAYER(1) PORT_NAME("X Button")
INPUT_PORTS_END

static INPUT_PORTS_START( vsmile )
	PORT_START("P1")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP )    PORT_PLAYER(1) PORT_NAME("Joypad Up")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN )  PORT_PLAYER(1) PORT_NAME("Joypad Down")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT )  PORT_PLAYER(1) PORT_NAME("Joypad Left")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(1) PORT_NAME("Joypad Right")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_BUTTON1 )        PORT_PLAYER(1) PORT_NAME("A Button")
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_BUTTON2 )        PORT_PLAYER(1) PORT_NAME("Menu")
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_BUTTON3 )        PORT_PLAYER(1) PORT_NAME("B Button")
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_BUTTON4 )        PORT_PLAYER(1) PORT_NAME("X Button")

	PORT_START("P2")
	PORT_DIPNAME( 0x0001, 0x0001, "POWER ON" )
	PORT_DIPSETTING(      0x0001, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0002, 0x0002, "POWER OFF" )
	PORT_DIPSETTING(      0x0002, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
INPUT_PORTS_END

static INPUT_PORTS_START( walle )
	PORT_START("P1")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP )    PORT_PLAYER(1) PORT_NAME("Joypad Up")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN )  PORT_PLAYER(1) PORT_NAME("Joypad Down")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT )  PORT_PLAYER(1) PORT_NAME("Joypad Left")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(1) PORT_NAME("Joypad Right")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_BUTTON1 )        PORT_PLAYER(1) PORT_NAME("A Button")
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_BUTTON2 )        PORT_PLAYER(1) PORT_NAME("B Button")
INPUT_PORTS_END

static INPUT_PORTS_START( wirels60 )
	PORT_START("P1")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP )    PORT_PLAYER(1) PORT_NAME("Joypad Up")
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN )  PORT_PLAYER(1) PORT_NAME("Joypad Down")
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT )  PORT_PLAYER(1) PORT_NAME("Joypad Left")
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(1) PORT_NAME("Joypad Right")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_BUTTON1 )        PORT_PLAYER(1) PORT_NAME("A Button")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_BUTTON2 )        PORT_PLAYER(1) PORT_NAME("B Button")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_BUTTON3 )        PORT_PLAYER(1) PORT_NAME("Menu")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_BUTTON4 )        PORT_PLAYER(1) PORT_NAME("Start")

	PORT_START("P2")
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP )    PORT_PLAYER(2) PORT_NAME("Joypad Up")
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN )  PORT_PLAYER(2) PORT_NAME("Joypad Down")
	PORT_BIT( 0x40, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT )  PORT_PLAYER(2) PORT_NAME("Joypad Left")
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(2) PORT_NAME("Joypad Right")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_BUTTON1 )        PORT_PLAYER(2) PORT_NAME("A Button")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_BUTTON2 )        PORT_PLAYER(2) PORT_NAME("B Button")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_BUTTON3 )        PORT_PLAYER(2) PORT_NAME("Menu")
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_BUTTON4 )        PORT_PLAYER(2) PORT_NAME("Start")
INPUT_PORTS_END

static INPUT_PORTS_START( rad_skat )
	PORT_START("P1")
	PORT_BIT( 0x0001, IP_ACTIVE_LOW, IPT_BUTTON4 ) PORT_NAME("Full Left")
	PORT_BIT( 0x0002, IP_ACTIVE_LOW, IPT_BUTTON5 ) PORT_NAME("Full Right")
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_NAME("Slight Left") // you have to use this for the menus (eg trick lists)
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_NAME("Slight Right")
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_NAME("Front")
	PORT_BIT( 0x0020, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_NAME("Back")
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_UNUSED )
	// there only seem to be 3 buttons on the pad part, so presumably all the above are the skateboard, and below are the pad?
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_NAME("M Button")
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_NAME("X Button")
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_BUTTON3 ) PORT_NAME("O Button")
	PORT_BIT( 0xf800, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("P2")
	PORT_BIT( 0xffff, IP_ACTIVE_LOW, IPT_UNUSED ) // read but unused?

	PORT_START("P3")
	PORT_BIT( 0xffff, IP_ACTIVE_LOW, IPT_CUSTOM ) // NTSC (1) / PAL (0) flag
INPUT_PORTS_END

static INPUT_PORTS_START( rad_skatp )
	PORT_INCLUDE(rad_skat)

	PORT_MODIFY("P3")
	PORT_BIT( 0xffff, IP_ACTIVE_HIGH, IPT_CUSTOM ) // NTSC (1) / PAL (0) flag
INPUT_PORTS_END

static INPUT_PORTS_START( rad_sktv )
	/* how does the Scanner connect? probably some serial port with comms protocol, not IO ports?
	   internal test mode shows 'uart' ports (which currently fail)

	   To access internal test hold DOWN and BUTTON1 together on startup until a coloured screen appears.
	   To cycle through the tests again hold DOWN and press BUTTON1 */

	PORT_START("P1")
	PORT_DIPNAME( 0x0001, 0x0001, "IN0" )
	PORT_DIPSETTING(      0x0001, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0002, 0x0002, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0002, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_BIT( 0x0004, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT )
	PORT_BIT( 0x0008, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT )
	PORT_BIT( 0x0010, IP_ACTIVE_LOW, IPT_JOYSTICK_UP )
	PORT_DIPNAME( 0x0020, 0x0020, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0020, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0040, 0x0040, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0040, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_BUTTON1 )
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_BUTTON2 )
	PORT_DIPNAME( 0x0400, 0x0400, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0400, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0800, 0x0800, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0800, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x1000, 0x1000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x1000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x2000, 0x2000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x2000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x4000, 0x4000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x4000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x8000, 0x8000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x8000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )

	PORT_START("P2")
	PORT_DIPNAME( 0x0001, 0x0001, "IN1" )
	PORT_DIPSETTING(      0x0001, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0002, 0x0002, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0002, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0004, 0x0004, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0004, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0008, 0x0008, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0008, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0010, 0x0010, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0010, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0020, 0x0020, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0020, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0040, 0x0040, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0040, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0080, 0x0080, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0080, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0100, 0x0100, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0100, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0200, 0x0200, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0200, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0400, 0x0400, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0400, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0800, 0x0800, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0800, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x1000, 0x1000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x1000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x2000, 0x2000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x2000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x4000, 0x4000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x4000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x8000, 0x8000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x8000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
INPUT_PORTS_END

/* hold 'Console Down' while powering up to get the test menu, including input tests
   the ball (Wired) and bat (IR) are read some other way as they don't seem to appear in the ports. */
static INPUT_PORTS_START( rad_crik )
	PORT_START("P1")
	PORT_BIT( 0x003f, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_NAME("Console Enter") // these are the controls on the base unit
	PORT_BIT( 0x0080, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_NAME("Console Down")
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_NAME("Console Left")
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_NAME("Console Right")
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_NAME("Console Up")
	PORT_BIT( 0xf800, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("P2")
	PORT_BIT( 0xffff, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("P3")
	PORT_BIT( 0xffff, IP_ACTIVE_LOW, IPT_UNKNOWN )
INPUT_PORTS_END

void spg2xx_cart_state::machine_start()
{
	spg2xx_game_state::machine_start();

	// if there's a cart, override the standard banking
	if (m_cart && m_cart->exists())
	{
		std::string region_tag;
		m_cart_region = memregion(region_tag.assign(m_cart->tag()).append(GENERIC_ROM_REGION_TAG).c_str());
		m_bank->configure_entries(0, ceilf((float)m_cart_region->bytes() / 0x800000), m_cart_region->base(), 0x800000);
		m_bank->set_entry(0);
	}
}

void vsmile_state::machine_start()
{
	// if there's a cart, override the standard banking
	if (m_cart && m_cart->exists())
	{
		std::string region_tag;
		m_cart_region = memregion(region_tag.assign(m_cart->tag()).append(GENERIC_ROM_REGION_TAG).c_str());
	}

	m_bankdev->set_bank(m_cart && m_cart->exists() ? 4 : 0);

	m_serial_eeprom = std::make_unique<uint8_t[]>(0x400);
	if (m_nvram)
		m_nvram->set_base(&m_serial_eeprom[0], 0x400);

	m_pad_timer = timer_alloc(TIMER_PAD);
	m_pad_timer->adjust(attotime::never);
}

void vsmile_state::machine_reset()
{
	m_portb_data = 0;
	m_portc_data = 0;

	m_pad_timer->adjust(attotime::from_hz(100), 0, attotime::from_hz(100));
	m_pad_counter = 0;
}

void spg2xx_cart_state::machine_reset()
{
	m_controller_input[0] = 0;
	m_controller_input[4] = 0;
	m_controller_input[6] = 0xff;
	m_controller_input[7] = 0;
}

void spg2xx_game_state::machine_start()
{
	m_bank->configure_entries(0, ceilf((float)memregion("maincpu")->bytes() / 0x800000), memregion("maincpu")->base(), 0x800000);
	m_bank->set_entry(0);

	m_serial_eeprom = std::make_unique<uint8_t[]>(0x400);
	if (m_nvram)
		m_nvram->set_base(&m_serial_eeprom[0], 0x400);
}

void spg2xx_game_state::machine_reset()
{
	m_current_bank = 0;

	m_w60_controller_input = -1;
	m_w60_porta_data = 0;
}

WRITE_LINE_MEMBER(spg2xx_game_state::poll_controls)
{
	if (!state)
		return;

	int32_t x = m_io_motionx ? ((int32_t)m_io_motionx->read() - 0x200) : 0;
	int32_t y = m_io_motiony ? ((int32_t)m_io_motiony->read() - 0x200) : 0;
	int32_t z = m_io_motionz ? ((int32_t)m_io_motionz->read() - 0x200) : 0;

	uint8_t old_input[8];
	memcpy(old_input, m_controller_input, 8);

	m_controller_input[0] = m_io_p1->read();
	m_controller_input[1] = (uint8_t)x;
	m_controller_input[2] = (uint8_t)y;
	m_controller_input[3] = (uint8_t)z;
	m_controller_input[4] = 0;
	x = (x >> 8) & 3;
	y = (y >> 8) & 3;
	z = (z >> 8) & 3;
	m_controller_input[5] = (z << 4) | (y << 2) | x;
	m_controller_input[6] = 0xff;
	m_controller_input[7] = 0;

	if (memcmp(old_input, m_controller_input, 8))
	{
		for(int i = 0; i < 8; i++)
			m_spg->uart_rx(m_controller_input[i]);
	}
}

DEVICE_IMAGE_LOAD_MEMBER(spg2xx_cart_state, vii_cart)
{
	uint32_t size = m_cart->common_get_size("rom");

	if (size < 0x800000)
	{
		image.seterror(IMAGE_ERROR_UNSPECIFIED, "Unsupported cartridge size");
		return image_init_result::FAIL;
	}

	m_cart->rom_alloc(size, GENERIC_ROM16_WIDTH, ENDIANNESS_LITTLE);
	m_cart->common_load_rom(m_cart->get_rom_base(), size, "rom");

	return image_init_result::PASS;
}

DEVICE_IMAGE_LOAD_MEMBER(vsmile_state, cart)
{
	uint32_t size = m_cart->common_get_size("rom");

	m_cart->rom_alloc(size, GENERIC_ROM16_WIDTH, ENDIANNESS_LITTLE);
	m_cart->common_load_rom(m_cart->get_rom_base(), size, "rom");

	return image_init_result::PASS;
}

void spg2xx_game_state::spg2xx_base(machine_config &config)
{
	UNSP(config, m_maincpu, XTAL(27'000'000));
	m_maincpu->set_addrmap(AS_PROGRAM, &spg2xx_game_state::mem_map);

	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	m_screen->set_refresh_hz(60);
	m_screen->set_size(320, 262);
	m_screen->set_visarea(0, 320-1, 0, 240-1);
	m_screen->set_screen_update("spg", FUNC(spg2xx_device::screen_update));
	m_screen->screen_vblank().set(m_spg, FUNC(spg2xx_device::vblank));

#if SPG2XX_VISUAL_AUDIO_DEBUG
	SCREEN(config, m_debug_screen, SCREEN_TYPE_RASTER);
	m_debug_screen->set_refresh_hz(60);
	m_debug_screen->set_size(1024, 768);
	m_debug_screen->set_visarea(0, 1024-1, 0, 768-1);
	m_debug_screen->set_screen_update("spg", FUNC(spg2xx_device::debug_screen_update));
#endif

	SPEAKER(config, "lspeaker").front_left();
	SPEAKER(config, "rspeaker").front_right();
	m_spg->add_route(ALL_OUTPUTS, "lspeaker", 0.5);
	m_spg->add_route(ALL_OUTPUTS, "rspeaker", 0.5);
}

void spg2xx_game_state::non_spg_base(machine_config &config)
{
#if SPG2XX_VISUAL_AUDIO_DEBUG
	SPG24X(config, m_spg, XTAL(27'000'000), m_maincpu, m_screen, m_debug_screen);
#else
	SPG24X(config, m_spg, XTAL(27'000'000), m_maincpu, m_screen);
#endif

	spg2xx_base(config);
}

void spg2xx_game_state::spg2xx_basep(machine_config &config)
{
	spg2xx_base(config);

	m_screen->set_refresh_hz(50);
	m_screen->set_size(320, 312);
}

void spg2xx_cart_state::vii(machine_config &config)
{
#if SPG2XX_VISUAL_AUDIO_DEBUG
	SPG24X(config, m_spg, XTAL(27'000'000), m_maincpu, m_screen, m_debug_screen);
#else
	SPG24X(config, m_spg, XTAL(27'000'000), m_maincpu, m_screen);
#endif

	spg2xx_base(config);
	m_screen->screen_vblank().append(FUNC(spg2xx_cart_state::poll_controls));

	m_spg->portb_out().set(FUNC(spg2xx_cart_state::vii_portb_w));
	m_spg->eeprom_w().set(FUNC(spg2xx_cart_state::eeprom_w));
	m_spg->eeprom_r().set(FUNC(spg2xx_cart_state::eeprom_r));

	NVRAM(config, m_nvram, nvram_device::DEFAULT_ALL_1);

	GENERIC_CARTSLOT(config, m_cart, generic_plain_slot, "vii_cart");
	m_cart->set_width(GENERIC_ROM16_WIDTH);
	m_cart->set_device_load(device_image_load_delegate(&spg2xx_cart_state::device_image_load_vii_cart, this));

	SOFTWARE_LIST(config, "vii_cart").set_original("vii");
}

void vsmile_state::vsmile(machine_config &config)
{
#if SPG2XX_VISUAL_AUDIO_DEBUG
	SPG24X(config, m_spg, XTAL(27'000'000), m_maincpu, m_screen, m_debug_screen);
#else
	SPG24X(config, m_spg, XTAL(27'000'000), m_maincpu, m_screen);
#endif
	spg2xx_base(config);
	m_maincpu->set_addrmap(AS_PROGRAM, &vsmile_state::mem_map);

	m_spg->portb_in().set(FUNC(vsmile_state::portb_r));
	m_spg->portc_in().set(FUNC(vsmile_state::portc_r));
	m_spg->portb_out().set(FUNC(vsmile_state::portb_w));
	m_spg->portc_out().set(FUNC(vsmile_state::portc_w));
	m_spg->chip_select().set(FUNC(vsmile_state::chip_sel_w));
	m_spg->uart_tx().set(FUNC(vsmile_state::uart_tx));

	GENERIC_CARTSLOT(config, m_cart, generic_plain_slot, "vsmile_cart");
	m_cart->set_width(GENERIC_ROM16_WIDTH);
	m_cart->set_device_load(device_image_load_delegate(&vsmile_state::device_image_load_cart, this));

	ADDRESS_MAP_BANK(config, m_bankdev);
	m_bankdev->set_addrmap(AS_PROGRAM, &vsmile_state::banked_map);
	m_bankdev->set_endianness(ENDIANNESS_LITTLE);
	m_bankdev->set_data_width(16);
	m_bankdev->set_shift(-1);
	m_bankdev->set_stride(0x400000);

	SOFTWARE_LIST(config, "cart_list").set_original("vsmile_cart");
}

void vsmile_state::vsmilep(machine_config &config)
{
	vsmile(config);
	m_spg->set_pal(true);
}

void spg2xx_game_state::wireless60(machine_config &config)
{
#if SPG2XX_VISUAL_AUDIO_DEBUG
	SPG24X(config, m_spg, XTAL(27'000'000), m_maincpu, m_screen, m_debug_screen);
#else
	SPG24X(config, m_spg, XTAL(27'000'000), m_maincpu, m_screen);
#endif
	spg2xx_base(config);
	m_screen->screen_vblank().append(FUNC(spg2xx_game_state::poll_controls));

	m_spg->porta_out().set(FUNC(spg2xx_game_state::wireless60_porta_w));
	m_spg->portb_out().set(FUNC(spg2xx_game_state::wireless60_portb_w));
	m_spg->porta_in().set(FUNC(spg2xx_game_state::wireless60_porta_r));
}

void spg2xx_game_state::jakks(machine_config &config)
{
#if SPG2XX_VISUAL_AUDIO_DEBUG
	SPG24X(config, m_spg, XTAL(27'000'000), m_maincpu, m_screen, m_debug_screen);
#else
	SPG24X(config, m_spg, XTAL(27'000'000), m_maincpu, m_screen);
#endif
	spg2xx_base(config);

	m_spg->porta_in().set(FUNC(spg2xx_cart_state::jakks_porta_r));

	I2CMEM(config, m_i2cmem, 0).set_data_size(0x200);
}

void spg2xx_game_state::walle(machine_config &config)
{
	jakks(config);

	m_spg->portc_in().set(FUNC(spg2xx_game_state::walle_portc_r));
	m_spg->portc_out().set(FUNC(spg2xx_game_state::walle_portc_w));
}

void spg2xx_game_state::rad_skat(machine_config &config)
{
#if SPG2XX_VISUAL_AUDIO_DEBUG
	SPG24X(config, m_spg, XTAL(27'000'000), m_maincpu, m_screen, m_debug_screen);
#else
	SPG24X(config, m_spg, XTAL(27'000'000), m_maincpu, m_screen);
#endif
	spg2xx_base(config);

	m_spg->porta_in().set_ioport("P1");
	m_spg->portb_in().set_ioport("P2");
	m_spg->portc_in().set_ioport("P3");
	m_spg->eeprom_w().set(FUNC(spg2xx_game_state::eeprom_w));
	m_spg->eeprom_r().set(FUNC(spg2xx_game_state::eeprom_r));

	NVRAM(config, m_nvram, nvram_device::DEFAULT_ALL_1);
}

void spg2xx_game_state::rad_skatp(machine_config &config)
{
	rad_skat(config);
	m_spg->set_pal(true);
}

void spg2xx_game_state::rad_crik(machine_config &config)
{
#if SPG2XX_VISUAL_AUDIO_DEBUG
	SPG28X(config, m_spg, XTAL(27'000'000), m_maincpu, m_screen, m_debug_screen);
#else
	SPG28X(config, m_spg, XTAL(27'000'000), m_maincpu, m_screen);
#endif
	spg2xx_base(config);

	m_spg->porta_in().set_ioport("P1");
	m_spg->portb_in().set_ioport("P2");
	m_spg->portc_in().set_ioport("P3");
	m_spg->eeprom_w().set(FUNC(spg2xx_game_state::eeprom_w));
	m_spg->eeprom_r().set(FUNC(spg2xx_game_state::eeprom_r));

	NVRAM(config, m_nvram, nvram_device::DEFAULT_ALL_1);
}

READ16_MEMBER(spg2xx_game_state::rad_crik_hack_r)
{
	int pc = m_maincpu->state_int(UNSP_PC);
	if (pc == 0xf851)
		return 0xf859;
	else
		return 0xf854;
}

void spg2xx_game_state::init_rad_crik()
{
	// not 100% sure what this is waiting on, could be eeprom as it seems to end up here frequently during the eeprom test, patch running code, not ROM, so that checksum can still pass
	m_maincpu->space(AS_PROGRAM).install_read_handler(0xf851, 0xf851, read16_delegate(FUNC(spg2xx_game_state::rad_crik_hack_r),this));
}

ROM_START( vii )
	ROM_REGION( 0x2000000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD16_WORD_SWAP( "vii.bin", 0x0000, 0x2000000, CRC(04627639) SHA1(f883a92d31b53c9a5b0cdb112d07cd793c95fc43))
ROM_END

ROM_START( batmantv )
	ROM_REGION( 0x800000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD16_WORD_SWAP( "batman.bin", 0x000000, 0x400000, CRC(46f848e5) SHA1(5875d57bb3fe0cac5d20e626e4f82a0e5f9bb94c) )
ROM_END

ROM_START( vsmile )
	ROM_REGION( 0x800000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD( "vsmilebios.bin", 0x000000, 0x200000, CRC(11f1b416) SHA1(11f77c4973d29c962567390e41879c86a759c93b) )
ROM_END

ROM_START( vsmileg )
	ROM_REGION( 0x800000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD16_WORD_SWAP( "bios german.bin", 0x000000, 0x200000, CRC(205c5296) SHA1(7fbcf761b5885c8b1524607aabaf364b4559c8cc) )
ROM_END

ROM_START( vsmilef )
	ROM_REGION( 0x800000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD16_WORD_SWAP( "sysrom_france", 0x000000, 0x200000, CRC(0cd0bdf5) SHA1(5c8d1eada1b6b545555b8d2b09325d7127681af8) )
ROM_END

ROM_START( vsmileb )
	ROM_REGION( 0x800000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD( "vbabybios.bin", 0x000000, 0x800000, CRC(ddc7f845) SHA1(2c17d0f54200070176d03d44a40c7923636e596a) )
ROM_END

ROM_START( walle )
	ROM_REGION( 0x800000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD16_WORD_SWAP( "walle.bin", 0x000000, 0x400000, BAD_DUMP CRC(bd554cba) SHA1(6cd06a036ab12e7b0e1fd8003db873b0bb783868) )
	// Alternate dump, we need to decide which one is correct.
	//ROM_LOAD16_WORD_SWAP( "walle.bin", 0x000000, 0x400000, CRC(6bc90b16) SHA1(184d72de059057aae7800da510fcf05ed1da9ec9))
ROM_END

ROM_START( zone40 )
	ROM_REGION( 0x4000000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD16_WORD_SWAP( "zone40.bin", 0x0000, 0x4000000, CRC(4ba1444f) SHA1(de83046ab93421486668a247972ad6d3cda19440) )
ROM_END

ROM_START( zone60 )
	ROM_REGION( 0x4000000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD16_WORD_SWAP( "zone60.bin", 0x0000, 0x4000000, CRC(4cb637d1) SHA1(1f97cbdb4299ac0fbafc2a3aa592066cb0727066))
ROM_END

ROM_START( wirels60 )
	ROM_REGION( 0x4000000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD16_WORD_SWAP( "wirels60.bin", 0x0000, 0x4000000, CRC(b4df8b28) SHA1(00e3da542e4bc14baf4724ad436f66d4c0f65c84))
ROM_END

ROM_START( rad_skat )
	ROM_REGION( 0x800000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD16_WORD_SWAP( "skateboarder.bin", 0x000000, 0x400000, CRC(08b9ab91) SHA1(6665edc4740804956136c68065890925a144626b) )
ROM_END

ROM_START( rad_skatp ) // rom was dumped from the NTSC version, but region comes from an io port, so ROM is probably the same
	ROM_REGION( 0x800000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD16_WORD_SWAP( "skateboarder.bin", 0x000000, 0x400000, CRC(08b9ab91) SHA1(6665edc4740804956136c68065890925a144626b) )
ROM_END

ROM_START( rad_sktv )
	ROM_REGION( 0x800000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD16_WORD_SWAP( "skannerztv.bin", 0x000000, 0x200000, CRC(e92278e3) SHA1(eb6bee5e661128d83784960dfff50379c36bfaeb) )

	/* The external scanner MCU is a Winbond from 2000: SA5641
	   the scanner plays sound effects when scanning, without being connected to the main unit, so a way to dump / emulate
	   this MCU is also needed for complete emulation

	   TODO: find details on MCU so that we know capacity etc. */
ROM_END

ROM_START( rad_crik ) // only released in EU?
	ROM_REGION( 0x800000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD16_WORD_SWAP( "cricket.bin", 0x000000, 0x200000, CRC(6fa0aaa9) SHA1(210d2d4f542181f59127ce2f516d0408dc6de7a8) )
ROM_END

/*
Wireless Air 60
(info provided with dump)

System: Wireless Air 60
ROM: Toshiba TC58NVG0S3ETA00
RAM: ESMT M12L128168A

This is a raw NAND flash dump

Interesting Strings:

GPnandnand; (GP is General Plus, which is Sunplus by another name)
GLB_GP-F_5B_USBD_1.0.0
SP_ToneMaker
GLB_GP-FS1_0405L_SPU_1.0.2.3
SPF2ALP

"GPnandnand" as a required signature appears to be referenced right here, in page 19 of a GeneralPlus document;
http://www.lcis.com.tw/paper_store/paper_store/GPL162004A-507A_162005A-707AV10_code_reference-20147131205102.pdf

*/

ROM_START( wlsair60 )
	ROM_REGION( 0x8400000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD16_WORD_SWAP( "wlsair60.nand", 0x0000, 0x8400000, CRC(eec23b97) SHA1(1bb88290cf54579a5bb51c08a02d793cd4d79f7a) )
ROM_END

/*
Wireless Hunting Video Game System
(info provided with dump)

System: Wireless Hunting Video Game System
Publisher: Hamy / Kids Station Toys Inc
Year: 2011
ROM: FDI MSP55LV100G
RAM: Micron Technology 48LC8M16A2

Games:

Secret Mission
Predator
Delta Force
Toy Land
Dream Forest
Trophy Season
Freedom Force
Be Careful
Net Power
Open Training
Super Archer
Ultimate Frisbee
UFO Shooting
Happy Darts
Balloon Shoot
Avatair
Angry Pirate
Penguin War
Ghost Shooter
Duck Hunt


ROM Board:

Package: SO44
Spacing: 1.27 mm
Width: 16.14 mm
Length: 27.78 mm
Voltage: 3V
Pinout:

          A25  A24
            |  |
      +--------------------------+
A21 --|==   #  # `.__.'        ==|-- A20
A18 --|==                      ==|-- A19
A17 --|==                      ==|-- A8
 A7 --|==                      ==|-- A9
 A6 --|==                  o   ==|-- A10
 A5 --|==  +----------------+  ==|-- A11
 A4 --|==  |                |  ==|-- A12
 A3 --|==  |  MSP55LV100G   |  ==|-- A13
 A2 --|==  |  0834 M02H     |  ==|-- A14
 A1 --|==  |  JAPAN         |  ==|-- A15
 A0 --|==  |                |  ==|-- A16
#CE --|==  |                |  ==|-- A23
GND --|==  |                |  ==|-- A22
#OE --|==  |                |  ==|-- Q15
 Q0 --|==  |                |  ==|-- Q7
 Q8 --|==  |                |  ==|-- Q14
 Q1 --|==  +----------------+  ==|-- Q6
 Q9 --|==                      ==|-- Q13
 Q2 --|==       M55L100G       ==|-- Q5
Q10 --|==                      ==|-- Q12
 Q3 --|==                      ==|-- Q4
Q11 --|==                      ==|-- VCC
      +--------------------------+


The only interesting string in this ROM is SPF2ALP,
which is also found in the Wireless Air 60 ROM.

*/

ROM_START( wrlshunt )
	ROM_REGION( 0x8000000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD16_WORD_SWAP( "wireless.bin", 0x0000, 0x8000000, CRC(a6ecc20e) SHA1(3645f23ba2bb218e92d4560a8ae29dddbaabf796) )
ROM_END

// year, name, parent, compat, machine, input, class, init, company, fullname, flags

// VTech systems
CONS( 2005, vsmile,  0,      0, vsmile,  vsmile, vsmile_state, empty_init, "VTech", "V.Smile (US)",      MACHINE_IMPERFECT_SOUND | MACHINE_IMPERFECT_GRAPHICS | MACHINE_NOT_WORKING )
CONS( 2005, vsmileg, vsmile, 0, vsmilep, vsmile, vsmile_state, empty_init, "VTech", "V.Smile (Germany)", MACHINE_IMPERFECT_SOUND | MACHINE_IMPERFECT_GRAPHICS | MACHINE_NOT_WORKING )
CONS( 2005, vsmilef, vsmile, 0, vsmilep, vsmile, vsmile_state, empty_init, "VTech", "V.Smile (France)",  MACHINE_IMPERFECT_SOUND | MACHINE_IMPERFECT_GRAPHICS | MACHINE_NOT_WORKING )
CONS( 2005, vsmileb, 0,      0, vsmile,  vsmile, vsmile_state, empty_init, "VTech", "V.Smile Baby (US)", MACHINE_IMPERFECT_SOUND | MACHINE_IMPERFECT_GRAPHICS | MACHINE_NOT_WORKING )

// Jungle Soft TV games
CONS( 2007, vii,      0, 0, vii,        vii,      spg2xx_cart_state, empty_init, "Jungle Soft / KenSingTon / Siatronics",    "Vii",         MACHINE_IMPERFECT_SOUND | MACHINE_IMPERFECT_GRAPHICS ) // motion controls are awkward, but playable for the most part
CONS( 2010, zone60,   0, 0, wireless60, wirels60, spg2xx_game_state, empty_init, "Jungle's Soft / Ultimate Products (HK) Ltd", "Zone 60",     MACHINE_IMPERFECT_SOUND | MACHINE_IMPERFECT_GRAPHICS )
CONS( 2010, wirels60, 0, 0, wireless60, wirels60, spg2xx_game_state, empty_init, "Jungle Soft / Kids Station Toys Inc",      "Wireless 60", MACHINE_IMPERFECT_SOUND | MACHINE_IMPERFECT_GRAPHICS )

// JAKKS Pacific Inc TV games
CONS( 2004, batmantv, 0, 0, jakks, batman, spg2xx_game_state, empty_init, "JAKKS Pacific Inc / HotGen Ltd", "The Batman", MACHINE_IMPERFECT_SOUND | MACHINE_IMPERFECT_GRAPHICS )
CONS( 2008, walle,    0, 0, walle, walle,  spg2xx_game_state, empty_init, "JAKKS Pacific Inc",              "Wall-E",     MACHINE_IMPERFECT_SOUND | MACHINE_IMPERFECT_GRAPHICS )

// Radica TV games
CONS( 2006, rad_skat,  0,        0, rad_skat, rad_skat,   spg2xx_game_state, empty_init, "Radica", "Play TV Skateboarder (NTSC)", MACHINE_IMPERFECT_SOUND | MACHINE_IMPERFECT_GRAPHICS )
CONS( 2006, rad_skatp, rad_skat, 0, rad_skatp, rad_skatp, spg2xx_game_state, empty_init, "Radica", "Connectv Skateboarder (PAL)", MACHINE_IMPERFECT_SOUND | MACHINE_IMPERFECT_GRAPHICS )
CONS( 2006, rad_crik,  0,        0, rad_crik, rad_crik,   spg2xx_game_state, empty_init, "Radica", "Connectv Cricket (PAL)",      MACHINE_IMPERFECT_SOUND | MACHINE_NOT_WORKING ) // Version 3.00 20/03/06 is listed in INTERNAL TEST
CONS( 2007, rad_sktv,  0,        0, rad_skat, rad_sktv,   spg2xx_game_state, empty_init, "Radica", "Skannerz TV",                 MACHINE_IMPERFECT_SOUND | MACHINE_NOT_WORKING )

// might not fit here.  First 0x8000 bytes are blank (not too uncommon for these) then rest of rom looks like it's probably encrypted at least
CONS( 2009, zone40,    0,       0,        non_spg_base, wirels60, spg2xx_game_state, empty_init, "Jungle Soft / Ultimate Products (HK) Ltd",          "Zone 40",           MACHINE_NO_SOUND | MACHINE_NOT_WORKING )

// NAND dumps w/ internal bootstrap. Almost certainly do not fit in this driver, as the SPG2xx can only address up to 4Mwords.
CONS( 2010, wlsair60,  0,       0,        non_spg_base, wirels60, spg2xx_game_state, empty_init, "Jungle Soft / Kids Station Toys Inc",               "Wireless Air 60",   MACHINE_NO_SOUND | MACHINE_NOT_WORKING )
CONS( 2011, wrlshunt,  0,       0,        non_spg_base, wirels60, spg2xx_game_state, empty_init, "Hamy / Kids Station Toys Inc",                      "Wireless Hunting Video Game System", MACHINE_NO_SOUND | MACHINE_NOT_WORKING )
