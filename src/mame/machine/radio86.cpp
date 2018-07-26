// license:BSD-3-Clause
// copyright-holders:Miodrag Milanovic
/***************************************************************************

        Radio-86RK machine driver by Miodrag Milanovic

        06/03/2008 Preliminary driver.

****************************************************************************/


#include "emu.h"
#include "cpu/i8085/i8085.h"
#include "machine/i8255.h"
#include "includes/radio86.h"



void radio86_state::radio86_init_keyboard()
{
	m_keyboard_mask = 0;
	m_tape_value = 0x10;
}

/* Driver initialization */
void radio86_state::init_radio86()
{
	/* set initialy ROM to be visible on first bank */
	uint8_t *RAM = m_region_maincpu->base();
	memset(RAM,0x0000,0x1000); // make frist page empty by default
	m_bank1->configure_entries(1, 2, RAM, 0x0000);
	m_bank1->configure_entries(0, 2, RAM, 0xf800);
	radio86_init_keyboard();
}

void radio86_state::init_radioram()
{
	init_radio86();
	m_radio_ram_disk = std::make_unique<uint8_t[]>(0x20000);
	memset(m_radio_ram_disk.get(),0,0x20000);
}

READ8_MEMBER(radio86_state::radio86_8255_portb_r2)
{
	uint8_t key = 0xff;
	for (int i = 0; i < 8; i++)
	{
		if (BIT(m_keyboard_mask, i)) {
			key &= m_io_line[i]->read();
		}
	}
	return key;
}

READ8_MEMBER(radio86_state::kr03_8255_portb_r2)
{
	uint8_t key = 0xff;
	uint16_t data;
	for (int i = 0; i < 8; i++)
	{
		if (BIT(m_keyboard_mask, i))
		{
			data = m_io_line[i]->read();
			if (!BIT(data, 8)) data &= ~3;
			if (!BIT(data, 9)) data &= ~5;
			if (!BIT(data, 10)) data &= ~7;
			key &= data;
		}
	}
	return key;
}

READ8_MEMBER(radio86_state::radio86_8255_portc_r2)
{
	double level = m_cassette->input();
	uint8_t dat = m_io_line[8]->read();
	if (level <  0) {
		dat ^= m_tape_value;
	}
	return dat;
}

WRITE8_MEMBER(radio86_state::radio86_8255_porta_w2)
{
	m_keyboard_mask = data ^ 0xff;
}

WRITE8_MEMBER(radio86_state::radio86_8255_portc_w2)
{
	m_cassette->output(data & 0x01 ? 1 : -1);
}


READ8_MEMBER(radio86_state::rk7007_8255_portc_r)
{
	double level = m_cassette->input();
	uint8_t key = 0xff;
	for (int i = 0; i < 8; i++)
	{
		if ((m_keyboard_mask & (1 << i))!=0) {
			key &= m_io_cline[i]->read();
		}
	}
	key &= 0xe0;
	if (level <  0) {
		key ^= m_tape_value;
	}
	return key;
}

WRITE_LINE_MEMBER(radio86_state::hrq_w)
{
	/* HACK - this should be connected to the BUSREQ line of Z80 */
	m_maincpu->set_input_line(INPUT_LINE_HALT, state);

	/* HACK - this should be connected to the BUSACK line of Z80 */
	m_dma8257->hlda_w(state);
}

READ8_MEMBER(radio86_state::memory_read_byte)
{
	address_space& prog_space = m_maincpu->space(AS_PROGRAM);
	return prog_space.read_byte(offset);
}

WRITE8_MEMBER(radio86_state::memory_write_byte)
{
	address_space& prog_space = m_maincpu->space(AS_PROGRAM);
	return prog_space.write_byte(offset, data);
}

void radio86_state::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	switch (id)
	{
	case TIMER_RESET:
		m_bank1->set_entry(0);
		break;
	default:
		assert_always(false, "Unknown id in radio86_state::device_timer");
	}
}


READ8_MEMBER(radio86_state::radio_cpu_state_r)
{
	// FIXME: the driver should handler the status callback rather than accessing this through the state interface
	return m_maincpu->state_int(i8080_cpu_device::I8085_STATUS);
}

READ8_MEMBER(radio86_state::radio_io_r)
{
	return m_maincpu->space(AS_PROGRAM).read_byte((offset << 8) + offset);
}

WRITE8_MEMBER(radio86_state::radio_io_w)
{
	m_maincpu->space(AS_PROGRAM).write_byte((offset << 8) + offset,data);
}

MACHINE_RESET_MEMBER(radio86_state,radio86)
{
	timer_set(attotime::from_usec(10), TIMER_RESET);
	m_bank1->set_entry(1);

	m_keyboard_mask = 0;
	m_disk_sel = 0;
}


WRITE8_MEMBER(radio86_state::radio86_pagesel)
{
	m_disk_sel = data;
}

READ8_MEMBER(radio86_state::radio86rom_romdisk_porta_r)
{
	uint16_t addr = (m_romdisk_msb << 8) | m_romdisk_lsb;
	if (m_cart->exists() && addr < m_cart->get_rom_size())
		return m_cart->read_rom(space, addr);
	else
		return 0xff;
}

READ8_MEMBER(radio86_state::radio86ram_romdisk_porta_r)
{
	uint8_t *romdisk = m_region_maincpu->base() + 0x10000;
	if ((m_disk_sel & 0x0f) ==0) {
		return romdisk[m_romdisk_msb*256+m_romdisk_lsb];
	} else {
		if (m_disk_sel==0xdf) {
			return m_radio_ram_disk[m_romdisk_msb*256+m_romdisk_lsb + 0x10000];
		} else {
			return m_radio_ram_disk[m_romdisk_msb*256+m_romdisk_lsb];
		}
	}
}

WRITE8_MEMBER(radio86_state::radio86_romdisk_portb_w)
{
	m_romdisk_lsb = data;
}

WRITE8_MEMBER(radio86_state::radio86_romdisk_portc_w)
{
	m_romdisk_msb = data;
}

WRITE8_MEMBER(radio86_state::mikrosha_8255_font_page_w)
{
	m_mikrosha_font_page = (data  > 7) & 1;
}

I8275_DRAW_CHARACTER_MEMBER(radio86_state::display_pixels)
{
	int i;
	const rgb_t *palette = m_palette->palette()->entry_list_raw();
	const uint8_t *charmap = m_charmap;
	uint8_t pixels = charmap[(linecount & 7) + (charcode << 3)] ^ 0xff;
	if (vsp) {
		pixels = 0;
	}
	if (lten) {
		pixels = 0xff;
	}
	if (rvv) {
		pixels ^= 0xff;
	}
	for(i=0;i<6;i++) {
		bitmap.pix32(y, x + i) = palette[(pixels >> (5-i)) & 1 ? (hlgt ? 2 : 1) : 0];
	}
}

static const rgb_t radio86_palette[3] = {
	rgb_t(0x00, 0x00, 0x00), // black
	rgb_t(0xa0, 0xa0, 0xa0), // white
	rgb_t(0xff, 0xff, 0xff)  // highlight
};

PALETTE_INIT_MEMBER(radio86_state,radio86)
{
	palette.set_pen_colors(0, radio86_palette, ARRAY_LENGTH(radio86_palette));
}

void radio86_state::video_start()
{
	m_charmap = memregion("gfx1")->base();
}
