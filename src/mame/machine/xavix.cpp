// license:BSD-3-Clause
// copyright-holders:David Haywood

#include "emu.h"
#include "includes/xavix.h"

// #define VERBOSE 1
#include "logmacro.h"

// general DMA to/from entire main map (not dedicated sprite DMA)
WRITE8_MEMBER(xavix_state::rom_dmatrg_w)
{
	// 0x80 is set in the IRQ routine, presumably to ack it
	if (data & 0x80)
	{
		if (m_irqsource & 0x20)
		{
			m_irqsource &= ~0x20;
			update_irqs();
		}
	}

	if (data & 0x01) // namcons2 writes 0x81, most of the time things write 0x01
	{
		LOG("%s: rom_dmatrg_w (do DMA?) %02x\n", machine().describe_context(), data);

		uint32_t source = (m_rom_dma_src[2] << 16) | (m_rom_dma_src[1] << 8) | m_rom_dma_src[0];
		uint16_t dest = (m_rom_dma_dst[1] << 8) | m_rom_dma_dst[0];
		uint16_t len = (m_rom_dma_len[1] << 8) | m_rom_dma_len[0];

		LOG("  (possible DMA op SRC %08x DST %04x LEN %04x)\n", source, dest, len);

		for (int i = 0; i < len; i++)
		{
			uint32_t m_tmpaddress = source + i;

			// many games explicitly want to access with the high bank bit set, so probably the same logic as when grabbing tile data
			// we have to be careful here or we get the zero page memory read, hence not just using read8 on the whole space
			// this again probably indicates there is 'data space' where those don't appear
			//uint8_t dat = m_maincpu->read_full_data_sp(m_tmpaddress);
			uint8_t dat = read_full_data_sp_bypass(m_tmpaddress);
			m_maincpu->write_full_data(dest+i, dat);
		}

		if (data & 0x40) // or merely the absense of 0x80 being set? (ttv_lotr and drgqst are the only games needing the IRQ and both set 0x40 tho)
		{
			m_irqsource |= 0x20;
			update_irqs();
		}

		// ekara definitely wants this cleared
		m_rom_dma_len[0] = 0;
		m_rom_dma_len[1] = 0;

	}
	else // the interrupt routine writes 0x80 to the trigger, maybe 'clear IRQ?'
	{
		LOG("%s: rom_dmatrg_w (unknown) %02x\n", machine().describe_context(), data);
	}
}


WRITE8_MEMBER(xavix_state::rom_dmasrc_w)
{
	// has_wamg expects to be able to read back the source to modify it (need to check if it expects it to change after an operation)
	LOG("%s: rom_dmasrc_w (%02x) %02x\n", machine().describe_context(), offset, data);
	m_rom_dma_src[offset] = data;
}

WRITE8_MEMBER(xavix_state::rom_dmadst_w)
{
	LOG("%s: rom_dmadst_w (%02x) %02x\n", machine().describe_context(), offset, data);
	m_rom_dma_dst[offset] = data;
}

WRITE8_MEMBER(xavix_state::rom_dmalen_w)
{
	LOG("%s: rom_dmalen_w (%02x) %02x\n", machine().describe_context(), offset, data);
	m_rom_dma_len[offset] = data;
}


READ8_MEMBER(xavix_state::rom_dmastat_r)
{
	LOG("%s: rom_dmastat_r (operation status?)\n", machine().describe_context());
	return 0x00;
}



WRITE8_MEMBER(xavix_state::vector_enable_w)
{
	LOG("%s: vector_enable_w %02x\n", machine().describe_context(), data);
	m_vectorenable = data;
}

WRITE8_MEMBER(xavix_state::nmi_vector_lo_w)
{
	LOG("%s: nmi_vector_lo_w %02x\n", machine().describe_context(), data);
	m_nmi_vector_lo_data = data;
}

WRITE8_MEMBER(xavix_state::nmi_vector_hi_w)
{
	LOG("%s: nmi_vector_hi_w %02x\n", machine().describe_context(), data);
	m_nmi_vector_hi_data = data;
}

WRITE8_MEMBER(xavix_state::irq_vector_lo_w)
{
	LOG("%s: irq_vector_lo_w %02x\n", machine().describe_context(), data);
	m_irq_vector_lo_data = data;
}

WRITE8_MEMBER(xavix_state::irq_vector_hi_w)
{
	LOG("%s: irq_vector_hi_w %02x\n", machine().describe_context(), data);
	m_irq_vector_hi_data = data;
}


WRITE8_MEMBER(xavix_state::extintrf_7900_w)
{
	LOG("%s: extintrf_7900_w %02x (---FIRST WRITE ON STARTUP---)\n", machine().describe_context(), data);
}

WRITE8_MEMBER(xavix_state::extintrf_7901_w)
{
	LOG("%s: extintrf_7901_w %02x\n", machine().describe_context(), data);
}

WRITE8_MEMBER(xavix_state::extintrf_7902_w)
{
	LOG("%s: extintrf_7902_w %02x\n", machine().describe_context(), data);
}



READ8_MEMBER(xavix_state::ioevent_enable_r)
{
	LOG("%s: ioevent_enable_r\n", machine().describe_context());
	return m_ioevent_enable;
}

WRITE8_MEMBER(xavix_state::ioevent_enable_w)
{
	LOG("%s: ioevent_enable_w %02x\n", machine().describe_context(), data);
	m_ioevent_enable = data;
}

void xavix_state::process_ioevent(uint8_t bits)
{
	if (m_ioevent_enable & bits)
	{
		m_ioevent_active |= bits;

		if (m_ioevent_active & 0x0f)
		{
			m_irqsource |= 0x08;
		}

		update_irqs();
	}
}

WRITE_LINE_MEMBER(xavix_state::ioevent_trg01)
{
	process_ioevent(0x01);
}

WRITE_LINE_MEMBER(xavix_state::ioevent_trg02)
{
	process_ioevent(0x02);
}

WRITE_LINE_MEMBER(xavix_state::ioevent_trg04)
{
	process_ioevent(0x04);
}

WRITE_LINE_MEMBER(xavix_state::ioevent_trg08)
{
	process_ioevent(0x08);
}

READ8_MEMBER(xavix_state::ioevent_irqstate_r)
{
	LOG("%s: ioevent_irqstate_r\n", machine().describe_context());
	return m_ioevent_active;
}

WRITE8_MEMBER(xavix_state::ioevent_irqack_w)
{
	LOG("%s: ioevent_irqack_w %02x\n", machine().describe_context(), data);

	if (data & 0x01)
	{
		m_ioevent_active &= ~0x01;
	}

	if (data & 0x02)
	{
		m_ioevent_active &= ~0x02;
	}

	if (data & 0x04)
	{
		m_ioevent_active &= ~0x04;
	}

	if (data & 0x08)
	{
		m_ioevent_active &= ~0x08;
	}

	if (!(m_ioevent_active & 0x0f))
	{
		m_irqsource &= ~0x08;
	}

	update_irqs();
}



WRITE8_MEMBER(xavix_state::adc_7b00_w)
{
	LOG("%s: adc_7b00_w %02x\n", machine().describe_context(), data);
}

READ8_MEMBER(xavix_state::adc_7b80_r)
{
	LOG("%s: adc_7b80_r\n", machine().describe_context());
	return 0x00;//0xff;
}

WRITE8_MEMBER(xavix_state::adc_7b80_w)
{
	LOG("%s: adc_7b80_w %02x\n", machine().describe_context(), data);
}

WRITE8_MEMBER(xavix_state::adc_7b81_w)
{
//  m_irqsource &= ~0x04;
//  update_irqs();

	LOG("%s: adc_7b81_w %02x\n", machine().describe_context(), data);
	m_adc_control = data;

//  m_adc_timer->adjust(attotime::from_usec(200));

}

READ8_MEMBER(xavix_state::adc_7b81_r)
{
//  has_wamg polls this if interrupt is enabled
	return machine().rand();
}



WRITE8_MEMBER(xavix_state::slotreg_7810_w)
{
	LOG("%s: slotreg_7810_w %02x\n", machine().describe_context(), data);
}


TIMER_DEVICE_CALLBACK_MEMBER(xavix_state::scanline_cb)
{

}

INTERRUPT_GEN_MEMBER(xavix_state::interrupt)
{
	if (m_video_ctrl & 0x20)
	{
		m_maincpu->set_input_line(INPUT_LINE_NMI, ASSERT_LINE);
		m_video_ctrl |= 0x80;
	}
}


WRITE8_MEMBER(xavix_state::colmix_sh_w)
{
	m_colmix_sh[offset] = data;
}

WRITE8_MEMBER(xavix_state::colmix_l_w)
{
	m_colmix_l[offset] = data;
}

WRITE8_MEMBER(xavix_state::colmix_6ff2_w)
{
	LOG("%s: colmix_6ff2_w %02x\n", machine().describe_context(), data);
	m_colmix_ctrl[0] = data;
}


READ8_MEMBER(xavix_state::dispctrl_6ff8_r)
{
	// 0x80 = main IRQ asserted flag
	// 0x40 = raster IRQ asserted flag
	// 0x20 = main IRQ enable
	// 0x10 = raster IRQ enable?

	//LOG("%s: dispctrl_6ff8_r\n", machine().describe_context());
	return m_video_ctrl;
}

WRITE8_MEMBER(xavix_state::dispctrl_6ff8_w)
{
	// 0x80 = main IRQ ack?
	// 0x40 = raster IRQ ack?
	// 0x20 = main IRQ enable
	// 0x10 = raster IRQ enable?

	if (data & 0x40)
	{
		m_irqsource &= ~0x40;
		update_irqs();
	}

	if (data & 0x80)
	{
		m_maincpu->set_input_line(INPUT_LINE_NMI, CLEAR_LINE);
	}

	m_video_ctrl = data & 0x3f;
	//  printf("%s: dispctrl_6ff8_w %02x\n", machine().describe_context(), data);
}

void xavix_state::update_irqs()
{
	if (m_irqsource != 0x00)
	{
		m_maincpu->set_input_line(0, ASSERT_LINE);
	}
	else
	{
		m_maincpu->set_input_line(0, CLEAR_LINE);
	}
}

TIMER_CALLBACK_MEMBER(xavix_state::interrupt_gen)
{
	if (m_video_ctrl & 0x10)
	{
		//printf("callback on scanline %d %d with IRQ enabled\n", m_screen->vpos(), m_screen->hpos());
		m_video_ctrl |= 0x40;
		m_irqsource |= 0x40;
		update_irqs();

		m_screen->update_partial(m_screen->vpos());
	}
	m_interrupt_timer->adjust(attotime::never, 0);
}


WRITE8_MEMBER(xavix_state::dispctrl_posirq_x_w)
{
	LOG("%s: dispctrl_posirq_x_w %02x\n", machine().describe_context(), data);
	m_posirq_x[0] = data;
}

WRITE8_MEMBER(xavix_state::dispctrl_posirq_y_w)
{
	LOG("%s: dispctrl_posirq_y_w %02x\n", machine().describe_context(), data);
	m_posirq_y[0] = data;

	m_interrupt_timer->adjust(m_screen->time_until_pos(m_posirq_y[0], m_posirq_x[0]), 0);
}

/* Per Game IO port callbacks */

uint8_t xavix_state::read_io0(uint8_t direction)
{
	// no special handling
	return m_in0->read();
}

uint8_t xavix_state::read_io1(uint8_t direction)
{
	// no special handling
	return m_in1->read();
}

void xavix_state::write_io0(uint8_t data, uint8_t direction)
{
	// no special handling
}

void xavix_state::write_io1(uint8_t data, uint8_t direction)
{
	// no special handling
}

uint8_t xavix_i2c_state::read_io1(uint8_t direction)
{
	uint8_t ret = m_in1->read();

	if (!(direction & 0x08))
	{
		ret &= ~0x08;
		ret |= (m_i2cmem->read_sda() & 1) << 3;
	}

	return ret;
}

void xavix_i2c_state::write_io1(uint8_t data, uint8_t direction)
{
	// ignore these writes so that epo_edfx can send read requests to the ee-prom and doesn't just report an error
	// TODO: check if these writes shouldn't be happening (the first is a direct write, the 2nd is from a port direction change)
	//  or if the i2cmem code is oversensitive, or if something else is missing to reset the state
	if (hackaddress1 != -1)
		if ((m_maincpu->pc() == hackaddress1) || (m_maincpu->pc() == hackaddress2))
			return;

	if (direction & 0x08)
	{
		m_i2cmem->write_sda((data & 0x08) >> 3);
	}

	if (direction & 0x10)
	{
		m_i2cmem->write_scl((data & 0x10) >> 4);
	}
}

uint8_t xavix_i2c_lotr_state::read_io1(uint8_t direction)
{
	uint8_t ret = m_in1->read();

	// some kind of comms with the IR sensor?
	ret ^= (machine().rand() & 0x02);
	ret ^= (machine().rand() & 0x04);

	if (!(direction & 0x08))
	{
		ret &= ~0x08;
		ret |= (m_i2cmem->read_sda() & 1) << 3;
	}

	return ret;
}

uint8_t xavix_ekara_state::read_io1(uint8_t direction)
{
	uint8_t extrainlatch0 = 0x00;
	uint8_t extrainlatch1 = 0x00;

	switch (m_extraioselect & 0x7f)
	{
	case 0x01:
		extrainlatch0 = (m_extra0->read() & 0x01) >> 0;
		extrainlatch1 = (m_extra0->read() & 0x02) >> 1;
		break;
	case 0x02:
		extrainlatch0 = (m_extra0->read() & 0x04) >> 2;
		extrainlatch1 = (m_extra0->read() & 0x08) >> 3;
		break;
	case 0x04:
		extrainlatch0 = (m_extra0->read() & 0x10) >> 4;
		extrainlatch1 = (m_extra0->read() & 0x20) >> 5;
		break;
	case 0x08:
		extrainlatch0 = (m_extra0->read() & 0x40) >> 6;
		extrainlatch1 = (m_extra0->read() & 0x80) >> 7;
		break;
	case 0x10:
		extrainlatch0 = (m_extra1->read() & 0x01) >> 0;
		extrainlatch1 = (m_extra1->read() & 0x02) >> 1;
		break;
	case 0x20:
		extrainlatch0 = (m_extra1->read() & 0x04) >> 2;
		extrainlatch1 = (m_extra1->read() & 0x08) >> 3;
		break;
	case 0x40:
		extrainlatch0 = (m_extra1->read() & 0x10) >> 4;
		extrainlatch1 = (m_extra1->read() & 0x20) >> 5;
		break;
	default:
		LOG("latching inputs with invalid m_extraioselect value of %02x\n", m_extraioselect);
		break;
	}

	uint8_t ret = m_in1->read();
	ret &= 0xfc;
	ret |= extrainlatch0 << 0;
	ret |= extrainlatch1 << 1;
	return ret;
}

void xavix_ekara_state::write_io0(uint8_t data, uint8_t direction)
{
	// is bit 0x80 an enable for something else? LED? Microphone? it doesn't seem related to the multiplexing
	m_extraioselect = data & direction;
}

void xavix_ekara_state::write_io1(uint8_t data, uint8_t direction)
{
	uint8_t extraiowrite = data & direction;
	m_extraiowrite = extraiowrite;
}

/* General IO port handling */

READ8_MEMBER(xavix_state::io0_data_r)
{
	uint8_t ret = read_io0(m_io0_direction) & ~m_io0_direction;
	ret |= m_io0_data & m_io0_direction;
	return ret;
}

READ8_MEMBER(xavix_state::io1_data_r)
{
	uint8_t ret = read_io1(m_io1_direction) & ~m_io1_direction;
	ret |= m_io1_data & m_io1_direction;
	return ret;
}

READ8_MEMBER(xavix_state::io0_direction_r)
{
	return m_io0_direction;
}

READ8_MEMBER(xavix_state::io1_direction_r)
{
	return m_io1_direction;
}


WRITE8_MEMBER(xavix_state::io0_data_w)
{
	m_io0_data = data;
	write_io0(data, m_io0_direction);
	LOG("%s: io0_data_w %02x\n", machine().describe_context(), data);
}

WRITE8_MEMBER(xavix_state::io1_data_w)
{
	m_io1_data = data;
	write_io1(data, m_io1_direction);
	LOG("%s: io1_data_w %02x\n", machine().describe_context(), data);
}


WRITE8_MEMBER(xavix_state::io0_direction_w)
{
	m_io0_direction = data;
	LOG("%s: io0_direction_w %02x\n", machine().describe_context(), data);
	io0_data_w(space, 0, m_io0_data);
}

WRITE8_MEMBER(xavix_state::io1_direction_w)
{
	m_io1_direction = data;
	LOG("%s: io1_direction_w %02x\n", machine().describe_context(), data);
	io1_data_w(space, 0, m_io1_data); // requires this for i2cmem to work, is it correct tho?
}

/* Arena (Visible Area + hblank?) handling */

READ8_MEMBER(xavix_state::arena_start_r)
{
	//LOG("%s: arena_start_r\n", machine().describe_context());
	return m_arena_start;
}

WRITE8_MEMBER(xavix_state::arena_start_w)
{
	LOG("%s: arena_start_w %02x\n", machine().describe_context(), data);
	m_arena_start = data; // expected to return data written

}
READ8_MEMBER(xavix_state::arena_end_r)
{
	LOG("%s: arena_end_r\n", machine().describe_context());
	return m_arena_end;
}

WRITE8_MEMBER(xavix_state::arena_end_w)
{
	LOG("%s: arena_end_w %02x\n", machine().describe_context(), data);
	m_arena_end = data; // expected to return data written
}

READ8_MEMBER(xavix_state::arena_control_r)
{
	// xavtenni expects 0x40 to go high (interlace related?)
	m_arena_control ^= 0x40;
	return m_arena_control;
}

WRITE8_MEMBER(xavix_state::arena_control_w)
{
	LOG("%s: arena_control_w %02x\n", machine().describe_context(), data);
	m_arena_control = data;

	// rad_bb2 waits on this in the IRQ (what does it want?) is this hblank related?
	if (data & 0x80)
		m_arena_control &= ~0x80;
}


READ8_MEMBER(xavix_state::timer_baseval_r)
{
	LOG("%s: timer_baseval_r\n", machine().describe_context());
	return m_timer_baseval;
}

READ8_MEMBER(xavix_state::timer_status_r)
{
	uint8_t ret = m_timer_control;
	LOG("%s: timer_status_r\n", machine().describe_context());
	return ret;
}

WRITE8_MEMBER(xavix_state::timer_control_w)
{
	/* timer is actively used by
	   ttv_lotr, ttv_sw, drgqst, has_wamg, rad_rh, eka_*, epo_efdx, rad_bass, rad_bb2

	   gets turned on briefly during the bootup of rad_crdn, but then off again

	   runs during rad_fb / rad_madf, but with IRQs turned off

	   disabled for rad_snow, rad_ping, rad_mtrk, rad_box, *nostalgia, ttv_mx, xavtenni
	   */

	//LOG("%s: timer_control_w %02x\n", machine().describe_context(), data);
	m_timer_control = data;

	if (data & 0x80) // tends to read+write address to ack interrupts, assume it's similar to other things and top bit will clear IRQ (usually gets written all the time when starting timer)
	{
		if (m_irqsource & 0x10)
		{
			m_irqsource &= ~0x10;
			update_irqs();
		}
	}

	// has_wamg, ttv_sw, eka_*, rad_bass set bit 0x02 too (maybe reload related?)

	// rad_fb / rad_madf don't set bit 0x40 (and doesn't seem to have a valid interrupt handler for timer, so probably means it generates no IRQ?)
	if (data & 0x01) // timer start?
	{
		// TODO: work out the proper calculation here
		// int divide = 1 << ((m_timer_freq&0x0f)+1);
		// uint32_t freq = m_maincpu->unscaled_clock()/2;
		// m_freq_timer->adjust(attotime::from_hz(freq / divide) * m_timer_baseval*20);
		//m_freq_timer->adjust(attotime::from_usec(1000));
		m_freq_timer->adjust(attotime::from_usec(50));
	}
	else
	{
		m_freq_timer->adjust(attotime::never, 0);
	}
}

WRITE8_MEMBER(xavix_state::timer_baseval_w)
{
	// expected to return data written
	m_timer_baseval = data;
	LOG("%s: timer_baseval_w %02x\n", machine().describe_context(), data);
}

READ8_MEMBER(xavix_state::timer_freq_r)
{
	LOG("%s: timer_freq_r\n", machine().describe_context());
	return m_timer_freq;
}

READ8_MEMBER(xavix_state::timer_curval_r)
{
	// TODO implement properly with timers etc. as rad_fb / rad_madfb rely on these values to calculate throw strength!
	LOG("%s: timer_curval_r\n", machine().describe_context());
	return machine().rand();
}


WRITE8_MEMBER(xavix_state::timer_freq_w)
{
	// 4-bit prescale
	LOG("%s: timer_freq_w %02x\n", machine().describe_context(), data);

	/* if master clock (MC) is XTAL(21'477'272) (NTSC master)

	   divide value        clock source
	   0x0 = MC / 2      = 10.738636 MHz (10738636 Hz)
	   0x1 = MC / 4      = 5.369318 MHz
	   0x2 = MC / 8      = 2.684659 MHz
	   0x3 = MC / 16     = 1.3423295 MHz
	   0x4 = MC / 32     = 671.16475 kHz
	   0x5 = MC / 64     = 335.582375 kHz
	   0x6 = MC / 128    = 167.7911875 kHz
	   0x7 = MC / 256    = 83.89559375 kHz
	   0x8 = MC / 512    = 41.947796875 kHz
	   0x9 = MC / 1024   = 20.9738984375 kHz
	   0xa = MC / 2048   = 10.48694921875 kHz
	   0xb = MC / 4096   = 5.243474609375 kHz
	   0xc = MC / 8192   = 2.6217373046875 kHz
	   0xd = MC / 16384  = 1.31086865234375 kHz
	   0xe = MC / 32768  = 655.434326171875 Hz
	   0xf = MC / 65536  = 327.7171630859375 Hz
	*/
	m_timer_freq = data & 0x0f;

	if (data & 0xf0)
		LOG("%s: unexpected upper bits in timer freq %02x\n", machine().describe_context(), data & 0xf0);
}

TIMER_CALLBACK_MEMBER(xavix_state::freq_timer_done)
{
	if (m_timer_control & 0x40) // Timer IRQ enable?
	{
		m_irqsource |= 0x10;
		m_timer_control |= 0x80;
		update_irqs();
	}

	//logerror("freq_timer_done\n");
	// reload
	//m_freq_timer->adjust(attotime::from_usec(50000));
}

TIMER_CALLBACK_MEMBER(xavix_state::adc_timer_done)
{
	//m_irqsource |= 0x04;
	//update_irqs();
}



READ8_MEMBER(xavix_state::mult_r)
{
	return m_multresults[offset];
}

WRITE8_MEMBER(xavix_state::mult_w)
{
	// rad_madf writes here to set the base value which the multiplication result gets added to
	m_multresults[offset] = data;
}

READ8_MEMBER(xavix_state::mult_param_r)
{
	return m_multparams[offset];
}

WRITE8_MEMBER(xavix_state::mult_param_w)
{
	COMBINE_DATA(&m_multparams[offset]);
	// there are NOPs after one of the writes, so presumably the operation is write triggerd and not intstant
	// see test code at 0184a4 in monster truck

	// offset0 is control

	// mm-- --Ss
	// mm = mode, S = sign for param1, s = sign for param2
	// modes 00 = multiply (regular?) 11 = add to previous 01 / 10 unknown (maybe subtract?)

	if (offset == 2)
	{
		// assume 0 is upper bits, might be 'mode' instead, check


		int signmode = (m_multparams[0] & 0x3f);

		uint16_t result = 0;

		// rad_madf uses this mode (add to previous result)
		if ((m_multparams[0] & 0xc0) == 0xc0)
		{
			const int param1 = signmode & 0x2 ? (int8_t)m_multparams[1] : (uint8_t)m_multparams[1];
			const int param2 = signmode & 0x1 ? (int8_t)m_multparams[2] : (uint8_t)m_multparams[2];

			result = param1 * param2;

			uint16_t oldresult = (m_multresults[1] << 8) | m_multresults[0];
			result = oldresult + result;
		}
		else if ((m_multparams[0] & 0xc0) == 0x00)
		{
			const int param1 = signmode & 0x2 ? (int8_t)m_multparams[1] : (uint8_t)m_multparams[1];
			const int param2 = signmode & 0x1 ? (int8_t)m_multparams[2] : (uint8_t)m_multparams[2];

			result = param1 * param2;
		}
		else
		{
			popmessage("unknown multiplier mode %02x", m_multparams[0] & 0xc0);
		}

		m_multresults[1] = (result >> 8) & 0xff;
		m_multresults[0] = result & 0xff;
	}
}


READ8_MEMBER(xavix_state::irq_source_r)
{
	/* the 2nd IRQ routine (regular IRQ) reads here before deciding what to do

	 the following bits have been seen to be checked (active high)
	 monster truck does most extensive checking

	  0x80 - Sound Irq
	  0x40 - Picture / Arena Irq? (including raster interrupt)
	  0x20 - DMA Irq  (most routines check this as first priority, and ignore other requests if it is set?)
	  0x10 - Timer / Counter IRQ
	  0x08 - IO Event Irq (uses 7a00 top bit to determine direction, enabled with 0x08 on 7a80, IRQ acked / cleared with 0x08 written to 7a81, 4 possible sources with different bits in 7a80 / 7a81 ? )
	         (this is the type of interrupt where the irq frequency adds a counter which determines analog value)
	  0x04 - ADC Conversion IRQ - loads/stores 7b81 (to ack interrupt)
	*/

	LOG("%s: irq_source_r\n", machine().describe_context());
	return m_irqsource;
}

WRITE8_MEMBER(xavix_state::irq_source_w)
{
	LOG("%s: irq_source_w %02x\n", machine().describe_context(), data);
	// cleared on startup in monster truck, no purpose?
}


void xavix_state::machine_start()
{
	// card night expects RAM to be initialized to 0xff or it will show the pause menu over the startup graphics?!
	// don't do this every reset or it breaks the baseball 2 secret mode toggle which flips a bit in RAM
	std::fill_n(&m_mainram[0], 0x4000, 0xff);

	m_interrupt_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(xavix_state::interrupt_gen), this));
	m_freq_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(xavix_state::freq_timer_done), this));
	m_adc_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(xavix_state::adc_timer_done), this));

	for (int i = 0; i < 4; i++)
	{
		m_sound_timer[i] = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(xavix_state::sound_timer_done), this));
	}
}

void xavix_state::machine_reset()
{
	m_rom_dma_src[0] = 0;
	m_rom_dma_src[1] = 0;
	m_rom_dma_src[2] = 0;

	m_rom_dma_dst[0] = 0;
	m_rom_dma_dst[1] = 0;

	m_rom_dma_len[0] = 0;
	m_rom_dma_len[1] = 0;

	m_vectorenable = 0;
	m_nmi_vector_lo_data = 0;
	m_nmi_vector_hi_data = 0;
	m_irq_vector_lo_data = 0;
	m_irq_vector_hi_data = 0;

	m_video_ctrl = 0;

	m_arena_control = 0;
	m_arena_start = 0;
	m_arena_end = 0;

	m_spritereg = 0;

	m_mastervol = 0x00;
	m_unk_snd75f8 = 0x00;
	m_unksnd75f9 = 0x00;
	m_unksnd75ff = 0x00;

	for (int i = 0; i < 4; i++)
	{
		m_sndtimer[i] = 0x00;
	}

	std::fill(std::begin(m_multparams), std::end(m_multparams), 0x00);
	std::fill(std::begin(m_multresults), std::end(m_multresults), 0x00);
	std::fill(std::begin(m_spritefragment_dmaparam1), std::end(m_spritefragment_dmaparam1), 0x00);
	std::fill(std::begin(m_tmap1_regs), std::end(m_tmap1_regs), 0x00);
	std::fill(std::begin(m_tmap2_regs), std::end(m_tmap2_regs), 0x00);
	std::fill(std::begin(m_txarray), std::end(m_txarray), 0x00);
	std::fill_n(&m_fragment_sprite[0], 0x800, 0x00); // taito nostalgia 1 never initializes the ram at 0x6400 but there's no condition on using it at present?

	//m_lowbus->set_bank(0);

	m_io0_data = 0x00;
	m_io1_data = 0x00;

	m_io0_direction = 0x00;
	m_io1_direction = 0x00;

	m_irqsource = 0x00;

	m_timer_control = 0x00;

	m_ioevent_enable = 0x00;
	m_ioevent_active = 0x00;

	m_sound_irqstatus = 0x00;

	m_sound_regbase = 0x00;

	m_adc_control = 0x00;

	m_sprite_xhigh_ignore_hack = true;
}

typedef device_delegate<uint8_t(int which, int half)> xavix_interrupt_vector_delegate;

int16_t xavix_state::get_vectors(int which, int half)
{
	//  LOG("get_vectors %d %d\n", which, half);
	if (m_vectorenable == 0)
		return -1;

	if (which == 0) // irq?
	{
		if (half == 0)
			return m_nmi_vector_hi_data;
		else
			return m_nmi_vector_lo_data;
	}
	else
	{
		if (half == 0)
			return m_irq_vector_hi_data;
		else
			return m_irq_vector_lo_data;
	}
}


