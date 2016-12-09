// license:BSD-3-Clause
// copyright-holders:Ted Green
#include "iteagle_fpga.h"
#include "coreutil.h"

#define LOG_FPGA            (0)
#define LOG_SERIAL          (0)
#define LOG_RTC             (0)
#define LOG_RAM             (0)
#define LOG_EEPROM          (0)
#define LOG_PERIPH          (0)

const device_type ITEAGLE_FPGA = &device_creator<iteagle_fpga_device>;

MACHINE_CONFIG_FRAGMENT(iteagle_fpga)
	MCFG_NVRAM_ADD_0FILL("eagle2_rtc")
MACHINE_CONFIG_END

DEVICE_ADDRESS_MAP_START(fpga_map, 32, iteagle_fpga_device)
	AM_RANGE(0x000, 0x01f) AM_READWRITE(fpga_r, fpga_w)
ADDRESS_MAP_END

DEVICE_ADDRESS_MAP_START(rtc_map, 32, iteagle_fpga_device)
	AM_RANGE(0x000, 0x7ff) AM_READWRITE(rtc_r, rtc_w)
ADDRESS_MAP_END

DEVICE_ADDRESS_MAP_START(ram_map, 32, iteagle_fpga_device)
	AM_RANGE(0x00000, 0x1ffff) AM_READWRITE(ram_r, ram_w)
ADDRESS_MAP_END

iteagle_fpga_device::iteagle_fpga_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: pci_device(mconfig, ITEAGLE_FPGA, "ITEagle FPGA", tag, owner, clock, "iteagle_fpga", __FILE__),
		m_rtc(*this, "eagle2_rtc"), m_version(0), m_seq_init(0)
{
}

machine_config_constructor iteagle_fpga_device::device_mconfig_additions() const
{
	return MACHINE_CONFIG_NAME(iteagle_fpga);
}

void iteagle_fpga_device::device_start()
{
	// RTC M48T02
	m_rtc->set_base(m_rtc_regs, sizeof(m_rtc_regs));

	pci_device::device_start();
	status = 0x5555;
	command = 0x5555;

	add_map(sizeof(m_fpga_regs), M_IO, FUNC(iteagle_fpga_device::fpga_map));
	// fpga defaults to base address 0x00000300
	bank_infos[0].adr = 0x00000300 & (~(bank_infos[0].size - 1));

	add_map(sizeof(m_rtc_regs), M_MEM, FUNC(iteagle_fpga_device::rtc_map));
	// RTC defaults to base address 0x000c0000
	bank_infos[1].adr = 0x000c0000 & (~(bank_infos[1].size - 1));

	add_map(sizeof(m_ram), M_MEM, FUNC(iteagle_fpga_device::ram_map));
	// RAM defaults to base address 0x000e0000
	bank_infos[2].adr = 0x000e0000 & (~(bank_infos[2].size - 1));

	m_timer = timer_alloc(0, nullptr);

	// virtpool nvram
	memset(m_ram, 0, sizeof(m_ram));
	// byte 0x10 is check sum of first 16 bytes
	// when corrupt the fw writes the following
	m_ram[0x00/4] = 0x00010207;
	m_ram[0x04/4] = 0x04010101;
	m_ram[0x08/4] = 0x01030101;
	m_ram[0x0c/4] = 0x00000001;
	m_ram[0x10/4] = 0x00000018;

}

void iteagle_fpga_device::device_reset()
{
	remap_cb();
	m_cpu = machine().device<cpu_device>(m_cpu_tag);
	memset(m_fpga_regs, 0, sizeof(m_fpga_regs));
	m_seq = m_seq_init;
	m_seq_rem1 = 0;
	m_seq_rem2 = 0;

	// Nibble starting at bit 20 is resolution, byte 0 is atmel response
	// 0x00080000 and interrupt starts reading from 0x14
	// 0x02000000 and interrupt starts reading from 0x18
	// Write 0x01000000 is a global interrupt clear
	m_fpga_regs[0x04/4] =  0x00000000;
	m_prev_reg = 0;

	m_serial_str.clear();
	m_serial_idx = 0;
	m_serial_data = false;
	memset(m_serial_com0, 0, sizeof(m_serial_com0));
	memset(m_serial_com1, 0, sizeof(m_serial_com1));
	memset(m_serial_com2, 0, sizeof(m_serial_com2));
	memset(m_serial_com3, 0, sizeof(m_serial_com3));
	m_serial_com0[0] = 0x2c;
	m_serial_com1[0] = 0x2c;
	m_serial_com2[0] = 0x2c;
	m_serial_com3[0] = 0x2c;
	m_serial_rx3.clear();
}

void iteagle_fpga_device::update_sequence(uint32_t data)
{
	uint32_t offset = 0x04/4;
	if (data & 0x80) {
		m_fpga_regs[offset] = (m_fpga_regs[offset]&0xFFFFFF00) | ((m_version>>(8*(data&3)))&0xff);
	} else {
		uint32_t val1, feed;
		feed = ((m_seq<<4) ^ m_seq)>>7;
		if (data & 0x1) {
			val1 = ((m_seq & 0x2)<<1) | ((m_seq & 0x4)>>1) | ((m_seq & 0x8)>>3);
			m_seq_rem1 = ((m_seq & 0x10)) | ((m_seq & 0x20)>>2) | ((m_seq & 0x40)>>4);
			m_seq_rem2 = ((m_seq & 0x80)>>1) | ((m_seq & 0x100)>>3) | ((m_seq & 0x200)>>5);
			m_seq = (m_seq>>9) | ((feed&0x1ff)<<15);
			m_fpga_regs[offset] = (m_fpga_regs[offset]&0xFFFFFF00) | ((val1 + m_seq_rem1 + m_seq_rem2)&0xFF);
		} else if (data & 0x2) {
			val1 = ((m_seq & 0x2)<<1) | ((m_seq & 0x4)>>1) | ((m_seq & 0x8)>>3);
			m_seq_rem1 = ((m_seq & 0x10)) | ((m_seq & 0x20)>>2) | ((m_seq & 0x40)>>4);
			m_seq = (m_seq>>6) | ((feed&0x3f)<<18);
			m_fpga_regs[offset] = (m_fpga_regs[offset]&0xFFFFFF00) | ((val1 + m_seq_rem1 + m_seq_rem2)&0xFF);
		} else {
			val1 = ((m_seq & 0x2)<<6) | ((m_seq & 0x4)<<4) | ((m_seq & 0x8)<<2) | ((m_seq & 0x10)<<0)
					| ((m_seq & 0x20)>>2) | ((m_seq & 0x40)>>4) | ((m_seq & 0x80)>>6) | ((m_seq & 0x100)>>8);
			m_seq = (m_seq>>8) | ((feed&0xff)<<16);
			m_fpga_regs[offset] = (m_fpga_regs[offset]&0xFFFFFF00) | ((val1 + m_seq_rem1 + m_seq_rem2) & 0xff);
		}
		if (0 && LOG_FPGA)
			logerror("%s:fpga update_sequence In: %02X Seq: %06X Out: %02X\n", machine().describe_context(), data, m_seq, m_fpga_regs[offset]&0xff);
	}
}

// Eagle 1 sequence generator
void iteagle_fpga_device::update_sequence_eg1(uint32_t data)
{
	uint32_t offset = 0x04/4;
	uint32_t val1, feed;
	feed = ((m_seq<<4) ^ m_seq)>>7;
	if (data & 0x1) {
		val1 = ((m_seq & 0x2)<<6) | ((m_seq & 0x4)<<4) | ((m_seq & 0x8)<<2) | ((m_seq & 0x10)<<0)
				| ((m_seq & 0x20)>>2) | ((m_seq & 0x40)>>4) | ((m_seq & 0x80)>>6) | ((m_seq & 0x100)>>8);
		m_seq = (m_seq>>8) | ((feed&0xff)<<16);
		m_fpga_regs[offset] = (m_fpga_regs[offset]&0xFFFFFF00) | ((val1 + m_seq_rem1 + m_seq_rem2)&0xFF);
	} else if (data & 0x2) {
		val1 = ((m_seq & 0x2)<<1) | ((m_seq & 0x4)>>1) | ((m_seq & 0x8)>>3);
		m_seq_rem1 = ((m_seq & 0x10)) | ((m_seq & 0x20)>>2) | ((m_seq & 0x40)>>4);
		m_seq = (m_seq>>6) | ((feed&0x3f)<<18);
		m_fpga_regs[offset] = (m_fpga_regs[offset]&0xFFFFFF00) | ((val1 + m_seq_rem1 + m_seq_rem2)&0xFF);
	} else {
		val1 = ((m_seq & 0x2)<<1) | ((m_seq & 0x4)>>1) | ((m_seq & 0x8)>>3);
		m_seq_rem1 = ((m_seq & 0x10)) | ((m_seq & 0x20)>>2) | ((m_seq & 0x40)>>4);
		m_seq_rem2 = ((m_seq & 0x80)>>1) | ((m_seq & 0x100)>>3) | ((m_seq & 0x200)>>5);
		m_seq = (m_seq>>9) | ((feed&0x1ff)<<15);
		m_fpga_regs[offset] = (m_fpga_regs[offset]&0xFFFFFF00) | ((val1 + m_seq_rem1 + m_seq_rem2) & 0xff);
	}
	if (0 && LOG_FPGA)
		logerror("%s:fpga update_sequence In: %02X Seq: %06X Out: %02X other %02X%02X%02X\n", machine().describe_context(),
			data, m_seq, m_fpga_regs[offset]&0xff, m_seq_rem2, m_seq_rem1, val1);
}

//-------------------------------------------------
//  device_timer - called when our device timer expires
//-------------------------------------------------
void iteagle_fpga_device::device_timer(emu_timer &timer, device_timer_id tid, int param, void *ptr)
{
	if (m_fpga_regs[0x4/4] & 0x01000000) {
		//m_fpga_regs[0x04/4] |=  0x02080000;
		m_fpga_regs[0x04/4] |=  0x00080000;
		m_cpu->set_input_line(m_irq_num, ASSERT_LINE);
		if (LOG_FPGA)
				logerror("%s:fpga device_timer Setting interrupt(%i)\n", machine().describe_context(), m_irq_num);
	}

}

READ32_MEMBER( iteagle_fpga_device::fpga_r )
{
	uint32_t result = m_fpga_regs[offset];

	switch (offset) {
		case 0x00/4:
			result = ((machine().root_device().ioport("SYSTEM")->read()&0xffff)<<16) | (machine().root_device().ioport("IN1")->read()&0xffff);
			if (LOG_FPGA && m_prev_reg!=offset)
				logerror("%s:fpga_r offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, result, mem_mask);
			break;
		case 0x04/4:
			result = (result & 0xFF0FFFFF) | ((machine().root_device().ioport("SW5")->read()&0xf)<<20);
			if (LOG_FPGA && !ACCESSING_BITS_0_7)
				logerror("%s:fpga_r offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, result, mem_mask);
			break;
		case 0x08/4:
			result = ((machine().root_device().ioport("TRACKY1")->read()&0xff)<<8) | (machine().root_device().ioport("TRACKX1")->read()&0xff);
			if (LOG_FPGA && m_prev_reg!=offset)
				logerror("%s:fpga_r offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, result, mem_mask);
			break;
		case 0x14/4: // GUN1-- Interrupt & 0x4==0x00080000
			result = ((machine().root_device().ioport("GUNY1")->read())<<16) | (machine().root_device().ioport("GUNX1")->read());
			if (LOG_FPGA)
				logerror("%s:fpga_r offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, result, mem_mask);
			break;
		case 0x18/4: // Interrupt & 0x4==0x02000000
			result = 0;
			if (LOG_FPGA)
				logerror("%s:fpga_r offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, result, mem_mask);
			break;
		case 0x0c/4: //
			result = (result & 0xFFFF0000) | ((m_serial_com1[m_serial_idx]&0xff)<<8) | (m_serial_com0[m_serial_idx]&0xff);
			if (ACCESSING_BITS_0_15) {
				m_serial_data = false;
				m_serial_idx = 0;
			}
			if (0 && LOG_FPGA)
				logerror("%s:fpga_r offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, result, mem_mask);
			break;
		case 0x1c/4: // 1d = modem byte
			result = (result & 0xFFFF0000) | ((m_serial_com3[m_serial_idx]&0xff)<<8) | (m_serial_com2[m_serial_idx]&0xff);
			if (ACCESSING_BITS_0_15) {
				m_serial_data = false;
				m_serial_idx = 0;
			}
			if (ACCESSING_BITS_24_31) {
				if (!m_serial_rx3.empty()) {
					logerror("fpga_r: read byte: %c\n", m_serial_rx3.at(0));
					result = (result & 0x00FFFFFF) | (m_serial_rx3.at(0)<<24);
					m_serial_rx3.erase(m_serial_rx3.begin());
				}
				if (m_serial_rx3.empty()) {
					m_serial_com3[0] &= ~0x1;
					m_serial_com3[3] &= ~0x20;
					m_cpu->set_input_line(m_serial_irq_num, CLEAR_LINE);
				}
			}
			if (LOG_FPGA)
				logerror("%s:fpga_r offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, result, mem_mask);
			break;
		default:
			if (LOG_FPGA)
				logerror("%s:fpga_r offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, result, mem_mask);
			osd_printf_debug("%s:fpga_r offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, result, mem_mask);
			break;
	}
	if (offset!=0x4/4)
		m_prev_reg = offset;
	return result;
}

WRITE32_MEMBER( iteagle_fpga_device::fpga_w )
{
	COMBINE_DATA(&m_fpga_regs[offset]);
	switch (offset) {
		case 0x04/4:
			if (ACCESSING_BITS_0_7) {
				if ((m_version & 0xff00) == 0x0200)
					update_sequence_eg1(data & 0xff);
				else
					// ATMEL Chip access.  Returns version id's when bit 7 is set.
					update_sequence(data & 0xff);
				if (0 && LOG_FPGA)
						logerror("%s:fpga_w offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, data, mem_mask);
			} else if (ACCESSING_BITS_8_15) {
				// Interrupt enable?
				if (LOG_FPGA)
						logerror("%s:fpga_w offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, data, mem_mask);
			} else if (ACCESSING_BITS_24_31 && (data & 0x01000000)) {
			// Interrupt clear/enable
				m_cpu->set_input_line(m_irq_num, CLEAR_LINE);
				// Not sure what value to use here, needed for lightgun
				m_timer->adjust(attotime::from_hz(59));
				if (LOG_FPGA)
						logerror("%s:fpga_w offset %04X = %08X & %08X Clearing interrupt(%i)\n", machine().describe_context(), offset*4, data, mem_mask, m_irq_num);
			} else {
				if (LOG_FPGA)
						logerror("%s:fpga_w offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, data, mem_mask);
			}
			break;
		case 0x14/4:
			if (ACCESSING_BITS_0_7 && (data&0x1)) {
				m_fpga_regs[0x04/4] &=  ~0x00080000;
			}
			if (LOG_FPGA)
					logerror("%s:fpga_w offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, data, mem_mask);
			break;
		case 0x18/4:
			if (ACCESSING_BITS_0_7 && (data&0x1)) {
				m_fpga_regs[0x04/4] &=  ~0x02000000;
			}
			if (LOG_FPGA)
					logerror("%s:fpga_w offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, data, mem_mask);
			break;
		case 0x0c/4:
			if (ACCESSING_BITS_0_7) {
				if (!m_serial_data) {
					m_serial_idx = data&0xf;
				} else {
					m_serial_com0[m_serial_idx] = data&0xff;
					m_serial_idx = 0;
				}
				m_serial_data = !m_serial_data;
			}
			if (ACCESSING_BITS_8_15) {
				if (!m_serial_data) {
					m_serial_idx = (data&0x0f00)>>8;
				} else {
					m_serial_com1[m_serial_idx] = (data&0xff00)>>8;
				}
				m_serial_data = !m_serial_data;
			}
			if (ACCESSING_BITS_16_23) {
				if (m_serial_str.size()==0)
					m_serial_str = "com0: ";
				m_serial_str += (data>>16)&0xff;
				if (((data>>16)&0xff)==0xd) {
					if (LOG_SERIAL) logerror("%s\n", m_serial_str.c_str());
					osd_printf_debug("%s\n", m_serial_str.c_str());
					m_serial_str.clear();
				}
			}
			if (ACCESSING_BITS_24_31) {
				if (m_serial_str.size()==0)
					m_serial_str = "com1: ";
				m_serial_str += (data>>24)&0xff;
				if (1) {
					if (LOG_SERIAL) logerror("%s\n", m_serial_str.c_str());
					osd_printf_debug("%s\n", m_serial_str.c_str());
					m_serial_str.clear();
				}
			}
			if (0 && LOG_FPGA)
					logerror("%s:fpga_w offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, data, mem_mask);
			break;
		case 0x1c/4:
			if (ACCESSING_BITS_0_7) {
				if (!m_serial_data) {
					m_serial_idx = data&0xf;
				} else {
					m_serial_com2[m_serial_idx] = data&0xff;
					m_serial_idx = 0;
				}
				m_serial_data = !m_serial_data;
			}
			if (ACCESSING_BITS_8_15) {
				if (!m_serial_data) {
					m_serial_idx = (data&0x0f00)>>8;
				} else {
					m_serial_com3[m_serial_idx] = (data&0xff00)>>8;
				}
				m_serial_data = !m_serial_data;
			}
			if (ACCESSING_BITS_16_23) {
				if (m_serial_str.size()==0)
					m_serial_str = "com2: ";
				m_serial_str += (data>>16)&0xff;
				if (1) {
					if (LOG_SERIAL) logerror("%s\n", m_serial_str.c_str());
					osd_printf_debug("%s\n", m_serial_str.c_str());
					m_serial_str.clear();
				}
			}
			if (ACCESSING_BITS_24_31) {
				if (m_serial_str.size()==0)
					m_serial_str = "com3: ";
				m_serial_str += (data>>24)&0xff;
				if (((data>>24)&0xff)==0xd) {
					if (LOG_SERIAL) logerror("%s\n", m_serial_str.c_str());
					osd_printf_debug("%s\n", m_serial_str.c_str());
					if (m_serial_str.find("ATI5") != -1)
						m_serial_rx3 += "OK\r181\r";
					else if (m_serial_str.find("ATS0?") != -1)
						m_serial_rx3 += "0\r";
					else
						m_serial_rx3 += "OK\r";
					m_serial_com3[0] |= 0x1;
					m_serial_com3[3] = 0x20;
					m_cpu->set_input_line(m_serial_irq_num, ASSERT_LINE);
					m_serial_str.clear();
				}
			}
			if (LOG_FPGA)
					logerror("%s:fpga_w offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, data, mem_mask);
			break;
		default:
			if (LOG_FPGA)
					logerror("%s:fpga_w offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, data, mem_mask);
			osd_printf_debug("%s:fpga_w offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, data, mem_mask);
			break;
	}
}
//*************************************
//*  RTC M48T02
//*************************************
READ32_MEMBER( iteagle_fpga_device::rtc_r )
{
	uint32_t result = m_rtc_regs[offset];

	switch (offset) {
		default:
			if (LOG_RTC)
				logerror("%s:RTC read from offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, result, mem_mask);
			break;
	}
	return result;
}

WRITE32_MEMBER( iteagle_fpga_device::rtc_w )
{
	system_time systime;
	int raw[8];

	COMBINE_DATA(&m_rtc_regs[offset]);
	switch (offset) {
		case 0x7F8/4: // M48T02 time
			if (data & mem_mask & 0x40) {
				// get the current date/time from the core
				machine().current_datetime(systime);
				raw[0] = 0x40;
				raw[1] = dec_2_bcd(systime.local_time.second);
				raw[2] = dec_2_bcd(systime.local_time.minute);
				raw[3] = dec_2_bcd(systime.local_time.hour);

				raw[4] = dec_2_bcd((systime.local_time.weekday != 0) ? systime.local_time.weekday : 7);
				raw[5] = dec_2_bcd(systime.local_time.mday);
				raw[6] = dec_2_bcd(systime.local_time.month + 1);
				raw[7] = dec_2_bcd(systime.local_time.year - 1900); // Epoch is 1900
				m_rtc_regs[0x7F8/4] = (raw[3]<<24) | (raw[2]<<16) | (raw[1]<<8) | (raw[0] <<0);
				m_rtc_regs[0x7FC/4] = (raw[7]<<24) | (raw[6]<<16) | (raw[5]<<8) | (raw[4] <<0);
				//m_rtc_regs[0x7FC/4] = (0x95<<24) | (raw[6]<<16) | (raw[5]<<8) | (raw[4] <<0);
			}
			if (LOG_RTC)
				logerror("%s:RTC write to offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, data, mem_mask);

			break;
		default:
			if (LOG_RTC)
				logerror("%s:RTC write to offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, data, mem_mask);
			break;
	}

}

//*************************************
//*  FPGA RAM -- Eagle 1 only
//*************************************
READ32_MEMBER( iteagle_fpga_device::ram_r )
{
	uint32_t result = m_ram[offset];
	if (LOG_RAM)
		logerror("%s:FPGA ram_r from offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, result, mem_mask);
	return result;
}

WRITE32_MEMBER( iteagle_fpga_device::ram_w )
{
	COMBINE_DATA(&m_ram[offset]);
	if (LOG_RAM)
		logerror("%s:FPGA ram_w to offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, data, mem_mask);
}

//************************************
// Attached serial EEPROM
//************************************

const device_type ITEAGLE_EEPROM = &device_creator<iteagle_eeprom_device>;

DEVICE_ADDRESS_MAP_START(eeprom_map, 32, iteagle_eeprom_device)
	AM_RANGE(0x0000, 0x000F) AM_READWRITE(eeprom_r, eeprom_w)
ADDRESS_MAP_END

MACHINE_CONFIG_FRAGMENT( iteagle_eeprom )
	MCFG_EEPROM_SERIAL_93C46_ADD("eeprom")
MACHINE_CONFIG_END

machine_config_constructor iteagle_eeprom_device::device_mconfig_additions() const
{
	return MACHINE_CONFIG_NAME( iteagle_eeprom );
}

iteagle_eeprom_device::iteagle_eeprom_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: pci_device(mconfig, ITEAGLE_EEPROM, "ITEagle EEPROM AT93C46", tag, owner, clock, "eeprom", __FILE__),
		m_eeprom(*this, "eeprom"), m_sw_version(0), m_hw_version(0)
{
	// When corrupt writes 0x3=2, 0x3e=2, 0xa=0, 0x30=0
	// 0x4 = HW Version - 6-8 is GREEN board PCB, 9 is RED board PCB
	// 0x5 = Serial Num + top byte of 0x4
	// 0x6 = OperID
	// 0xe = SW Version
	// 0xf = 0x01 for extra courses
	// 0x3e = 0x0002 for good nvram
	// 0x3f = checksum
	m_iteagle_default_eeprom =
	{ {
		0xd000,0x0022,0x0000,0x0003,0x1209,0x1111,0x2222,0x1234,
		0x0000,0x0000,0x0000,0x0000,0xcd00,0x0000,0x0000,0x0001,
		0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,
		0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,
		0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,
		0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,
		0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,
		0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0002,0x0000
	} };
}

void iteagle_eeprom_device::device_start()
{
	// EEPROM: Set software version and calc crc
	m_iteagle_default_eeprom[0xe] = m_sw_version;
	m_iteagle_default_eeprom[0x4] = (m_iteagle_default_eeprom[0x4] & 0xff00) | m_hw_version;
	uint16_t checkSum = 0;
	for (int i=0; i<0x3f; i++) {
		checkSum += m_iteagle_default_eeprom[i];
	//logerror("eeprom init i: %x data: %04x\n", i, iteagle_default_eeprom[i]);
	}
	m_iteagle_default_eeprom[0x3f] = checkSum;

	eeprom_base_device::static_set_default_data(*m_eeprom, m_iteagle_default_eeprom.data(), 0x80);

	pci_device::device_start();
	skip_map_regs(1);
	add_map(0x10, M_IO, FUNC(iteagle_eeprom_device::eeprom_map));
}

void iteagle_eeprom_device::device_reset()
{
	pci_device::device_reset();
}

void iteagle_eeprom_device::map_extra(uint64_t memory_window_start, uint64_t memory_window_end, uint64_t memory_offset, address_space *memory_space,
							uint64_t io_window_start, uint64_t io_window_end, uint64_t io_offset, address_space *io_space)
{
	m_memory_space = memory_space;
}

READ32_MEMBER( iteagle_eeprom_device::eeprom_r )
{
	uint32_t result = 0;

	switch (offset) {
		case 0xC/4: // I2C Handler
			if (ACCESSING_BITS_16_23) {
				result = m_eeprom->do_read()<<(16+3);
				if (LOG_EEPROM)
					logerror("%s:eeprom_r from offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, result, mem_mask);
			}   else {
					logerror("%s:eeprom_r from offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, result, mem_mask);
			}
			break;
		default:
				logerror("%s:eeprom read from offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, result, mem_mask);
			break;
	}
	return result;
}

WRITE32_MEMBER( iteagle_eeprom_device::eeprom_w )
{
	switch (offset) {
		case 0x8/4: // 8255x PORT command
			if ((data&0xf)==0x1) {
				// Self test for ethernet controller
				m_memory_space->write_dword((data&0xfffffff0) | 0x4, 0x0);
				logerror("%s:eeprom_w to offset %04X = %08X & %08X Self Test\n", machine().describe_context(), offset*4, data, mem_mask);
			}
			break;
		case 0xC/4: // I2C Handler
			if (ACCESSING_BITS_16_23) {
				m_eeprom->di_write((data  & 0x040000) >> (16+2));
				m_eeprom->cs_write((data  & 0x020000) ? ASSERT_LINE : CLEAR_LINE);
				m_eeprom->clk_write((data & 0x010000) ? ASSERT_LINE : CLEAR_LINE);
				if (LOG_EEPROM)
					logerror("%s:eeprom_w to offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, data, mem_mask);
			}   else {
				//if (LOG_EEPROM)
					logerror("%s:eeprom_w to offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, data, mem_mask);
			}
			break;
		default:
			//if (LOG_EEPROM)
				logerror("%s:eeprom write to offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, data, mem_mask);
			break;
	}
}

//************************************
// Attached Peripheral Controller
//************************************

MACHINE_CONFIG_FRAGMENT(eagle1)
	MCFG_NVRAM_ADD_0FILL("eagle1_rtc")
MACHINE_CONFIG_END

machine_config_constructor iteagle_periph_device::device_mconfig_additions() const
{
	return MACHINE_CONFIG_NAME(eagle1);
}

const device_type ITEAGLE_PERIPH = &device_creator<iteagle_periph_device>;

DEVICE_ADDRESS_MAP_START(ctrl_map, 32, iteagle_periph_device)
	AM_RANGE(0x000, 0x0cf) AM_READWRITE(ctrl_r, ctrl_w)
ADDRESS_MAP_END

iteagle_periph_device::iteagle_periph_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: pci_device(mconfig, ITEAGLE_PERIPH, "ITEagle Peripheral Controller", tag, owner, clock, "periph", __FILE__),
	m_rtc(*this, "eagle1_rtc")
{
}

void iteagle_periph_device::device_start()
{
	pci_device::device_start();
	pci_device::set_multifunction_device(true);
	add_map(sizeof(m_ctrl_regs), M_IO, FUNC(iteagle_periph_device::ctrl_map));
	// ctrl defaults to base address 0x00000000
	bank_infos[0].adr = 0x000;

	m_rtc_regs[0xa] = 0x20; // 32.768 MHz
	m_rtc_regs[0xb] = 0x02; // 24-hour format
	m_rtc->set_base(m_rtc_regs, sizeof(m_rtc_regs));
}

void iteagle_periph_device::device_reset()
{
	pci_device::device_reset();
	memset(m_ctrl_regs, 0, sizeof(m_ctrl_regs));
	m_ctrl_regs[0x10/4] =  0x00000000; // 0x6=No SIMM, 0x2, 0x1, 0x0 = SIMM .  Top 16 bits are compared to 0x3. Bit 0 might be lan chip present.
}

READ32_MEMBER( iteagle_periph_device::ctrl_r )
{
	system_time systime;
	uint32_t result = m_ctrl_regs[offset];
	switch (offset) {
		case 0x0/4:
			if (LOG_PERIPH)
				logerror("%s:fpga ctrl_r from offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, result, mem_mask);
			osd_printf_debug("%s:fpga ctrl_r from offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, result, mem_mask);
			break;
		case 0x70/4:
			if (ACCESSING_BITS_8_15) {
				// get the current date/time from the core
				machine().current_datetime(systime);
				m_rtc_regs[0] = dec_2_bcd(systime.local_time.second);
				m_rtc_regs[1] = 0x00; // Seconds Alarm
				m_rtc_regs[2] = dec_2_bcd(systime.local_time.minute);
				m_rtc_regs[3] = 0x00; // Minutes Alarm
				m_rtc_regs[4] = dec_2_bcd(systime.local_time.hour);
				m_rtc_regs[5] = 0x00; // Hours Alarm

				m_rtc_regs[6] = dec_2_bcd((systime.local_time.weekday != 0) ? systime.local_time.weekday : 7);
				m_rtc_regs[7] = dec_2_bcd(systime.local_time.mday);
				m_rtc_regs[8] = dec_2_bcd(systime.local_time.month + 1);
				m_rtc_regs[9] = dec_2_bcd(systime.local_time.year - 1900); // Epoch is 1900
				//m_rtc_regs[9] = 0x99; // Use 1998
				//m_rtc_regs[0xa] &= ~0x10; // Reg A Status
				//m_ctrl_regs[0xb] &= 0x10; // Reg B Status
				//m_ctrl_regs[0xc] &= 0x10; // Reg C Interrupt Status
				m_rtc_regs[0xd] = 0x80; // Reg D Valid time/ram Status
				result = (result & 0xffff00ff) | (m_rtc_regs[m_ctrl_regs[0x70/4]&0xff]<<8);
			}
			if (LOG_PERIPH)
				logerror("%s:fpga ctrl_r from offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, result, mem_mask);
			break;
		default:
			if (LOG_PERIPH)
				logerror("%s:fpga ctrl_r from offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, result, mem_mask);
			osd_printf_debug("%s:fpga ctrl_r from offset %04X = %08X & %08X\n", machine().describe_context(), offset*4, result, mem_mask);
			break;
	}
	return result;
}

WRITE32_MEMBER( iteagle_periph_device::ctrl_w )
{
	COMBINE_DATA(&m_ctrl_regs[offset]);
	switch (offset) {
		case 0x20/4: // IDE LED
			if (ACCESSING_BITS_16_23) {
				// Sets register index
			} else if (ACCESSING_BITS_24_31) {
				// Bit 25 is IDE LED
			} else {
			}
			break;
		case 0x70/4:
			if (ACCESSING_BITS_8_15) {
				m_rtc_regs[m_ctrl_regs[0x70/4]&0xff] = (data>>8)&0xff;
			}
		default:
			break;
	}
	if (LOG_PERIPH)
		logerror("%s:fpga ctrl_w to offset %04X = %08X & %08X\n", machine().describe_context(), offset * 4, data, mem_mask);
}
