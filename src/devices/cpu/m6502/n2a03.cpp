// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
/***************************************************************************

    n2a03.cpp

    6502, NES variant

***************************************************************************/

#include "emu.h"
#include "n2a03.h"
#include "n2a03d.h"

DEFINE_DEVICE_TYPE(N2A03, n2a03_device, "n2a03", "Ricoh N2A03")

READ8_MEMBER(n2a03_device::psg1_4014_r)
{
	return m_apu->read(space, 0x14);
}

READ8_MEMBER(n2a03_device::psg1_4015_r)
{
	return m_apu->read(space, 0x15);
}

WRITE8_MEMBER(n2a03_device::psg1_4015_w)
{
	m_apu->write(space, 0x15, data);
}

WRITE8_MEMBER(n2a03_device::psg1_4017_w)
{
	m_apu->write(space, 0x17, data);
}


// on various drivers          output port 0x4014 is used for external hardware   (not used by APU?)
//                       input/output port 0x4016  ^                              (not used by APU?)
//                       input        port 0x4017  ^                              ( APU_IRQCTRL )
// is there a fall through where every write is seen by other hw, or do these addresses really not touch the APU?? APU_IRQCTRL can definitely be written by can it be read back?

void n2a03_device::n2a03_map(address_map &map)
{
	map(0x4000, 0x4013).rw("nesapu", FUNC(nesapu_device::read), FUNC(nesapu_device::write));
	map(0x4014, 0x4014).r(FUNC(n2a03_device::psg1_4014_r)); // AM_WRITE(sprite_dma_0_w)
	map(0x4015, 0x4015).rw(FUNC(n2a03_device::psg1_4015_r), FUNC(n2a03_device::psg1_4015_w)); /* PSG status / first control register */
	//map(0x4016, 0x4016).rw(FUNC(n2a03_device::vsnes_in0_r), FUNC(n2a03_device::vsnes_in0_w));
	map(0x4017, 0x4017) /*.r(FUNC(n2a03_device::vsnes_in1_r))*/ .w(FUNC(n2a03_device::psg1_4017_w));
}



n2a03_device::n2a03_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: m6502_device(mconfig, N2A03, tag, owner, clock)
	, m_apu(*this, "nesapu")
{
	program_config.m_internal_map = address_map_constructor(FUNC(n2a03_device::n2a03_map), this);
}

std::unique_ptr<util::disasm_interface> n2a03_device::create_disassembler()
{
	return std::make_unique<n2a03_disassembler>();
}

void n2a03_device::device_start()
{
	if(!m_apu->started())
		throw device_missing_dependencies();

	m6502_device::device_start();
}

void n2a03_device::device_clock_changed()
{
	m_apu->set_unscaled_clock(clock());
}

WRITE_LINE_MEMBER(n2a03_device::apu_irq)
{
	// games relying on the APU_IRQ don't seem to work anyway? (nes software list : timelord, mig29sf, firehawk)
	set_input_line(N2A03_APU_IRQ_LINE, state ? ASSERT_LINE : CLEAR_LINE);
}

READ8_MEMBER(n2a03_device::apu_read_mem)
{
	return mintf->program->read_byte(offset);
}

MACHINE_CONFIG_START(n2a03_device::device_add_mconfig)
	MCFG_DEVICE_ADD("nesapu", NES_APU, DERIVED_CLOCK(1,1) )
	MCFG_NES_APU_IRQ_HANDLER(WRITELINE(*this, n2a03_device, apu_irq))
	MCFG_NES_APU_MEM_READ_CALLBACK(READ8(*this, n2a03_device, apu_read_mem))

	MCFG_SOUND_ROUTE(ALL_OUTPUTS, ":mono", 0.50)

MACHINE_CONFIG_END


#include "cpu/m6502/n2a03.hxx"
