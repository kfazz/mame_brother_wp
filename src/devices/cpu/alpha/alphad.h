// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

#ifndef MAME_CPU_ALPHA_ALPHAD_H
#define MAME_CPU_ALPHA_ALPHAD_H

#pragma once

class alpha_disassembler : public util::disasm_interface
{
public:
	alpha_disassembler() = default;
	virtual ~alpha_disassembler() = default;

	virtual u32 opcode_alignment() const override { return 4; };
	virtual offs_t disassemble(std::ostream &stream, offs_t pc, const data_buffer &opcodes, const data_buffer &params) override;

private:
	static char const *const R[];
	static char const *const F[];
};

#endif // MAME_CPU_ALPHA_ALPHAD_H
