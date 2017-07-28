// license:BSD-3-Clause
// copyright-holders:Juergen Buchmueller,Ernesto Corvi
#ifndef MAME_CPU_Z8000_Z8000_H
#define MAME_CPU_Z8000_Z8000_H

#pragma once

#include "cpu/z80/z80daisy.h"


enum
{
	Z8000_PC=1,
	Z8000_NSPSEG, Z8000_NSPOFF, Z8000_FCW,
	Z8000_PSAPSEG, Z8000_PSAPOFF, Z8000_REFRESH,
	Z8000_IRQ_REQ, Z8000_IRQ_SRV, Z8000_IRQ_VEC,
	Z8000_R0, Z8000_R1, Z8000_R2, Z8000_R3,
	Z8000_R4, Z8000_R5, Z8000_R6, Z8000_R7,
	Z8000_R8, Z8000_R9, Z8000_R10, Z8000_R11,
	Z8000_R12, Z8000_R13, Z8000_R14, Z8000_R15
};

/* Interrupt Types that can be generated by outside sources */
#define Z8000_EPU       0x8000  /* extended instruction trap */
#define Z8000_TRAP      0x4000  /* privileged instruction trap */
#define Z8000_NMI       0x2000  /* non maskable interrupt */
#define Z8000_SEGTRAP   0x1000  /* segment trap (Z8001) */
#define Z8000_NVI       0x0800  /* non vectored interrupt */
#define Z8000_VI        0x0400  /* vectored interrupt (LSB is vector)  */
#define Z8000_SYSCALL   0x0200  /* system call (lsb is vector) */
#define Z8000_HALT      0x0100  /* halted flag  */

#define MCFG_Z8000_MO(_devcb) \
	devcb = &z8002_device::set_mo_callback(*device, DEVCB_##_devcb);

class z8002_device : public cpu_device, public z80_daisy_chain_interface
{
public:
	// construction/destruction
	z8002_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	~z8002_device();

	template <class Object> static devcb_base &set_mo_callback(device_t &device, Object &&cb) { return downcast<z8002_device &>(device).m_mo_out.set_callback(std::forward<Object>(cb)); }
	DECLARE_WRITE_LINE_MEMBER(mi_w) { m_mi = state; } // XXX: this has to apply in the middle of an insn for now

	struct Z8000_dasm { char const *dasm; uint32_t flags; int size; };

	static void init_tables();
	static void deinit_tables();
	static Z8000_dasm dasm(unsigned w);

protected:
	z8002_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, int addrbits, int iobits, int vecmult);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_execute_interface overrides
	virtual uint32_t execute_min_cycles() const override { return 2; }
	virtual uint32_t execute_max_cycles() const override { return 744; }
	virtual uint32_t execute_input_lines() const override { return 2; }
	virtual uint32_t execute_default_irq_vector() const override { return 0xff; }
	virtual void execute_run() override;
	virtual void execute_set_input(int inputnum, int state) override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// device_state_interface overrides
	virtual void state_string_export(const device_state_entry &entry, std::string &str) const override;

	// device_disasm_interface overrides
	virtual uint32_t disasm_min_opcode_bytes() const override { return 2; }
	virtual uint32_t disasm_max_opcode_bytes() const override { return 6; }
	virtual offs_t disasm_disassemble(std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, uint32_t options) override;

	address_space_config m_program_config;
	address_space_config m_io_config;
	devcb_write_line m_mo_out;

	uint32_t  m_op[4];      /* opcodes/data of current instruction */
	uint32_t  m_ppc;        /* previous program counter */
	uint32_t  m_pc;         /* program counter */
	uint16_t  m_psapseg;    /* program status pointer, segment (Z8001 only) */
	uint16_t  m_psapoff;    /* program status pointer, offset */
	uint16_t  m_fcw;        /* flags and control word */
	uint16_t  m_refresh;    /* refresh timer/counter */
	uint16_t  m_nspseg;     /* system stack pointer, segment (Z8001 only) */
	uint16_t  m_nspoff;     /* system stack pointer, offset */
	uint16_t  m_irq_req;    /* CPU is halted, interrupt or trap request */
	uint16_t  m_irq_vec;    /* interrupt vector */
	uint32_t  m_op_valid;   /* bit field indicating if given op[] field is already initialized */
	union
	{
		uint8_t   B[16]; /* RL0,RH0,RL1,RH1...RL7,RH7 */
		uint16_t  W[16]; /* R0,R1,R2...R15 */
		uint32_t  L[8];  /* RR0,RR2,RR4..RR14 */
		uint64_t  Q[4];  /* RQ0,RQ4,..RQ12 */
	} m_regs;             /* registers */
	int m_nmi_state;      /* NMI line state */
	int m_irq_state[2];   /* IRQ line states (NVI, VI) */
	int m_mi;
	address_space *m_program;
	address_space *m_data;
	direct_read_data *m_direct;
	address_space *m_io;
	int m_icount;
	int m_vector_mult;

	void clear_internal_state();
	void register_debug_state();
	virtual int segmented_mode();
	static inline uint32_t addr_add(uint32_t addr, uint32_t addend);
	static inline uint32_t addr_sub(uint32_t addr, uint32_t subtrahend);
	inline uint16_t RDOP();
	inline uint32_t get_operand(int opnum);
	inline uint32_t get_addr_operand(int opnum);
	inline uint32_t get_raw_addr_operand(int opnum);
	virtual uint32_t adjust_addr_for_nonseg_mode(uint32_t addr);
	inline uint8_t RDMEM_B(int spacenum, uint32_t addr);
	inline uint16_t RDMEM_W(int spacenum, uint32_t addr);
	inline uint32_t RDMEM_L(int spacenum, uint32_t addr);
	inline void WRMEM_B(int spacenum, uint32_t addr, uint8_t value);
	inline void WRMEM_W(int spacenum, uint32_t addr, uint16_t value);
	inline void WRMEM_L(int spacenum, uint32_t addr, uint32_t value);
	inline uint8_t RDPORT_B(int mode, uint16_t addr);
	virtual uint16_t RDPORT_W(int mode, uint16_t addr);
	inline void WRPORT_B(int mode, uint16_t addr, uint8_t value);
	virtual void WRPORT_W(int mode, uint16_t addr, uint16_t value);
	inline void cycles(int cycles);
	inline void set_irq(int type);
	virtual void PUSH_PC();
	virtual void CHANGE_FCW(uint16_t fcw);
	static inline uint32_t make_segmented_addr(uint32_t addr);
	static inline uint32_t segmented_addr(uint32_t addr);
	inline uint32_t addr_from_reg(int regno);
	inline void addr_to_reg(int regno, uint32_t addr);
	inline void add_to_addr_reg(int regno, uint16_t addend);
	inline void sub_from_addr_reg(int regno, uint16_t subtrahend);
	inline void set_pc(uint32_t addr);
	inline void PUSHW(uint8_t dst, uint16_t value);
	inline uint16_t POPW(uint8_t src);
	inline void PUSHL(uint8_t dst, uint32_t value);
	inline uint32_t POPL(uint8_t src);
	inline uint8_t ADDB(uint8_t dest, uint8_t value);
	inline uint16_t ADDW(uint16_t dest, uint16_t value);
	inline uint32_t ADDL(uint32_t dest, uint32_t value);
	inline uint8_t ADCB(uint8_t dest, uint8_t value);
	inline uint16_t ADCW(uint16_t dest, uint16_t value);
	inline uint8_t SUBB(uint8_t dest, uint8_t value);
	inline uint16_t SUBW(uint16_t dest, uint16_t value);
	inline uint32_t SUBL(uint32_t dest, uint32_t value);
	inline uint8_t SBCB(uint8_t dest, uint8_t value);
	inline uint16_t SBCW(uint16_t dest, uint16_t value);
	inline uint8_t ORB(uint8_t dest, uint8_t value);
	inline uint16_t ORW(uint16_t dest, uint16_t value);
	inline uint8_t ANDB(uint8_t dest, uint8_t value);
	inline uint16_t ANDW(uint16_t dest, uint16_t value);
	inline uint8_t XORB(uint8_t dest, uint8_t value);
	inline uint16_t XORW(uint16_t dest, uint16_t value);
	inline void CPB(uint8_t dest, uint8_t value);
	inline void CPW(uint16_t dest, uint16_t value);
	inline void CPL(uint32_t dest, uint32_t value);
	inline uint8_t COMB(uint8_t dest);
	inline uint16_t COMW(uint16_t dest);
	inline uint8_t NEGB(uint8_t dest);
	inline uint16_t NEGW(uint16_t dest);
	inline void TESTB(uint8_t result);
	inline void TESTW(uint16_t dest);
	inline void TESTL(uint32_t dest);
	inline uint8_t INCB(uint8_t dest, uint8_t value);
	inline uint16_t INCW(uint16_t dest, uint16_t value);
	inline uint8_t DECB(uint8_t dest, uint8_t value);
	inline uint16_t DECW(uint16_t dest, uint16_t value);
	inline uint32_t MULTW(uint16_t dest, uint16_t value);
	inline uint64_t MULTL(uint32_t dest, uint32_t value);
	inline uint32_t DIVW(uint32_t dest, uint16_t value);
	inline uint64_t DIVL(uint64_t dest, uint32_t value);
	inline uint8_t RLB(uint8_t dest, uint8_t twice);
	inline uint16_t RLW(uint16_t dest, uint8_t twice);
	inline uint8_t RLCB(uint8_t dest, uint8_t twice);
	inline uint16_t RLCW(uint16_t dest, uint8_t twice);
	inline uint8_t RRB(uint8_t dest, uint8_t twice);
	inline uint16_t RRW(uint16_t dest, uint8_t twice);
	inline uint8_t RRCB(uint8_t dest, uint8_t twice);
	inline uint16_t RRCW(uint16_t dest, uint8_t twice);
	inline uint8_t SDAB(uint8_t dest, int8_t count);
	inline uint16_t SDAW(uint16_t dest, int8_t count);
	inline uint32_t SDAL(uint32_t dest, int8_t count);
	inline uint8_t SDLB(uint8_t dest, int8_t count);
	inline uint16_t SDLW(uint16_t dest, int8_t count);
	inline uint32_t SDLL(uint32_t dest, int8_t count);
	inline uint8_t SLAB(uint8_t dest, uint8_t count);
	inline uint16_t SLAW(uint16_t dest, uint8_t count);
	inline uint32_t SLAL(uint32_t dest, uint8_t count);
	inline uint8_t SLLB(uint8_t dest, uint8_t count);
	inline uint16_t SLLW(uint16_t dest, uint8_t count);
	inline uint32_t SLLL(uint32_t dest, uint8_t count);
	inline uint8_t SRAB(uint8_t dest, uint8_t count);
	inline uint16_t SRAW(uint16_t dest, uint8_t count);
	inline uint32_t SRAL(uint32_t dest, uint8_t count);
	inline uint8_t SRLB(uint8_t dest, uint8_t count);
	inline uint16_t SRLW(uint16_t dest, uint8_t count);
	inline uint32_t SRLL(uint32_t dest, uint8_t count);
	inline void Interrupt();
	virtual uint32_t GET_PC(uint32_t VEC);
	virtual uint16_t GET_FCW(uint32_t VEC);
	virtual uint32_t F_SEG_Z8001();
	virtual uint32_t PSA_ADDR();
	virtual uint32_t read_irq_vector();

	void zinvalid();
	void Z00_0000_dddd_imm8();
	void Z00_ssN0_dddd();
	void Z01_0000_dddd_imm16();
	void Z01_ssN0_dddd();
	void Z02_0000_dddd_imm8();
	void Z02_ssN0_dddd();
	void Z03_0000_dddd_imm16();
	void Z03_ssN0_dddd();
	void Z04_0000_dddd_imm8();
	void Z04_ssN0_dddd();
	void Z05_0000_dddd_imm16();
	void Z05_ssN0_dddd();
	void Z06_0000_dddd_imm8();
	void Z06_ssN0_dddd();
	void Z07_0000_dddd_imm16();
	void Z07_ssN0_dddd();
	void Z08_0000_dddd_imm8();
	void Z08_ssN0_dddd();
	void Z09_0000_dddd_imm16();
	void Z09_ssN0_dddd();
	void Z0A_0000_dddd_imm8();
	void Z0A_ssN0_dddd();
	void Z0B_0000_dddd_imm16();
	void Z0B_ssN0_dddd();
	void Z0C_ddN0_0000();
	void Z0C_ddN0_0001_imm8();
	void Z0C_ddN0_0010();
	void Z0C_ddN0_0100();
	void Z0C_ddN0_0101_imm8();
	void Z0C_ddN0_0110();
	void Z0C_ddN0_1000();
	void Z0D_ddN0_0000();
	void Z0D_ddN0_0001_imm16();
	void Z0D_ddN0_0010();
	void Z0D_ddN0_0100();
	void Z0D_ddN0_0101_imm16();
	void Z0D_ddN0_0110();
	void Z0D_ddN0_1000();
	void Z0D_ddN0_1001_imm16();
	void Z0E_imm8();
	void Z0F_imm8();
	void Z10_0000_dddd_imm32();
	void Z10_ssN0_dddd();
	void Z11_ddN0_ssN0();
	void Z12_0000_dddd_imm32();
	void Z12_ssN0_dddd();
	void Z13_ddN0_ssN0();
	void Z14_0000_dddd_imm32();
	void Z14_ssN0_dddd();
	void Z15_ssN0_ddN0();
	void Z16_0000_dddd_imm32();
	void Z16_ssN0_dddd();
	void Z17_ssN0_ddN0();
	void Z18_00N0_dddd_imm32();
	void Z18_ssN0_dddd();
	void Z19_0000_dddd_imm16();
	void Z19_ssN0_dddd();
	void Z1A_0000_dddd_imm32();
	void Z1A_ssN0_dddd();
	void Z1B_0000_dddd_imm16();
	void Z1B_ssN0_dddd();
	void Z1C_ddN0_1000();
	void Z1C_ddN0_1001_0000_ssss_0000_nmin1();
	void Z1C_ssN0_0001_0000_dddd_0000_nmin1();
	void Z1D_ddN0_ssss();
	void Z1E_ddN0_cccc();
	void Z1F_ddN0_0000();
	void Z20_ssN0_dddd();
	void Z21_0000_dddd_imm16();
	void Z21_ssN0_dddd();
	void Z22_0000_ssss_0000_dddd_0000_0000();
	void Z22_ddN0_imm4();
	void Z23_0000_ssss_0000_dddd_0000_0000();
	void Z23_ddN0_imm4();
	void Z24_0000_ssss_0000_dddd_0000_0000();
	void Z24_ddN0_imm4();
	void Z25_0000_ssss_0000_dddd_0000_0000();
	void Z25_ddN0_imm4();
	void Z26_0000_ssss_0000_dddd_0000_0000();
	void Z26_ddN0_imm4();
	void Z27_0000_ssss_0000_dddd_0000_0000();
	void Z27_ddN0_imm4();
	void Z28_ddN0_imm4m1();
	void Z29_ddN0_imm4m1();
	void Z2A_ddN0_imm4m1();
	void Z2B_ddN0_imm4m1();
	void Z2C_ssN0_dddd();
	void Z2D_ssN0_dddd();
	void Z2E_ddN0_ssss();
	void Z2F_ddN0_ssss();
	void Z30_0000_dddd_dsp16();
	void Z30_ssN0_dddd_imm16();
	void Z31_0000_dddd_dsp16();
	void Z31_ssN0_dddd_imm16();
	void Z32_0000_ssss_dsp16();
	void Z32_ddN0_ssss_imm16();
	void Z33_0000_ssss_dsp16();
	void Z33_ddN0_ssss_imm16();
	void Z34_0000_dddd_dsp16();
	void Z34_ssN0_dddd_imm16();
	void Z35_0000_dddd_dsp16();
	void Z35_ssN0_dddd_imm16();
	void Z36_0000_0000();
	void Z36_imm8();
	void Z37_0000_ssss_dsp16();
	void Z37_ddN0_ssss_imm16();
	void Z38_imm8();
	void Z39_ssN0_0000();
	void Z3A_ssss_0000_0000_aaaa_dddd_x000();
	void Z3A_ssss_0001_0000_aaaa_dddd_x000();
	void Z3A_ssss_0010_0000_aaaa_dddd_x000();
	void Z3A_ssss_0011_0000_aaaa_dddd_x000();
	void Z3A_dddd_0100_imm16();
	void Z3A_dddd_0101_imm16();
	void Z3A_ssss_0110_imm16();
	void Z3A_ssss_0111_imm16();
	void Z3A_ssss_1000_0000_aaaa_dddd_x000();
	void Z3A_ssss_1001_0000_aaaa_dddd_x000();
	void Z3A_ssss_1010_0000_aaaa_dddd_x000();
	void Z3A_ssss_1011_0000_aaaa_dddd_x000();
	void Z3B_ssss_0000_0000_aaaa_dddd_x000();
	void Z3B_ssss_0001_0000_aaaa_dddd_x000();
	void Z3B_ssss_0010_0000_aaaa_dddd_x000();
	void Z3B_ssss_0011_0000_aaaa_dddd_x000();
	void Z3B_dddd_0100_imm16();
	void Z3B_dddd_0101_imm16();
	void Z3B_ssss_0110_imm16();
	void Z3B_ssss_0111_imm16();
	void Z3B_ssss_1000_0000_aaaa_dddd_x000();
	void Z3B_ssss_1001_0000_aaaa_dddd_x000();
	void Z3B_ssss_1010_0000_aaaa_dddd_x000();
	void Z3B_ssss_1011_0000_aaaa_dddd_x000();
	void Z3C_ssss_dddd();
	void Z3D_ssss_dddd();
	void Z3E_dddd_ssss();
	void Z3F_dddd_ssss();
	void Z40_0000_dddd_addr();
	void Z40_ssN0_dddd_addr();
	void Z41_0000_dddd_addr();
	void Z41_ssN0_dddd_addr();
	void Z42_0000_dddd_addr();
	void Z42_ssN0_dddd_addr();
	void Z43_0000_dddd_addr();
	void Z43_ssN0_dddd_addr();
	void Z44_0000_dddd_addr();
	void Z44_ssN0_dddd_addr();
	void Z45_0000_dddd_addr();
	void Z45_ssN0_dddd_addr();
	void Z46_0000_dddd_addr();
	void Z46_ssN0_dddd_addr();
	void Z47_0000_dddd_addr();
	void Z47_ssN0_dddd_addr();
	void Z48_0000_dddd_addr();
	void Z48_ssN0_dddd_addr();
	void Z49_0000_dddd_addr();
	void Z49_ssN0_dddd_addr();
	void Z4A_0000_dddd_addr();
	void Z4A_ssN0_dddd_addr();
	void Z4B_0000_dddd_addr();
	void Z4B_ssN0_dddd_addr();
	void Z4C_0000_0000_addr();
	void Z4C_0000_0001_addr_imm8();
	void Z4C_0000_0010_addr();
	void Z4C_0000_0100_addr();
	void Z4C_0000_0101_addr_imm8();
	void Z4C_0000_0110_addr();
	void Z4C_0000_1000_addr();
	void Z4C_ddN0_0000_addr();
	void Z4C_ddN0_0001_addr_imm8();
	void Z4C_ddN0_0010_addr();
	void Z4C_ddN0_0100_addr();
	void Z4C_ddN0_0101_addr_imm8();
	void Z4C_ddN0_0110_addr();
	void Z4C_ddN0_1000_addr();
	void Z4D_0000_0000_addr();
	void Z4D_0000_0001_addr_imm16();
	void Z4D_0000_0010_addr();
	void Z4D_0000_0100_addr();
	void Z4D_0000_0101_addr_imm16();
	void Z4D_0000_0110_addr();
	void Z4D_0000_1000_addr();
	void Z4D_ddN0_0000_addr();
	void Z4D_ddN0_0001_addr_imm16();
	void Z4D_ddN0_0010_addr();
	void Z4D_ddN0_0100_addr();
	void Z4D_ddN0_0101_addr_imm16();
	void Z4D_ddN0_0110_addr();
	void Z4D_ddN0_1000_addr();
	void Z4E_ddN0_ssN0_addr();
	void Z50_0000_dddd_addr();
	void Z50_ssN0_dddd_addr();
	void Z51_ddN0_0000_addr();
	void Z51_ddN0_ssN0_addr();
	void Z52_0000_dddd_addr();
	void Z52_ssN0_dddd_addr();
	void Z53_ddN0_0000_addr();
	void Z53_ddN0_ssN0_addr();
	void Z54_0000_dddd_addr();
	void Z54_ssN0_dddd_addr();
	void Z55_ssN0_0000_addr();
	void Z55_ssN0_ddN0_addr();
	void Z56_0000_dddd_addr();
	void Z56_ssN0_dddd_addr();
	void Z57_ssN0_0000_addr();
	void Z57_ssN0_ddN0_addr();
	void Z58_0000_dddd_addr();
	void Z58_ssN0_dddd_addr();
	void Z59_0000_dddd_addr();
	void Z59_ssN0_dddd_addr();
	void Z5A_0000_dddd_addr();
	void Z5A_ssN0_dddd_addr();
	void Z5B_0000_dddd_addr();
	void Z5B_ssN0_dddd_addr();
	void Z5C_0000_0001_0000_dddd_0000_nmin1_addr();
	void Z5C_0000_1000_addr();
	void Z5C_0000_1001_0000_ssss_0000_nmin1_addr();
	void Z5C_ddN0_1000_addr();
	void Z5C_ddN0_1001_0000_ssN0_0000_nmin1_addr();
	void Z5C_ssN0_0001_0000_dddd_0000_nmin1_addr();
	void Z5D_0000_ssss_addr();
	void Z5D_ddN0_ssss_addr();
	void Z5E_0000_cccc_addr();
	void Z5E_ddN0_cccc_addr();
	void Z5F_0000_0000_addr();
	void Z5F_ddN0_0000_addr();
	void Z60_0000_dddd_addr();
	void Z60_ssN0_dddd_addr();
	void Z61_0000_dddd_addr();
	void Z61_ssN0_dddd_addr();
	void Z62_0000_imm4_addr();
	void Z62_ddN0_imm4_addr();
	void Z63_0000_imm4_addr();
	void Z63_ddN0_imm4_addr();
	void Z64_0000_imm4_addr();
	void Z64_ddN0_imm4_addr();
	void Z65_0000_imm4_addr();
	void Z65_ddN0_imm4_addr();
	void Z66_0000_imm4_addr();
	void Z66_ddN0_imm4_addr();
	void Z67_0000_imm4_addr();
	void Z67_ddN0_imm4_addr();
	void Z68_0000_imm4m1_addr();
	void Z68_ddN0_imm4m1_addr();
	void Z69_0000_imm4m1_addr();
	void Z69_ddN0_imm4m1_addr();
	void Z6A_0000_imm4m1_addr();
	void Z6A_ddN0_imm4m1_addr();
	void Z6B_0000_imm4m1_addr();
	void Z6B_ddN0_imm4m1_addr();
	void Z6C_0000_dddd_addr();
	void Z6C_ssN0_dddd_addr();
	void Z6D_0000_dddd_addr();
	void Z6D_ssN0_dddd_addr();
	void Z6E_0000_ssss_addr();
	void Z6E_ddN0_ssss_addr();
	void Z6F_0000_ssss_addr();
	void Z6F_ddN0_ssss_addr();
	void Z70_ssN0_dddd_0000_xxxx_0000_0000();
	void Z71_ssN0_dddd_0000_xxxx_0000_0000();
	void Z72_ddN0_ssss_0000_xxxx_0000_0000();
	void Z73_ddN0_ssss_0000_xxxx_0000_0000();
	void Z74_ssN0_dddd_0000_xxxx_0000_0000();
	void Z75_ssN0_dddd_0000_xxxx_0000_0000();
	void Z76_0000_dddd_addr();
	void Z76_ssN0_dddd_addr();
	void Z77_ddN0_ssss_0000_xxxx_0000_0000();
	void Z78_imm8();
	void Z79_0000_0000_addr();
	void Z79_ssN0_0000_addr();
	void Z7A_0000_0000();
	void Z7B_0000_0000();
	void Z7B_0000_1000();
	void Z7B_0000_1001();
	void Z7B_0000_1010();
	void Z7B_dddd_1101();
	void Z7C_0000_00ii();
	void Z7C_0000_01ii();
	void Z7D_dddd_0ccc();
	void Z7D_ssss_1ccc();
	void Z7E_imm8();
	void Z7F_imm8();
	void Z80_ssss_dddd();
	void Z81_ssss_dddd();
	void Z82_ssss_dddd();
	void Z83_ssss_dddd();
	void Z84_ssss_dddd();
	void Z85_ssss_dddd();
	void Z86_ssss_dddd();
	void Z87_ssss_dddd();
	void Z88_ssss_dddd();
	void Z89_ssss_dddd();
	void Z8A_ssss_dddd();
	void Z8B_ssss_dddd();
	void Z8C_dddd_0000();
	void Z8C_dddd_0010();
	void Z8C_dddd_0100();
	void Z8C_dddd_0110();
	void Z8C_dddd_0001();
	void Z8C_dddd_1000();
	void Z8C_dddd_1001();
	void Z8D_0000_0111();
	void Z8D_dddd_0000();
	void Z8D_dddd_0010();
	void Z8D_dddd_0100();
	void Z8D_dddd_0110();
	void Z8D_dddd_1000();
	void Z8D_imm4_0001();
	void Z8D_imm4_0011();
	void Z8D_imm4_0101();
	void Z8E_imm8();
	void Z8F_imm8();
	void Z90_ssss_dddd();
	void Z91_ddN0_ssss();
	void Z92_ssss_dddd();
	void Z93_ddN0_ssss();
	void Z94_ssss_dddd();
	void Z95_ssN0_dddd();
	void Z96_ssss_dddd();
	void Z97_ssN0_dddd();
	void Z98_ssss_dddd();
	void Z99_ssss_dddd();
	void Z9A_ssss_dddd();
	void Z9B_ssss_dddd();
	void Z9C_dddd_1000();
	void Z9D_imm8();
	void Z9E_0000_cccc();
	void Z9F_imm8();
	void ZA0_ssss_dddd();
	void ZA1_ssss_dddd();
	void ZA2_dddd_imm4();
	void ZA3_dddd_imm4();
	void ZA4_dddd_imm4();
	void ZA5_dddd_imm4();
	void ZA6_dddd_imm4();
	void ZA7_dddd_imm4();
	void ZA8_dddd_imm4m1();
	void ZA9_dddd_imm4m1();
	void ZAA_dddd_imm4m1();
	void ZAB_dddd_imm4m1();
	void ZAC_ssss_dddd();
	void ZAD_ssss_dddd();
	void ZAE_dddd_cccc();
	void ZAF_dddd_cccc();
	void ZB0_dddd_0000();
	void ZB1_dddd_0000();
	void ZB1_dddd_0111();
	void ZB1_dddd_1010();
	void ZB2_dddd_0001_imm8();
	void ZB2_dddd_0011_0000_ssss_0000_0000();
	void ZB2_dddd_00I0();
	void ZB2_dddd_01I0();
	void ZB2_dddd_1001_imm8();
	void ZB2_dddd_1011_0000_ssss_0000_0000();
	void ZB2_dddd_10I0();
	void ZB2_dddd_11I0();
	void ZB3_dddd_0001_imm8();
	void ZB3_dddd_0011_0000_ssss_0000_0000();
	void ZB3_dddd_00I0();
	void ZB3_dddd_0101_imm8();
	void ZB3_dddd_0111_0000_ssss_0000_0000();
	void ZB3_dddd_01I0();
	void ZB3_dddd_1001_imm8();
	void ZB3_dddd_1011_0000_ssss_0000_0000();
	void ZB3_dddd_10I0();
	void ZB3_dddd_1101_imm8();
	void ZB3_dddd_1111_0000_ssss_0000_0000();
	void ZB3_dddd_11I0();
	void ZB4_ssss_dddd();
	void ZB5_ssss_dddd();
	void ZB6_ssss_dddd();
	void ZB7_ssss_dddd();
	void ZB8_ddN0_0010_0000_rrrr_ssN0_0000();
	void ZB8_ddN0_0110_0000_rrrr_ssN0_1110();
	void ZB8_ddN0_1010_0000_rrrr_ssN0_0000();
	void ZB8_ddN0_1110_0000_rrrr_ssN0_1110();
	void ZB8_ddN0_0000_0000_rrrr_ssN0_0000();
	void ZB8_ddN0_0100_0000_rrrr_ssN0_0000();
	void ZB8_ddN0_1000_0000_rrrr_ssN0_0000();
	void ZB8_ddN0_1100_0000_rrrr_ssN0_0000();
	void ZB9_imm8();
	void ZBA_ssN0_0000_0000_rrrr_dddd_cccc();
	void ZBA_ssN0_0001_0000_rrrr_ddN0_x000();
	void ZBA_ssN0_0010_0000_rrrr_ddN0_cccc();
	void ZBA_ssN0_0100_0000_rrrr_dddd_cccc();
	void ZBA_ssN0_0110_0000_rrrr_ddN0_cccc();
	void ZBA_ssN0_1000_0000_rrrr_dddd_cccc();
	void ZBA_ssN0_1001_0000_rrrr_ddN0_x000();
	void ZBA_ssN0_1010_0000_rrrr_ddN0_cccc();
	void ZBA_ssN0_1100_0000_rrrr_dddd_cccc();
	void ZBA_ssN0_1110_0000_rrrr_ddN0_cccc();
	void ZBB_ssN0_0000_0000_rrrr_dddd_cccc();
	void ZBB_ssN0_0001_0000_rrrr_ddN0_x000();
	void ZBB_ssN0_0010_0000_rrrr_ddN0_cccc();
	void ZBB_ssN0_0100_0000_rrrr_dddd_cccc();
	void ZBB_ssN0_0110_0000_rrrr_ddN0_cccc();
	void ZBB_ssN0_1000_0000_rrrr_dddd_cccc();
	void ZBB_ssN0_1001_0000_rrrr_ddN0_x000();
	void ZBB_ssN0_1010_0000_rrrr_ddN0_cccc();
	void ZBB_ssN0_1100_0000_rrrr_dddd_cccc();
	void ZBB_ssN0_1110_0000_rrrr_ddN0_cccc();
	void ZBC_aaaa_bbbb();
	void ZBD_dddd_imm4();
	void ZBE_aaaa_bbbb();
	void ZBF_imm8();
	void Z20_0000_dddd_imm8();
	void ZC_dddd_imm8();
	void ZD_dsp12();
	void ZE_cccc_dsp8();
	void ZF_dddd_0dsp7();
	void ZF_dddd_1dsp7();

private:
	// structure for the opcode definition table
	typedef void (z8002_device::*opcode_func)();

	struct Z8000_init {
		int     beg, end, step;
		int     size, cycles;
		opcode_func opcode;
		const char  *dasm;
		uint32_t dasmflags;
	};

	/* structure for the opcode execution table / disassembler */
	struct Z8000_exec {
		opcode_func opcode;
		int     cycles;
		int     size;
		const char    *dasm;
		uint32_t dasmflags;
	};

	/* opcode execution table */
	static const Z8000_init table[];
	static std::unique_ptr<Z8000_exec const []> z8000_exec;
};


class z8001_device : public z8002_device
{
public:
	// construction/destruction
	z8001_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// device_disasm_interface overrides
	virtual uint32_t disasm_max_opcode_bytes() const override { return 8; }

	address_space_config m_data_config;

	virtual int segmented_mode() override;
	virtual uint32_t adjust_addr_for_nonseg_mode(uint32_t addr) override;
	virtual uint16_t RDPORT_W(int mode, uint16_t addr) override;
	virtual void WRPORT_W(int mode, uint16_t addr, uint16_t value) override;
	virtual void PUSH_PC() override;
	virtual void CHANGE_FCW(uint16_t fcw) override;
	virtual uint32_t GET_PC(uint32_t VEC) override;
	virtual uint16_t GET_FCW(uint32_t VEC) override;
	virtual uint32_t F_SEG_Z8001() override;
	virtual uint32_t PSA_ADDR() override;
	virtual uint32_t read_irq_vector() override;

private:
	void z8k_disass_mode(int ref, const std::vector<std::string> &params);
};


DECLARE_DEVICE_TYPE(Z8001, z8001_device)
DECLARE_DEVICE_TYPE(Z8002, z8002_device)


/* possible values for z8k_segm_mode */
#define Z8K_SEGM_MODE_NONSEG 0
#define Z8K_SEGM_MODE_SEG    1
#define Z8K_SEGM_MODE_AUTO   2

#endif // MAME_CPU_Z8000_Z8000_H
