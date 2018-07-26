// license:BSD-3-Clause
// copyright-holders:Barry Rodewald
/*
    Hitachi HD63450 DMA Controller
*/
#ifndef MAME_MACHINE_HD63450_H
#define MAME_MACHINE_HD63450_H

#pragma once



#define MCFG_HD63450_DMA_END_CB(_devcb) \
	downcast<hd63450_device &>(*device).set_dma_end_callback(DEVCB_##_devcb);

#define MCFG_HD63450_DMA_ERROR_CB(_devcb) \
	downcast<hd63450_device &>(*device).set_dma_error_callback(DEVCB_##_devcb);

#define MCFG_HD63450_DMA_READ_0_CB(_devcb) \
	downcast<hd63450_device &>(*device).set_dma_read_callback<0>(DEVCB_##_devcb);

#define MCFG_HD63450_DMA_READ_1_CB(_devcb) \
	downcast<hd63450_device &>(*device).set_dma_read_callback<1>(DEVCB_##_devcb);

#define MCFG_HD63450_DMA_READ_2_CB(_devcb) \
	downcast<hd63450_device &>(*device).set_dma_read_callback<2>(DEVCB_##_devcb);

#define MCFG_HD63450_DMA_READ_3_CB(_devcb) \
	downcast<hd63450_device &>(*device).set_dma_read_callback<3>(DEVCB_##_devcb);

#define MCFG_HD63450_DMA_WRITE_0_CB(_devcb) \
	downcast<hd63450_device &>(*device).set_dma_write_callback<0>(DEVCB_##_devcb);

#define MCFG_HD63450_DMA_WRITE_1_CB(_devcb) \
	downcast<hd63450_device &>(*device).set_dma_write_callback<1>(DEVCB_##_devcb);

#define MCFG_HD63450_DMA_WRITE_2_CB(_devcb) \
	downcast<hd63450_device &>(*device).set_dma_write_callback<2>(DEVCB_##_devcb);

#define MCFG_HD63450_DMA_WRITE_3_CB(_devcb) \
	downcast<hd63450_device &>(*device).set_dma_write_callback<3>(DEVCB_##_devcb);

#define MCFG_HD63450_CPU(_tag) \
	downcast<hd63450_device &>(*device).set_cpu_tag(_tag);

#define MCFG_HD63450_CLOCKS(_clk1, _clk2, _clk3, _clk4) \
	downcast<hd63450_device &>(*device).set_our_clocks(_clk1, _clk2, _clk3, _clk4);

#define MCFG_HD63450_BURST_CLOCKS(_clk1, _clk2, _clk3, _clk4) \
	downcast<hd63450_device &>(*device).set_burst_clocks(_clk1, _clk2, _clk3, _clk4);

class hd63450_device : public device_t
{
public:
	hd63450_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> devcb_base &set_dma_end_callback(Object &&cb) { return m_dma_end.set_callback(std::forward<Object>(cb)); }
	template <class Object> devcb_base &set_dma_error_callback(Object &&cb) { return m_dma_error.set_callback(std::forward<Object>(cb)); }
	template <int Ch, class Object> devcb_base &set_dma_read_callback(Object &&cb) { return m_dma_read[Ch].set_callback(std::forward<Object>(cb)); }
	template <int Ch, class Object> devcb_base &set_dma_write_callback(Object &&cb) { return m_dma_write[Ch].set_callback(std::forward<Object>(cb)); }

	template <typename T> void set_cpu_tag(T &&cpu_tag) { m_cpu.set_tag(std::forward<T>(cpu_tag)); }
	void set_our_clocks(const attotime &clk1, const attotime &clk2, const attotime &clk3, const attotime &clk4)
	{
		m_our_clock[0] = clk1;
		m_our_clock[1] = clk2;
		m_our_clock[2] = clk3;
		m_our_clock[3] = clk4;
	}
	void set_burst_clocks(const attotime &clk1, const attotime &clk2, const attotime &clk3, const attotime &clk4)
	{
		m_burst_clock[0] = clk1;
		m_burst_clock[1] = clk2;
		m_burst_clock[2] = clk3;
		m_burst_clock[3] = clk4;
	}

	DECLARE_READ16_MEMBER( read );
	DECLARE_WRITE16_MEMBER( write );
	DECLARE_WRITE_LINE_MEMBER(drq0_w);
	DECLARE_WRITE_LINE_MEMBER(drq1_w);
	DECLARE_WRITE_LINE_MEMBER(drq2_w);
	DECLARE_WRITE_LINE_MEMBER(drq3_w);

	void single_transfer(int x);
	void set_timer(int channel, const attotime &tm);
	int get_vector(int channel);
	int get_error_vector(int channel);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	struct hd63450_regs
	{  // offsets in bytes
		uint8_t csr;  // [00] Channel status register (R/W)
		uint8_t cer;  // [01] Channel error register (R)
		uint8_t dcr;  // [04] Device control register (R/W)
		uint8_t ocr;  // [05] Operation control register (R/W)
		uint8_t scr;  // [06] Sequence control register (R/W)
		uint8_t ccr;  // [07] Channel control register (R/W)
		uint16_t mtc; // [0a,0b]  Memory Transfer Counter (R/W)
		uint32_t mar; // [0c-0f]  Memory Address Register (R/W)
		uint32_t dar; // [14-17]  Device Address Register (R/W)
		uint16_t btc; // [1a,1b]  Base Transfer Counter (R/W)
		uint32_t bar; // [1c-1f]  Base Address Register (R/W)
		uint8_t niv;  // [25]  Normal Interrupt Vector (R/W)
		uint8_t eiv;  // [27]  Error Interrupt Vector (R/W)
		uint8_t mfc;  // [29]  Memory Function Code (R/W)
		uint8_t cpr;  // [2d]  Channel Priority Register (R/W)
		uint8_t dfc;  // [31]  Device Function Code (R/W)
		uint8_t bfc;  // [39]  Base Function Code (R/W)
		uint8_t gcr;  // [3f]  General Control Register (R/W)
	};

	devcb_write8 m_dma_end;
	devcb_write8 m_dma_error;
	devcb_read8 m_dma_read[4];
	devcb_write8 m_dma_write[4];

	attotime m_our_clock[4];
	attotime m_burst_clock[4];

	// internal state
	hd63450_regs m_reg[4];
	emu_timer* m_timer[4];  // for timing data reading/writing each channel
	uint16_t m_transfer_size[4];
	bool m_halted[4];  // non-zero if a channel has been halted, and can be continued later.
	required_device<cpu_device> m_cpu;
	bool m_drq_state[4];

	// tell if a channel is in use
	bool dma_in_progress(int channel) const { return (m_reg[channel].csr & 0x08) != 0; }

	TIMER_CALLBACK_MEMBER(dma_transfer_timer);
	void dma_transfer_abort(int channel);
	void dma_transfer_halt(int channel);
	void dma_transfer_continue(int channel);
	void dma_transfer_start(int channel);
};

DECLARE_DEVICE_TYPE(HD63450, hd63450_device)

#endif // MAME_MACHINE_HD63450_H
