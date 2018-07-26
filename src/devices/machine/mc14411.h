// license:BSD-3-Clause
// copyright-holders:Joakim Larsson Edstrom
/**********************************************************************

    Motorola MC14411 Bit Rate Generator

***********************************************************************
                            _____   _____
                     F1  1 |*    \_/     | 24  VDD
                     F3  2 |             | 23  Rate Select A
                     F5  3 |             | 22  Rate Select B
                     F7  4 |             | 21  Xtal In
                     F8  5 |             | 20  Xtal Out
                    F10  6 |             | 19  F16
                     F9  7 |   MC14411   | 18  F15
                    F11  8 |             | 17  F2
                    F14  9 |             | 16  F4
                 Reset* 10 |             | 15  F6
              Not Used  11 |             | 14  F12
                   VSS  12 |_____________| 13  F13

**********************************************************************/

#ifndef MAME_MACHINE_MC14411_H
#define MAME_MACHINE_MC14411_H

#pragma once


//**************************************************************************
//  DEVICE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_MC14411_F1_CB(_devcb)  downcast<mc14411_device &>(*device).set_out_fx_cb( 0, DEVCB_##_devcb);
#define MCFG_MC14411_F2_CB(_devcb)  downcast<mc14411_device &>(*device).set_out_fx_cb( 1, DEVCB_##_devcb);
#define MCFG_MC14411_F3_CB(_devcb)  downcast<mc14411_device &>(*device).set_out_fx_cb( 2, DEVCB_##_devcb);
#define MCFG_MC14411_F4_CB(_devcb)  downcast<mc14411_device &>(*device).set_out_fx_cb( 3, DEVCB_##_devcb);
#define MCFG_MC14411_F5_CB(_devcb)  downcast<mc14411_device &>(*device).set_out_fx_cb( 4, DEVCB_##_devcb);
#define MCFG_MC14411_F6_CB(_devcb)  downcast<mc14411_device &>(*device).set_out_fx_cb( 5, DEVCB_##_devcb);
#define MCFG_MC14411_F7_CB(_devcb)  downcast<mc14411_device &>(*device).set_out_fx_cb( 6, DEVCB_##_devcb);
#define MCFG_MC14411_F8_CB(_devcb)  downcast<mc14411_device &>(*device).set_out_fx_cb( 7, DEVCB_##_devcb);
#define MCFG_MC14411_F9_CB(_devcb)  downcast<mc14411_device &>(*device).set_out_fx_cb( 8, DEVCB_##_devcb);
#define MCFG_MC14411_F10_CB(_devcb) downcast<mc14411_device &>(*device).set_out_fx_cb( 9, DEVCB_##_devcb);
#define MCFG_MC14411_F11_CB(_devcb) downcast<mc14411_device &>(*device).set_out_fx_cb(10, DEVCB_##_devcb);
#define MCFG_MC14411_F12_CB(_devcb) downcast<mc14411_device &>(*device).set_out_fx_cb(11, DEVCB_##_devcb);
#define MCFG_MC14411_F13_CB(_devcb) downcast<mc14411_device &>(*device).set_out_fx_cb(12, DEVCB_##_devcb);
#define MCFG_MC14411_F14_CB(_devcb) downcast<mc14411_device &>(*device).set_out_fx_cb(13, DEVCB_##_devcb);
#define MCFG_MC14411_F15_CB(_devcb) downcast<mc14411_device &>(*device).set_out_fx_cb(14, DEVCB_##_devcb);
#define MCFG_MC14411_F16_CB(_devcb) downcast<mc14411_device &>(*device).set_out_fx_cb(15, DEVCB_##_devcb);

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************
class mc14411_device : public device_t
{
public:
	// timers
	enum timer_id // indexes
	{
		TIMER_F1 = 0,
		TIMER_F2 = 1,
		TIMER_F3 = 2,
		TIMER_F4 = 3,
		TIMER_F5 = 4,
		TIMER_F6 = 5,
		TIMER_F7 = 6,
		TIMER_F8 = 7,
		TIMER_F9 = 8,
		TIMER_F10 = 9,
		TIMER_F11 = 10,
		TIMER_F12 = 11,
		TIMER_F13 = 12,
		TIMER_F14 = 13,
		TIMER_F15 = 14,
		TIMER_F16 = 15,
		TIMER_ID_RESET = 16
	};

	// rate select inputs
	enum
	{
		RSA = 0x01,
		RSB = 0x02
	};

	// construction/destruction
	mc14411_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> devcb_base &set_out_fx_cb(int index, Object &&cb) { return m_out_fx_cbs[index].set_callback(std::forward<Object>(cb)); }
	auto out_f1_cb() { return m_out_fx_cbs[0].bind(); }
	auto out_f2_cb() { return m_out_fx_cbs[1].bind(); }
	auto out_f3_cb() { return m_out_fx_cbs[2].bind(); }
	auto out_f4_cb() { return m_out_fx_cbs[3].bind(); }
	auto out_f5_cb() { return m_out_fx_cbs[4].bind(); }
	auto out_f6_cb() { return m_out_fx_cbs[5].bind(); }
	auto out_f7_cb() { return m_out_fx_cbs[6].bind(); }
	auto out_f8_cb() { return m_out_fx_cbs[7].bind(); }
	auto out_f9_cb() { return m_out_fx_cbs[8].bind(); }
	auto out_f10_cb() { return m_out_fx_cbs[9].bind(); }
	auto out_f11_cb() { return m_out_fx_cbs[10].bind(); }
	auto out_f12_cb() { return m_out_fx_cbs[11].bind(); }
	auto out_f13_cb() { return m_out_fx_cbs[12].bind(); }
	auto out_f14_cb() { return m_out_fx_cbs[13].bind(); }
	auto out_f15_cb() { return m_out_fx_cbs[14].bind(); }
	auto out_f16_cb() { return m_out_fx_cbs[15].bind(); }

	DECLARE_WRITE_LINE_MEMBER(reset_w);
	DECLARE_WRITE8_MEMBER(rate_select_w);
	DECLARE_WRITE_LINE_MEMBER(rsa_w);
	DECLARE_WRITE_LINE_MEMBER(rsb_w);

	void timer_enable(timer_id i, bool enable);
	void timer_disable_all();

protected:
	mc14411_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_clock_changed() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	void arm_timer(int i);

	emu_timer *m_fx_timer[16];
	emu_timer *m_reset_timer;

	uint32_t m_fx_state[16]; // F1-F16 output line states

	// divider matrix
	static const int s_counter_divider[16];
	static const int s_divider_select[4];

	devcb_write_line m_out_fx_cbs[16];

	uint32_t m_divider; // main divider to use, 0-3 column index into counter_divider
	uint32_t m_reset;   // Reset line state

	bool m_timer_enabled[16];
};

// device type definition
DECLARE_DEVICE_TYPE(MC14411, mc14411_device)

#endif // MAME_MACHINE_MC14411_H
