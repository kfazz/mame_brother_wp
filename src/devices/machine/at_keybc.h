// license:BSD-3-Clause
// copyright-holders:Vas Crabb
/***************************************************************************

    IBM PC/AT and PS/2 keyboard controllers

***************************************************************************/
#ifndef MAME_MACHINE_AT_KEYBC_H
#define MAME_MACHINE_AT_KEYBC_H

#pragma once

#include "cpu/mcs48/mcs48.h"


//**************************************************************************
//  KEYBOARD CONTROLLER DEVICE BASE
//**************************************************************************

class at_kbc_device_base : public device_t
{
public:
	// outputs to host
	auto hot_res() { return m_hot_res_cb.bind(); }
	auto gate_a20() { return m_gate_a20_cb.bind(); }
	auto kbd_irq() { return m_kbd_irq_cb.bind(); }

	// outputs to keyboard
	auto kbd_clk() { return m_kbd_clk_cb.bind(); } // open collector with 10kΩ pull-up
	auto kbd_data() { return m_kbd_data_cb.bind(); } // open collector with 10kΩ pull-up

	// host interface
	virtual DECLARE_READ8_MEMBER(data_r);
	virtual DECLARE_READ8_MEMBER(status_r);
	DECLARE_WRITE8_MEMBER(data_w);
	DECLARE_WRITE8_MEMBER(command_w);

	// inputs from keyboard
	DECLARE_WRITE_LINE_MEMBER(kbd_clk_w);
	DECLARE_WRITE_LINE_MEMBER(kbd_data_w);

protected:
	// trampoline constructor
	at_kbc_device_base(machine_config const &mconfig, device_type type, char const *tag, device_t *owner, u32 clock);

	// device_t implementation
	virtual void device_resolve_objects() override;
	virtual void device_start() override;

	// host outputs - use 1 = asserted, 0 = deasserted
	void set_hot_res(u8 state);
	void set_gate_a20(u8 state);
	void set_kbd_irq(u8 state);

	// keyboard line drive - use 1 = pulled up, 0 = driven low
	void set_kbd_clk_out(u8 state);
	void set_kbd_data_out(u8 state);
	u8 kbd_clk_r() const;
	u8 kbd_data_r() const;

	required_device<upi41_cpu_device> m_mcu;

private:
	// internal sync helpers
	TIMER_CALLBACK_MEMBER(write_data);
	TIMER_CALLBACK_MEMBER(write_command);
	TIMER_CALLBACK_MEMBER(set_kbd_clk_in);
	TIMER_CALLBACK_MEMBER(set_kbd_data_in);

	devcb_write_line m_hot_res_cb, m_gate_a20_cb, m_kbd_irq_cb;
	devcb_write_line m_kbd_clk_cb, m_kbd_data_cb;

	u8 m_hot_res, m_gate_a20, m_kbd_irq;
	u8 m_kbd_clk_in, m_kbd_clk_out, m_kbd_data_in, m_kbd_data_out;
};


//**************************************************************************
//  PC/AT KEYBOARD CONTROLLER DEVICE
//**************************************************************************

class at_keyboard_controller_device : public at_kbc_device_base
{
public:
	// standard constructor
	at_keyboard_controller_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual tiny_rom_entry const *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual ioport_constructor device_input_ports() const override;

private:
	// MCU I/O handlers
	DECLARE_WRITE8_MEMBER(p2_w);
};


//**************************************************************************
//  PS/2 KEYBOARD/MOUSE CONTROLLER DEVICE
//**************************************************************************

class ps2_keyboard_controller_device : public at_kbc_device_base
{
public:
	// outputs to host
	auto mouse_irq() { return m_mouse_irq_cb.bind(); }

	// outputs to mouse
	auto mouse_clk() { return m_mouse_clk_cb.bind(); } // open collector with 10kΩ pull-up
	auto mouse_data() { return m_mouse_data_cb.bind(); } // open collector with 10kΩ pull-up

	// host interface
	virtual DECLARE_READ8_MEMBER(data_r) override;

	// inputs from mouse
	DECLARE_WRITE_LINE_MEMBER(mouse_clk_w);
	DECLARE_WRITE_LINE_MEMBER(mouse_data_w);

	// standard constructor
	ps2_keyboard_controller_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual tiny_rom_entry const *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_resolve_objects() override;
	virtual void device_start() override;

private:
	// host outputs - use 1 = asserted, 0 = deasserted
	void set_mouse_irq(u8 state);

	// mouse line drive - use 1 = pulled up, 0 = driven low
	void set_mouse_clk_out(u8 state);
	void set_mouse_data_out(u8 state);
	u8 mouse_clk_r() const;
	u8 mouse_data_r() const;

	// internal sync helpers
	TIMER_CALLBACK_MEMBER(set_mouse_clk_in);
	TIMER_CALLBACK_MEMBER(set_mouse_data_in);

	// MCU I/O handlers
	DECLARE_READ8_MEMBER(p1_r);
	DECLARE_WRITE8_MEMBER(p2_w);

	devcb_write_line m_mouse_irq_cb;
	devcb_write_line m_mouse_clk_cb, m_mouse_data_cb;

	u8 m_mouse_irq;
	u8 m_mouse_clk_in, m_mouse_clk_out, m_mouse_data_in, m_mouse_data_out;
	u8 m_p2_data;
};


//**************************************************************************
//  DEVICE TYPES
//**************************************************************************

DECLARE_DEVICE_TYPE(AT_KEYBOARD_CONTROLLER, at_keyboard_controller_device)
DECLARE_DEVICE_TYPE(PS2_KEYBOARD_CONTROLLER, ps2_keyboard_controller_device)

#endif // MAME_MACHINE_AT_KEYBC_H
