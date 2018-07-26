// license:BSD-3-Clause
// copyright-holders:Aaron Giles, Nathan Woods
/**********************************************************************

    Motorola 6821 PIA interface and emulation

    Notes:
        * get_port_b_z_mask() gives the caller the bitmask that shows
          which bits are high-impedance when reading port B, and thus
          neither 0 or 1. get_output_cb2_z() returns the same info
          for the CB2 pin.
        * set_port_a_z_mask allows the input callback to indicate
          which port A bits are disconnected. For these bits, the
          read operation will return the output buffer's contents.
        * The 'alt' interface functions are used when the A0 and A1
          address bits are swapped.
        * All 'int' data or return values are bool, and should be
          converted to bool at some point.

**********************************************************************/

#ifndef MAME_DEVICES_MACHINE_6821PIA_H
#define MAME_DEVICES_MACHINE_6821PIA_H

#pragma once




/***************************************************************************
    DEVICE CONFIGURATION MACROS
***************************************************************************/

// TODO: REMOVE THESE
#define MCFG_PIA_READPA_HANDLER(_devcb) \
	downcast<pia6821_device &>(*device).set_readpa_handler(DEVCB_##_devcb);

#define MCFG_PIA_READPB_HANDLER(_devcb) \
	downcast<pia6821_device &>(*device).set_readpb_handler(DEVCB_##_devcb);

#define MCFG_PIA_READCA1_HANDLER(_devcb) \
	downcast<pia6821_device &>(*device).set_readca1_handler(DEVCB_##_devcb);

#define MCFG_PIA_READCA2_HANDLER(_devcb) \
	downcast<pia6821_device &>(*device).set_readca2_handler(DEVCB_##_devcb);

#define MCFG_PIA_READCB1_HANDLER(_devcb) \
	downcast<pia6821_device &>(*device).set_readcb1_handler(DEVCB_##_devcb);

// TODO: CONVERT THESE TO WRITE LINE
#define MCFG_PIA_WRITEPA_HANDLER(_devcb) \
	downcast<pia6821_device &>(*device).set_writepa_handler(DEVCB_##_devcb);

#define MCFG_PIA_WRITEPB_HANDLER(_devcb) \
	downcast<pia6821_device &>(*device).set_writepb_handler(DEVCB_##_devcb);

#define MCFG_PIA_CA2_HANDLER(_devcb) \
	downcast<pia6821_device &>(*device).set_ca2_handler(DEVCB_##_devcb);

#define MCFG_PIA_CB2_HANDLER(_devcb) \
	downcast<pia6821_device &>(*device).set_cb2_handler(DEVCB_##_devcb);

#define MCFG_PIA_IRQA_HANDLER(_devcb) \
	downcast<pia6821_device &>(*device).set_irqa_handler(DEVCB_##_devcb);

#define MCFG_PIA_IRQB_HANDLER(_devcb) \
	downcast<pia6821_device &>(*device).set_irqb_handler(DEVCB_##_devcb);


/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

// ======================> pia6821_device

class pia6821_device :  public device_t
{
public:
	// construction/destruction
	pia6821_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	// static configuration helpers
	// TODO: REMOVE THESE
	template<class Obj> devcb_base &set_readpa_handler(Obj &&object) { return m_in_a_handler.set_callback(std::forward<Obj>(object)); }
	template<class Obj> devcb_base &set_readpb_handler(Obj &&object) { return m_in_b_handler.set_callback(std::forward<Obj>(object)); }
	template<class Obj> devcb_base &set_readca1_handler(Obj &&object) { return m_in_ca1_handler.set_callback(std::forward<Obj>(object)); }
	template<class Obj> devcb_base &set_readca2_handler(Obj &&object) { return m_in_ca2_handler.set_callback(std::forward<Obj>(object)); }
	template<class Obj> devcb_base &set_readcb1_handler(Obj &&object) { return m_in_cb1_handler.set_callback(std::forward<Obj>(object)); }
	auto readpa_handler() { return m_in_a_handler.bind(); }
	auto readpb_handler() { return m_in_b_handler.bind(); }
	auto readca1_handler() { return m_in_ca1_handler.bind(); }
	auto readca2_handler() { return m_in_ca2_handler.bind(); }
	auto readcb1_handler() { return m_in_cb1_handler.bind(); }

	// TODO: CONVERT THESE TO WRITE LINE
	template<class Obj> devcb_base &set_writepa_handler(Obj &&object) { return m_out_a_handler.set_callback(std::forward<Obj>(object)); }
	template<class Obj> devcb_base &set_writepb_handler(Obj &&object) { return m_out_b_handler.set_callback(std::forward<Obj>(object)); }
	auto writepa_handler() { return m_out_a_handler.bind(); }
	auto writepb_handler() { return m_out_b_handler.bind(); }

	template<class Obj> devcb_base &set_ca2_handler(Obj &&object) { return m_ca2_handler.set_callback(std::forward<Obj>(object)); }
	template<class Obj> devcb_base &set_cb2_handler(Obj &&object) { return m_cb2_handler.set_callback(std::forward<Obj>(object)); }
	template<class Obj> devcb_base &set_irqa_handler(Obj &&object) { return m_irqa_handler.set_callback(std::forward<Obj>(object)); }
	template<class Obj> devcb_base &set_irqb_handler(Obj &&object) { return m_irqb_handler.set_callback(std::forward<Obj>(object)); }
	auto ca2_handler() { return m_ca2_handler.bind(); }
	auto cb2_handler() { return m_cb2_handler.bind(); }
	auto irqa_handler() { return m_irqa_handler.bind(); }
	auto irqb_handler() { return m_irqb_handler.bind(); }

	uint8_t reg_r(uint8_t offset);
	void reg_w(uint8_t offset, uint8_t data);

	DECLARE_READ8_MEMBER( read ) { return reg_r(offset); }
	DECLARE_WRITE8_MEMBER( write ) { reg_w(offset, data); }
	DECLARE_READ8_MEMBER( read_alt ) { return reg_r(((offset << 1) & 0x02) | ((offset >> 1) & 0x01)); }
	DECLARE_WRITE8_MEMBER( write_alt ) { reg_w(((offset << 1) & 0x02) | ((offset >> 1) & 0x01), data); }

	uint8_t port_b_z_mask() const { return ~m_ddr_b; }          // see first note in .c
	void set_port_a_z_mask(uint8_t data) { m_port_a_z_mask = data; }// see second note in .c

	DECLARE_WRITE8_MEMBER( porta_w ) { write_porta(data); }
	void write_porta(uint8_t data);
	void set_a_input(uint8_t data, uint8_t z_mask);
	uint8_t a_output();

	DECLARE_WRITE_LINE_MEMBER( ca1_w );

	DECLARE_WRITE_LINE_MEMBER( ca2_w );
	bool ca2_output();
	bool ca2_output_z();

	DECLARE_WRITE8_MEMBER( portb_w ) { write_portb(data); }
	void write_portb(uint8_t data);
	uint8_t b_output();

	DECLARE_WRITE_LINE_MEMBER( cb1_w );

	DECLARE_WRITE_LINE_MEMBER( cb2_w );
	bool cb2_output();
	bool cb2_output_z();

	int irq_a_state() const { return m_irq_a_state; }
	int irq_b_state() const { return m_irq_b_state; }

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:

	void update_interrupts();

	uint8_t get_in_a_value();
	uint8_t get_in_b_value();

	uint8_t get_out_a_value();
	uint8_t get_out_b_value();

	void set_out_ca2(int data);
	void set_out_cb2(int data);

	uint8_t port_a_r();
	uint8_t ddr_a_r();
	uint8_t control_a_r();

	uint8_t port_b_r();
	uint8_t ddr_b_r();
	uint8_t control_b_r();

	void send_to_out_a_func(const char* message);
	void send_to_out_b_func(const char* message);

	void port_a_w(uint8_t data);
	void ddr_a_w(uint8_t data);

	void port_b_w(uint8_t data);
	void ddr_b_w(uint8_t data);

	void control_a_w(uint8_t data);
	void control_b_w(uint8_t data);

	static bool irq1_enabled(uint8_t c);
	static bool c1_low_to_high(uint8_t c);
	static bool c1_high_to_low(uint8_t c);
	static bool output_selected(uint8_t c);
	static bool irq2_enabled(uint8_t c);
	static bool strobe_e_reset(uint8_t c);
	static bool strobe_c1_reset(uint8_t c);
	static bool c2_set(uint8_t c);
	static bool c2_low_to_high(uint8_t c);
	static bool c2_high_to_low(uint8_t c);
	static bool c2_set_mode(uint8_t c);
	static bool c2_strobe_mode(uint8_t c);
	static bool c2_output(uint8_t c);
	static bool c2_input(uint8_t c);

	devcb_read8 m_in_a_handler;
	devcb_read8 m_in_b_handler;
	devcb_read_line m_in_ca1_handler;
	devcb_read_line m_in_cb1_handler;
	devcb_read_line m_in_ca2_handler;
	devcb_write8 m_out_a_handler;
	devcb_write8 m_out_b_handler;
	devcb_write_line m_ca2_handler;
	devcb_write_line m_cb2_handler;
	devcb_write_line m_irqa_handler;
	devcb_write_line m_irqb_handler;

	uint8_t m_in_a;
	uint8_t m_in_ca1;
	uint8_t m_in_ca2;
	uint8_t m_out_a;
	uint8_t m_out_ca2;
	uint8_t m_port_a_z_mask;
	uint8_t m_ddr_a;
	uint8_t m_ctl_a;
	uint8_t m_irq_a1;
	uint8_t m_irq_a2;
	uint8_t m_irq_a_state;

	uint8_t m_in_b;
	uint8_t m_in_cb1;
	uint8_t m_in_cb2;
	uint8_t m_out_b;
	uint8_t m_out_cb2;
	uint8_t m_last_out_cb2_z;
	uint8_t m_ddr_b;
	uint8_t m_ctl_b;
	uint8_t m_irq_b1;
	uint8_t m_irq_b2;
	uint8_t m_irq_b_state;

	// variables that indicate if access a line externally -
	// used to for logging purposes ONLY
	bool m_in_a_pushed;
	bool m_out_a_needs_pulled;
	bool m_in_ca1_pushed;
	bool m_in_ca2_pushed;
	bool m_out_ca2_needs_pulled;
	bool m_in_b_pushed;
	bool m_out_b_needs_pulled;
	bool m_in_cb1_pushed;
	bool m_in_cb2_pushed;
	bool m_out_cb2_needs_pulled;
	bool m_logged_port_a_not_connected;
	bool m_logged_port_b_not_connected;
	bool m_logged_ca1_not_connected;
	bool m_logged_ca2_not_connected;
	bool m_logged_cb1_not_connected;
	bool m_logged_cb2_not_connected;
};


// device type definition
DECLARE_DEVICE_TYPE(PIA6821, pia6821_device)


#endif // MAME_DEVICES_MACHINE_6821PIA_H
