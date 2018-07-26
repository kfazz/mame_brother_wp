// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/***************************************************************************

  a8sio.h - Atari 8 bit SIO bus interface


              1 1
      2 4 6 8 0 2
     +-----------+
    / o o o o o o \
   / o o o o o o o \
  +-----------------+
     1 3 5 7 9 1 1
               1 3

  1 - clock in (to computer)
  2 - clock out
  3 - data in
  4 - GND
  5 - data out
  6 - GND
  7 - command (active low)
  8 - motor
  9 - proceed (active low)
 10 - +5V/ready
 11 - audio in
 12 - +12V (A400/A800)
 13 - interrupt (active low)

***************************************************************************/

#ifndef MAME_BUS_A800_A8SIO_H
#define MAME_BUS_A800_A8SIO_H

#pragma once


//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_A8SIO_SLOT_ADD(_nbtag, _tag, _def_slot) \
	MCFG_DEVICE_ADD(_tag, A8SIO_SLOT, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(a8sio_cards, _def_slot, false) \
	downcast<a8sio_slot_device &>(*device).set_a8sio_slot(_nbtag, _tag);

#define MCFG_A8SIO_DATA_IN_CB(_devcb) \
	downcast<a8sio_device &>(*device).set_data_in_callback(DEVCB_##_devcb);


class a8sio_slot_device : public device_t,
							public device_slot_interface
{
public:
	// construction/destruction
	a8sio_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// inline configuration
	void set_a8sio_slot(const char *tag, const char *slottag) { m_a8sio_tag = tag; m_a8sio_slottag = slottag; }

protected:
	a8sio_slot_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;

	// configuration
	const char *m_a8sio_tag;
	const char *m_a8sio_slottag;
};


// device type definition
DECLARE_DEVICE_TYPE(A8SIO_SLOT, a8sio_slot_device)


class device_a8sio_card_interface;

class a8sio_device : public device_t
{
public:
	// construction/destruction
	a8sio_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// inline configuration
	template <class Object> devcb_base &set_clock_in_callback(Object &&cb) { return m_out_clock_in_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> devcb_base &set_data_in_callback(Object &&cb) { return m_out_data_in_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> devcb_base &set_audio_in_callback(Object &&cb) { return m_out_audio_in_cb.set_callback(std::forward<Object>(cb)); }

	void add_a8sio_card(device_a8sio_card_interface *card);
	device_a8sio_card_interface *get_a8sio_card();

	DECLARE_WRITE_LINE_MEMBER( clock_in_w );  // pin 1
	//virtual DECLARE_WRITE_LINE_MEMBER( clock_out_w ); // pin 2
	DECLARE_WRITE_LINE_MEMBER( data_in_w );   // pin 3
	//DECLARE_WRITE_LINE_MEMBER( data_out_wi ); // pin 5
	//DECLARE_WRITE_LINE_MEMBER( command_w );   // pin 7
	DECLARE_WRITE_LINE_MEMBER( motor_w );     // pin 8
	//DECLARE_WRITE_LINE_MEMBER( proceed_w );   // pin 9
	DECLARE_WRITE8_MEMBER( audio_in_w );      // pin 11

protected:
	a8sio_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	devcb_write_line    m_out_clock_in_cb; // pin 1
	devcb_write_line    m_out_data_in_cb; // pin 3
	devcb_write8        m_out_audio_in_cb; // pin 11

	device_a8sio_card_interface *m_device;
};


// device type definition
DECLARE_DEVICE_TYPE(A8SIO, a8sio_device)


class device_a8sio_card_interface : public device_slot_card_interface
{
	friend class a8sio_device;
public:
	// construction/destruction
	virtual ~device_a8sio_card_interface();

	void set_a8sio_device();

	// inline configuration
	void set_a8sio_tag(const char *tag, const char *slottag) { m_a8sio_tag = tag; m_a8sio_slottag = slottag; }

	virtual DECLARE_WRITE_LINE_MEMBER( motor_w );

public:
	device_a8sio_card_interface(const machine_config &mconfig, device_t &device);

	a8sio_device  *m_a8sio;
	const char *m_a8sio_tag;
	const char *m_a8sio_slottag;
};


void a8sio_cards(device_slot_interface &device);

#endif // MAME_BUS_A800_A8SIO_H
