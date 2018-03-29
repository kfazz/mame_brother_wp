// license:BSD-3-Clause
// copyright-holders:Nigel Barnes
/****************************************************************************

    BBC Master Compact Joystick/Mouse port

*****************************************************************************/

#include "emu.h"
#include "joyport.h"


//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(BBC_JOYPORT_SLOT, bbc_joyport_slot_device, "bbc_joyport_slot", "BBC Master Compact Joystick/Mouse port")


//**************************************************************************
//  DEVICE BBC_JOYPORT INTERFACE
//**************************************************************************

//-------------------------------------------------
//  device_bbc_joyport_interface - constructor
//-------------------------------------------------

device_bbc_joyport_interface::device_bbc_joyport_interface(const machine_config &mconfig, device_t &device)
	: device_slot_card_interface(mconfig, device)
{
	m_slot = dynamic_cast<bbc_joyport_slot_device *>(device.owner());
}


//-------------------------------------------------
//  ~device_bbc_joyport_interface - destructor
//-------------------------------------------------

device_bbc_joyport_interface::~device_bbc_joyport_interface()
{
}


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  bbcmc_joyport_slot_device - constructor
//-------------------------------------------------

bbc_joyport_slot_device::bbc_joyport_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, BBC_JOYPORT_SLOT, tag, owner, clock),
	device_slot_interface(mconfig, *this),
	m_device(nullptr),
	m_cb1_handler(*this),
	m_cb2_handler(*this)
{
}


//-------------------------------------------------
//  device_validity_check -
//-------------------------------------------------

void bbc_joyport_slot_device::device_validity_check(validity_checker &valid) const
{
	device_t *const carddev = get_card_device();
	if (carddev && !dynamic_cast<device_bbc_joyport_interface *>(carddev))
		osd_printf_error("Card device %s (%s) does not implement device_bbc_joyport_interface\n", carddev->tag(), carddev->name());
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void bbc_joyport_slot_device::device_start()
{
	device_t *const carddev = get_card_device();
	m_device = dynamic_cast<device_bbc_joyport_interface *>(carddev);
	if (carddev && !m_device)
		fatalerror("Card device %s (%s) does not implement device_bbc_joyport_interface\n", carddev->tag(), carddev->name());

  // resolve callbacks
	m_cb1_handler.resolve_safe();
	m_cb2_handler.resolve_safe();
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void bbc_joyport_slot_device::device_reset()
{
}


//-------------------------------------------------
//  pb_r
//-------------------------------------------------

READ8_MEMBER(bbc_joyport_slot_device::pb_r)
{
	if (m_device)
		return m_device->pb_r(space, 0);
	else
		return 0xff;
}


//-------------------------------------------------
//  pb_w
//-------------------------------------------------

WRITE8_MEMBER(bbc_joyport_slot_device::pb_w)
{
	if (m_device)
		m_device->pb_w(space, 0, data);
}


//-------------------------------------------------
//  SLOT_INTERFACE( bbc_joyport_devices )
//-------------------------------------------------


// slot devices
#include "joystick.h"
//#include "mouse.h"


SLOT_INTERFACE_START( bbc_joyport_devices )
	SLOT_INTERFACE("joystick", BBCMC_JOYSTICK)
	//SLOT_INTERFACE("mouse", BBCMC_MOUSE)
SLOT_INTERFACE_END
