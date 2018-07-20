// license:BSD-3-Clause
// copyright-holders:Nigel Barnes
/**********************************************************************

    Solidisk EFS

    http://chrisacorns.computinghistory.org.uk/8bit_Upgrades/Solidisk_EFS.html

    TODO:
    - add Winchester slot
    - unknown how 16K RAM is paged as SWR (adverts claim it was unreliable)

**********************************************************************/


#include "emu.h"
#include "stlefs.h"


//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(ELECTRON_STLEFS, electron_stlefs_device, "electron_stlefs", "Solidisk EFS")


//-------------------------------------------------
//  MACHINE_DRIVER( stlefs )
//-------------------------------------------------

FLOPPY_FORMATS_MEMBER(electron_stlefs_device::floppy_formats)
	FLOPPY_ACORN_SSD_FORMAT,
	FLOPPY_ACORN_DSD_FORMAT,
	FLOPPY_ACORN_ADFS_OLD_FORMAT
FLOPPY_FORMATS_END

void stlefs_floppies(device_slot_interface &device)
{
	device.option_add("35dd",  FLOPPY_35_DD);
	device.option_add("525qd", FLOPPY_525_QD);
}

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_START(electron_stlefs_device::device_add_mconfig)
	/* fdc */
	MCFG_DEVICE_ADD("fdc", WD1770, 16_MHz_XTAL / 2)
	MCFG_WD_FDC_INTRQ_CALLBACK(WRITELINE(*this, electron_stlefs_device, fdc_intrq_w))
	MCFG_WD_FDC_DRQ_CALLBACK(WRITELINE(*this, electron_stlefs_device, fdc_drq_w))
	MCFG_FLOPPY_DRIVE_ADD("fdc:0", stlefs_floppies, "525qd", electron_stlefs_device::floppy_formats)
	MCFG_FLOPPY_DRIVE_SOUND(true)
	MCFG_FLOPPY_DRIVE_ADD("fdc:1", stlefs_floppies, nullptr, electron_stlefs_device::floppy_formats)
	MCFG_FLOPPY_DRIVE_SOUND(true)
MACHINE_CONFIG_END

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  electron_stlefs_device - constructor
//-------------------------------------------------

electron_stlefs_device::electron_stlefs_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, ELECTRON_STLEFS, tag, owner, clock)
	, device_electron_cart_interface(mconfig, *this)
	, m_fdc(*this, "fdc")
	, m_floppy0(*this, "fdc:0")
	, m_floppy1(*this, "fdc:1")
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void electron_stlefs_device::device_start()
{
}

//-------------------------------------------------
//  read - cartridge data read
//-------------------------------------------------

uint8_t electron_stlefs_device::read(address_space &space, offs_t offset, int infc, int infd, int romqa)
{
	uint8_t data = 0xff;

	if (infc)
	{
		switch (offset & 0xff)
		{
		case 0xc4:
		case 0xc5:
		case 0xc6:
		case 0xc7:
			data = m_fdc->read(space, offset & 0x03);
			break;
		}
	}

	if (!infc && !infd)
	{
		if (offset >= 0x0000 && offset < 0x4000)
		{
			data = m_rom[(offset & 0x3fff) + (romqa * 0x4000)];
		}
	}

	return data;
}

//-------------------------------------------------
//  write - cartridge data write
//-------------------------------------------------

void electron_stlefs_device::write(address_space &space, offs_t offset, uint8_t data, int infc, int infd, int romqa)
{
	if (infc)
	{
		switch (offset & 0xff)
		{
		case 0xc0:
			wd1770_control_w(space, 0, data);
			break;
		case 0xc4:
		case 0xc5:
		case 0xc6:
		case 0xc7:
			m_fdc->write(space, offset & 0x03, data);
			break;
		//case 0xcb:
			//m_page_register = data;
		}
	}
}


//**************************************************************************
//  IMPLEMENTATION
//**************************************************************************

WRITE8_MEMBER(electron_stlefs_device::wd1770_control_w)
{
	floppy_image_device *floppy = nullptr;

	// bit 0, 1: drive select
	if (BIT(data, 0)) floppy = m_floppy0->get_device();
	if (BIT(data, 1)) floppy = m_floppy1->get_device();
	m_fdc->set_floppy(floppy);

	// bit 2: side select
	if (floppy)
		floppy->ss_w(BIT(data, 2));

	// bit 3: density
	m_fdc->dden_w(BIT(data, 3));

	// bit 5: reset
	if (!BIT(data, 5)) m_fdc->soft_reset();
}

void electron_stlefs_device::fdc_intrq_w(int state)
{
	m_slot->irq_w(state);
}

void electron_stlefs_device::fdc_drq_w(int state)
{
	m_slot->nmi_w(state);
}
