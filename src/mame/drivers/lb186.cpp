// license:BSD-3-Clause
// copyright-holders:Carl
// TODO: SCSI, requires NCR5380 BSY IRQs
#include "emu.h"
#include "cpu/i86/i186.h"
#include "machine/wd_fdc.h"
#include "machine/mc68681.h"
#include "bus/rs232/rs232.h"
#include "machine/nscsi_hd.h"
#include "machine/ncr5380n.h"
#include "imagedev/flopdrv.h"
#include "formats/pc_dsk.h"
#include "formats/naslite_dsk.h"

class lb186_state : public driver_device
{
public:
	lb186_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_fdc(*this, "fdc")
		, m_scsi(*this, "scsibus:7:ncr5380")
	{
	}

	void lb186(machine_config &config);

private:
	DECLARE_WRITE8_MEMBER(sio_out_w);
	DECLARE_WRITE8_MEMBER(drive_sel_w);
	DECLARE_READ8_MEMBER(scsi_dack_r);
	DECLARE_WRITE8_MEMBER(scsi_dack_w);
	DECLARE_FLOPPY_FORMATS( floppy_formats );
	static void ncr5380(device_t *device);
	void lb186_io(address_map &map);
	void lb186_map(address_map &map);

	required_device<i80186_cpu_device> m_maincpu;
	required_device<wd1772_device> m_fdc;
	required_device<ncr5380n_device> m_scsi;
};

WRITE8_MEMBER(lb186_state::scsi_dack_w)
{
	m_scsi->dma_w(data);
}

READ8_MEMBER(lb186_state::scsi_dack_r)
{
	return m_scsi->dma_r();
}

WRITE8_MEMBER(lb186_state::sio_out_w)
{
	if(!BIT(data, 5))
		m_fdc->soft_reset();
	m_maincpu->tmrin1_w(BIT(data, 3) ? ASSERT_LINE : CLEAR_LINE);
}

WRITE8_MEMBER(lb186_state::drive_sel_w)
{
	m_fdc->dden_w(BIT(data, 5));

	floppy_image_device *floppy;
	char devname[8];
	unsigned int drive = data & 0xf;
	switch(drive)
	{
		case 0:
			return;
		case 1:
			drive = 0;
			break;
		case 2:
			drive = 1;
			break;
		case 4:
			drive = 2;
			break;
		case 8:
			drive = 3;
			break;
		default:
			logerror("More than one drive enabled!\n");
			return;
	}

	sprintf(devname, "%d", drive);
	floppy = m_fdc->subdevice<floppy_connector>(devname)->get_device();
	m_fdc->set_floppy(floppy);
	floppy->ss_w(BIT(data, 4));
}

void lb186_state::lb186_map(address_map &map)
{
	map(0x00000, 0x3ffff).ram(); // fixed 256k for now
	map(0xfc000, 0xfffff).rom().region("bios", 0);
}

void lb186_state::lb186_io(address_map &map)
{
	map.unmap_value_high();
	map(0x1000, 0x101f).rw("duart", FUNC(scn2681_device::read), FUNC(scn2681_device::write)).umask16(0x00ff);
	map(0x1080, 0x108f).rw(m_scsi, FUNC(ncr5380n_device::read), FUNC(ncr5380n_device::write)).umask16(0x00ff);
	map(0x1100, 0x1107).rw(m_fdc, FUNC(wd1772_device::read), FUNC(wd1772_device::write)).umask16(0x00ff);
	map(0x1180, 0x1180).rw(FUNC(lb186_state::scsi_dack_r), FUNC(lb186_state::scsi_dack_w));
	map(0x1200, 0x1200).w(FUNC(lb186_state::drive_sel_w));
}

static void lb186_floppies(device_slot_interface &device)
{
	device.option_add("525dd", FLOPPY_525_DD);
}

void lb186_state::ncr5380(device_t *device)
{
	devcb_base *devcb;
	(void)devcb;
	MCFG_DEVICE_CLOCK(10000000)
	MCFG_NCR5380N_IRQ_HANDLER(WRITELINE(":maincpu", i80186_cpu_device, int1_w))
	MCFG_NCR5380N_DRQ_HANDLER(WRITELINE(":maincpu", i80186_cpu_device, drq0_w))
}

static void scsi_devices(device_slot_interface &device)
{
	device.option_add("harddisk", NSCSI_HARDDISK);
	device.option_add_internal("ncr5380", NCR5380N);
}

FLOPPY_FORMATS_MEMBER( lb186_state::floppy_formats )
	FLOPPY_PC_FORMAT,
	FLOPPY_NASLITE_FORMAT
FLOPPY_FORMATS_END

MACHINE_CONFIG_START(lb186_state::lb186)
	MCFG_DEVICE_ADD("maincpu", I80186, 16_MHz_XTAL / 2)
	MCFG_DEVICE_PROGRAM_MAP(lb186_map)
	MCFG_DEVICE_IO_MAP(lb186_io)

	MCFG_DEVICE_ADD("duart", SCN2681, 3.6864_MHz_XTAL)
	MCFG_MC68681_IRQ_CALLBACK(WRITELINE("maincpu", i80186_cpu_device, int0_w))
	MCFG_MC68681_A_TX_CALLBACK(WRITELINE("rs232_1", rs232_port_device, write_txd))
	MCFG_MC68681_B_TX_CALLBACK(WRITELINE("rs232_2", rs232_port_device, write_txd))
	MCFG_MC68681_OUTPORT_CALLBACK(WRITE8(*this, lb186_state, sio_out_w))

	MCFG_DEVICE_ADD("rs232_1", RS232_PORT, default_rs232_devices, "terminal")
	MCFG_RS232_RXD_HANDLER(WRITELINE("duart", scn2681_device, rx_a_w))
	MCFG_DEVICE_ADD("rs232_2", RS232_PORT, default_rs232_devices, nullptr)
	MCFG_RS232_RXD_HANDLER(WRITELINE("duart", scn2681_device, rx_b_w))

	MCFG_DEVICE_ADD("fdc", WD1772, 16_MHz_XTAL / 2)
	MCFG_WD_FDC_INTRQ_CALLBACK(WRITELINE("maincpu", i80186_cpu_device, int2_w))
	MCFG_WD_FDC_DRQ_CALLBACK(WRITELINE("maincpu", i80186_cpu_device, drq0_w))
	MCFG_FLOPPY_DRIVE_ADD("fdc:0", lb186_floppies, "525dd", lb186_state::floppy_formats)
	MCFG_FLOPPY_DRIVE_ADD("fdc:1", lb186_floppies, nullptr, lb186_state::floppy_formats)
	MCFG_FLOPPY_DRIVE_ADD("fdc:2", lb186_floppies, nullptr, lb186_state::floppy_formats)
	MCFG_FLOPPY_DRIVE_ADD("fdc:3", lb186_floppies, nullptr, lb186_state::floppy_formats)

	MCFG_NSCSI_BUS_ADD("scsibus")
	MCFG_NSCSI_ADD("scsibus:0", scsi_devices, "harddisk", false)
	MCFG_NSCSI_ADD("scsibus:1", scsi_devices, nullptr, false)
	MCFG_NSCSI_ADD("scsibus:2", scsi_devices, nullptr, false)
	MCFG_NSCSI_ADD("scsibus:3", scsi_devices, nullptr, false)
	MCFG_NSCSI_ADD("scsibus:4", scsi_devices, nullptr, false)
	MCFG_NSCSI_ADD("scsibus:5", scsi_devices, nullptr, false)
	MCFG_NSCSI_ADD("scsibus:6", scsi_devices, nullptr, false)
	MCFG_NSCSI_ADD("scsibus:7", scsi_devices, "ncr5380", true)
	MCFG_SLOT_OPTION_MACHINE_CONFIG("ncr5380", lb186_state::ncr5380)
MACHINE_CONFIG_END

ROM_START( lb186 )
	ROM_REGION(0x4000, "bios", 0)
	ROM_LOAD16_BYTE("a75515_v3.35.rom", 0x0000, 0x2000, CRC(245824fb) SHA1(b39ed91d421513f5912fdbc290aaa3f1b7d4f1e0))
	ROM_LOAD16_BYTE("a75516_v3.35.rom", 0x0001, 0x2000, CRC(9d9a5e22) SHA1(070be31c622f50508e8cbdb797c79978b6a4b8f6))
ROM_END

COMP( 1985, lb186, 0, 0, lb186, 0, lb186_state, empty_init, "Ampro Computers", "Little Board/186", MACHINE_NO_SOUND_HW )
