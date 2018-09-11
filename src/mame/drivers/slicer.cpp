// license:BSD-3-Clause
// copyright-holders:Carl

// Slicer Computers Slicer 80186 SBC
// The bios makefile refers to a "exe3bin" utility, this can be substituted with FreeDOS exe2bin and the /l=0xf800 option
// which will fixup the relocations

#include "emu.h"
#include "cpu/i86/i186.h"
#include "machine/74259.h"
#include "machine/wd_fdc.h"
#include "machine/mc68681.h"
#include "bus/rs232/rs232.h"
#include "bus/isa/isa.h"
#include "bus/scsi/scsi.h"

class slicer_state : public driver_device
{
public:
	slicer_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_fdc(*this, "fdc"),
		m_sasi(*this, "sasi")
	{
	}

	void slicer(machine_config &config);

private:
	DECLARE_WRITE8_MEMBER(sio_out_w);
	DECLARE_WRITE_LINE_MEMBER(drive_size_w);
	template<unsigned int drive> DECLARE_WRITE_LINE_MEMBER(drive_sel_w);

	void slicer_io(address_map &map);
	void slicer_map(address_map &map);

	required_device<fd1797_device> m_fdc;
	required_device<scsi_port_device> m_sasi;
};

WRITE8_MEMBER(slicer_state::sio_out_w)
{
	floppy_image_device *floppy;
	int state = (data & 0x80) ? 0 : 1;
	char devname[8];

	for(int i = 0; i < 4; i++)
	{
		sprintf(devname, "%d", i);
		floppy = m_fdc->subdevice<floppy_connector>(devname)->get_device();
		if(floppy)
			floppy->mon_w(state);
	}
}

template<unsigned int drive>
WRITE_LINE_MEMBER(slicer_state::drive_sel_w)
{
	floppy_image_device *floppy;
	char devname[8];

	if (!state)
		return;

	sprintf(devname, "%d", drive);
	floppy = m_fdc->subdevice<floppy_connector>(devname)->get_device();
	m_fdc->set_floppy(floppy);
}

WRITE_LINE_MEMBER(slicer_state::drive_size_w)
{
	m_fdc->set_unscaled_clock (state ? 1'000'000 : 2'000'000);
}

void slicer_state::slicer_map(address_map &map)
{
	map(0x00000, 0x3ffff).ram(); // fixed 256k for now
	map(0xf8000, 0xfffff).rom().region("bios", 0);
}

void slicer_state::slicer_io(address_map &map)
{
	map.unmap_value_high();
	map(0x0000, 0x007f).rw(m_fdc, FUNC(fd1797_device::read), FUNC(fd1797_device::write)).umask16(0x00ff); //PCS0
	map(0x0080, 0x00ff).rw("duart", FUNC(scn2681_device::read), FUNC(scn2681_device::write)).umask16(0x00ff); //PCS1
	map(0x0100, 0x010f).mirror(0x0070).w("drivelatch", FUNC(ls259_device::write_d0)).umask16(0x00ff); //PCS2
	// TODO: 0x180 sets ack
	map(0x0180, 0x0180).r("sasi_data_in", FUNC(input_buffer_device::bus_r)).w("sasi_data_out", FUNC(output_latch_device::bus_w)).umask16(0x00ff); //PCS3
	map(0x0181, 0x0181).r("sasi_ctrl_in", FUNC(input_buffer_device::bus_r));
	map(0x0184, 0x0184).r("sasi_data_in", FUNC(input_buffer_device::bus_r)).w("sasi_data_out", FUNC(output_latch_device::bus_w)).umask16(0x00ff);
	map(0x0185, 0x0185).r("sasi_ctrl_in", FUNC(input_buffer_device::bus_r));
}

static void slicer_floppies(device_slot_interface &device)
{
	device.option_add("525dd", FLOPPY_525_DD);
	device.option_add("8dsdd", FLOPPY_8_DSDD);
}

MACHINE_CONFIG_START(slicer_state::slicer)
	MCFG_DEVICE_ADD("maincpu", I80186, 16_MHz_XTAL / 2)
	MCFG_DEVICE_PROGRAM_MAP(slicer_map)
	MCFG_DEVICE_IO_MAP(slicer_io)

	MCFG_DEVICE_ADD("duart", SCN2681, 3.6864_MHz_XTAL)
	MCFG_MC68681_IRQ_CALLBACK(WRITELINE("maincpu", i80186_cpu_device, int0_w))
	MCFG_MC68681_A_TX_CALLBACK(WRITELINE("rs232_1", rs232_port_device, write_txd))
	MCFG_MC68681_B_TX_CALLBACK(WRITELINE("rs232_2", rs232_port_device, write_txd))
	MCFG_MC68681_OUTPORT_CALLBACK(WRITE8(*this, slicer_state, sio_out_w))

	MCFG_DEVICE_ADD("rs232_1", RS232_PORT, default_rs232_devices, "terminal")
	MCFG_RS232_RXD_HANDLER(WRITELINE("duart", scn2681_device, rx_a_w))
	MCFG_DEVICE_ADD("rs232_2", RS232_PORT, default_rs232_devices, nullptr)
	MCFG_RS232_RXD_HANDLER(WRITELINE("duart", scn2681_device, rx_b_w))

	MCFG_DEVICE_ADD("fdc", FD1797, 16_MHz_XTAL / 2 / 8)
	MCFG_WD_FDC_INTRQ_CALLBACK(WRITELINE("maincpu", i80186_cpu_device, int1_w))
	MCFG_WD_FDC_DRQ_CALLBACK(WRITELINE("maincpu", i80186_cpu_device, drq0_w))
	MCFG_FLOPPY_DRIVE_ADD("fdc:0", slicer_floppies, "525dd", floppy_image_device::default_floppy_formats)
	MCFG_FLOPPY_DRIVE_ADD("fdc:1", slicer_floppies, nullptr, floppy_image_device::default_floppy_formats)
	MCFG_FLOPPY_DRIVE_ADD("fdc:2", slicer_floppies, nullptr, floppy_image_device::default_floppy_formats)
	MCFG_FLOPPY_DRIVE_ADD("fdc:3", slicer_floppies, nullptr, floppy_image_device::default_floppy_formats)

	ls259_device &drivelatch(LS259(config, "drivelatch")); // U29
	drivelatch.q_out_cb<0>().set("sasi", FUNC(scsi_port_device::write_sel));
	drivelatch.q_out_cb<1>().set("sasi", FUNC(scsi_port_device::write_rst));
	drivelatch.q_out_cb<2>().set(FUNC(slicer_state::drive_sel_w<3>));
	drivelatch.q_out_cb<3>().set(FUNC(slicer_state::drive_sel_w<2>));
	drivelatch.q_out_cb<4>().set(FUNC(slicer_state::drive_sel_w<1>));
	drivelatch.q_out_cb<5>().set(FUNC(slicer_state::drive_sel_w<0>));
	drivelatch.q_out_cb<6>().set(FUNC(slicer_state::drive_size_w));
	drivelatch.q_out_cb<7>().set("fdc", FUNC(fd1797_device::dden_w));

	MCFG_DEVICE_ADD("sasi", SCSI_PORT, 0)
	MCFG_SCSI_DATA_INPUT_BUFFER("sasi_data_in")
	MCFG_SCSI_BSY_HANDLER(WRITELINE("sasi_ctrl_in", input_buffer_device, write_bit3))
	MCFG_SCSI_MSG_HANDLER(WRITELINE("sasi_ctrl_in", input_buffer_device, write_bit4))
	MCFG_SCSI_CD_HANDLER(WRITELINE("sasi_ctrl_in", input_buffer_device, write_bit5))
	MCFG_SCSI_REQ_HANDLER(WRITELINE("sasi_ctrl_in", input_buffer_device, write_bit6))
	MCFG_SCSI_IO_HANDLER(WRITELINE("sasi_ctrl_in", input_buffer_device, write_bit7))

	MCFG_SCSI_OUTPUT_LATCH_ADD("sasi_data_out", "sasi")
	MCFG_DEVICE_ADD("sasi_data_in", INPUT_BUFFER, 0)
	MCFG_DEVICE_ADD("sasi_ctrl_in", INPUT_BUFFER, 0)
MACHINE_CONFIG_END

ROM_START( slicer )
	ROM_REGION(0x8001, "bios", 0)
	// built from sources, reset.asm adds an extra byte
	ROM_LOAD("epbios.bin", 0x0000, 0x8001, CRC(96fe9dd4) SHA1(5fc43454fe7d51f2ae97aef822155dcd28eb7f23))

	ROM_REGION(0x10000, "user1", 0)
	//slicer_h.bin : main slicer board, high byte
	//slicer_l.bin : main slicer board, low byte
	//slvid_cg.bin : slicer video/keyboard expansion board, character generator
	//slvid_e.bin : slicer video/keyboard expansion board, even byte
	//slvid_o.bin : slicer video/keyboard expansion board, odd byte
	ROM_LOAD( "slicer_h.bin", 0x000000, 0x004000, CRC(1f9a79b7) SHA1(2070c6818d39fe7ec4370fc2304469793a126731) )
	ROM_LOAD( "slicer_l.bin", 0x000000, 0x004000, CRC(6feef94b) SHA1(174488591b727a4130166bcb2e83c0e74323d43b) )
	ROM_LOAD( "slvid_cg.bin", 0x000000, 0x001000, CRC(d4d9ac2f) SHA1(866c760320b224ba8670501ea905de32193acedc) )
	ROM_LOAD( "slvid_o.bin",  0x000000, 0x001000, CRC(c62dda77) SHA1(1d0b9abc53412b0725072d4c33c478fb5358ab5c) )
	ROM_LOAD( "slvid_e.bin",  0x000000, 0x001000, CRC(8694274f) SHA1(8373baaea8d689bf52699b587942a57f26baf740) )
ROM_END

COMP( 1983, slicer, 0, 0, slicer, 0, slicer_state, empty_init, "Slicer Computers", "Slicer", MACHINE_NO_SOUND )
