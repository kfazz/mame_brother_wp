// license:BSD-3-Clause
// copyright-holders:Robbbert
/******************************************************************************************************************

Nutting Icebox

2018-08-19 Skeleton driver. It's all guesswork.

It's a case with many slots for daughter boards, and 2 floppy drives. It's an "in-circuit emulator"
for Bally / Astrocade development.

The unit has 5 daughter boards: 3 boards were photographed and are: a cpu board (Z80 and unmarked XTAL),
                    a RS232 board (2x 8251, 8 dipswitches connected to a BR1941L, 4.9152 XTAL),
                    and a board covered with TTL.

Commands:
- At the main % prompt:
-- A = Arcade
-- C = Commercial
-- D = Debug
-- ^C = load data to 0 and jump there (disk boot?)
-- ^T = load data from disk?
-- ^X = new line
- At the debug prompt: (to be done)   Q to quit

Status:
- Machine boots up, you can enter commands.
- Disk track 00 can be read into memory, hangs when track 01 is requested.
- Note that disk won't actually do anything until Z-80 IM0 is fixed.

To Do:
- Find out more what ports F2-FF do.

The floppy drives are a special type containing a FDC within. The FDC is a FD1771-B01 with a 4MHz XTAL.


NOTES FROM AL KOSSOW
--------------------

This is the Nutting Associates ICE box used for Bally arcade and Astrocade development.

The games are written in "Terse", a dialect of FORTH with graphics and sound extensions.

The game graphics and sound hardware were in a separate chassis connected through 50 pin ribbon cables.
There is 64K of ram that can be mapped to the addresses used where the proms in the game would be.

Development appeared to have started under CP/M, then they migrated to development in the Terse environment.
The sector size also changed from 128 to 1024 bytes. For some reason, the hardware inverts the sector data.
CP/M flipped the bits, Terse does not.

There is a 122 pin backplane, and five cards. The bus buffers for the floppies and the external hardware
are on the backplane. There is an I/O, memory mapper, dual 32k static ram, and CPU cards. The I/O card has
a three byte FIFO and a buffer for forcing 00 onto the data bus, which appears to be related to handling
breakpoints from looking at the code in ICE.ASM.

The eprom dump is for booting Terse, and earlier version of the prom is documented in ICE.ASM which may be for CP/M

The CPU card has 1K of ram, mapped to FC00. They must have moved the sector buffering to main RAM, since a
1K sector obviously wouldn't fit.

The crystal on the CPU is 9.8304MHz. It may be possible to alter the cpu speed. There is a FAST signal on the bp.

I/O registers from the ICE.ASM listing

CRTD    E0    terminal uart data
CRTS    E1    terminal uart control
LPTD    E2    printer uart data
LPTS    E3    printer uart control

DATA    E4    1771 regs
SECTOR  E5
TRACK   E6
COMMAND E7

CENABE  F0
SELECT  F1    bit 0 & 1 (select drive 0-3), bit 2 (enable floppy interrupt)
FIFO    F2    3 BYTE FIFO

EXTL    F8    These are probably for the mapping registers
EXTH    F9

WPROTL  FA
WPROTH  FB

BADRH   FC    BASE ADR? breakpoint
BADRL   FD

BTYPE   FE
MISC    FF

the 0 is forced onto the bus when bp signal IRESET/ is asserted. it appears to come from the CPU board (only
the cpu and io board are connected to it)

the box doesn't do remapping, it just has two 16 bit write-only registers that can unmap and write protect memory
in 16 4k chunks, (extl,exth), (wprotl,wproth) .

breakpoint (badrh, barl) specifies an adr to nmi on, with a 4 bit reference type in btype (wr, req, iorq, mreq)

it looks like zeroing btype disables it.

io port 0xff (misc) is on the I/O board.

******************************************************************************************************************/

#include "emu.h"
#include "cpu/z80/z80.h"
#include "machine/i8251.h"
#include "machine/com8116.h"
#include "bus/rs232/rs232.h"
#include "machine/wd_fdc.h"


class icebox_state : public driver_device
{
public:
	icebox_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_uart0(*this, "uart0")
		, m_uart1(*this, "uart1")
		, m_brg(*this, "brg")
		, m_fdc(*this, "fdc")
		, m_floppy0(*this, "fdc:0")
		, m_floppy1(*this, "fdc:1")
	{ }

	void icebox(machine_config &config);

private:
	DECLARE_WRITE_LINE_MEMBER(intrq_w);
	DECLARE_WRITE_LINE_MEMBER(drq_w);
	void mem_map(address_map &map);
	void io_map(address_map &map);
	void machine_reset() override;
	void port_f1_w(u8 data);
	u8 m_f1;

	required_device<cpu_device> m_maincpu;
	required_device<i8251_device> m_uart0;
	required_device<i8251_device> m_uart1;
	required_device<com8116_device> m_brg;
	required_device<fd1771_device> m_fdc;
	required_device<floppy_connector> m_floppy0;
	required_device<floppy_connector> m_floppy1;
};


void icebox_state::mem_map(address_map &map)
{
	map(0x0000, 0xefff).ram();
	map(0xf000, 0xfbff).rom().region("maincpu", 0);
	map(0xfc00, 0xffff).ram();
}

void icebox_state::io_map(address_map &map)
{
	map.global_mask(0xff);
	map.unmap_value_high();
	map(0xe0, 0xe0).rw(m_uart0, FUNC(i8251_device::data_r), FUNC(i8251_device::data_w));
	map(0xe1, 0xe1).rw(m_uart0, FUNC(i8251_device::status_r), FUNC(i8251_device::control_w));
	map(0xe2, 0xe2).rw(m_uart1, FUNC(i8251_device::data_r), FUNC(i8251_device::data_w));
	map(0xe3, 0xe3).rw(m_uart1, FUNC(i8251_device::status_r), FUNC(i8251_device::control_w));
	map(0xe4, 0xe7).lrw8("fdc_usage",
					[this](offs_t offset) { return m_fdc->read(offset^3); },
					[this](offs_t offset, u8 data) { m_fdc->write(offset^3, data); });
	map(0xf1, 0xf1).lw8("port_F1", [this](u8 data) { port_f1_w(data); } );
}

/* Input ports */
static INPUT_PORTS_START( icebox )
	PORT_START("BAUD")
	PORT_DIPNAME( 0x0f, 0x0e, "Baud Rate for Terminal")
	PORT_DIPSETTING(    0x00, "50")
	PORT_DIPSETTING(    0x01, "75")
	PORT_DIPSETTING(    0x02, "110")
	PORT_DIPSETTING(    0x03, "134.5")
	PORT_DIPSETTING(    0x04, "150")
	PORT_DIPSETTING(    0x05, "300")
	PORT_DIPSETTING(    0x06, "600")
	PORT_DIPSETTING(    0x07, "1200")
	PORT_DIPSETTING(    0x08, "1800")
	PORT_DIPSETTING(    0x09, "2000")
	PORT_DIPSETTING(    0x0a, "2400")
	PORT_DIPSETTING(    0x0b, "3600")
	PORT_DIPSETTING(    0x0c, "4800")
	PORT_DIPSETTING(    0x0e, "9600")
	PORT_DIPSETTING(    0x0f, "19200")
	PORT_DIPNAME( 0xf0, 0x70, "Baud Rate for Printer")
	PORT_DIPSETTING(    0x00, "50")
	PORT_DIPSETTING(    0x10, "75")
	PORT_DIPSETTING(    0x20, "110")
	PORT_DIPSETTING(    0x30, "134.5")
	PORT_DIPSETTING(    0x40, "150")
	PORT_DIPSETTING(    0x50, "300")
	PORT_DIPSETTING(    0x60, "600")
	PORT_DIPSETTING(    0x70, "1200")
	PORT_DIPSETTING(    0x80, "1800")
	PORT_DIPSETTING(    0x90, "2000")
	PORT_DIPSETTING(    0xa0, "2400")
	PORT_DIPSETTING(    0xb0, "3600")
	PORT_DIPSETTING(    0xc0, "4800")
	PORT_DIPSETTING(    0xe0, "9600")
	PORT_DIPSETTING(    0xf0, "19200")
INPUT_PORTS_END

void icebox_state::machine_reset()
{
	u8 data = ioport("BAUD")->read();
	m_brg->write_str(data & 15); // Terminal
	m_brg->write_stt((data >> 4) & 15); // Printer
	m_maincpu->set_pc(0xf000);
	m_f1 = 0;
}

void icebox_state::port_f1_w(u8 data)
{
	m_f1 = data;

	floppy_image_device *floppy = nullptr;

	switch (data & 7)
	{
		case 4:
			floppy = m_floppy0->get_device();
			break;
		case 5:
			floppy = m_floppy1->get_device();
			break;
		default:
			break;
	}

	m_fdc->set_floppy(floppy);

	if (floppy)
	{
		floppy->mon_w(0);
		floppy->ss_w(0);            // side 0 ?
	}

	m_fdc->dden_w(1);                 // single density?
}

// Command complete: release CPU from halt, via IM0.
WRITE_LINE_MEMBER(icebox_state::intrq_w)
{
	if (BIT(m_f1, 2) && state)
		m_maincpu->set_input_line_and_vector(INPUT_LINE_IRQ0, HOLD_LINE, 0xc9);
}

// The next byte from floppy is available. Enable CPU so it can get the byte, via IM0.
WRITE_LINE_MEMBER(icebox_state::drq_w)
{
	if (BIT(m_f1, 2) && state)
		m_maincpu->set_input_line_and_vector(INPUT_LINE_IRQ0, HOLD_LINE, 0xc9);
}

static void floppies(device_slot_interface &device)
{
	device.option_add("flop", FLOPPY_8_SSDD); // Pertec "iCOM FD5200"
}

static DEVICE_INPUT_DEFAULTS_START( terminal ) // we need to remove bit 7 which is on the last character of each message
	DEVICE_INPUT_DEFAULTS( "RS232_RXBAUD", 0xff, RS232_BAUD_9600 )
	DEVICE_INPUT_DEFAULTS( "RS232_TXBAUD", 0xff, RS232_BAUD_9600 )
	DEVICE_INPUT_DEFAULTS( "RS232_STARTBITS", 0xff, RS232_STARTBITS_1 )
	DEVICE_INPUT_DEFAULTS( "RS232_DATABITS", 0xff, RS232_DATABITS_7 )
	DEVICE_INPUT_DEFAULTS( "RS232_PARITY", 0xff, RS232_PARITY_EVEN )
	DEVICE_INPUT_DEFAULTS( "RS232_STOPBITS", 0xff, RS232_STOPBITS_2 )
DEVICE_INPUT_DEFAULTS_END

MACHINE_CONFIG_START(icebox_state::icebox)
	/* basic machine hardware */
	MCFG_DEVICE_ADD("maincpu",Z80, 9'830'400 / 4) // unknown divisor
	MCFG_DEVICE_PROGRAM_MAP(mem_map)
	MCFG_DEVICE_IO_MAP(io_map)

	I8251(config, m_uart0, 0);
	m_uart0->txd_handler().set("rs232a", FUNC(rs232_port_device::write_txd));
	m_uart0->dtr_handler().set("rs232a", FUNC(rs232_port_device::write_dtr));
	m_uart0->rts_handler().set("rs232a", FUNC(rs232_port_device::write_rts));

	rs232_port_device &rs232a(RS232_PORT(config, "rs232a", default_rs232_devices, "terminal"));
	rs232a.rxd_handler().set(m_uart0, FUNC(i8251_device::write_rxd));
	rs232a.dsr_handler().set(m_uart0, FUNC(i8251_device::write_dsr));
	rs232a.cts_handler().set(m_uart0, FUNC(i8251_device::write_cts));
	rs232a.set_option_device_input_defaults("terminal", DEVICE_INPUT_DEFAULTS_NAME(terminal)); // must be exactly here

	I8251(config, m_uart1, 0);
	m_uart1->txd_handler().set("rs232b", FUNC(rs232_port_device::write_txd));
	m_uart1->dtr_handler().set("rs232b", FUNC(rs232_port_device::write_dtr));
	m_uart1->rts_handler().set("rs232b", FUNC(rs232_port_device::write_rts));

	rs232_port_device &rs232b(RS232_PORT(config, "rs232b", default_rs232_devices, nullptr));
	rs232b.rxd_handler().set(m_uart1, FUNC(i8251_device::write_rxd));
	rs232b.dsr_handler().set(m_uart1, FUNC(i8251_device::write_dsr));
	rs232b.cts_handler().set(m_uart1, FUNC(i8251_device::write_cts));

	COM5016_5(config, m_brg, 4.9152_MHz_XTAL);   // BR1941L
	m_brg->fr_handler().set(m_uart0, FUNC(i8251_device::write_txc));
	m_brg->fr_handler().append(m_uart0, FUNC(i8251_device::write_rxc));
	m_brg->ft_handler().set(m_uart1, FUNC(i8251_device::write_txc));
	m_brg->ft_handler().append(m_uart1, FUNC(i8251_device::write_rxc));

	MCFG_DEVICE_ADD(m_fdc, FD1771, 4_MHz_XTAL / 2)
	MCFG_WD_FDC_INTRQ_CALLBACK(WRITELINE(*this, icebox_state, intrq_w))
	MCFG_WD_FDC_DRQ_CALLBACK(WRITELINE(*this, icebox_state, drq_w))
	MCFG_FLOPPY_DRIVE_ADD(m_floppy0, floppies, "flop", floppy_image_device::default_floppy_formats)
	MCFG_FLOPPY_DRIVE_SOUND(true)
	MCFG_FLOPPY_DRIVE_ADD(m_floppy1, floppies, "flop", floppy_image_device::default_floppy_formats)
	MCFG_FLOPPY_DRIVE_SOUND(true)
MACHINE_CONFIG_END

/* ROM definition */
ROM_START( icebox )
	ROM_REGION( 0x1000, "maincpu", 0 )
	ROM_LOAD( "ice0.bin", 0x0000, 0x0800, CRC(252092c0) SHA1(8b6c53994dbb1aa76fdb6b72961e12071c100809) )
	ROM_LOAD( "ice1.bin", 0x0800, 0x0800, CRC(f4dc4b93) SHA1(cd8c3b2a1ceb4e5efb35af9bcac7ebaab6a8a308) )
ROM_END

/* Driver */

//    YEAR  NAME     PARENT  COMPAT  MACHINE  INPUT     CLASS         INIT          COMPANY             FULLNAME         FLAGS
COMP( 19??, icebox,  0,      0,      icebox,  icebox,   icebox_state, empty_init, "Nutting Associates", "ICEBOX",   MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW ) // Terminal beeps
