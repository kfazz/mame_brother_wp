// license:BSD-3-Clause
// copyright-holders:Nathan Woods
/***************************************************************************

    coco_dcmodem.cpp

    Code for emulating the CoCo Direct Connect Modem PAK

***************************************************************************/

#include "emu.h"
#include "coco_dcmodem.h"

#include "machine/mos6551.h"
#include "bus/rs232/rs232.h"


/***************************************************************************
    CONSTANTS
***************************************************************************/

#define UART_TAG        "uart"
#define PORT_TAG        "port"


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> coco_dc_modem_device

namespace
{
	class coco_dc_modem_device :
		public device_t,
		public device_cococart_interface
	{
	public:
		// construction/destruction
		coco_dc_modem_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
			: device_t(mconfig, COCO_DCMODEM, tag, owner, clock)
			, device_cococart_interface(mconfig, *this)
			, m_uart(*this, UART_TAG)
		{
		}

		// optional information overrides
		virtual void device_add_mconfig(machine_config &config) override;

		// callbacks
		WRITE_LINE_MEMBER(uart_irq_w)
		{
			set_line_value(line::CART, state != 0);
		}

	protected:
		// device-level overrides
		virtual void device_start() override
		{
			install_readwrite_handler(0xFF6C, 0xFF6F,
				read8_delegate(FUNC(mos6551_device::read), (mos6551_device *)m_uart),
				write8_delegate(FUNC(mos6551_device::write), (mos6551_device *)m_uart));
		}

		virtual const tiny_rom_entry *device_rom_region() const override;

		// CoCo cartridge level overrides
		virtual uint8_t *get_cart_base() override
		{
			return memregion("eprom")->base();
		}

	private:
		// internal state
		required_device<mos6551_device> m_uart;
	};
};


/***************************************************************************
    IMPLEMENTATION
***************************************************************************/

MACHINE_CONFIG_START(coco_dc_modem_device::device_add_mconfig)
	MCFG_DEVICE_ADD(UART_TAG, MOS6551, 0)
	MCFG_MOS6551_XTAL(XTAL(1'843'200))
	MCFG_MOS6551_IRQ_HANDLER(WRITELINE(*this, coco_dc_modem_device, uart_irq_w))
	MCFG_MOS6551_TXD_HANDLER(WRITELINE(PORT_TAG, rs232_port_device, write_txd))

	MCFG_DEVICE_ADD(PORT_TAG, RS232_PORT, default_rs232_devices, nullptr)
	MCFG_RS232_RXD_HANDLER(WRITELINE(UART_TAG, mos6551_device, write_rxd))
	MCFG_RS232_DCD_HANDLER(WRITELINE(UART_TAG, mos6551_device, write_dcd))
	MCFG_RS232_DSR_HANDLER(WRITELINE(UART_TAG, mos6551_device, write_dsr))
	MCFG_RS232_CTS_HANDLER(WRITELINE(UART_TAG, mos6551_device, write_cts))
MACHINE_CONFIG_END


ROM_START(coco_dcmodem)
	ROM_REGION(0x2000, "eprom", ROMREGION_ERASE00)
	ROM_LOAD("direct connect modem pak,1985,26-2228,tandy.rom", 0x0000, 0x2000, CRC(667bc55d) SHA1(703fe0aba4a603591078cb675ffd26a67c02df88))
ROM_END

DEFINE_DEVICE_TYPE_PRIVATE(COCO_DCMODEM, device_cococart_interface, coco_dc_modem_device, "coco_dcmodem", "CoCo Direct Connect Modem PAK")


//-------------------------------------------------
//  device_rom_region
//-------------------------------------------------

const tiny_rom_entry *coco_dc_modem_device::device_rom_region() const
{
	return ROM_NAME(coco_dcmodem);
}


