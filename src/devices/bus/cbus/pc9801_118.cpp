// license:BSD-3-Clause
// copyright-holders:Angelo Salese
/***************************************************************************

    NEC PC-9801-118 sound card

    YMF288 + some extra ports

    TODO:
    - preliminary, presumably needs CS-4231 too, it's an extended clone of the already emulated AD1848 used on the Windows Sound System
    - verify sound irq;

***************************************************************************/

#include "emu.h"
#include "bus/cbus/pc9801_118.h"

#include "sound/2608intf.h"
#include "speaker.h"


#define MAIN_CLOCK_X2 XTAL(2'457'600)

//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

// device type definition
DEFINE_DEVICE_TYPE(PC9801_118, pc9801_118_device, "pc9801_118", "pc9801_118")

WRITE_LINE_MEMBER(pc9801_118_device::sound_irq)
{
	/* TODO: seems to die very often */
	m_bus->int_w<5>(state);
}

//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

MACHINE_CONFIG_START(pc9801_118_device::device_add_mconfig)
	SPEAKER(config, "lspeaker").front_left();
	SPEAKER(config, "rspeaker").front_right();
	MCFG_DEVICE_ADD("opn3", YM2608, MAIN_CLOCK_X2*4) // actually YMF288, unknown clock / divider, might be X1 x 5 actually
	MCFG_YM2608_IRQ_HANDLER(WRITELINE(*this, pc9801_118_device, sound_irq))
	MCFG_AY8910_PORT_A_READ_CB(READ8(*this, pc9801_118_device, opn_porta_r))
	//MCFG_AY8910_PORT_B_READ_CB(READ8(*this, pc9801_state, opn_portb_r))
	//MCFG_AY8910_PORT_A_WRITE_CB(WRITE8(*this, pc9801_state, opn_porta_w))
	MCFG_AY8910_PORT_B_WRITE_CB(WRITE8(*this, pc9801_118_device, opn_portb_w))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "lspeaker", 1.00)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "rspeaker", 1.00)
MACHINE_CONFIG_END


//-------------------------------------------------
//  input_ports - device-specific input ports
//-------------------------------------------------

static INPUT_PORTS_START( pc9801_118 )
	PORT_INCLUDE( pc9801_joy_port )

	PORT_START("OPN3_DSW")
	PORT_CONFNAME( 0x01, 0x00, "PC-9801-118: Port Base" )
	PORT_CONFSETTING(    0x00, "0x088" )
	PORT_CONFSETTING(    0x01, "0x188" )
INPUT_PORTS_END

ioport_constructor pc9801_118_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( pc9801_118 );
}

// RAM
ROM_START( pc9801_118 )
	ROM_REGION( 0x100000, "opn3", ROMREGION_ERASE00 )
ROM_END

const tiny_rom_entry *pc9801_118_device::device_rom_region() const
{
	return ROM_NAME( pc9801_118 );
}

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  pc9801_118_device - constructor
//-------------------------------------------------

pc9801_118_device::pc9801_118_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: pc9801_snd_device(mconfig, PC9801_118, tag, owner, clock),
		m_bus(*this, DEVICE_SELF_OWNER),
		m_opn3(*this, "opn3")
{
}


//-------------------------------------------------
//  device_validity_check - perform validity checks
//  on this device
//-------------------------------------------------

void pc9801_118_device::device_validity_check(validity_checker &valid) const
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------


void pc9801_118_device::device_start()
{
	m_bus->install_io(0xa460, 0xa463, read8_delegate(FUNC(pc9801_118_device::id_r), this), write8_delegate(FUNC(pc9801_118_device::ext_w), this) );

	save_item(NAME(m_ext_reg));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void pc9801_118_device::device_reset()
{
	uint16_t port_base = (ioport("OPN3_DSW")->read() & 1) << 8;
	m_bus->io_space().unmap_readwrite(0x0088, 0x008b, 0x100);
	m_bus->install_io(port_base + 0x0088, port_base + 0x008f, read8_delegate(FUNC(pc9801_118_device::opn3_r), this), write8_delegate(FUNC(pc9801_118_device::opn3_w), this) );
	m_ext_reg = 1; // TODO: enabled or disabled?
}


//**************************************************************************
//  READ/WRITE HANDLERS
//**************************************************************************


READ8_MEMBER(pc9801_118_device::opn3_r)
{
	if(((offset & 5) == 0) || m_ext_reg)
		return m_opn3->read(space, offset >> 1);
	else // odd
	{
		//printf("PC9801-118: Read to undefined port [%02x]\n",offset+0x188);
		return 0xff;
	}
}


WRITE8_MEMBER(pc9801_118_device::opn3_w)
{
	if(((offset & 5) == 0) || m_ext_reg)
		m_opn3->write(space, offset >> 1,data);
	//else // odd
	//  printf("PC9801-118: Write to undefined port [%02x] %02x\n",offset+0x188,data);
}

READ8_MEMBER( pc9801_118_device::id_r )
{
	if(offset == 0)
	{
		printf("OPN3 EXT read ID [%02x]\n",offset);
		return 0x80 | (m_ext_reg & 1);
	}

	printf("OPN3 EXT read unk [%02x]\n",offset);
	return 0xff;
}

WRITE8_MEMBER( pc9801_118_device::ext_w )
{
	if(offset == 0)
	{
		m_ext_reg = data & 1;
		/* TODO: apparently writing a 1 doubles the available channels (and presumably enables CS-4231 too) */
		if(data)
			printf("PC-9801-118: extended register %02x write\n",data);
		return;
	}

	printf("OPN3 EXT write unk %02x -> [%02x]\n",data,offset);
}
