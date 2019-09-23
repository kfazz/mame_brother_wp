// license:BSD-3-Clause
// copyright-holders:Ken Fazzone
/***************************************************************************

        TIM-011

        04/09/2010 Skeleton driver.

****************************************************************************/

#include "emu.h"
#include "cpu/z180/z180.h"
#include "imagedev/floppy.h"
#include "machine/upd765.h"
#include "emupal.h"
#include "screen.h"

#define FDC9266_TAG "fdc"

class wp75_state : public driver_device
{
public:
	wp75_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		//, m_videoram(*this, "vram")
		, m_fdc(*this, FDC9266_TAG)
		, m_floppy0(*this, FDC9266_TAG ":0:35dd")
	{ }

	void init_wp75();

	void wp75(machine_config &config);

private:
	virtual void machine_reset() override;
	virtual void video_start() override;
	uint32_t screen_update_wp75(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	DECLARE_WRITE8_MEMBER(vpl_w);
	DECLARE_WRITE8_MEMBER(vph_w);
	DECLARE_WRITE8_MEMBER(vram_w);
	DECLARE_READ8_MEMBER(vram_r);
	DECLARE_WRITE8_MEMBER(vctl_w);
	DECLARE_WRITE8_MEMBER(vcrl_w);
	DECLARE_WRITE8_MEMBER(vcrh_w);
	DECLARE_WRITE8_MEMBER(bank_w);
	INTERRUPT_GEN_MEMBER(wp75_timer_interrupt);
	DECLARE_READ8_MEMBER(portb_r); //limit switches
	DECLARE_READ8_MEMBER(sys_r); //port c
	DECLARE_WRITE8_MEMBER(irq1_clear_w);

	DECLARE_READ8_MEMBER(unk_r);
	DECLARE_WRITE8_MEMBER(unk_w);
	//fake floppy state for now
	DECLARE_READ8_MEMBER(fdc_r);
	DECLARE_WRITE8_MEMBER(fdc_w);

	DECLARE_WRITE8_MEMBER(kb_w);
	DECLARE_READ8_MEMBER(kb_r);
	DECLARE_WRITE8_MEMBER(ca_w);
	DECLARE_WRITE8_MEMBER(wh_w);
	DECLARE_WRITE8_MEMBER(lf_w);

	DECLARE_WRITE8_MEMBER(window_w);
	DECLARE_READ8_MEMBER(window_r);
	DECLARE_READ8_MEMBER(rom_wind_r);

	DECLARE_WRITE8_MEMBER(fdd_w);
	DECLARE_READ8_MEMBER(fdd_r);

	DECLARE_WRITE8_MEMBER(pg_w);
	DECLARE_WRITE8_MEMBER(pk_w);

	DECLARE_WRITE_LINE_MEMBER(drq_w);

	uint8_t ga_irq_mask;

	uint16_t v_ptr; //addr vram io window points at
	uint8_t vl; //addr vram io window points at
	uint8_t vh; //addr vram io window points at

	bool ca_idx = false; //true when carriage is left homed or csol is true
	bool rsol = false; //ribbon solenoid
	bool csol = false; //correction solenoid

	//carriage homing routine appears to go 40 steps left then decides 'check printer'
	uint16_t carriage_position = 40; //120 steps per inch

	//used to determing whether carriage is going left or right
	uint8_t ca_prev = 0x00; //previous ca latch contents
	uint8_t ca_now; //current ca latch contents


//	b7	6	54	32	1	0
//	crsr?	c.blnk  cr.sz   vr incr. disp.	reverse
	
	uint8_t v_ctl;

	uint8_t kb_matrix;

	required_device<cpu_device> m_maincpu;
	required_device<upd765a_device> m_fdc;
	required_device<floppy_image_device> m_floppy0;
	void wp75_io(address_map &map);
	void wp75_mem(address_map &map);
};

void wp75_state::wp75_mem(address_map &map)
{
	/*
ram 0x60000-0x61FFF mirror mask = 0x10000
mir 0x70000-0x71FFF

ram 0x02000-0x05FFF mirror mask = 0x60000
mir 0x62000-0x65FFF

ram 0x66000-0x6FFFF
	*/

	map.unmap_value_high();
//	map(0x0000, 0x1FFF).rom();
	map(0x0000, 0x7FFFF).rom();
	map(0x2000, 0x5FFF).ram().rw(FUNC(wp75_state::window_r), FUNC(wp75_state::window_w)).share("window");
//	map(0x6000,0x3FFFF).rom();

//	map(0x40000, 0x5FFFF).rom(); // Dict

//	map(0x40000, 0x5FFFF).bankr("rom1"); //dictionary & bank program
//	map(0x2000, 0x5FFF).ram(); //mirror(0x60000).ram();
//	map(0x60000, 0x6FFFF).ram();

	map(0x60000,0x61FFF).mirror(0x10000).ram();
	map(0x62000,0x65FFF).ram(); // <== window points here
	map(0x66000,0x6FFFF).ram();
//	map(0x70000,0x71FFF).ram(); //.mirror(0x10000).ram();

	//this window points at the rom that is shadowed by ram from 0x2000-0x5FFF
	map(0x72000, 0x75FFF).r(FUNC(wp75_state::rom_wind_r)).share("romwindow");

//	map(0x72000, 0x75FFF).rom();
//	map(0x76000, 0x7FFFF).rom(); //supposedly Unused?
}

void wp75_state::wp75_io(address_map &map)
{
	map.unmap_value_high();
	map(0x0000, 0x003f).ram(); /* Z180 internal registers */

	map(0x70, 0x70).mirror(0xff00).w(FUNC(wp75_state::vpl_w));
	map(0x71, 0x71).mirror(0xff00).w(FUNC(wp75_state::vph_w));
	map(0x72, 0x72).mirror(0xff00).w(FUNC(wp75_state::vram_w));
	map(0x73, 0x73).mirror(0xff00).r(FUNC(wp75_state::vram_r));
	map(0x74, 0x74).mirror(0xff00).w(FUNC(wp75_state::vctl_w));
	map(0x75, 0x75).mirror(0xff00).w(FUNC(wp75_state::vcrl_w));
	map(0x76, 0x76).mirror(0xff00).w(FUNC(wp75_state::vcrh_w));
//	map(0x80, 0xA0).mirror(0xff00).r(FUNC(wp75_state::unk_r));
//	map(0x80, 0xA0).mirror(0xff00).w(FUNC(wp75_state::unk_w));

//	map(0x78, 0x7b).mirror(0xff00).m(m_fdc, FUNC(upd765a_device::map)); //.umask16(0x0Ff);
//	map(0x7c, 0x7F).mirror(0xff00).m(m_fdc, FUNC(upd765a_device::map)); //.umask16(0x0Ff);

	map(0x78, 0x7b).mirror(0xff00).r(FUNC(wp75_state::fdc_r));
	map(0x78, 0x7b).mirror(0xff00).w(FUNC(wp75_state::fdc_w));

	map(0xB8, 0xB8).mirror(0xff00).r(FUNC(wp75_state::kb_r));
	map(0xB8, 0xB8).mirror(0xff00).w(FUNC(wp75_state::kb_w));

	map(0xC0, 0xC0).mirror(0xff00).w(FUNC(wp75_state::ca_w));
	map(0xC8, 0xC8).mirror(0xff00).w(FUNC(wp75_state::wh_w));
	map(0xD0, 0xD0).mirror(0xff00).w(FUNC(wp75_state::lf_w));

	map(0xD8, 0xD8).mirror(0xff00).w(FUNC(wp75_state::pg_w)); //solenoids and dc motor control
	map(0xF0, 0xF0).mirror(0xff00).w(FUNC(wp75_state::pk_w)); //buzzer and ?

	map(0xB0, 0xB0).mirror(0xff00).r(FUNC(wp75_state::sys_r)); //spec strap
	map(0xE0, 0xE0).mirror(0xff00).w(FUNC(wp75_state::bank_w)); //rom1 bank switch control
	map(0xA8,0xA8).mirror(0xff00).r(FUNC(wp75_state::portb_r));
	map(0xF8,0xF8).mirror(0xff00).w(FUNC(wp75_state::irq1_clear_w));

}

/* Input ports */
static INPUT_PORTS_START( wp75 )
INPUT_PORTS_END

void wp75_state::machine_reset()
{
}

void wp75_state::video_start()
{
	v_ctl &= (1 << 2);

}

void wp75_state::init_wp75()
{
	//8 banks of 128k, but A16 is don't care
	//membank("rom1")->configure_entries(0, 8, memregion("rom1")->base(), 0x20000);
	//wp70 starts with entry 0?
	//2540DS starts with entry 2???
	//membank("rom1")->set_entry(0);
}

uint32_t wp75_state::screen_update_wp75(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	uint8_t *vram = memregion("vram")->base();

	uint32_t *scanline;
	int x, y;
	uint8_t pixels;
	static const uint32_t palette[2] = {0x282828, 0xffCC00 };

	for (y = 0; y < 240; y++)
	{
		scanline = &bitmap.pix32(y);
		for (x = 0; x < 91; x++)
		{
			pixels = vram[(y * 91) + x];

			*scanline++ = palette[(pixels>>7)&1];
			*scanline++ = palette[(pixels>>6)&1];
			*scanline++ = palette[(pixels>>5)&1];
			*scanline++ = palette[(pixels>>4)&1];
			*scanline++ = palette[(pixels>>3)&1];
			*scanline++ = palette[(pixels>>2)&1];
			*scanline++ = palette[(pixels>>1)&1];
			*scanline++ = palette[(pixels>>0)&1];
		}
	}
	return 0;
}


	WRITE8_MEMBER(wp75_state::bank_w)
{
	printf("Rom1 bank dat:%02x\n",data ); //, (data >>1) & 3);
	//membank("rom1")->set_entry((data >> 1) & 3 ); //(data >> 1) & 3);
	//membank("rom1")->set_entry(data);

}

	WRITE8_MEMBER(wp75_state::vpl_w)
{
	vl = data & 0xff;
	v_ptr = (vh << 8) | (vl & 0xff);
//	printf("vpl_w :%02x   inc:%x \n", vl, (v_ctl & 0xC) >> 2);
}
	WRITE8_MEMBER(wp75_state::vph_w)
{
	vh = data & 0xFF;
	v_ptr = (vh << 8) | (vl & 0xff);
//	printf("vph_w :%02x   inc:%x\n",vh, (v_ctl & 0xC) >> 2);
}
	WRITE8_MEMBER(wp75_state::vram_w)
{
	uint8_t *vram = memregion("vram")->base();
	uint16_t index = (vh << 8) | (vl & 0xff);
//	printf("vram_w @:%02x :%02x inc %d\n",index, data, (v_ctl & 0xC) >> 2);

	if (index > 0x7FFF)
	{
		index = 0;
	}
	vram[index] = (data & 0xFF);
	if (index < 0x7FFF)
	{
		if ((v_ctl & 0xC) == 0) { //if vram addr auto increment
		v_ptr = index + 1;
		vl = (v_ptr & 0xFF);
		vh = (v_ptr >> 8);
		}
	}
	else {
		v_ptr = 0;
		vl = 0;
		vh = 0;
	}

}
	READ8_MEMBER(wp75_state::vram_r)
{
	uint8_t *vram = memregion("vram")->base();
	uint16_t index = (vh << 8) | (vl & 0xff);
	uint8_t data;
	if (index > 0x7FFF)
	{
		index = 0;
	}
	data = vram[index];
	if (index < 0x7FFF)
	{
		if ((v_ctl & 0xC) == 0) { //if vram addr auto increment
		v_ptr = index + 1;
		vl = (v_ptr & 0xFF);
		vh = (v_ptr >> 8);
		}
	}
	else {
		v_ptr = 0;
		vl = 0;
		vh = 0;
	}
	return data;
}
	WRITE8_MEMBER(wp75_state::vctl_w)
{
	v_ctl = data;
//	printf("vctl:%02x\n",data);
}
	WRITE8_MEMBER(wp75_state::vcrl_w)
{
//	printf("cursr addr l:%02x\n",data);
}

	WRITE8_MEMBER(wp75_state::vcrh_w)
{
//	printf("cursr addr h:%02x\n",data);
}

	READ8_MEMBER(wp75_state::sys_r)
{
	//11 = s5 70 or 66 line s4 0, s3,2,1,0 country code 
	uint8_t data = 0xcf; //0xc2
//	printf("sysr h:%02x\n",data);
	return data;
}

	WRITE8_MEMBER(wp75_state::pg_w)
{
	if (data & 4)
	{
	csol = false;
	}
	else {
	csol = true;
	}
	//printf("pg_w h:%02x\n",data);
}

	WRITE8_MEMBER(wp75_state::pk_w)
{
	if (data & 1)
	{
	printf("pg_w BEEP h:%02x\n",data);
	}
	else {
	//printf("pg_w h:%02x\n",data);
	}
	
}


	READ8_MEMBER(wp75_state::portb_r)
{
	uint8_t data = 0xa;//e
	if(ca_idx)
		data = 0xe;//a

	if (data != 0xa)
	printf("portb h:%02x\n",data);
	return data;
}



        READ8_MEMBER(wp75_state::fdc_r)
{
        uint8_t data = 0;
	switch (offset)
	{
	case 0: data = 00; break;
	case 1: data = 00; break;
	case 2: data = 00; break;
	case 3: data = 00; break;
	}
//        printf("unk a:%02x h:%02x\n",offset,data);
        return data;
}

        WRITE8_MEMBER(wp75_state::fdc_w)
{
//        printf("fdc a:%02x h:%02x\n",offset,data);
}


	READ8_MEMBER(wp75_state::unk_r)
{
	uint8_t data = 0x00;
//	printf("unk a:%02x h:%02x\n",offset,data);
	return data;
}

	WRITE8_MEMBER(wp75_state::unk_w)
{
//	printf("unk a:%02x h:%02x\n",offset,data);
}

WRITE8_MEMBER(wp75_state::irq1_clear_w)
{
	ga_irq_mask = data;
	//printf("irq clr h:%02x\n",data);
	m_maincpu->set_input_line(INPUT_LINE_IRQ1, CLEAR_LINE);	
}


WRITE_LINE_MEMBER(wp75_state::drq_w)
{
	if (state)
	{
		m_maincpu->set_input_line(Z180_INPUT_LINE_DREQ0, ASSERT_LINE);	
	}
	else
	{
		m_maincpu->set_input_line(Z180_INPUT_LINE_DREQ0, CLEAR_LINE);	
	}
}


INTERRUPT_GEN_MEMBER(wp75_state::wp75_timer_interrupt)
{
	if (carriage_position <= 1 || csol)
	{
	ca_idx = true;
	}
	else {
	ca_idx = false;
	}

	m_maincpu->set_input_line(INPUT_LINE_IRQ1, HOLD_LINE);
}



	WRITE8_MEMBER(wp75_state::kb_w)
{
	kb_matrix = data;
//	printf("kb_w h:%02x\n",data);
}

	READ8_MEMBER(wp75_state::kb_r)
{

	uint8_t data = 0xfF;
        if(kb_matrix == 8) data = 0xef;
        if(kb_matrix == 7) data = 0xFF;
        if(kb_matrix == 6) data = 0x7f;
        if(kb_matrix == 5) data = 0xef;
        if(kb_matrix == 4) data = 0x7f;
        if(kb_matrix == 3) data = 0x7f;
        if(kb_matrix == 2) data = 0x7f;
        if(kb_matrix == 1) data = 0x7f;
        if(kb_matrix == 0) data = 0x7f;//8B?
//	printf("kb_r m:%x h:%02x\n", kb_matrix,data);
	return data;
}

	WRITE8_MEMBER(wp75_state::ca_w)
{
	int8_t dir = 0;
	ca_now = data;
	switch (data)
	{  // 33-> 99 -> CC ->66 going left
	  //  33 ->66 -> CC ->99 going right
	case 0x33:
	if (ca_prev == 0x66) dir = -1;
	if (ca_prev == 0x99) dir = 1;
	break;
	case 0x66:
	if (ca_prev == 0xCC) dir = -1;
	if (ca_prev == 0x33) dir = 1;
	break;
	case 0x99:
	if (ca_prev == 0x33) dir = -1;
	if (ca_prev == 0xCC) dir = 1;
	break;
	case 0xCC:
	if (ca_prev == 0x99) dir = -1;
	if (ca_prev == 0x66) dir = 1;
	break;
	}
	ca_prev = data;
	if (dir == 1) carriage_position++;
	if (dir == -1) carriage_position--;

	if (data!=0xff)
		printf("ca_w h:%02x  cpos: %d\n",data, carriage_position);
}

	WRITE8_MEMBER(wp75_state::wh_w)
{
 	if(data != 0xff)
	printf("wh_w h:%02x\n",data);
}

	WRITE8_MEMBER(wp75_state::lf_w)
{
	if(data != 0xff)
		printf("lf_w h:%02x\n",data);
}




	WRITE8_MEMBER(wp75_state::window_w)
{
	uint8_t *rom = memregion("maincpu")->base();	
	rom[0x60000 + offset] = data;
	//printf("window_w a:%02x h:%02x\n",offset ,data);
}
	READ8_MEMBER(wp75_state::window_r)
{
	uint8_t *rom = memregion("maincpu")->base();
	//printf("window_r a:%02x h:%02x\n",offset ,rom[0x60000 & offset]);
	return rom[0x60000 + offset];
}

	READ8_MEMBER(wp75_state::rom_wind_r)
{
	uint8_t *rom = memregion("rom1")->base();
	//printf("romwind__r a:%02x h:%02x\n",offset ,rom[offset & 0xFFFF]);
	return rom[0x2000 + offset];// & 0x77FFF]; .
	//0x2000 seems to relate to FDC, gets stuck
	
	//0x62000 runs off the rails quicky and reboots, doesn't seem to be correct
	//0x72000 suceeds through sheer blind luck, crashes later

}



	WRITE8_MEMBER(wp75_state::fdd_w)
{
	printf("fdd_w p:%02x h:%02x\n",offset ,data);
}
	READ8_MEMBER(wp75_state::fdd_r)
{
	uint8_t data = 0xff;
	printf("fdd_r p:%02x h:%02x\n",offset ,data);
	return data;
}


static void wp75_floppies(device_slot_interface &device)
{
	device.option_add("35dd", FLOPPY_35_DD);
}

static const floppy_format_type wp75_floppy_formats[] = {
	FLOPPY_IMD_FORMAT,
	FLOPPY_MFI_FORMAT,
	FLOPPY_MFM_FORMAT,
	nullptr
};

MACHINE_CONFIG_START(wp75_state::wp75)
	/* basic machine hardware */
	MCFG_DEVICE_ADD("maincpu",Z180, XTAL(12'288'000) / 2) // location U17 HD64180
	MCFG_DEVICE_PROGRAM_MAP(wp75_mem)
	MCFG_DEVICE_IO_MAP(wp75_io)
//	MCFG_DEVICE_VBLANK_INT_DRIVER("screen", wp75_state, irq1_line_hold)
	MCFG_DEVICE_PERIODIC_INT_DRIVER(wp75_state, wp75_timer_interrupt,60)

	// FDC9266 location U43
	UPD765A(config, m_fdc, XTAL(8'000'000), true, true);
	//m_fdc->intrq_wr_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ2);
	m_fdc->drq_wr_callback().set(FUNC(wp75_state::drq_w));


	/* floppy drives */
	MCFG_FLOPPY_DRIVE_ADD(FDC9266_TAG ":0", wp75_floppies, "35dd", wp75_floppy_formats)

	/* video hardware */
	// Video hardware
//	screen_device &screen(SCREEN(config, "screen", SCREEN_TYPE_RASTER));
//	screen.set_color(0xFFCC00);
//	screen.set_physical_aspect(5, 9);
//	screen.set_raw(XTAL(15'300'000), 819, 0, 819-1, 240, 0, 240-1);
//	screen.set_refresh_hz(30); // Two interlaced fields at 60Hz => 30Hz frame rate
//	screen.set_screen_update("maincpu", FUNC(wp75_state::screen_update_wp75));
//	screen.set_palette("palette");

//	screen_device &screen(SCREEN(config, "screen", SCREEN_TYPE_RASTER));
//	screen.set_physical_aspect(5, 9);
	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500)) /* not accurate */
	MCFG_SCREEN_SIZE(819, 240)
	MCFG_SCREEN_VISIBLE_AREA(0, 819-1, 0, 240-1)
	MCFG_SCREEN_UPDATE_DRIVER(wp75_state, screen_update_wp75)
	MCFG_SCREEN_PALETTE("palette")

	PALETTE(config, "palette", palette_device::MONOCHROME);
MACHINE_CONFIG_END

/* ROM definition */
ROM_START( wp70 )
	ROM_REGION( 0x80000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD( "ua6386.bin", 0x0000, 0x80000, CRC(6d61e49d) SHA1(4f2ee2b479e8b630dad9abeb82aa86ac3ab63ef7))
	ROM_REGION( 0x80000, "rom1", ROMREGION_ERASEFF )
	ROM_LOAD( "UA2849-dict-ger.BIN", 0x0000, 0x80000, CRC(fa8712eb) SHA1(2d3454138c79e75604b30229c05ed8fb8e7d15fe))
	ROM_REGION( 0x8000, "vram", ROMREGION_ERASEFF )
ROM_END

ROM_START( wp75 )
	ROM_REGION( 0x80000, "maincpu", ROMREGION_ERASEFF )
	ROM_LOAD( "UA8504001.BIN", 0x0000, 0x80000, CRC(c515ab07) SHA1(2e99eb22e9e8ed613d1490f56b2a9296474db2e0))
	ROM_REGION( 0x80000, "rom1", ROMREGION_ERASEFF )
	ROM_LOAD( "UA8504001.BIN", 0x0000, 0x80000, CRC(c515ab07) SHA1(2e99eb22e9e8ed613d1490f56b2a9296474db2e0))
	ROM_REGION( 0x8000, "vram", ROMREGION_ERASEFF )
ROM_END


/* Driver */

/*    YEAR  NAME    PARENT  COMPAT  MACHINE  INPUT   CLASS         INIT        COMPANY                    FULLNAME   FLAGS */
COMP( 1989, wp75, 0,      0,      wp75,  wp75, wp75_state, init_wp75, "Brother", "WP-75", MACHINE_NOT_WORKING | MACHINE_NO_SOUND)
//COMP( 1989, wp70, 0,   wp75,      wp75,  wp75, wp75_state, init_wp75, "Brother", "WP-70", MACHINE_NOT_WORKING | MACHINE_NO_SOUND)
