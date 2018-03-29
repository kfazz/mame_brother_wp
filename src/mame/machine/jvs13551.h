// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
#ifndef MAME_MACHINE_JVS13551_H
#define MAME_MACHINE_JVS13551_H

#pragma once

#include "machine/jvsdev.h"

#define MCFG_SEGA_837_13551_DEVICE_ADD(_tag, _host, tilt, d0, d1, a0, a1, a2, a3, a4, a5, a6, a7, out) \
	MCFG_JVS_DEVICE_ADD(_tag, SEGA_837_13551, _host) \
	downcast<sega_837_13551_device &>(*device).set_port_tag(0, tilt);  \
	downcast<sega_837_13551_device &>(*device).set_port_tag(1, d0); \
	downcast<sega_837_13551_device &>(*device).set_port_tag(2, d1); \
	downcast<sega_837_13551_device &>(*device).set_port_tag(3, a0); \
	downcast<sega_837_13551_device &>(*device).set_port_tag(4, a1); \
	downcast<sega_837_13551_device &>(*device).set_port_tag(5, a2); \
	downcast<sega_837_13551_device &>(*device).set_port_tag(6, a3); \
	downcast<sega_837_13551_device &>(*device).set_port_tag(7, a4); \
	downcast<sega_837_13551_device &>(*device).set_port_tag(8, a5); \
	downcast<sega_837_13551_device &>(*device).set_port_tag(9, a6); \
	downcast<sega_837_13551_device &>(*device).set_port_tag(10, a7); \
	downcast<sega_837_13551_device &>(*device).set_port_tag(11, out);

class jvs_host;

class sega_837_13551_device : public jvs_device
{
public:
	sega_837_13551_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	void set_port_tag(int port, const char *tag) { port_tag[port] = tag; }

	virtual const tiny_rom_entry *device_rom_region() const override;

	DECLARE_WRITE_LINE_MEMBER(jvs13551_coin_1_w);
	DECLARE_WRITE_LINE_MEMBER(jvs13551_coin_2_w);
	void inc_coin(int coin);

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual ioport_constructor device_input_ports() const override;

	// JVS device overrides
	virtual const char *device_id() override;
	virtual uint8_t command_format_version() override;
	virtual uint8_t jvs_standard_version() override;
	virtual uint8_t comm_method_version() override;
	virtual void function_list(uint8_t *&buf) override;
	virtual bool switches(uint8_t *&buf, uint8_t count_players, uint8_t bytes_per_switch) override;
	virtual bool coin_counters(uint8_t *&buf, uint8_t count) override;
	virtual bool coin_add(uint8_t slot, int32_t count) override;
	virtual bool analogs(uint8_t *&buf, uint8_t count) override;
	virtual bool swoutputs(uint8_t count, const uint8_t *vals) override;
	virtual bool swoutputs(uint8_t id, uint8_t val) override;

private:
	const char *port_tag[12];
	ioport_port *port[12];
	uint16_t coin_counter[2];
};

DECLARE_DEVICE_TYPE(SEGA_837_13551, sega_837_13551_device)

#endif // MAME_MACHINE_JVS13551_H
