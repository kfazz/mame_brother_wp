// license:BSD-3-Clause
// copyright-holders:Barry Rodewald
/*
 * wpc_flip1.h
 *
 */

#ifndef MAME_INCLUDES_WPC_FLIP1_H
#define MAME_INCLUDES_WPC_FLIP1_H

#include "includes/wpc_dot.h"

class wpc_flip1_state : public wpc_dot_state
{
public:
	wpc_flip1_state(const machine_config &mconfig, device_type type, const char *tag)
		: wpc_dot_state(mconfig, type, tag)
	{ }
public:
	DECLARE_DRIVER_INIT(wpc_flip1);
	void wpc_flip1(machine_config &config);
	void wpc_flip1_map(address_map &map);
};

#endif // MAME_INCLUDES_WPC_FLIP1_H
