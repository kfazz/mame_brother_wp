// license:BSD-3-Clause
// copyright-holders:R. Belmont
/******************************************************************************
*
*   MIPS DECstation I/O Gate Array emulation
*
*
*/
#ifndef MAME_MACHINE_DECIOGA_H
#define MAME_MACHINE_DECIOGA_H

#pragma once


class dec_ioga_device : public device_t
{
public:
	dec_ioga_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	void map(address_map &map);

	// irq inputs
	DECLARE_WRITE_LINE_MEMBER(rtc_irq_w);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

	DECLARE_READ32_MEMBER(csr_r);
	DECLARE_WRITE32_MEMBER(csr_w);
	DECLARE_READ32_MEMBER(intr_r);
	DECLARE_WRITE32_MEMBER(intr_w);
	DECLARE_READ32_MEMBER(imsk_r);
	DECLARE_WRITE32_MEMBER(imsk_w);
private:
	uint32_t m_csr, m_intr, m_imsk;
};

DECLARE_DEVICE_TYPE(DECSTATION_IOGA, dec_ioga_device)

#endif // MAME_MACHINE_DECIOGA_H
