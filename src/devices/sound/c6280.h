// license:BSD-3-Clause
// copyright-holders:Charles MacDonald
#ifndef MAME_SOUND_C6280_H
#define MAME_SOUND_C6280_H

#pragma once

class c6280_device : public device_t, public device_sound_interface
{
public:
	c6280_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// write only
	DECLARE_WRITE8_MEMBER( c6280_w );

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_clock_changed() override;

	// sound stream update overrides
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

private:
	void calculate_clocks();

	struct channel {
		uint16_t m_frequency;
		uint8_t m_control;
		uint8_t m_balance;
		uint8_t m_waveform[32];
		uint8_t m_index;
		int16_t m_dda;
		uint8_t m_noise_control;
		uint32_t m_noise_counter;
		uint32_t m_counter;
	};

	// internal state
	sound_stream *m_stream;
	uint8_t m_select;
	uint8_t m_balance;
	uint8_t m_lfo_frequency;
	uint8_t m_lfo_control;
	channel m_channel[8];
	int16_t m_volume_table[32];
	uint32_t m_noise_freq_tab[32];
	uint32_t m_wave_freq_tab[4096];
};

DECLARE_DEVICE_TYPE(C6280, c6280_device)

#define MCFG_C6280_CPU(tag) \
	downcast<c6280_device &>(*device).set_devicecpu_tag(tag);

#endif // MAME_SOUND_C6280_H
