// license:BSD-3-Clause
// copyright-holders: Samuele Zannoli
#ifndef MAME_MACHINE_NFORCEPC_H
#define MAME_MACHINE_NFORCEPC_H

#pragma once

// NVIDIA Corporation nForce CPU bridge

class crush11_host_device : public pci_host_device {
public:
	template <typename T>
	crush11_host_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock, T &&cpu_tag, int ram_size)
		: crush11_host_device(mconfig, tag, owner, clock)
	{
		set_ids_host(0x10de01a4, 0x01, 0x10430c11);
		set_cpu_tag(std::forward<T>(cpu_tag));
		set_ram_size(ram_size);
	}
	crush11_host_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <typename T> void set_cpu_tag(T &&tag) { cpu.set_tag(std::forward<T>(tag)); }
	void set_ram_size(int ram_size);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

	virtual void reset_all_mappings() override;

	virtual void map_extra(uint64_t memory_window_start, uint64_t memory_window_end, uint64_t memory_offset, address_space *memory_space,
		uint64_t io_window_start, uint64_t io_window_end, uint64_t io_offset, address_space *io_space) override;

	virtual void config_map(address_map &map) override;

private:
	int ddr_ram_size;
	required_device<device_memory_interface> cpu;
	std::vector<uint32_t> ram;

	DECLARE_READ8_MEMBER(test_r);
	DECLARE_WRITE8_MEMBER(test_w);
};

DECLARE_DEVICE_TYPE(CRUSH11, crush11_host_device)

#endif
