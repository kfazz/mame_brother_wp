// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************/
/**
  * @file mconfig.h
  * @defgroup MACHINE_CONFIG Machine configuration macros and functions
  * @{
  */
/***************************************************************************/


#pragma once

#ifndef __EMU_H__
#error Dont include this file directly; include emu.h instead.
#endif

#ifndef MAME_EMU_MCONFIG_H
#define MAME_EMU_MCONFIG_H

#include <cassert>
#include <memory>
#include <tuple>
#include <type_traits>
#include <typeinfo>
#include <utility>


//**************************************************************************
//  CONSTANTS
//**************************************************************************

// by convention, tags should all be lowercase
#define MIN_TAG_LENGTH          1

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

namespace emu { namespace detail {

struct machine_config_replace { machine_config &config; };

} } // namesapce emu::detail


struct internal_layout
{
	size_t decompressed_size;
	size_t compressed_size;
	u8 compression_type;
	const u8* data;
};


// ======================> machine_config

// machine configuration definition
class machine_config
{
	DISABLE_COPYING(machine_config);

	friend class running_machine;

public:
	class token
	{
	public:
		token(machine_config &host, device_t &device) : m_host(host), m_device(&device)
		{
			assert(m_device == m_host.m_current_device);
		}
		token(token &&that) : m_host(that.m_host), m_device(that.m_device)
		{
			that.m_device = nullptr;
			assert(!m_device || (m_device == m_host.m_current_device));
		}
		token(token const &) = delete;
		token &operator=(token &&) = delete;
		token &operator=(token const &) = delete;
		~token()
		{
			if (m_device)
			{
				assert(m_device == m_host.m_current_device);
				m_host.m_current_device = nullptr;
			}
		}
	private:
		machine_config &m_host;
		device_t *m_device;
	};

	// construction/destruction
	machine_config(const game_driver &gamedrv, emu_options &options);
	~machine_config();

	// getters
	const game_driver &gamedrv() const { return m_gamedrv; }
	device_t &root_device() const { assert(m_root_device); return *m_root_device; }
	device_t &current_device() const { assert(m_current_device); return *m_current_device; }
	emu_options &options() const { return m_options; }
	inline device_t *device(const char *tag) const { return root_device().subdevice(tag); }
	template <class DeviceClass> inline DeviceClass *device(const char *tag) const { return downcast<DeviceClass *>(device(tag)); }

	// public state
	attotime                m_minimum_quantum;          // minimum scheduling quantum
	std::string             m_perfect_cpu_quantum;      // tag of CPU to use for "perfect" scheduling

	// other parameters
	const internal_layout *            m_default_layout;           // default layout for this machine

	// helpers during configuration; not for general use
	token begin_configuration(device_t &device)
	{
		assert(!m_current_device);
		m_current_device = &device;
		return token(*this, device);
	}
	emu::detail::machine_config_replace replace() { return emu::detail::machine_config_replace{ *this }; };
	device_t *device_add(const char *tag, device_type type, u32 clock);
	template <typename Creator>
	device_t *device_add(const char *tag, Creator &&type, u32 clock)
	{
		return device_add(tag, device_type(type), clock);
	}
	template <typename Creator, typename... Params>
	auto device_add(const char *tag, Creator &&type, Params &&... args)
	{
		std::pair<const char *, device_t *> const owner(resolve_owner(tag));
		auto device(type.create(*this, owner.first, owner.second, std::forward<Params>(args)...));
		auto &result(*device);
		assert(type.type() == typeid(result));
		add_device(std::move(device), owner.second);
		return &result;
	}
	template <typename Creator, typename... Params>
	auto device_add(const char *tag, Creator &&type, XTAL clock, Params &&... args)
	{
		clock.validate(std::string("Instantiating device ") + tag);
		return device_add(tag, std::forward<Creator>(type), clock.value(), std::forward<Params>(args)...);
	}
	device_t *device_replace(const char *tag, device_type type, u32 clock);
	template <typename Creator>
	device_t *device_replace(const char *tag, Creator &&type, u32 clock)
	{
		return device_replace(tag, device_type(type), clock);
	}
	template <typename Creator, typename... Params>
	auto device_replace(const char *tag, Creator &&type, Params &&... args)
	{
		std::tuple<const char *, device_t *, device_t *> const existing(prepare_replace(tag));
		auto device(type.create(*this, std::get<0>(existing), std::get<1>(existing), std::forward<Params>(args)...));
		auto &result(*device);
		assert(type.type() == typeid(result));
		replace_device(std::move(device), *std::get<1>(existing), std::get<2>(existing));
		return &result;
	}
	template <typename Creator, typename... Params>
	auto device_replace(const char *tag, Creator &&type, XTAL clock, Params &&... args)
	{
		clock.validate(std::string("Replacing device ") + tag);
		return device_replace(tag, std::forward<Creator>(type), clock.value(), std::forward<Params>(args)...);
	}
	device_t *device_remove(const char *tag);
	device_t *device_find(device_t *owner, const char *tag);

private:
	class current_device_stack;

	// internal helpers
	std::pair<const char *, device_t *> resolve_owner(const char *tag) const;
	std::tuple<const char *, device_t *, device_t *> prepare_replace(const char *tag);
	device_t &add_device(std::unique_ptr<device_t> &&device, device_t *owner);
	device_t &replace_device(std::unique_ptr<device_t> &&device, device_t &owner, device_t *existing);
	void remove_references(ATTR_UNUSED device_t &device);

	// internal state
	game_driver const &         m_gamedrv;
	emu_options &               m_options;
	std::unique_ptr<device_t>   m_root_device;
	device_t *                  m_current_device;
};


namespace emu { namespace detail {

template <typename Tag, typename Creator, typename... Params>
inline std::enable_if_t<emu::detail::is_device_implementation<typename std::remove_reference_t<Creator>::exposed_type>::value, typename std::remove_reference_t<Creator>::exposed_type *> device_add_impl(machine_config &mconfig, Tag &&tag, Creator &&type, Params &&... args)
{
	return &type(mconfig, std::forward<Tag>(tag), std::forward<Params>(args)...);
}
template <typename Tag, typename Creator, typename... Params>
inline std::enable_if_t<emu::detail::is_device_interface<typename std::remove_reference_t<Creator>::exposed_type>::value, device_t *> device_add_impl(machine_config &mconfig, Tag &&tag, Creator &&type, Params &&... args)
{
	return &type(mconfig, std::forward<Tag>(tag), std::forward<Params>(args)...).device();
}
template <typename Tag, typename Creator, typename... Params>
inline std::enable_if_t<emu::detail::is_device_implementation<typename std::remove_reference_t<Creator>::exposed_type>::value, typename std::remove_reference_t<Creator>::exposed_type *> device_replace_impl(machine_config &mconfig, Tag &&tag, Creator &&type, Params &&... args)
{
	return &type(mconfig.replace(), std::forward<Tag>(tag), std::forward<Params>(args)...);
}
template <typename Tag, typename Creator, typename... Params>
inline std::enable_if_t<emu::detail::is_device_interface<typename std::remove_reference_t<Creator>::exposed_type>::value, device_t *> device_replace_impl(machine_config &mconfig, Tag &&tag, Creator &&type, Params &&... args)
{
	return &type(mconfig.replace(), std::forward<Tag>(tag), std::forward<Params>(args)...).device();
}

} } // namespace emu::detail


//*************************************************************************/
/** @name Machine config start/end macros */
//*************************************************************************/

/**
 @def MACHINE_CONFIG_START(_name)
 Begins a device machine configuration member
 @param _name name of this config
 @hideinitializer
*/
#define MACHINE_CONFIG_START(_name) \
ATTR_COLD void _name(machine_config &config) \
{ \
	device_t *device = nullptr; \
	devcb_base *devcb = nullptr; \
	(void)device; \
	(void)devcb;

/**
@def MACHINE_CONFIG_END
Ends a machine_config.
@hideinitializer
*/
#define MACHINE_CONFIG_END \
}

//*************************************************************************/
/** @name Core machine config options */
//*************************************************************************/

// scheduling parameters
#define MCFG_QUANTUM_TIME(_time) \
	config.m_minimum_quantum = _time;
#define MCFG_QUANTUM_PERFECT_CPU(_cputag) \
	config.m_perfect_cpu_quantum = subtag(_cputag);

// core video parameters
#define MCFG_DEFAULT_LAYOUT(_layout) \
	config.m_default_layout = &(_layout);

// add/remove devices
#define MCFG_DEVICE_ADD(_tag, ...) \
	device = emu::detail::device_add_impl(config, _tag, __VA_ARGS__);
#define MCFG_DEVICE_REPLACE(_tag, ...) \
	device = emu::detail::device_replace_impl(config, _tag, __VA_ARGS__);
#define MCFG_DEVICE_REMOVE(_tag) \
	device = config.device_remove(_tag);
#define MCFG_DEVICE_MODIFY(_tag)    \
	device = config.device_find(this, _tag);

#endif  /* MAME_EMU_MCONFIG_H */
/** @} */
