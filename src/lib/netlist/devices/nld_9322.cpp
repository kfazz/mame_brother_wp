// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
/*
 * nld_9322.cpp
 *
 */

#include "nld_9322.h"

namespace netlist
{
	namespace devices
	{
	NETLIB_OBJECT(9322_selector)
	{
		NETLIB_CONSTRUCTOR(9322_selector)
		, m_A(*this, "A")
		, m_B(*this, "B")
		, m_Y(*this, "Y")
		{
		}

		void update_outputs(const netlist_sig_t strobe, const netlist_sig_t select);

	public:
		logic_input_t m_A;
		logic_input_t m_B;
		logic_output_t m_Y;
	};

	// FIXME: Timing
	void NETLIB_NAME(9322_selector)::update_outputs(const netlist_sig_t strobe, const netlist_sig_t select)
	{
		if (strobe)
			m_Y.push(0, NLTIME_FROM_NS(21));
		else if (select)
			m_Y.push(m_B(), NLTIME_FROM_NS(14));
		else
			m_Y.push(m_A(), NLTIME_FROM_NS(14));
	}

	NETLIB_OBJECT(9322)
	{
		NETLIB_CONSTRUCTOR(9322)
		, m_1(*this, "1")
		, m_2(*this, "2")
		, m_3(*this, "3")
		, m_4(*this, "4")
		, m_SELECT(*this, "SELECT")
		, m_STROBE(*this, "STROBE")
		{
			register_subalias("A1", m_1.m_A);
			register_subalias("B1", m_1.m_B);
			register_subalias("Y1", m_1.m_Y);
			register_subalias("A2", m_2.m_A);
			register_subalias("B2", m_2.m_B);
			register_subalias("Y2", m_2.m_Y);
			register_subalias("A3", m_3.m_A);
			register_subalias("B3", m_3.m_B);
			register_subalias("Y3", m_3.m_Y);
			register_subalias("A4", m_4.m_A);
			register_subalias("B4", m_4.m_B);
			register_subalias("Y4", m_4.m_Y);
		}

		NETLIB_UPDATEI();

	protected:
		NETLIB_SUB(9322_selector) m_1;
		NETLIB_SUB(9322_selector) m_2;
		NETLIB_SUB(9322_selector) m_3;
		NETLIB_SUB(9322_selector) m_4;

		logic_input_t m_SELECT;
		logic_input_t m_STROBE;
	};

	NETLIB_OBJECT_DERIVED(9322_dip, 9322)
	{
		NETLIB_CONSTRUCTOR_DERIVED(9322_dip, 9322)
		{
			register_subalias("1", m_SELECT);
			register_subalias("2", m_1.m_A);
			register_subalias("3", m_1.m_B);
			register_subalias("4", m_1.m_Y);
			register_subalias("5", m_2.m_A);
			register_subalias("6", m_2.m_B);
			register_subalias("7", m_2.m_Y);

			register_subalias("9",  m_3.m_Y);
			register_subalias("10", m_3.m_B);
			register_subalias("11", m_3.m_A);
			register_subalias("12", m_4.m_Y);
			register_subalias("13", m_4.m_B);
			register_subalias("14", m_4.m_A);
			register_subalias("15", m_STROBE);
		}
	};

	NETLIB_UPDATE(9322)
	{
		const netlist_sig_t strobe = m_STROBE();
		const netlist_sig_t select = m_SELECT();
		m_1.update_outputs(strobe, select);
		m_2.update_outputs(strobe, select);
		m_3.update_outputs(strobe, select);
		m_4.update_outputs(strobe, select);
	}

	NETLIB_DEVICE_IMPL(9322)
	NETLIB_DEVICE_IMPL(9322_dip)

	} //namespace devices
} // namespace netlist
