// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * nllists.h
 *
 */

#pragma once

#ifndef NLLISTS_H_
#define NLLISTS_H_

#include <atomic>

#include "nl_config.h"
#include "plib/plists.h"
#include "plib/pchrono.h"

// ----------------------------------------------------------------------------------------
// timed queue
// ----------------------------------------------------------------------------------------


namespace netlist
{

	//FIXME: move to an appropriate place
	template<bool enabled_ = true>
	class pspin_lock
	{
	public:
		pspin_lock() { }
		void acquire() noexcept{ while (m_lock.test_and_set(std::memory_order_acquire)) { } }
		void release() noexcept { m_lock.clear(std::memory_order_release); }
	private:
		std::atomic_flag m_lock = ATOMIC_FLAG_INIT;
	};

	template<>
	class pspin_lock<false>
	{
	public:
		void acquire() const noexcept { }
		void release() const noexcept { }
	};

	#if HAS_OPENMP && USE_OPENMP
		using tqlock = pspin_lock<true>;
	#else
		using tqlock = pspin_lock<false>;
	#endif

	template <class Element, class Time>
	class timed_queue
	{
		P_PREVENT_COPYING(timed_queue)
	public:

		struct entry_t
		{
			Time m_exec_time;
			Element m_object;
		};

		timed_queue(unsigned list_size)
		: m_list(list_size)
		{
			m_lock.acquire();
			clear();
			m_lock.release();
		}

		std::size_t capacity() const { return m_list.size(); }
		bool empty() const { return (m_end == &m_list[1]); }

		void push(Element o, const Time t) noexcept
		{
			/* Lock */
			m_lock.acquire();
			entry_t * i = m_end;
			for (; t > (i - 1)->m_exec_time; --i)
			{
				*(i) = *(i-1);
				m_prof_sortmove.inc();
			}
			*i = { t, o };
			++m_end;
			m_prof_call.inc();
			m_lock.release();
		}

		entry_t pop() noexcept              { return *(--m_end); }
		const entry_t &top() const noexcept { return *(m_end-1); }

		void remove(const Element &elem) noexcept
		{
			/* Lock */
			m_lock.acquire();
			for (entry_t * i = m_end - 1; i > &m_list[0]; i--)
			{
				if (i->m_object == elem)
				{
					m_end--;
					while (i < m_end)
					{
						*i = *(i+1);
						++i;
					}
					m_lock.release();
					return;
				}
			}
			m_lock.release();
		}

		void retime(const Element &elem, const Time t) noexcept
		{
			remove(elem);
			push(elem, t);
		}

		void clear()
		{
			m_end = &m_list[0];
			/* put an empty element with maximum time into the queue.
			 * the insert algo above will run into this element and doesn't
			 * need a comparison with queue start.
			 */
			m_list[0] = { Time::never(), Element(0) };
			m_end++;
		}

		// save state support & mame disasm

		const entry_t *listptr() const { return &m_list[1]; }
		std::size_t size() const noexcept { return static_cast<std::size_t>(m_end - &m_list[1]); }
		const entry_t & operator[](const std::size_t index) const { return m_list[ 1 + index]; }

	private:

		tqlock m_lock;
		entry_t * m_end;
		std::vector<entry_t> m_list;

	public:
		// profiling
		nperfcount_t m_prof_sortmove;
		nperfcount_t m_prof_call;
};

}

#endif /* NLLISTS_H_ */
