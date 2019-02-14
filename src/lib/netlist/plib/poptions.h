// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * poptions.h
 *
 */

#pragma once

#ifndef POPTIONS_H_
#define POPTIONS_H_

#include "pstring.h"
#include "plists.h"
#include "putil.h"

namespace plib {
/***************************************************************************
    Options
***************************************************************************/

class options;

class option_base
{
public:
	option_base(options &parent, pstring help);
	virtual ~option_base();

	pstring help() { return m_help; }
private:
	pstring m_help;
};

class option_group : public option_base
{
public:
	option_group(options &parent, pstring group, pstring help)
	: option_base(parent, help), m_group(group) { }
	~option_group();

	pstring group() { return m_group; }
private:
	pstring m_group;
};

class option_example : public option_base
{
public:
	option_example(options &parent, pstring group, pstring help)
	: option_base(parent, help), m_example(group) { }
	~option_example();

	pstring example() { return m_example; }
private:
	pstring m_example;
};


class option : public option_base
{
public:
	option(options &parent, pstring ashort, pstring along, pstring help, bool has_argument);
	~option();

	/* no_argument options will be called with "" argument */

	pstring short_opt() { return m_short; }
	pstring long_opt() { return m_long; }
	bool has_argument() { return m_has_argument ; }
	bool was_specified() { return m_specified; }

	int do_parse(const pstring &argument)
	{
		m_specified = true;
		return parse(argument);
	}

protected:
	virtual int parse(const pstring &argument) = 0;

private:
	pstring m_short;
	pstring m_long;
	bool m_has_argument;
	bool m_specified;
};

class option_str : public option
{
public:
	option_str(options &parent, pstring ashort, pstring along, pstring defval, pstring help)
	: option(parent, ashort, along, help, true), m_val(defval)
	{}

	pstring operator ()() { return m_val; }

protected:
	virtual int parse(const pstring &argument) override;

private:
	pstring m_val;
};

class option_str_limit_base : public option
{
public:
	option_str_limit_base(options &parent, pstring ashort, pstring along, std::vector<pstring> &&limit, pstring help)
	: option(parent, ashort, along, help, true)
	, m_limit(limit)
	{
	}
	const std::vector<pstring> &limit() const { return m_limit; }

protected:

private:
	std::vector<pstring> m_limit;
};


template <typename T>
class option_str_limit : public option_str_limit_base
{
public:
	option_str_limit(options &parent, pstring ashort, pstring along, const T &defval, std::vector<pstring> &&limit, pstring help)
	: option_str_limit_base(parent, ashort, along, std::move(limit), help), m_val(defval)
	{
	}

	T operator ()() { return m_val; }

	pstring as_string() const { return limit()[m_val]; }

protected:
	virtual int parse(const pstring &argument) override
	{
		auto raw = plib::container::indexof(limit(), argument);

		if (raw != plib::container::npos)
		{
			m_val = static_cast<T>(raw);
			return 0;
		}
		else
			return 1;
	}

private:
	T m_val;
};

class option_bool : public option
{
public:
	option_bool(options &parent, pstring ashort, pstring along, pstring help)
	: option(parent, ashort, along, help, false), m_val(false)
	{}

	bool operator ()() { return m_val; }

protected:
	virtual int parse(const pstring &argument) override;

private:
	bool m_val;
};

template <typename T>
class option_num : public option
{
public:
	option_num(options &parent, pstring ashort, pstring along, T defval,
			pstring help,
			T minval = std::numeric_limits<T>::min(),
			T maxval = std::numeric_limits<T>::max() )
	: option(parent, ashort, along, help, true)
	, m_val(defval)
	, m_min(minval)
	, m_max(maxval)
	{}

	T operator ()() { return m_val; }

protected:
	virtual int parse(const pstring &argument) override
	{
		bool err;
		m_val = pstonum_ne<T>(argument, err);
		return (err ? 1 : (m_val < m_min || m_val > m_max));
	}

private:
	T m_val;
	T m_min;
	T m_max;
};

class option_vec : public option
{
public:
	option_vec(options &parent, pstring ashort, pstring along, pstring help)
	: option(parent, ashort, along, help, true)
	{}

	std::vector<pstring> operator ()() { return m_val; }

protected:
	virtual int parse(const pstring &argument) override;

private:
	std::vector<pstring> m_val;
};

class option_args : public option_vec
{
public:
	option_args(options &parent, pstring help)
	: option_vec(parent, "", "", help)
	{}
};

class options
{
public:

	options();
	explicit options(option *o[]);

	~options();

	void register_option(option_base *opt);
	int parse(int argc, char *argv[]);

	pstring help(pstring description, pstring usage,
			unsigned width = 72, unsigned indent = 20);

	pstring app() { return m_app; }

private:
	static pstring split_paragraphs(pstring text, unsigned width, unsigned indent,
			unsigned firstline_indent);

	void check_consistency();

	template <typename T>
	T *getopt_type()
	{
		for (auto & optbase : m_opts )
		{
			if (auto opt = dynamic_cast<T *>(optbase))
				return opt;
		}
		return nullptr;
	}

	option *getopt_short(pstring arg);
	option *getopt_long(pstring arg);

	std::vector<option_base *> m_opts;
	pstring m_app;
	option_args * m_other_args;
};

}

#endif /* POPTIONS_H_ */
