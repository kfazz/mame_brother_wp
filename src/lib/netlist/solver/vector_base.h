// license:GPL-2.0+
// copyright-holders:Couriersud
/*
 * vector_base.h
 *
 * Base vector operations
 *
 */

#ifndef VECTOR_BASE_H_
#define VECTOR_BASE_H_

#include <algorithm>
#include <cmath>
#include <type_traits>
#include "../plib/pconfig.h"

#if 0
template <unsigned storage_N>
struct pvector
{
	pvector(unsigned size)
	: m_N(size) { }

	unsigned size() {
		if (storage_N)
	}

	double m_V[storage_N];
private:
	unsigned m_N;
};
#endif

#if !defined(__clang__) && !defined(_MSC_VER) && (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ > 6))
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif

template<typename VT, typename T>
void vec_set_scalar (const std::size_t n, VT &v, const T & scalar)
{
	for ( std::size_t i = 0; i < n; i++ )
		v[i] = scalar;
}

template<typename VT, typename VS>
void vec_set (const std::size_t n, VT &v, const VS & source)
{
	for ( std::size_t i = 0; i < n; i++ )
		v[i] = source [i];
}

template<typename T, typename V1, typename V2>
T vec_mult (const std::size_t n, const V1 & v1, const V2 & v2 )
{
	T value = 0.0;
	for ( std::size_t i = 0; i < n; i++ )
		value += v1[i] * v2[i];
	return value;
}

template<typename T, typename VT>
T vec_mult2 (const std::size_t n, const VT &v)
{
	T value = 0.0;
	for ( std::size_t i = 0; i < n; i++ )
		value += v[i] * v[i];
	return value;
}

template<typename VV, typename T, typename VR>
void vec_mult_scalar (const std::size_t n, const VV & v, const T & scalar, VR & result)
{
	for ( std::size_t i = 0; i < n; i++ )
		result[i] = scalar * v[i];
}

template<typename VV, typename T, typename VR>
void vec_add_mult_scalar (const std::size_t n, const VV & v, const T scalar, VR & result)
{
	for ( std::size_t i = 0; i < n; i++ )
		result[i] = result[i] + scalar * v[i];
}

template<typename T>
void vec_add_mult_scalar_p(const std::size_t & n, const T * RESTRICT v, const T scalar, T * RESTRICT result)
{
	for ( std::size_t i = 0; i < n; i++ )
		result[i] += scalar * v[i];
}

template<typename V, typename R>
void vec_add_ip(const std::size_t n, const V & v, R & result)
{
	for ( std::size_t i = 0; i < n; i++ )
		result[i] += v[i];
}

template<typename V1, typename V2, typename VR>
void vec_sub(const std::size_t n, const V1 &v1, const V2 & v2, VR & result)
{
	for ( std::size_t i = 0; i < n; i++ )
		result[i] = v1[i] - v2[i];
}

template<typename V, typename T>
void vec_scale(const std::size_t n, V & v, const T scalar)
{
	for ( std::size_t i = 0; i < n; i++ )
		v[i] = scalar * v[i];
}

template<typename T, typename V>
T vec_maxabs(const std::size_t n, const V & v)
{
	T ret = 0.0;
	for ( std::size_t i = 0; i < n; i++ )
		ret = std::max(ret, std::abs(v[i]));

	return ret;
}
#if !defined(__clang__) && !defined(_MSC_VER) && (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ > 6))
#pragma GCC diagnostic pop
#endif

#endif /* MAT_CR_H_ */
