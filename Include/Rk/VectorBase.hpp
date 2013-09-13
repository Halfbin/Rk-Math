//
// Copyright (C) 2013 Rk
// Permission to use, copy, modify and distribute this file for any purpose is hereby granted without fee or other
// limitation, provided that
//  - this notice is reproduced verbatim in all copies and modifications of this file, and
//  - neither the names nor trademarks of the copyright holder are used to endorse any work which includes or is derived
//    from this file.
// No warranty applies to the use, either of this software, or any modification of it, and Rk disclaims all liabilities
// in relation to such use.
//

#pragma once

#include <Rk/Memory.hpp>
#include <Rk/Types.hpp>
#include <Rk/Lerp.hpp>

#include <type_traits>
#include <functional>
#include <algorithm>
#include <numeric>

namespace Rk
{
  template <typename T, uint N>
  class Vector
  {
    T components [N];

  public:
    enum : uint { Size = N };
    static uint size () { return Size; }

    Vector ()
    { }

    Vector (Nil) :
      components ()
    { }
    
    template <typename U>
    Vector (const Vector <U, N>& other)
    {
      copy (components, N, other.components);
    }
    
    T& operator [] (uint i)
    {
      return components [i];
    }

    const T& operator [] (uint i) const
    {
      return components [i];
    }

    T*       begin   ()       { return components; }
    const T* begin   () const { return components; }
    const T* cbegin  ()       { return components; }
    T*       rbegin  ()       { return end () - 1; }
    const T* rbegin  () const { return end () - 1; }
    const T* crbegin ()       { return end () - 1; }

    T*       end   ()       { return components + N; }
    const T* end   () const { return components + N; }
    const T* cend  ()       { return components + N; }
    T*       rend  ()       { return begin () - 1; }
    const T* rend  () const { return begin () - 1; }
    const T* crend ()       { return begin () - 1; }

  };
  
  // Sparse read/make
  template <typename T, uint N, typename Iter>
  auto read_vector (Vector <T, N>& dest, Iter iter, uint stride = 1)
    -> Vector <T, N>&
  {
    for (auto i = 0; i != N; i++)
      dest [i] = *(iter + i * stride);
    return dest;
  }

  template <uint N, typename Iter>
  auto make_vector (Iter iter, uint stride = 1)
    -> Vector <decltype (*iter), N>
  {
    Vector <decltype (*iter), N> result;
    return read_vector (result, iter, stride);
  }

  // Sparse swizzled read/make
  template <typename T, typename I, uint N, typename Iter>
  auto read_vector_swizzle (Vector <T, N>& dest, Vector <I, N> indices, Iter iter, uint stride = 1)
    -> Vector <T, N>&
  {
    for (auto i = 0; i != N; i++)
      dest [i] = *(iter + indices [i] * stride);
    return dest;
  }

  template <typename Iter, typename I, uint N>
  auto make_vector_swizzle (Iter iter, Vector <I, N> indices, uint stride = 1)
    -> Vector <decltype (*iter), N>
  {
    Vector <decltype (*iter), N> result;
    return read_vector_swizzle (result, indices, iter, stride);
  }

  template <uint N, typename Iter>
  auto make_ref_vector (Iter iter, uint stride = 1)
    -> Vector <decltype (*iter)&, N>;

  // Equality
  template <typename T, typename U, uint N>
  bool operator == (Vector <T, N> lhs, Vector <U, N> rhs)
  {
    for (uint i = 0; i != N; i++)
    {
      if (lhs [i] != rhs [i])
        return false;
    }

    return true;
  }

  template <typename T, typename U, uint N>
  bool operator != (Vector <T, N> lhs, Vector <U, N> rhs)
  {
    return !(lhs == rhs);
  }

  template <typename T, typename I, uint N>
  auto swizzle (
    Vector <T, N> value,
    Vector <I, N> indices,
    typename std::enable_if <std::is_integral <I>::value>::type* = 0)
    -> Vector <T, N>
  {
    Vector <T, N> result;
    for (auto i = 0; i != N; i++)
      result [i] = value [indices [i]];
    return result;
  }

  namespace Swizzlers
  {
    enum : uint
    {
      X = 0, Y, Z, W,
      R = 0, G, B, A,
      S = 0, T, P, Q
    };

  }

  // transform
  template <typename T, uint N, typename Func>
  auto transform (Vector <T, N> value, Func func)
    -> Vector <decltype (func (lhs [0])), N>
  {
    Vector <decltype (func (lhs [0])), N> result;

    std::transform (
      value.components,
      value.components + N
      result.components,
      func
    );
    
    return result;
  }

  template <typename T, typename U, uint N, typename Func>
  auto transform (Vector <T, N> lhs, Vector <U, N> rhs, Func func)
    -> Vector <decltype (func (lhs [0], rhs [0])), N>
  {
    Vector <decltype (func (lhs [0], rhs [0])), N> result;

    std::transform (
      lhs.components,
      lhs.components + N
      rhs.components,
      result.components,
      func
    );
    
    return result;
  }

  // sum
  template <typename T, typename U, uint N>
  auto operator + (Vector <T, N> lhs, Vector <U, N> rhs)
    -> Vector <decltype (lhs [0] + rhs [0]), N>
  {
    return transform (lhs, rhs, [] (T lh, U rh) { return lh + rh; });
  }

  template <typename T, typename U, uint N>
  auto operator += (Vector <T, N> lhs, U&& rhs)
    -> Vector <T, N>&
  {
    return lhs = lhs + std::forward <U> (rhs);
  }

  // difference
  template <typename T, typename U, uint N>
  auto operator - (Vector <T, N> lhs, Vector <U, N> rhs)
    -> Vector <decltype (lhs [0] - rhs [0]), N>
  {
    return transform (lhs, rhs, [] (T lh, U rh) { return lh - rh; });
  }

  template <typename T, typename U, uint N>
  auto operator -= (Vector <T, N> lhs, U&& rhs)
    -> Vector <T, N>&
  {
    return lhs = lhs - std::forward <U> (rhs);
  }

  // product
  template <typename T, typename U, uint N>
  auto operator * (Vector <T, N> lhs, Vector <U, N> rhs)
    -> Vector <decltype (lhs [0] * rhs [0]), N>
  {
    return transform (lhs, rhs, [] (T lh, U rh) { return lh * rh; });
  }

  template <typename T, typename U, uint N>
  auto operator * (Vector <T, N> lhs, U rhs)
    -> typename std::enable_if <
      std::is_arithmetic <U>::value,
      Vector <decltype (lhs [0] * rhs), N>
    >::type
  {
    return transform (lhs, [rhs] (T x) { return x * rhs; });
  }

  template <typename T, typename U, uint N>
  auto operator * (T lhs, Vector <U, N> rhs)
    -> typename std::enable_if <
      std::is_arithmetic <T>::value,
      Vector <decltype (lhs * rhs [0]), N>
    >::type
  {
    return rhs * lhs;
  }

  template <typename T, typename U, uint N>
  auto operator *= (Vector <T, N> lhs, U&& rhs)
    -> Vector <T, N>&
  {
    return lhs = lhs * std::forward <U> (rhs);
  }

  // quotient
  template <typename T, typename U, uint N>
  auto operator / (Vector <T, N> lhs, Vector <U, N> rhs)
    -> Vector <decltype (lhs [0] / rhs [0]), N>
  {
    return transform (lhs, rhs, [] (T lh, U rh) { return lh / rh; });
  }

  template <typename T, typename U, uint N>
  auto operator / (Vector <T, N> lhs, U rhs)
    -> typename std::enable_if <
      std::is_arithmetic <U>::value &&
      std::is_integral   <U>,
      Vector <decltype (lhs [0] / rhs), N>
    >::type
  {
    return transform (lhs, [rhs] (T x) { return x / rhs; });
  }

  template <typename T, typename U, uint N>
  auto operator / (Vector <T, N> lhs, U rhs)
    -> typename std::enable_if <
      std::is_arithmetic     <U>::value &&
      std::is_floating_point <U>,
      Vector <decltype (lhs [0] * rhs), N>
    >::type
  {
    auto recip = U (1) / rhs;
    return lhs * recip;
  }

  template <typename T, typename U, uint N>
  auto operator /= (Vector <T, N> lhs, U&& rhs)
    -> Vector <T, N>&
  {
    return lhs = lhs / std::forward <U> (rhs);
  }

  // remainder
  template <typename T, typename U, uint N>
  auto operator % (Vector <T, N> lhs, Vector <U, N> rhs)
    -> typename std::enable_if <
      std::is_integral <U>::value &&
      std::is_integral <T>::value,
      Vector <decltype (lhs [0] % rhs [0]), N>
    >::type
  {
    return transform (lhs, rhs, [] (T lh, U rh) { return lh % rh; });
  }

  template <typename T, typename U, uint N>
  auto operator % (Vector <T, N> lhs, U rhs)
    -> typename std::enable_if <
      std::is_integral <U>::value &&
      std::is_integral <T>::value,
      Vector <decltype (lhs [0] % rhs), N>
    >::type
  {
    return transform (lhs, [rhs] (T x) { return x % rhs; });
  }

  template <typename T, typename U, uint N>
  auto operator %= (Vector <T, N> lhs, U&& rhs)
    -> Vector <T, N>&
  {
    return lhs = lhs % std::forward <U> (rhs);
  }

  // negation
  template <typename T, uint N>
  auto operator - (Vector <T, N> v)
    -> Vector <T, N>
  {
    return transform (v, [] (T x) { return -x; });
  }

  // dot product
  template <typename T, typename U, uint N>
  auto dot (Vector <T, N> lhs, Vector <U, N> rhs) ->
    decltype (lhs [0] * rhs [0])
  {
    return std::inner_product (
      lhs.components,
      lhs.components + N,
      rhs.components,
      decltype (lhs [0] * rhs [0]) ()
    );
  }

  // magnitude
  template <typename T, uint N>
  T magnitude (Vector <T, N> v)
  {
    return std::sqrt (dot (v, v));
  }

  template <typename T, uint N>
  T length (Vector <T, N> v)
  {
    return magnitude (v);
  }

  template <typename T, uint N>
  T abs (Vector <T, N> v)
  {
    return magnitude (v);
  }

  template <typename T, uint N>
  T norm (Vector <T, N> v)
  {
    return magnitude (v);
  }

  // unit
  template <typename T, uint N>
  auto unit (Vector <T, N> v)
    -> Vector <T, N>
  {
    return v / norm (v);
  }

  template <typename T, uint N>
  auto normalize (Vector <T, N>& v)
    -> Vector <T, N>&
  {
    return v = unit (v);
  }

  // lerp
  template <typename T, typename U, typename F, uint N>
  auto lerp (
    Vector <T, N> a,
    Vector <U, N> b,
    F t)
    -> Vector <decltype (a [0] * t + b [0] * t), N>
  {
    return transform (a, b, [t] (T lhs, U rhs) { return lerp (lhs, rhs, t); });
  }

  // floor
  template <typename T, uint N>
  auto floor (
    Vector <T, N> v,
    typename std::enable_if <
      std::is_floating_point <T>::value
    >::type* = 0)
    -> Vector <T, N>
  {
    return transform (v, [] (T x) { return std::floor (x); });
  }

  // ceil
  template <typename T, uint N>
  auto ceil (
    Vector <T, N> v,
    typename std::enable_if <
      std::is_floating_point <T>::value
    >::type* = 0)
    -> Vector <T, N>
  {
    return transform (v, [] (T x) { return std::ceil (x); });
  }

}
