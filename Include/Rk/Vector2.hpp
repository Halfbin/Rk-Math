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

#include <Rk/VectorBase.hpp>

namespace Rk
{
  template <typename T>
  class Vector <T, 2>
  {
  public:
    union {
      T components [2];
      struct { T x, y; };
      struct { T s, t; };
      struct { T r, g; };
    };

    Vector ()
    { }

    Vector (Nil) :
      components ()
    { }
  
    template <typename X, typename Y>
    Vector (X&& nx, Y&& ny) :
      x (std::forward <X> (nx)),
      y (std::forward <Y> (ny))
    { }
  
    template <typename U>
    Vector (const Vector <U, 2>& other) :
      x (other.x),
      y (other.y)
    { }
  
    T& operator [] (uint i)
    {
      return components [i];
    }

    const T& operator [] (uint i) const
    {
      return components [i];
    }

  };

  template <typename T>
  auto make_vector (T x, T y)
    -> Vector <T, 2>
  {
    return Vector <T, 2> (x, y);
  }

  template <typename T>
  auto make_ref_vector (T& x, T& y)
    -> Vector <T&, 2>
  {
    return Vector <T&, 2> (x, y);
  }

  template <typename T>
  auto make_ref_vector (const T& x, const T& y)
    -> Vector <const T&, 2>
  {
    return Vector <const T&, 2> (x, y);
  }

  template <typename Iter>
  auto make_ref_vector (Iter iter, uint stride = 1)
    -> Vector <decltype (*iter)&, 2>
  {
    return make_ref_vector (
      *(iter + 0 * stride),
      *(iter + 1 * stride)
    );
  }

  template <typename Iter, typename I>
  auto make_ref_vector_swizzle (Iter iter, Vector <I, 2> indices, uint stride = 1)
    -> Vector <decltype (*iter)&, 2>
  {
    return make_ref_vector (
      *(iter + indices [0] * stride),
      *(iter + indices [1] * stride)
    );
  }

  template <typename T, typename U>
  bool operator == (Vector <T, 2> lhs, Vector <U, 2> rhs)
  {
    return lhs.x == rhs.x && lhs.y == rhs.y;
  }

  template <typename T>
  auto swizzle (Vector <T, 2> value, uint sx, uint sy)
    -> Vector <T, 2>
  {
    return make_vector (value [sx], value [sy]);
  }

  template <typename T, typename I>
  auto swizzle (
    Vector <T, 2> value,
    Vector <I, 2> indices,
    typename std::enable_if <std::is_integral <I>::value>::type* = 0)
    -> Vector <T, 2>
  {
    return make_vector (value [indices.x], value [indices.y]);
  }
  
  // sum
  template <typename T, typename U>
  auto operator + (Vector <T, 2> lhs, Vector <U, 2> rhs)
    -> Vector <decltype (lhs [0] + rhs [0]), 2>
  {
    return make_vector (lhs.x + rhs.x, lhs.y + rhs.y);
  }

  // difference
  template <typename T, typename U>
  auto operator - (Vector <T, 2> lhs, Vector <U, 2> rhs)
    -> Vector <decltype (lhs [0] - rhs [0]), 2>
  {
    return make_vector (lhs.x - rhs.x, lhs.y - rhs.y);
  }

  // product
  template <typename T, typename U>
  auto operator * (Vector <T, 2> lhs, Vector <U, 2> rhs)
    -> Vector <decltype (lhs [0] * rhs [0]), 2>
  {
    return make_vector (lhs.x * rhs.x, lhs.y * rhs.y);
  }

  template <typename T, typename U>
  auto operator * (Vector <T, 2> lhs, U rhs)
    -> typename std::enable_if <
      std::is_arithmetic <U>::value,
      Vector <decltype (lhs [0] * rhs), 2>
    >::type
  {
    return make_vector (lhs.x * rhs, lhs.y * rhs);
  }

  // quotient
  template <typename T, typename U>
  auto operator / (Vector <T, 2> lhs, Vector <U, 2> rhs)
    -> Vector <decltype (lhs [0] / rhs [0]), 2>
  {
    return make_vector (lhs.x / rhs.x, lhs.y / rhs.y);
  }

  template <typename T, typename U>
  auto operator / (Vector <T, 2> lhs, U rhs)
    -> typename std::enable_if <
      std::is_arithmetic <U>::value &&
      std::is_integral   <U>,
      Vector <decltype (lhs [0] / rhs), 2>
    >::type
  {
    return make_vector (lhs.x / rhs, lhs.y / rhs);
  }

  // remainder
  template <typename T, typename U>
  auto operator % (Vector <T, 2> lhs, Vector <U, 2> rhs)
    -> typename std::enable_if <
      std::is_integral <U>::value &&
      std::is_integral <T>::value,
      Vector <decltype (lhs [0] % rhs [0]), 2>
    >::type
  {
    return make_vector (lhs.x % rhs.x, lhs.y % rhs.y);
  }

  template <typename T, typename U>
  auto operator % (Vector <T, 2> lhs, U rhs)
    -> typename std::enable_if <
      std::is_integral <U>::value &&
      std::is_integral <T>::value,
      Vector <decltype (lhs [0] % rhs), 2>
    >::type
  {
    return make_vector (lhs.x % rhs, lhs.y % rhs);
  }

  // negation
  template <typename T>
  auto operator - (Vector <T, 2> v)
    -> Vector <T, 2>
  {
    return make_vector (-v.x, -v.y);
  }

  // dot product
  template <typename T, typename U>
  auto dot (Vector <T, 2> lhs, Vector <U, 2> rhs) ->
    decltype (lhs [0] * rhs [0])
  {
    return lhs.x * rhs.x + lhs.y * rhs.y;
  }

  // lerp
  template <typename T, typename U, typename F>
  auto lerp (Vector <T, 2> a, Vector <U, 2> b, F t)
    -> Vector <decltype (a [0] * t + b [0] * t), 2>
  {
    F omt = F (1) - t;
    return make_vector (
      lerp (a.x, b.x, t),
      lerp (a.y, b.y, t)
    );
  }

  // floor
  template <typename T>
  auto floor (
    Vector <T, 2> v,
    typename std::enable_if <
      std::is_floating_point <T>::value
    >::type* = 0)
    -> Vector <T, 2>
  {
    return make_vector (
      std::floor (v.x),
      std::floor (v.y)
    );
  }

  // ceil
  template <typename T>
  auto ceil (
    Vector <T, 2> v,
    typename std::enable_if <
      std::is_floating_point <T>::value
    >::type* = 0)
    -> Vector <T, 2>
  {
    return make_vector (
      std::ceil (v.x),
      std::ceil (v.y)
    );
  }

  namespace VectorTypes
  {
    typedef Vector <float,  2> Vector2f, vec2f, v2f;
    typedef Vector <double, 2> Vector2d, vec2d, v2d;
    typedef Vector <int,    2> Vector2i, vec2i, v2i;

    typedef Vector <i8,  2> Vector2i8,  vec2i8,  v2i8;
    typedef Vector <i16, 2> Vector2i16, vec2i16, v2i16;
    typedef Vector <i32, 2> Vector2i32, vec2i32, v2i32;

    typedef Vector <u8,  2> Vector2u8,  vec2u8,  v2u8;
    typedef Vector <u16, 2> Vector2u16, vec2u16, v2u16;
    typedef Vector <u32, 2> Vector2u32, vec2u32, v2u32;

  }
  
  using namespace VectorTypes;

}

#ifndef RK_VECTOR_NO_GLOBAL
using namespace Rk::VectorTypes;
#endif
