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
  class Vector <T, 3>
  {
  public:
    union {
      T components [3];
      struct { T x, y, z; };
      struct { T s, t, p; };
      struct { T r, g, b; };
    };

    Vector ()
    { }

    Vector (Nil) :
      components ()
    { }
    
    // x y z
    template <typename X, typename Y, typename Z>
    Vector (X&& nx, Y&& ny, Z&& nz) :
      x (std::forward <X> (nx)),
      y (std::forward <Y> (ny)),
      z (std::forward <Z> (nz))
    { }
    
    // xyz
    template <typename U>
    Vector (const Vector <U, 3>& other) :
      x (other.x),
      y (other.y),
      z (other.z)
    { }
    
    // xy z
    template <typename XY, typename Z>
    Vector (Vector <XY, 2> nxy, Z&& nz) :
      x (nxy.x),
      y (nxy.y),
      z (std::forward <Z> (nz))
    { }
    
    // x yz
    template <typename X, typename YZ>
    Vector (X&& nx, Vector <YZ, 2> nyz) :
      x (std::forward <X> (nx)),
      y (nyz.x),
      z (nyz.y)
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
  auto make_vector (T x, T y, T z)
    -> Vector <T, 3>
  {
    return Vector <T, 3> (x, y, z);
  }

  template <typename T>
  auto make_ref_vector (T& x, T& y, T& z)
    -> Vector <T&, 3>
  {
    return Vector <T&, 3> (x, y, z);
  }

  template <typename T>
  auto make_ref_vector (const T& x, const T& y, const T& z)
    -> Vector <const T&, 3>
  {
    return Vector <const T&, 3> (x, y, z);
  }
  
  template <typename Iter>
  auto make_ref_vector (Iter iter, uint stride = 1)
    -> Vector <decltype (*iter)&, 3>
  {
    return make_ref_vector (
      *(iter + 0 * stride),
      *(iter + 1 * stride),
      *(iter + 2 * stride)
    );
  }

  template <typename Iter, typename I>
  auto make_ref_vector_swizzle (Iter iter, Vector <I, 3> indices, uint stride = 1)
    -> Vector <decltype (*iter)&, 3>
  {
    return make_ref_vector (
      *(iter + indices [0] * stride),
      *(iter + indices [1] * stride),
      *(iter + indices [2] * stride)
    );
  }

  template <typename T, typename U>
  bool operator == (Vector <T, 3> lhs, Vector <U, 3> rhs)
  {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
  }

  template <typename T>
  auto swizzle (Vector <T, 3> value, uint sx, uint sy, uint sz)
    -> Vector <T, 3>
  {
    return Vector <T, 3> (value [sx], value [sy], value [sz]);
  }

  template <typename T, typename I>
  auto swizzle (
    Vector <T, 3> value,
    Vector <I, 3> indices,
    typename std::enable_if <std::is_integral <I>::value>::type* = 0)
    -> Vector <T, 3>
  {
    return Vector <T, 3> (value [indices.x], value [indices.y], value [indices.z]);
  }

  // sum
  template <typename T, typename U>
  auto operator + (Vector <T, 3> lhs, Vector <U, 3> rhs)
    -> Vector <decltype (lhs [0] + rhs [0]), 3>
  {
    return make_vector (lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
  }

  // difference
  template <typename T, typename U>
  auto operator - (Vector <T, 3> lhs, Vector <U, 3> rhs)
    -> Vector <decltype (lhs [0] - rhs [0]), 3>
  {
    return make_vector (lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
  }

  // product
  template <typename T, typename U>
  auto operator * (Vector <T, 3> lhs, Vector <U, 3> rhs)
    -> Vector <decltype (lhs [0] * rhs [0]), 3>
  {
    return make_vector (lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z);
  }

  template <typename T, typename U>
  auto operator * (Vector <T, 3> lhs, U rhs)
    -> typename std::enable_if <
      std::is_arithmetic <U>::value,
      Vector <decltype (lhs [0] * rhs), 3>
    >::type
  {
    return make_vector (lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
  }

  // quotient
  template <typename T, typename U>
  auto operator / (Vector <T, 3> lhs, Vector <U, 3> rhs)
    -> Vector <decltype (lhs [0] / rhs [0]), 3>
  {
    return make_vector (lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z);
  }

  template <typename T, typename U>
  auto operator / (Vector <T, 3> lhs, U rhs)
    -> typename std::enable_if <
      std::is_arithmetic <U>::value &&
      std::is_integral   <U>,
      Vector <decltype (lhs [0] / rhs), 3>
    >::type
  {
    return make_vector (lhs.x / rhs, lhs.y / rhs, lhs.z / rhs);
  }

  // remainder
  template <typename T, typename U>
  auto operator % (Vector <T, 3> lhs, Vector <U, 3> rhs)
    -> typename std::enable_if <
      std::is_integral <U>::value &&
      std::is_integral <T>::value,
      Vector <decltype (lhs [0] % rhs [0]), 3>
    >::type
  {
    return make_vector (lhs.x % rhs.x, lhs.y % rhs.y, lhs.z % rhs.z);
  }

  template <typename T, typename U>
  auto operator % (Vector <T, 3> lhs, U rhs)
    -> typename std::enable_if <
      std::is_integral <U>::value &&
      std::is_integral <T>::value,
      Vector <decltype (lhs [0] % rhs), 3>
    >::type
  {
    return make_vector (lhs.x % rhs, lhs.y % rhs, lhs.z % rhs);
  }

  // negation
  template <typename T>
  auto operator - (Vector <T, 3> v)
    -> Vector <T, 3>
  {
    return make_vector (-v.x, -v.y, -v.z);
  }

  // dot product
  template <typename T, typename U>
  auto dot (Vector <T, 3> lhs, Vector <U, 3> rhs) ->
    decltype (lhs [0] * rhs [0])
  {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
  }

  // lerp
  template <typename T, typename U, typename F>
  auto lerp (
    Vector <T, 3> a,
    Vector <U, 3> b,
    F t)
    -> Vector <decltype (a [0] * t + b [0] * t), 3>
  {
    return make_vector (
      lerp (a.x, b.x, t),
      lerp (a.y, b.y, t),
      lerp (a.z, b.z, t)
    );
  }

  // floor
  template <typename T>
  auto floor (
    Vector <T, 3> v,
    typename std::enable_if <
      std::is_floating_point <T>::value
    >::type* = 0)
    -> Vector <T, 3>
  {
    return make_vector (
      std::floor (v.x),
      std::floor (v.y),
      std::floor (v.z)
    );
  }

  // ceil
  template <typename T>
  auto ceil (
    Vector <T, 3> v,
    typename std::enable_if <
      std::is_floating_point <T>::value
    >::type* = 0)
    -> Vector <T, 3>
  {
    return make_vector (
      std::ceil (v.x),
      std::ceil (v.y),
      std::ceil (v.z)
    );
  }

  // cross product
  template <typename T, typename U>
  auto cross (Vector <T, 3> lhs, Vector <U, 3> rhs)
    -> Vector <decltype (lhs.x * rhs.x), 3>
  {
    return make_vector (
      lhs.y * rhs.z - lhs.z * rhs.y,
      lhs.z * rhs.x - lhs.x * rhs.z,
      lhs.x * rhs.y - lhs.y * rhs.x
    );
  }

  namespace VectorTypes
  {
    typedef Vector <float,  3> Vector3f, vec3f, v3f;
    typedef Vector <double, 3> Vector3d, vec3d, v3d;
    typedef Vector <int,    3> Vector3i, vec3i, v3i;

    typedef Vector <i8,  3> Vector3i8,  vec3i8,  v3i8;
    typedef Vector <i16, 3> Vector3i16, vec3i16, v3i16;
    typedef Vector <i32, 3> Vector3i32, vec3i32, v3i32;

    typedef Vector <u8,  3> Vector3u8,  vec3u8,  v3u8;
    typedef Vector <u16, 3> Vector3u16, vec3u16, v3u16;
    typedef Vector <u32, 3> Vector3u32, vec3u32, v3u32;

  }
  
  using namespace VectorTypes;

}

#ifndef RK_VECTOR_NO_GLOBAL
using namespace Rk::VectorTypes;
#endif
