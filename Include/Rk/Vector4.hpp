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
  class Vector <T, 4>
  {
  public:
    union {
      T components [4];
      struct { T x, y, z, w; };
      struct { T s, t, p, q; };
      struct { T r, g, b, a; };
    };

    Vector ()
    { }

    Vector (Nil) :
      components ()
    { }
    
    // x y z w
    template <typename X, typename Y, typename Z, typename W>
    Vector (X&& nx, Y&& ny, Z&& nz, W&& nw) :
      x (std::forward <X> (nx)),
      y (std::forward <Y> (ny)),
      z (std::forward <Z> (nz)),
      w (std::forward <W> (nw))
    { }
    
    // xyzw
    template <typename U>
    Vector (const Vector <U, 4>& other) :
      x (other.x),
      y (other.y),
      z (other.z),
      w (other.w)
    { }
    
    // xy z w
    template <typename XY, typename Z, typename W>
    Vector (Vector <XY, 2> nxy, Z&& nz, W&& nw) :
      x (nxy.x),
      y (nxy.y),
      z (std::forward <Z> (nz)),
      w (std::forward <W> (nw))
    { }
    
    // x yz w
    template <typename X, typename YZ, typename W>
    Vector (X&& nx, Vector <YZ, 2> nyz, W&& nw) :
      x (std::forward <X> (nx)),
      y (nyz.x),
      z (nyz.y),
      w (std::forward <W> (nw))
    { }
    
    // x y zw
    template <typename X, typename Y, typename ZW>
    Vector (X&& nx, Y&& ny, Vector <ZW, 2> nzw) :
      x (std::forward <X> (nx)),
      y (std::forward <Y> (ny)),
      z (nzw.x),
      w (nzw.y)
    { }
    
    // xy zw
    template <typename XY, typename ZW>
    Vector (Vector <XY, 2> nxy, Vector <ZW, 2> nzw) :
      x (nxy.x),
      y (nxy.y),
      z (nzw.x),
      w (nzw.y)
    { }
    
    // xyz w
    template <typename XYZ, typename W>
    Vector (Vector <XYZ, 3> nxyz, W&& nw) :
      x (nxyz.x),
      y (nxyz.y),
      z (nxyz.z),
      w (std::forward <W> (nw))
    { }
    
    // x yzw
    template <typename X, typename YZW>
    Vector (X&& nx, Vector <YZW, 3> nyzw) :
      x (std::forward <X> (nx)),
      y (nyzw.x),
      z (nyzw.y),
      w (nyzw.z)
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
  auto make_vector (T x, T y, T z, T w)
    -> Vector <T, 4>
  {
    return Vector <T, 4> (x, y, z, w);
  }

  template <typename T>
  auto make_ref_vector (T& x, T& y, T& z, T& w)
    -> Vector <T&, 4>
  {
    return Vector <T&, 4> (x, y, z, w);
  }

  template <typename T>
  auto make_ref_vector (const T& x, const T& y, const T& z, const T& w)
    -> Vector <const T&, 4>
  {
    return Vector <const T&, 4> (x, y, z, w);
  }
  
  template <typename Iter>
  auto make_ref_vector (Iter iter, uint stride = 1)
    -> Vector <decltype (*iter)&, 4>
  {
    return make_ref_vector (
      *(iter + 0 * stride),
      *(iter + 1 * stride),
      *(iter + 2 * stride),
      *(iter + 3 * stride)
    );
  }

  template <typename Iter, typename I>
  auto make_ref_vector_swizzle (Iter iter, Vector <I, 4> indices, uint stride = 1)
    -> Vector <decltype (*iter)&, 4>
  {
    return make_ref_vector (
      *(iter + indices [0] * stride),
      *(iter + indices [1] * stride),
      *(iter + indices [2] * stride),
      *(iter + indices [3] * stride)
    );
  }

  template <typename T, typename U>
  bool operator == (Vector <T, 4> lhs, Vector <U, 4> rhs)
  {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.w == rhs.w;
  }

  template <typename T>
  auto swizzle (Vector <T, 4> value, uint sx, uint sy, uint sz, uint sw)
    -> Vector <T, 4>
  {
    return Vector <T, 4> (value [sx], value [sy], value [sz], value [sw]);
  }

  template <typename T, typename I>
  auto swizzle (
    Vector <T, 4> value,
    Vector <I, 4> indices,
    typename std::enable_if <std::is_integral <I>::value>::type* = 0)
    -> Vector <T, 4>
  {
    return Vector <T, 4> (value [indices.x], value [indices.y], value [indices.z], value [indices.w]);
  }

  // sum
  template <typename T, typename U>
  auto operator + (Vector <T, 4> lhs, Vector <U, 4> rhs)
    -> Vector <decltype (lhs [0] + rhs [0]), 4>
  {
    return make_vector (lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w);
  }

  // difference
  template <typename T, typename U>
  auto operator - (Vector <T, 4> lhs, Vector <U, 4> rhs)
    -> Vector <decltype (lhs [0] - rhs [0]), 4>
  {
    return make_vector (lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z, lhs.w - rhs.w);
  }

  // product
  template <typename T, typename U>
  auto operator * (Vector <T, 4> lhs, Vector <U, 4> rhs)
    -> Vector <decltype (lhs [0] * rhs [0]), 4>
  {
    return make_vector (lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z, lhs.w * rhs.w);
  }

  template <typename T, typename U>
  auto operator * (Vector <T, 4> lhs, U rhs)
    -> typename std::enable_if <
      std::is_arithmetic <U>::value,
      Vector <decltype (lhs [0] * rhs), 4>
    >::type
  {
    return make_vector (lhs.x * rhs, lhs.y * rhs, lhs.z * rhs, lhs.w * rhs);
  }

  // quotient
  template <typename T, typename U>
  auto operator / (Vector <T, 4> lhs, Vector <U, 4> rhs)
    -> Vector <decltype (lhs [0] / rhs [0]), 4>
  {
    return make_vector (lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z, lhs.w / rhs.w);
  }

  template <typename T, typename U>
  auto operator / (Vector <T, 4> lhs, U rhs)
    -> typename std::enable_if <
      std::is_arithmetic <U>::value &&
      std::is_integral   <U>,
      Vector <decltype (lhs [0] / rhs), 4>
    >::type
  {
    return make_vector (lhs.x / rhs, lhs.y / rhs, lhs.z / rhs, lhs.w / rhs);
  }

  // remainder
  template <typename T, typename U>
  auto operator % (Vector <T, 4> lhs, Vector <U, 4> rhs)
    -> typename std::enable_if <
      std::is_integral <U>::value &&
      std::is_integral <T>::value,
      Vector <decltype (lhs [0] % rhs [0]), 4>
    >::type
  {
    return make_vector (lhs.x % rhs.x, lhs.y % rhs.y, lhs.z % rhs.z, lhs.w % rhs.w);
  }

  template <typename T, typename U>
  auto operator % (Vector <T, 4> lhs, U rhs)
    -> typename std::enable_if <
      std::is_integral <U>::value &&
      std::is_integral <T>::value,
      Vector <decltype (lhs [0] % rhs), 4>
    >::type
  {
    return make_vector (lhs.x % rhs, lhs.y % rhs, lhs.z % rhs, lhs.w % rhs);
  }

  // negation
  template <typename T>
  auto operator - (Vector <T, 4> v)
    -> Vector <T, 4>
  {
    return make_vector (-v.x, -v.y, -v.z, -v.w);
  }

  // dot product
  template <typename T, typename U>
  auto dot (Vector <T, 4> lhs, Vector <U, 4> rhs) ->
    decltype (lhs [0] * rhs [0])
  {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;
  }

  // lerp
  template <typename T, typename U, typename F>
  auto lerp (
    Vector <T, 4> a,
    Vector <U, 4> b,
    F t)
    -> Vector <decltype (a [0] * t + b [0] * t), 4>
  {
    return make_vector (
      lerp (a.x, b.x, t),
      lerp (a.y, b.y, t),
      lerp (a.z, b.z, t),
      lerp (a.w, b.w, t)
    );
  }

  // floor
  template <typename T>
  auto floor (
    Vector <T, 4> v,
    typename std::enable_if <
      std::is_floating_point <T>::value
    >::type* = 0)
    -> Vector <T, 4>
  {
    return make_vector (
      std::floor (v.x),
      std::floor (v.y),
      std::floor (v.z),
      std::floor (v.w)
    );
  }

  // ceil
  template <typename T>
  auto ceil (
    Vector <T, 4> v,
    typename std::enable_if <
      std::is_floating_point <T>::value
    >::type* = 0)
    -> Vector <T, 4>
  {
    return make_vector (
      std::ceil (v.x),
      std::ceil (v.y),
      std::ceil (v.z),
      std::ceil (v.w)
    );
  }

  namespace VectorTypes
  {
    typedef Vector <float,  4> Vector4f, vec4f, v4f;
    typedef Vector <double, 4> Vector4d, vec4d, v4d;
    typedef Vector <int,    4> Vector4i, vec4i, v4i;

    typedef Vector <i8,  4> Vector4i8,  vec4i8,  v4i8;
    typedef Vector <i16, 4> Vector4i16, vec4i16, v4i16;
    typedef Vector <i32, 4> Vector4i32, vec4i32, v4i32;

    typedef Vector <u8,  4> Vector4u8,  vec4u8,  v4u8;
    typedef Vector <u16, 4> Vector4u16, vec4u16, v4u16;
    typedef Vector <u32, 4> Vector4u32, vec4u32, v4u32;

  }
  
  using namespace VectorTypes;

}

#ifndef RK_VECTOR_NO_GLOBAL
using namespace Rk::VectorTypes;
#endif
