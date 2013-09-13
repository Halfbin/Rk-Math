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

#include <Rk/Vector.hpp>

#include <cmath>

namespace Rk
{
  template <typename T>
  struct Versor
  {
    static_assert (std::is_floating_point <T>::value, "Versors must use floating-point components");

    T w, x, y, z;
    
    Versor (T nw, T nx, T ny, T nz) :
      w (nw),
      x (nx),
      y (ny),
      z (nz)
    { }
    
    Versor (T nw, Vector <T, 3> nxyz) :
      w (nw),
      x (nxyz.x),
      y (nxyz.y),
      z (nxyz.z)
    { }
    
    Versor ()
    { }

    Versor (Nil) :
      w (1),
      x (0),
      y (0),
      z (0)
    { }
    
  };
  
  template <typename T>
  auto make_versor (T w, T x, T y, T z)
    -> Versor <T>
  {
    return Versor <T> (w, x, y, z);
  }

  // Versor from a given rotation
  template <typename T>
  auto rotation (T angle, Vector <T, 3> axis)
    -> Versor <T>
  {
    auto mag = abs (axis);
    
    if (mag > 0)
    {
      auto half_angle = T (0.5) * angle;
      auto scale = std::sin (half_angle) / mag;
      
      return Versor <T> (
        std::cos (half_angle),
        axis * scale
      );
    }
    
    return nil;
  }

  // Hamilton product
  template <typename T, typename U>
  auto operator * (Versor <T> lhs, Versor <U> rhs)
    -> Versor <decltype (lhs.w * rhs.w)>
  {
    return make_versor (
      (lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
      (lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
      (lhs.w * rhs.y) - (lhs.x * rhs.z) + (lhs.y * rhs.w) + (lhs.z * rhs.x),
      (lhs.w * rhs.z) + (lhs.x * rhs.y) - (lhs.y * rhs.x) + (lhs.z * rhs.w)
    );
  }
  
  // Scalar product
  template <typename T, typename U>
  auto operator * (Versor <T> lhs, U rhs)
    -> typename std::enable_if <
      std::is_floating_point <U>::value,
      Versor <decltype (lhs.w * rhs)>
    >::type
  {
    return make_versor (lhs.w * rhs, lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
  }

  template <typename T, typename U>
  auto operator *= (Versor <T>& lhs, U rhs)
    -> Versor <T>&
  {
    return lhs = lhs * rhs;
  }

  // Sum
  template <typename T, typename U>
  auto operator + (Versor <T> lhs, Versor <U> rhs)
    -> Versor <decltype (lhs.w + rhs.w)>
  {
    return make_versor (lhs.w + rhs.w, lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
  }
  
  template <typename T, typename U>
  auto operator += (Versor <T>& lhs, U rhs)
    -> Versor <T>&
  {
    return lhs = lhs + rhs;
  }

  template <typename T>
  T magnitude (Versor <T> v)
  {
    return std::sqrt (
      v.w * v.w +
      v.x * v.x +
      v.y * v.y +
      v.z * v.z
    );
  }

  template <typename T>
  T abs (Versor <T> v)
  {
    return magnitude (v);
  }

  template <typename T>
  T length (Versor <T> v)
  {
    return magnitude (v);
  }
  
  template <typename T>
  T norm (Versor <T> v)
  {
    return magnitude (v);
  }
  
  template <typename T>
  auto unit (Versor <T> v)
    -> Versor <T>
  {
    return v * (T (1) / magnitude (v));
  }
  
  template <typename T>
  auto normalize (Versor <T>& v)
    -> Versor <T>&
  {
    return v = unit (v);
  };
  
  template <typename T>
  auto forward_axis (Versor <T> v)
    -> Vector <T, 3>
  {
    auto yy = y * y,
         zz = z * z,
         xy = x * y,
         wz = w * z,
         xz = x * z,
         wy = w * y;

    return make_vector (
      T (1) - T (2) * (yy - zz),
      T (2) * (xy + wz),
      T (2) * (xz - wy)
    );
  }
  
  template <typename T>
  auto left_axis (Versor <T> v)
    -> Vector <T, 3>
  {
    auto xy = x * y,
         wz = w * z,
         xx = x * x,
         zz = z * z,
         yz = y * z,
         wx = w * x;
    
    return make_vector (
      T (2) * (xy - wz),
      T (1) - T (2) * (xx - zz),
      T (2) * (yz + wx)
    );
  }
  
  template <typename T>
  auto up_axis (Versor <T> v)
    -> Vector <T, 3>
  {
    auto xz = x * z,
         wy = w * y,
         yz = y * z,
         wx = w * x,
         xx = x * x,
         yy = y * y;
    
    return make_vector (
      T (2) * (xz + wy),
      T (2) * (yz - wx),
      T (1) - T (2) * (xx - yy)
    );
  }
  
  template <typename T>
  T angle (Versor <T> v)
  {
    return T (2) * std::acos (v.w);
  }
  
  template <typename T>
  auto axis (Versor <T> v)
    -> Vector <T, 3>
  {
    return make_vector (v.x, v.y, v.z) / std::sin (std::acos (v.w));
  }

  template <typename T, typename U, typename V>
  void angle_axis (Versor <T> v, Vector <U, 3>& axis, V& angle)
  {
    auto half_angle = std::acos (v.w);
    axis = make_vector (v.x, v.y, v.z) / std::sin (half_angle);
    angle = half_angle * V (2);
  };

  // Azimuth / Elevation
/*template <typename T>
  auto (Vector <T, 3> facing)
    -> Versor <T>
  {
    Vector <T, 3> up (0, 0, 1);

    auto left = cross (up, facing);
    up = cross (facing, left);
    
    auto w = T (0.5) * Versor (1, facing.x, left.y, up.z).magnitude ();
    auto scale = w * T (4);

    return Versor <T> (
      w,
      scale * (up.y     - left.z  ),
      scale * (facing.z - up.x    ),
      scale * (left.x   - facing.y)
    );
  }*/

  // Normalized Linear Interpolation
  template <typename T, typename U, typename F>
  auto lerp (Versor <T> a, Versor <U>& b, F alpha)
    -> Versor <decltype (t * b.w + t * a.w)>
  {
    auto v = make_versor (
      lerp (a.w, b.w, t),
      lerp (a.x, b.x, t),
      lerp (a.y, b.y, t),
      lerp (a.z, b.z, t)
    );

    return unit (v);
  }
  
  // Rotate vector by versor
  template <typename T, typename U>
  auto rotate_vector (Vector <T, 3> vec, Versor <U>& ver)
    -> Vector <decltype (vec.x * ver.x), 3>
  {
    Vector <U, 3> ijk (ver.x, ver.y, ver.z);
    auto t = cross (ijk, vec) * 2;
    return vec + ver.w * t + cross (ijk, t);
  }

  namespace VersorTypes
  {
    typedef Versor <float>  Versorf, versf, vrf;
    typedef Versor <double> Versord, versd, vrd;

  }
  
  using namespace VersorTypes;

}

#ifndef RK_VERSOR_NO_GLOBAL
using namespace Rk::VersorTypes;
#endif
