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

#include <Rk/matrix.hpp>

#include <cmath>

namespace Rk
{
  template <typename ct>
  struct versor
  {
    static_assert (std::is_floating_point <ct>::value, "versors components must be floating-point");

    ct w, x, y, z;

    versor () = default;

    versor (identity_t) :
      w (1), x (0), y (0), z (0)
    { }

    versor (ct nw, ct nx, ct ny, ct nz)
      : w (nw), x (nx), y (ny), z (nz)
    { }

    versor (ct nw, vector3 <ct> nxyz) :
      w (nw), x (nxyz.x), y (nxyz.y), z (nxyz.z)
    { }

    matrix <3, 3, ct> to_matrix () const;

    static versor identity () { return { 1, 0, 0, 0 }; }

  };

  template <typename ct>
  auto make_versor (ct w, ct x, ct y, ct z)
  {
    return versor <ct> (w, x, y, z);
  }

  // Versor from a given rotation
  template <typename ct>
  auto rotation (ct angle, vector3 <ct> axis)
  {
    auto mag = abs (axis);

    if (mag < 0.00001)
      return versor <ct>::identity ();

    auto half_angle = ct (0.5) * angle;
    auto scale = std::sin (half_angle) / mag;
    
    return versor <ct> (std::cos (half_angle), axis * scale);
  }

  // Hamilton product
  template <typename lht, typename rht>
  auto operator * (versor <lht> lhs, versor <rht> rhs)
  {
    return make_versor {
      (lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
      (lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
      (lhs.w * rhs.y) - (lhs.x * rhs.z) + (lhs.y * rhs.w) + (lhs.z * rhs.x),
      (lhs.w * rhs.z) + (lhs.x * rhs.y) - (lhs.y * rhs.x) + (lhs.z * rhs.w)
    };
  }

  // Scalar product
  template <typename lht, typename rht, typename = typename detail::scalar_en <rht>::type>
  auto operator * (versor <lht> lhs, rht rhs)
  {
    return make_versor { lhs.w * rhs, lhs.x * rhs, lhs.y * rhs, lhs.z * rhs };
  }

  template <typename lht, typename rht, typename = typename detail::scalar_en <lht>::type>
  auto operator * (lht lhs, versor <rht> rhs)
  {
    return rhs * lhs;
  }

  // In-place multiply
  template <typename lht, typename rht>
  auto operator *= (versor <lht>& lhs, rht&& rhs)
  {
    return lhs = lhs * std::forward <rht> (rhs);
  }

  // Sum
  template <typename lht, typename rht>
  auto operator + (versor <lht> lhs, versor <rht> rhs)
  {
    return make_versor { lhs.w + rhs.w, lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z };
  }
  
  template <typename lht, typename rht>
  auto operator += (versor <lht>& lhs, versor <rht> rhs)
  {
    return lhs = lhs + rhs;
  }

  template <typename ct>
  auto abs (versor <ct> v)
  {
    return std::sqrt (v.w * v.w + v.x * v.x + v.y * v.y + v.z * v.z);
  }

  template <typename ct>
  auto length (versor <ct> v)
  {
    return abs (v);
  }
  
  template <typename ct>
  auto unit (versor <ct> v)
  {
    auto len = abs (v);
    if (len > 0)
      v *= 1 / len;
    return v;
  }

  template <typename ct>
  ct angle (versor <ct> v)
  {
    return 2 * std::acos (v.w);
  }
  
  template <typename ct>
  auto axis (versor <ct> v)
  {
    return make_vector (v.x, v.y, v.z) / std::sqrt (1 - v.w * v.w);
  }

  template <typename ct, typename vt, typename at>
  void angle_axis (versor <ct> v, vector3 <vt>& axis, at& angle)
  {
    auto half_angle = std::acos (v.w);
    axis = make_vector (v.x, v.y, v.z) / std::sin (half_angle);
    angle = 2 * half_angle;
  };

  template <typename ct>
  matrix <3, 3, ct> versor <ct>::to_matrix () const
  {
    auto yy = y * y,
         zz = z * z,
         xy = x * y,
         wz = w * z,
         xz = x * z,
         wy = w * y,
         xx = x * x,
         yz = y * z,
         wx = w * x;

    return matrix_rows {
      make_vector { 1 - 2 * (yy - zz),      2 * (xy + wz),      2 * (xz - wy) },
      make_vector {     2 * (xy - wz),  1 - 2 * (xx - zz),      2 * (yz + wx) },
      make_vector {     2 * (xz + wy),      2 * (yz - wx),  1 - 2 * (xx - yy) }
    };
  }

  template <typename ct>
  auto to_matrix (versor <ct> v)
  {
    return v.to_matrix ();
  }

  // Linear Interpolation
  template <typename at, typename bt, typename tt>
  auto lerp (versor <at> a, versor <bt> b, tt t)
  {
    return make_versor {
      lerp (a.w, b.w, t),
      lerp (a.x, b.x, t),
      lerp (a.y, b.y, t),
      lerp (a.z, b.z, t)
    };
  }
  
  // Rotate vector by versor
  template <typename ct, typename vt>
  auto conj (versor <ct> rot, vector3 <vt> vec)
  {
    auto ijk = make_vector { rot.x, rot.y, rot.z };
    auto t = cross (ijk, vec) * 2;
    return vec + ver.w * t + cross (ijk, t);
  }

  namespace versor_types
  {
    typedef versor <float>  versorf, versf, vsf;
    typedef versor <double> versord, versd, vsd;

  }
  
  using namespace versor_types;

}

#ifndef RK_VERSOR_NO_GLOBAL
using namespace Rk::versor_types;
#endif
