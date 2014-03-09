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
#include <Rk/versor.hpp>

namespace Rk
{
  template <typename ct>
  auto translation (vector3 <ct> v)
  {
    auto result = matrix <4, 4, ct>::identity ();
    result.set_column (3, compose_vector { v, 1 });
    return result;
  }

  template <typename ct>
  auto affine (vector3 <ct> trn, versor <ct> rot)
  {
    auto rmat = to_matrix (rot);

    return matrix_rows {
      compose_vector { rmat.row (0), trn.x },
      compose_vector { rmat.row (1), trn.y },
      compose_vector { rmat.row (2), trn.z },
      make_vector    {   0, 0, 0,      1   }
    };
  }

  template <typename ct>
  auto world_to_eye (vector3 <ct> pos, versor <ct> ori)
  {
    auto rm = to_matrix (ori);
    auto np = -pos;

    auto i = -rm.row (1),
         j =  rm.row (2),
         k = -rm.row (0);

    return matrix_rows {
      compose_vector {   i,   dot (i, np) },
      compose_vector {   j,   dot (j, np) },
      compose_vector {   k,   dot (k, np) },
      make_vector    { 0,0,0,      1      }
    };
  }

  template <typename t>
  auto eye_to_clip (t fov, t asp, t zn, t zf)
  {
    auto pi_over_360 = 3.14159265358979323846 / 360;

    t h   = 1 / std::tan (fov * pi_over_360),
      w   = h / asp,
      nrd = 1 / (zf - zn),
      q   =  (zf + zn)  * nrd,
      qn  = 2 * zn * zf * nrd;

    return matrix_rows {
      make_vector { w, 0,  0,  0 },
      make_vector { 0, h,  0,  0 },
      make_vector { 0, 0,  q, qn },
      make_vector { 0, 0, -1,  0 }
    };
  }

  template <typename t>
  auto ui_to_clip (t w, t h)
  {
    return matrix_rows {
      make_vector { 2 / w,    0,   -1 },
      make_vector {   0,   -2 / h,  1 },
      make_vector {   0,      0,    1 }
    };
  }

}
