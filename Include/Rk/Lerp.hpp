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

#include <type_traits>

namespace Rk
{
  template <typename F>
  auto lerp (F a, F b, F t)
    -> typename std::enable_if <
      std::is_floating_point <F>::value, F
    >::type
  {
    return a + (b - a) * t;
  }

  template <typename T, typename F>
  T lerp (const T& a, const T& b, F t)
  {
    return a * (F (1) - t) + b * t;
  }

  // Normalized Linear Interpolation
  template <typename T, typename F>
  T nlerp (const T& a, const T& b, F t)
  {
    return abs (t * b + (F (1) - t) * a);
  }

}
