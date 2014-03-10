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
#include <complex>

namespace Rk
{
  template <typename at, typename bt, typename tt, typename = typename std::enable_if <
    std::is_scalar <at>::value &&
    std::is_scalar <bt>::value &&
    std::is_floating_point <tt>::value >::type>
  auto lerp (at a, bt b, tt t)
  {
    return a + (b - a) * t;
  }

  template <typename at, typename bt, typename tt, typename = typename std::enable_if <
    std::is_floating_point <tt>::value >::type>
  auto lerp (std::complex <at> a, std::complex <bt> b, tt t)
  {
    return a * (1 - t) + b * t;
  }

  template <typename ct>
  auto unit (std::complex <ct> z)
  {
    auto len = abs (z);
    if (len > 0)
      z /= len
    return z;
  }

}
