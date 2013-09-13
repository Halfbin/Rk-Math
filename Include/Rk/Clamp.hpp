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

namespace Rk
{
  template <typename T, typename U, typename V>
  T clamp (T&& val, U&& lower, V&& upper)
  {
    if      (val < lower) return std::forward <T> (lower);
    else if (val > upper) return std::forward <U> (upper);
    else                  return std::forward <V> (val);
  }

}
