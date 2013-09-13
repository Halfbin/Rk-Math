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

#include <Rk/Types.hpp>

namespace Rk
{
  template <typename T, uint M, uint N>
  class Matrix;

  namespace MatrixTypes
  {
    typedef Matrix <float,  2, 2> Matrix2f, mat2f;
    typedef Matrix <float,  3, 3> Matrix3f, mat3f;
    typedef Matrix <float,  4, 4> Matrix4f, mat4f;
    typedef Matrix <double, 2, 2> Matrix2d, mat2d;
    typedef Matrix <double, 3, 3> Matrix3d, mat3d;
    typedef Matrix <double, 4, 4> Matrix4d, mat4d;
  
    typedef Matrix <float,  2, 3> Matrix2x3f, mat2x3f;
    typedef Matrix <float,  2, 4> Matrix2x4f, mat2x4f;
    typedef Matrix <float,  3, 2> Matrix3x2f, mat3x2f;
    typedef Matrix <float,  3, 4> Matrix3x4f, mat3x4f;
    typedef Matrix <float,  4, 2> Matrix4x2f, mat4x2f;
    typedef Matrix <float,  4, 3> Matrix4x3f, mat4x3f;
  
    typedef Matrix <double, 2, 3> Matrix2x3d, mat2x3d;
    typedef Matrix <double, 2, 4> Matrix2x4d, mat2x4d;
    typedef Matrix <double, 3, 2> Matrix3x2d, mat3x2d;
    typedef Matrix <double, 3, 4> Matrix3x4d, mat3x4d;
    typedef Matrix <double, 4, 2> Matrix4x2d, mat4x2d;
    typedef Matrix <double, 4, 3> Matrix4x3d, mat4x3d;

  }

  using namespace MatrixTypes;

}

#ifndef RK_MATRIX_NO_GLOBAL
using namespace Rk::MatrixTypes;
#endif
