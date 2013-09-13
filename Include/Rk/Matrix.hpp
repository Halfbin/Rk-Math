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

#include <Rk/MatrixForward.hpp>

#include <Rk/Versor.hpp>
#include <Rk/Vector.hpp>

namespace Rk
{
  template <typename T, uint M, uint N>
  class Matrix
  {
    T data [M][N]; // Row Major - M rows, N columns
    
  public:
    typedef T              Element;
    typedef Vector <T&, N> Row;
    typedef Vector <T&, M> Column;
    typedef Vector <T,  N> RowConst;
    typedef Vector <T,  M> ColumnConst;
    
    Matrix ()
    { }

    Matrix (Nil) :
      data ()
    { }
    
    template <typename U>
    Matrix (const Matrix <U, M, N>& other)
    {
      for (uint i = 0; i < M; i++)
      for (uint j = 0; j < N; j++)
        data [i][j] = T (other.data [i][j]);
    }
    
    Matrix (const T (&array) [M][N])
    {
      for (uint i = 0; i < M; i++)
      for (uint j = 0; j < N; j++)
        data [i][j] = array [i][j];
    }

    static Matrix identity ()
    {
      return nil;
    }
    
    Row row (uint i)
    {
      return make_ref_vector <N> ((T*) &data [i][0], 1);
    }
    
    Column column (uint j)
    {
      return make_ref_vector <M> ((T*) &data [0][j], N);
    }
    
    RowConst row (uint i) const
    {
      return make_vector <N> ((const T*) &data [i][0], 1);
    }
    
    ColumnConst column (uint j) const
    {
      return make_vector <M> ((const T*) &data [0][j], N);
    }
    
    T& element (uint i, uint j)
    {
      return data [i][j];
    }
    
    T element (uint i, uint j) const
    {
      return data [i][j];
    }
    
    T& operator () (uint i, uint j)
    {
      return element (i, j);
    }

    T operator () (uint i, uint j) const
    {
      return element (i, j);
    }
    
  };

  template <
    typename TL, uint ML,
    typename TR, uint NR,
    uint NLMR>
  auto operator * (
    const Matrix <TL, ML,   NLMR>& lhs,
    const Matrix <TR, NLMR, NR  >& rhs)
    -> Matrix <
      decltype (TL () * TR ()),
      ML, NR
    >
  {
    Matrix <decltype (TL () * TR ()), ML, NR> result;
    for (uint i = 0; i != ML; i++)
    for (uint j = 0; j != NR; j++)
      result.element (i, j) = dot (lhs.row (i), rhs.column (j));
    return result;
  }
  
  template <typename T, uint M, typename U>
  auto operator * (
    const Matrix <T, M, 4> lhs,
    Vector <U, 4>          rhs)
    -> Vector <
      decltype (T () * U ()),
      4
    >
  {
    return make_vector (
      dot (lhs.row (0), rhs),
      dot (lhs.row (1), rhs),
      dot (lhs.row (2), rhs),
      dot (lhs.row (3), rhs)
    );
  }

  template <typename T>
  auto translation (Vector <T, 3> xln)
    -> Matrix <T, 4, 4>
  {
    Matrix <T, 4, 4> result = nil;
    result.column (3) = Vector <T, 4> (xln, 1);
    return result;
  }
  
  template <typename T>
  auto affine (Vector <T, 3> xln, Versor <T> rot)
    -> Matrix <T, 4, 4>
  {
    auto fwd  = forward_axis (rot),
         left = left_axis    (rot),
         up   = up_axis      (rot);
    
    Matrix <T, 4, 4> result;
    result.row (0) = Vector <T, 4> (fwd,  xln.x);
    result.row (1) = Vector <T, 4> (left, xln.y);
    result.row (2) = Vector <T, 4> (up,   xln.z);
    result.row (3) = Vector <T, 4> (0, 0, 0, 1 );
    return result;
  }
  
  template <typename T>
  auto world_to_eye (Vector <T, 3> xln, Versor <T> rot)
    -> Matrix <T, 4, 4>
  {
    auto fwd  = forward_axis (rot),
         left = left_axis    (rot),
         up   = up_axis      (rot);
    
    Matrix <T, 4, 4> result;
    result.row (0) = Vector <T, 4> (-left,   0);
    result.row (1) = Vector <T, 4> (up,      0);
    result.row (2) = Vector <T, 4> (-fwd,    0);
    result.row (3) = Vector <T, 4> (0, 0, 0, 1);
    return result * translation (-xln);
  }
  
  template <typename T>
  auto eye_to_clip (T fov, T aspect, T z_near, T z_far)
    -> Matrix <T, 4, 4>
  {
    T two_z_near = T (2) * z_near,
      height = two_z_near * std::tan (fov * T (3.14159265358979323846) / T (360)),
      width = height * aspect,
      recip_depth = T (1) / (z_far - z_near),
      q = -(z_far + z_near) * recip_depth,
      qn = -two_z_near * z_far * recip_depth,
      w = two_z_near / width,
      h = two_z_near / height;
    
    Matrix <T, 4, 4> result;
    result.row (0) = Vector <T, 4> (w, 0,  0,  0);
    result.row (1) = Vector <T, 4> (0, h,  0,  0);
    result.row (2) = Vector <T, 4> (0, 0,  q, qn);
    result.row (3) = Vector <T, 4> (0, 0, -1,  0);
    return result;
  }
  
  template <typename T>
  auto ui_to_clip (T width, T height)
    -> Matrix <T, 3, 3>
  {
    Matrix <T, 3, 3> result;
    result.row (0) = Vector <T, 3> (T (2) / width,               0, -1);
    result.row (1) = Vector <T, 3> (            0, -T (2) / height,  1);
    result.row (2) = Vector <T, 3> (            0,               0,  1);
    return result;
  }

}
