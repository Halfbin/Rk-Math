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

//#include <Rk/versor.hpp>
#include <Rk/vector.hpp>

#include <array>

namespace Rk {
  template <uint m, uint n, typename et>
  class matrix {
    static_assert (!std::is_reference <et>::value, "matrix elements must not be references");
    static_assert (std::is_scalar <et>::value,     "matrix elements must be scalar");

  public:
    typedef et             elem_t;
    typedef vector <n, et> row_t;
    typedef vector <m, et> col_t;

  private:
    std::array <row_t, m> rows;

  /*template <uint... idxs>
    auto row_impl (uint i, detail::idx_seq <idxs...>) const {
      return make_vector { rows [i][idxs]... };
    }*/

    template <uint... idxs>
    auto col_impl (uint j, detail::idx_seq <idxs...>) const {
      return make_vector (rows [idxs][j]...);
    }

  /*template <uint... idxs>
    auto set_row_impl (uint i, row_t v, detail::idx_seq <idxs...>) {
      typedef char sequence [n + 1];
      sequence { 0,
        (void (elements [i][idxs] = v [idxs]), 0)
        ...
      };
    }*/

    template <uint... idxs>
    auto set_col_impl (uint j, col_t v, detail::idx_seq <idxs...>) {
      typedef char sequence [m + 1];
      sequence { 0,
        (void (rows [idxs][j] = v [idxs]), 0)
        ...
      };
    }

    template <bool is_square = m == n>
    static matrix identity_impl () {
      static_assert (is_square, "Identity is not defined for non-square matrices");
      return generate ([] (uint i, uint j) { return (i == j) ? et (1) : et (0); });
    }

  public:
    typedef detail::make_idxs <0, m> zero_to_m_t;
    typedef detail::make_idxs <1, m> one_to_m_t;
    typedef detail::make_idxs <0, n> zero_to_n_t;
    typedef detail::make_idxs <1, n> one_to_n_t;

    static zero_to_m_t zero_to_m () { return zero_to_m_t (); }
    static one_to_m_t  one_to_m  () { return one_to_m_t  (); }
    static zero_to_n_t zero_to_n () { return zero_to_n_t (); }
    static one_to_n_t  one_to_n  () { return one_to_n_t  (); }

    matrix () = default;

    template <typename ct, typename... cts>
    explicit matrix (vector <n, ct> first_row, vector <n, cts>... new_rows) :
      rows ({ static_cast <row_t> (first_row), static_cast <row_t> (new_rows)... })
    { }

    template <typename ft>
    static matrix generate (ft f) {
      matrix result;
      for (uint i = 0; i != m; i++)
      for (uint j = 0; j != n; j++)
        result (i, j) = f (i, j);
      return result;
    }

    static matrix identity () {
      return identity_impl <> ();
    }

    et& operator () (uint i, uint j) {
      return rows [i][j];
    }

    et operator () (uint i, uint j) const {
      return rows [i][j];
    }

    et& at (uint i, uint j) {
      if (i >= m || j >= n) throw std::out_of_range ("matrix index out-of-range");
      return rows [i][j];
    }

    et at (uint i, uint j) const {
      if (i >= m || j >= n) throw std::out_of_range ("matrix index out-of-range");
      return rows [i][j];
    }

    row_t row (uint i) const {
      return rows [i]; //row_impl (i, zero_to_n ());
    }

    col_t col (uint j) const {
      return col_impl (j, zero_to_m ());
    }

    void set_row (uint i, row_t v) {
      rows [i] = v; //set_row_impl (i, v, zero_to_n ());
    }

    void set_col (uint j, col_t v) {
      set_col_impl (j, v, zero_to_m ());
    }

    et const* raw () const {
      return (et const*) &rows;
    }
  };

  template <uint n, typename... cts>
  auto matrix_rows (vector <n, cts>&&... rows) {
    return matrix <sizeof... (rows), n, std::common_type_t <cts...>> { rows... };
  }

  template <uint m, uint n, uint p, typename lht, typename rht>
  auto operator * (const matrix <m, n, lht>& lhs, const matrix <n, p, rht>& rhs) {
    matrix <m, p, product_t <lht, rht>> result;
    for (uint i = 0; i != m; i++)
    for (uint j = 0; j != p; j++)
      result (i, j) = dot (lhs.row (i), rhs.col (j));
    return result;
  }

  template <uint m, uint n, uint p, typename lht, typename rht>
  auto operator *= (matrix <m, n, lht>& lhs, const matrix <n, p, rht>& rhs) {
    lhs = lhs * rhs;
  };

  namespace detail {
    template <uint m, uint n, typename lht, typename rht, uint... idxs>
    auto mul_cvec_impl (const matrix <m, n, lht>& lhs, vector <n, rht> rhs, idx_seq <idxs...>) {
      return make_vector (dot (lhs.row (idxs), rhs)...);
    }

    template <uint m, uint n, typename lht, typename rht, uint... idxs>
    auto mul_rvec_impl (vector <m, lht> lhs, const matrix <m, n, rht>& rhs, idx_seq <idxs...>) {
      return make_vector (dot (lhs, rhs.col (idxs))...);
    }
  }

  template <uint m, uint n, typename lht, typename rht>
  auto operator * (const matrix <m, n, lht>& lhs, vector <n, rht> rhs) {
    using namespace detail;
    return mul_cvec_impl (lhs, rhs, lhs.zero_to_m ());
  }

  template <uint m, uint n, typename lht, typename rht>
  auto operator * (vector <m, lht> lhs, const matrix <m, n, rht>& rhs) {
    using namespace detail;
    return mul_rvec_impl (lhs, rhs, rhs.zero_to_n ());
  }

  namespace matrix_types {
    typedef matrix <2, 2, float> matrix2f, mat2f, m2f;
    typedef matrix <3, 3, float> matrix3f, mat3f, m3f;
    typedef matrix <4, 4, float> matrix4f, mat4f, m4f;
    typedef matrix <2, 2, double> matrix2d, mat2d, m2d;
    typedef matrix <3, 3, double> matrix3d, mat3d, m3d;
    typedef matrix <4, 4, double> matrix4d, mat4d, m4d;

    typedef matrix <2, 3, float> matrix2x3f, mat2x3f, m23f;
    typedef matrix <2, 4, float> matrix2x4f, mat2x4f, m24f;
    typedef matrix <3, 2, float> matrix3x2f, mat3x2f, m32f;
    typedef matrix <3, 4, float> matrix3x4f, mat3x4f, m34f;
    typedef matrix <4, 2, float> matrix4x2f, mat4x2f, m42f;
    typedef matrix <4, 3, float> matrix4x3f, mat4x3f, m43f;

    typedef matrix <2, 3, double> matrix2x3d, mat2x3d, m23d;
    typedef matrix <2, 4, double> matrix2x4d, mat2x4d, m24d;
    typedef matrix <3, 2, double> matrix3x2d, mat3x2d, m32d;
    typedef matrix <3, 4, double> matrix3x4d, mat3x4d, m34d;
    typedef matrix <4, 2, double> matrix4x2d, mat4x2d, m42d;
    typedef matrix <4, 3, double> matrix4x3d, mat4x3d, m43d;
  }

  using namespace matrix_types;
}

#ifndef RK_MATRIX_NO_GLOBAL
using namespace Rk::matrix_types;
#endif

