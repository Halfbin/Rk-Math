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

#include <Rk/type_traits.hpp>
#include <Rk/types.hpp>
#include <Rk/lerp.hpp>

#include <initializer_list>
#include <functional>
#include <algorithm>
#include <stdexcept>
#include <cstdint>
#include <array>
#include <cmath>

namespace Rk {
  namespace detail {
    template <uint...>
    struct idx_seq { };

    template <uint base, uint n, uint... rest>
    struct idx_seq_maker :
      idx_seq_maker <base, n - 1, base + n - 1, rest...>
    { };

    template <uint base, uint... rest>
    struct idx_seq_maker <base, 0, rest...> {
      typedef idx_seq <rest...> seq;
    };

    template <uint base, uint n>
    using make_idxs = typename idx_seq_maker <base, n - base>::seq;

    struct vector_tag { };
  }

  template <typename t>
  struct is_vector :
    std::is_base_of <detail::vector_tag, t>
  { };

  template <uint n, typename ct>
  class vector;

  template <uint n, typename ct>
  class vector_facade :
    detail::vector_tag
  {
    typedef vector <n, ct> self_t;

    static_assert (!std::is_reference <ct>::value, "vector components must not be references");
    static_assert (std::is_scalar <ct>::value,     "vector components must be scalar");

    constexpr self_t&       self ()       { return static_cast <self_t&>       (*this); }
    constexpr const self_t& self () const { return static_cast <const self_t&> (*this); }

  protected:
    typedef vector_facade facade_t;

  public:
    typedef ct        comp_t;
    typedef ct*       ptr_t;
    typedef const ct* cptr_t;
    typedef ptr_t     iterator;
    typedef cptr_t    const_iterator;

    typedef detail::make_idxs <0, n> zero_to_n_t;
    typedef detail::make_idxs <1, n> one_to_n_t;

    static constexpr auto zero_to_n () {
      return zero_to_n_t ();
    }

    static constexpr auto one_to_n () {
      return one_to_n_t ();
    }

    constexpr ct& operator [] (uint i)       { return self ().components [i]; }
    constexpr ct  operator [] (uint i) const { return self ().components [i]; }

    ct& at (uint i) {
      if (i >= n) throw std::out_of_range ("vector index out-of-range");
      return self ().components [i];
    }

    ct at (uint i) const {
      if (i >= n) throw std::out_of_range ("vector index out-of-range");
      return self ().components [i];
    }

    ptr_t  begin  ()       { return std::begin (self ().components); }
    cptr_t begin  () const { return std::begin (self ().components); }
    cptr_t cbegin () const { return std::begin (self ().components); }

    ptr_t  end  ()       { return std::end (self ().components); }
    cptr_t end  () const { return std::end (self ().components); }
    cptr_t cend () const { return std::end (self ().components); }

  //auto length () const; // gives C3779 on CTP11-2013

    template <typename... sw_ts>
    auto swizzle (sw_ts&&... sws) const;

    template <typename... sw_ts>
    auto operator () (sw_ts&&... sws) const {
      return swizzle (std::forward <sw_ts> (sws)...);
    }
  };

  template <uint n, typename ct>
  class vector :
    public vector_facade <n, ct>
  {
  public:
    std::array <ct, n> components;

    constexpr vector () = default;

    template <typename t>
    constexpr vector (const vector <n, t>& other) :
      components (other.components)
    { }

    constexpr vector (Nil) :
      components { }
    { }

    template <typename init_t>
    vector (std::initializer_list <init_t> init) {
      if (init.size () != n)
        throw std::length_error ("Cannot initialize vector with wrong number of components");
      std::copy (init.begin (), init.end (), components);
    }
  };

  template <typename ct>
  class vector <2, ct> :
    public vector_facade <2, ct>
  {
  public:
    union {
      std::array <ct, 2> components;
      struct { ct x, y; };
    };

    constexpr vector () = default;

    constexpr vector (Nil) :
      components {0, 0}
    { }

    template <typename t>
    constexpr vector (const vector <2, t>& other) :
      components { (ct) other[0], (ct) other[1] }
    { }

    constexpr vector (ct nx, ct ny) :
      components {nx, ny}
    { }
  };

  template <typename ct>
  class vector <3, ct> :
    public vector_facade <3, ct>
  {
  public:
    union {
      std::array <ct, 3> components;
      struct { ct x, y, z; };
    };

    constexpr vector () = default;

    constexpr vector (Nil) :
      components {0, 0, 0}
    { }

    template <typename t>
    constexpr vector (const vector <3, t>& other) :
      components { (ct) other[0], (ct) other[1], (ct) other[2] }
    { }

    constexpr vector (ct nx, ct ny, ct nz) :
      components { nx, ny, nz }
    { }
  };

  template <typename ct>
  class vector <4, ct> :
    public vector_facade <4, ct>
  {
  public:
    union {
      std::array <ct, 4> components;
      struct { ct x, y, z, w; };
    };

    constexpr vector () = default;

    constexpr vector (Nil) :
      components {0, 0, 0, 0}
    { }

    template <typename t>
    constexpr vector (const vector <4, t>& other) :
      components { (ct) other[0], (ct) other[1], (ct) other[2], (ct) other[3] }
    { }

    constexpr vector (ct nx, ct ny, ct nz, ct nw) :
      components { nx, ny, nz, nw }
    { }
  };

  template <typename ct, typename... arg_ts>
  constexpr auto make_vector_as (arg_ts... args) {
    return vector <sizeof... (arg_ts), ct> { static_cast <ct> (args)... };
  }

  template <typename... arg_ts>
  constexpr auto make_vector (arg_ts... args) {
    return make_vector_as <std::common_type_t <arg_ts...>> (args...);
  }

  namespace detail {
    template <uint n, typename ct, uint... idxs>
    constexpr auto make_diagonal_impl (ct x, idx_seq <idxs...>) {
      return make_vector (((void) idxs, x) ...);
    }
  }

  template <uint n, typename ct>
  [[deprecated ("prefer diag")]]
  constexpr auto make_diagonal (ct x) {
    return detail::make_diagonal_impl <n> (x, vector <n, ct>::zero_to_n ());
  }

  template <uint n, typename ct>
  constexpr auto diag (ct x) {
    return detail::make_diagonal_impl <n> (x, vector <n, ct>::zero_to_n ());
  }

  template <uint n, typename ct>
  static constexpr auto const ones_vector = make_diagonal<n, ct> (1);

  template <uint n, typename ct>
  static constexpr auto const zero_vector = make_diagonal<n, ct> (0);

  namespace detail {
    template <typename st, typename rt = void>
    struct scalar_en :
      std::enable_if <std::is_scalar <st>::value, rt>
    { };

    template <typename st, typename rt = void>
    struct fp_en :
      std::enable_if <std::is_floating_point <st>::value, rt>
    { };

    template <uint n, typename vt, uint m, typename ut, uint... v_idxs, uint... u_idxs>
    auto vector_cat (vector <n, vt> v, vector <m, ut> u, idx_seq <v_idxs...>, idx_seq <u_idxs...>) {
      return make_vector (v [v_idxs]..., u [u_idxs]...);
    }

    template <uint n, typename ct, typename t, uint... idxs>
    auto vector_cat (vector <n, ct> v, t x, idx_seq <idxs...>) {
      return make_vector (v [idxs]..., x);
    }

    template <uint n, typename ct, uint m>
    auto compose_impl (vector <n, ct> v) {
      return v;
    }

    template <uint n, typename ct, typename t>
    auto compose_impl (vector <n, ct> v, t x) {
      return vector_cat (v, x, v.zero_to_n ());
    }

    template <uint n, typename ct, uint m, typename ut, typename... ts>
    auto compose_impl (vector <n, ct> v, vector <m, ut> u, ts... xs) {
      return compose_impl (vector_cat (v, u, v.zero_to_n (), u.zero_to_n ()), xs...);
    }

    template <uint n, typename ct, typename t, typename... ts>
    auto compose_impl (vector <n, ct> v, t x, ts... xs) {
      return compose_impl (vector_cat (v, x, v.zero_to_n ()), xs...);
    }

    template <typename t, typename... ts, typename en_t = typename scalar_en <t>::type>
    auto compose_impl (t x, ts... xs) {
      return compose_impl (vector <1, t> { x }, xs...);
    }
  }

  template <typename... xts>
  auto compose_vector (xts... xs) {
    return detail::compose_impl (xs...);
  }

  // Detail
  namespace detail {
    template <typename ft, uint n, typename lht, typename rht, uint... idxs>
    constexpr auto transform_impl (ft f, vector <n, lht> lhs, vector <n, rht> rhs, idx_seq <idxs...>) {
      return make_vector (f (lhs [idxs], rhs [idxs]) ...);
    }

    template <typename ft, uint n, typename ct, uint... idxs>
    constexpr auto transform_impl (ft f, vector <n, ct> v, idx_seq <idxs...>) {
      return make_vector (f (v [idxs]) ...);
    }

    template <typename ft, uint n, typename ct, uint... idxs>
    auto reduce_impl (ft f, vector <n, ct> v, idx_seq <idxs...>) {
      typedef std::result_of_t <ft (ct, ct)> rt;
      rt accum = v [0];
      typedef char sequence [sizeof... (idxs) + 1];
      (void) sequence { '\0', // sequencing trick
        (void (accum = f (accum, v [idxs])), '\0')
        ...
      };
      return accum;
    }
  }

  template <typename ft, uint n, typename lht, typename rht>
  constexpr auto transform (ft f, vector <n, lht> lhs, vector <n, rht> rhs) {
    using namespace detail;
    return transform_impl (f, lhs, rhs, lhs.zero_to_n ());
  }

  template <typename ft, uint n, typename ct>
  constexpr auto transform (ft f, vector <n, ct> v) {
    using namespace detail;
    return transform_impl (f, v, v.zero_to_n ());
  }

  template <typename ft, uint n, typename ct>
  auto reduce (ft f, vector <n, ct> v) {
    using namespace detail;
    return reduce_impl (f, v, v.one_to_n ());
  }

  template <typename tft, typename rft, uint n, typename lht, typename rht>
  auto inner (rft rf, tft tf, vector <n, lht> lhs, vector <n, rht> rhs) {
    return reduce (rf, transform (tf, lhs, rhs));
  }

  // Componentwise operations

  // Comparison
  template <uint n, typename lht, typename rht>
  bool constexpr operator == (vector <n, lht> lhs, vector <n, rht> rhs) {
    return inner (std::logical_and <> (), std::equal_to <> (), lhs, rhs);
  }

  template <uint n, typename lht, typename rht>
  bool constexpr operator != (vector <n, lht> lhs, vector <n, rht> rhs) {
    return !(lhs == rhs);
  }

  // Addition
  template <uint n, typename lht, typename rht>
  auto constexpr operator + (vector <n, lht> lhs, vector <n, rht> rhs) {
    return transform (std::plus <> (), lhs, rhs);
  }

  template <uint n, typename lht, typename rht>
  auto& operator += (vector <n, lht>& lhs, vector <n, rht> rhs) {
    return lhs = lhs + rhs;
  }

  // Subtraction
  template <uint n, typename lht, typename rht>
  auto constexpr operator - (vector <n, lht> lhs, vector <n, rht> rhs) {
    return transform (std::minus <> (), lhs, rhs);
  }

  template <uint n, typename lht, typename rht>
  auto& operator -= (vector <n, lht>& lhs, vector <n, rht> rhs) {
    return lhs = lhs - rhs;
  }

  // Negation
  template <uint n, typename ct>
  auto constexpr operator - (vector <n, ct> v) {
    return transform (std::negate <> (), v);
  }

  // Multiplication
  template <uint n, typename lht, typename rht>
  auto constexpr operator * (vector <n, lht> lhs, vector <n, rht> rhs) {
    return transform (std::multiplies <> (), lhs, rhs);
  }

  template <uint n, typename lht, typename rht, typename = typename detail::scalar_en <rht>::type>
  auto constexpr operator * (vector <n, lht> lhs, rht rhs) {
    return transform ([rhs] (lht x) { return x * rhs; }, lhs);
  }

  template <uint n, typename lht, typename rht, typename = typename detail::scalar_en <lht>::type>
  auto constexpr operator * (lht lhs, vector <n, rht> rhs) {
    return rhs * lhs;
  }

  template <uint n, typename lht, typename rht>
  auto& operator *= (vector <n, lht>& lhs, rht&& rhs) {
    return lhs = lhs * std::forward <rht> (rhs);
  }

  // Division
  template <uint n, typename lht, typename rht>
  auto constexpr operator / (vector <n, lht> lhs, vector <n, rht> rhs) {
    return transform (std::divides <> (), lhs, rhs);
  }

  template <uint n, typename lht, typename rht, typename = typename detail::scalar_en <rht>::type>
  auto constexpr operator / (vector <n, lht> lhs, rht rhs) {
    return transform ([rhs] (lht x) { return x / rhs; }, lhs);
  }

  template <uint n, typename lht, typename rht>
  auto& operator /= (vector <n, lht>& lhs, rht&& rhs) {
    return lhs = lhs / std::forward <rht> (rhs);
  }

  // Modulo
  template <uint n, typename lht, typename rht>
  auto constexpr operator % (vector <n, lht> lhs, vector <n, rht> rhs) {
    return transform (std::modulus <> (), lhs, rhs);
  }

  template <uint n, typename lht, typename rht, typename = typename detail::scalar_en <rht>::type>
  auto constexpr operator % (vector <n, lht> lhs, rht rhs) {
    return transform ([rhs] (lht x) { return x % rhs; }, lhs);
  }

  template <uint n, typename lht, typename rht>
  auto& operator %= (vector <n, lht>& lhs, rht&& rhs) {
    return lhs = lhs % std::forward <rht> (rhs);
  }

  // Bitwise and
  template <uint n, typename lht, typename rht>
  auto constexpr operator & (vector <n, lht> lhs, vector <n, rht> rhs) {
    return transform (std::bit_and <> (), lhs, rhs);
  }

  template <uint n, typename lht, typename rht, typename = typename detail::scalar_en <rht>::type>
  auto constexpr operator & (vector <n, lht> lhs, rht rhs) {
    return transform ([rhs] (lht x) { return x & rhs; }, lhs);
  }

  template <uint n, typename lht, typename rht>
  auto& operator &= (vector <n, lht>& lhs, rht&& rhs) {
    return lhs = lhs & std::forward <rht> (rhs);
  }

  // Bitwise or
  template <uint n, typename lht, typename rht>
  auto constexpr operator | (vector <n, lht> lhs, vector <n, rht> rhs) {
    return transform (std::bit_or <> (), lhs, rhs);
  }

  template <uint n, typename lht, typename rht, typename = typename detail::scalar_en <rht>::type>
  auto constexpr operator | (vector <n, lht> lhs, rht rhs) {
    return transform ([rhs] (lht x) { return x | rhs; }, lhs);
  }

  template <uint n, typename lht, typename rht>
  auto& operator |= (vector <n, lht>& lhs, rht&& rhs) {
    return lhs = lhs | std::forward <rht> (rhs);
  }

  // Bitwise xor
  template <uint n, typename lht, typename rht>
  auto constexpr operator ^ (vector <n, lht> lhs, vector <n, rht> rhs) {
    return transform (std::bit_xor <> (), lhs, rhs);
  }

  template <uint n, typename lht, typename rht, typename = typename detail::scalar_en <rht>::type>
  auto constexpr operator ^ (vector <n, lht> lhs, rht rhs) {
    return transform ([rhs] (lht x) { return x ^ rhs; }, lhs);
  }

  template <uint n, typename lht, typename rht>
  auto& operator ^= (vector <n, lht>& lhs, rht&& rhs) {
    return lhs = lhs ^ std::forward <rht> (rhs);
  }

  // Shift
  template <uint n, typename lht, typename rht>
  auto constexpr operator >> (vector <n, lht> lhs, vector <n, rht> rhs) {
    return transform ([] (lht a, rht b) { return a >> b; }, lhs, rhs);
  }

  template <uint n, typename lht, typename rht, typename = typename detail::scalar_en <rht>::type>
  auto constexpr operator >> (vector <n, lht> lhs, rht rhs) {
    return transform ([rhs] (lht x) { return x >> rhs; }, lhs);
  }

  template <uint n, typename lht, typename rht>
  auto& operator >>= (vector <n, lht>& lhs, rht&& rhs) {
    return lhs = lhs >> std::forward <rht> (rhs);
  }

  template <uint n, typename lht, typename rht>
  auto constexpr operator << (vector <n, lht> lhs, vector <n, rht> rhs) {
    return transform ([] (lht a, rht b) { return a << b; }, lhs, rhs);
  }

  template <uint n, typename lht, typename rht, typename = typename detail::scalar_en <rht>::type>
  auto constexpr operator << (vector <n, lht> lhs, rht rhs) {
    return transform ([rhs] (lht x) { return x << rhs; }, lhs);
  }

  template <uint n, typename lht, typename rht>
  auto& operator <<= (vector <n, lht>& lhs, rht&& rhs) {
    return lhs = lhs << std::forward <rht> (rhs);
  }

  // Dot product
  template <uint n, typename lht, typename rht>
  auto constexpr dot (vector <n, lht> lhs, vector <n, rht> rhs) {
    return reduce (std::plus <> (), lhs * rhs);
  }

  // Length squared (faster than length; monotonic)
  template <uint n, typename ct>
  auto constexpr abs2 (vector <n, ct> v) {
    return dot (v, v);
  }

  template <uint n, typename ct>
  auto constexpr length2 (vector <n, ct> v) {
    return abs2 (v);
  }

  // Length
  template <uint n, typename ct>
  auto constexpr abs (vector <n, ct> v) {
    return std::sqrt (abs2 (v));
  }

  template <uint n, typename ct>
  auto constexpr length (vector <n, ct> v) {
    return abs (v);
  }

  template <uint n, typename ct>
  auto constexpr unit (vector <n, ct> v) {
    auto len = abs (v);
    if (len > 0)
      v *= 1 / len;
    return v;
  }

/*template <uint n, typename ct>
  auto vector_facade <n, ct>::length () const
  {
    return Rk::length (self ());
  }*/

  // Cross product
  template <typename lht, typename rht>
  auto constexpr cross (vector <3, lht> lhs, vector <3, rht> rhs) {
    return make_vector (
      lhs.y * rhs.z - lhs.z * rhs.y,
      lhs.z * rhs.x - lhs.x * rhs.z,
      lhs.x * rhs.y - lhs.y * rhs.x
    );
  }

  // Linear interpolate
  template <uint n, typename lht, typename rht, typename tt, typename = typename detail::fp_en <tt>::type>
  auto lerp (vector <n, lht> lhs, vector <n, rht> rhs, tt t) {
    return transform ([t] (lht x, rht y) { return lerp (x, y, t); }, lhs, rhs);
  }

  // Floor / Ceiling
  template <uint n, typename ct>
  auto floor (vector <n, ct> v) {
    return transform ([] (ct x) { return std::floor (x); }, v);
  }

  template <uint n, typename ct>
  auto ceil (vector <n, ct> v) {
    return transform ([] (ct x) { return std::ceil (x); }, v);
  }

  // min / max
  template <uint n, typename ct>
  auto min (vector <n, ct> v) {
    return reduce ([] (ct a, ct b) { return std::min (a, b); }, v);
  }

  template <uint n, typename ct>
  auto max (vector <n, ct> v) {
    return reduce ([] (ct a, ct b) { return std::max (a, b); }, v);
  }

  // Swizzling
  namespace xyzw_swiz { enum : uint { X = 0, Y, Z, W }; }
  namespace rgba_swiz { enum : uint { R = 0, G, B, A }; }
  namespace stpq_swiz { enum : uint { S = 0, T, P, Q }; }

  namespace swiz {
    using namespace xyzw_swiz;
  }

  template <uint n, typename ct, uint m, typename swt>
  auto swizzle (vector <n, ct> v, vector <m, swt> sw) {
    return transform ([v] (uint i) { return v [i]; }, sw);
  }

  template <uint n, typename ct, typename... sw_ts>
  auto swizzle (vector <n, ct> v, uint i, uint j, sw_ts... sws) {
    return swizzle (v, make_vector_as <uint> (i, j, sws...));
  }

  template <uint n, typename ct>
  template <typename... sw_ts>
  auto vector_facade <n, ct>::swizzle (sw_ts&&... sws) const {
    return Rk::swizzle (self (), std::forward <sw_ts> (sws)...);
  }

  // Convenience types
  namespace vector_types {
    template <typename ct>
    using vector2 = vector <2, ct>;

    template <typename ct>
    using vector3 = vector <3, ct>;

    template <typename ct>
    using vector4 = vector <4, ct>;

    template<uint n> using vectori = vector<n, int>;
    template<uint n> using vectorf = vector<n, float>;
    template<uint n> using vectord = vector<n, double>;
    template<uint n> using vectori8 = vector<n, int8_t>;
    template<uint n> using vectori16 = vector<n, int16_t>;
    template<uint n> using vectori32 = vector<n, int32_t>;
    template<uint n> using vectoru8 = vector<n, uint8_t>;
    template<uint n> using vectoru16 = vector<n, uint16_t>;
    template<uint n> using vectoru32 = vector<n, uint32_t>;

    typedef vector2<int>    vector2i, vec2i, v2i;
    typedef vector3<int>    vector3i, vec3i, v3i;
    typedef vector4<int>    vector4i, vec4i, v4i;

    typedef vector2<float>  vector2f, vec2f, v2f;
    typedef vector3<float>  vector3f, vec3f, v3f;
    typedef vector4<float>  vector4f, vec4f, v4f;

    typedef vector2<double> vector2d, vec2d, v2d;
    typedef vector3<double> vector3d, vec3d, v3d;
    typedef vector4<double> vector4d, vec4d, v4d;

    typedef vector2<int8_t> vector2i8, vec2i8, v2i8;
    typedef vector3<int8_t> vector3i8, vec3i8, v3i8;
    typedef vector4<int8_t> vector4i8, vec4i8, v4i8;

    typedef vector2<int16_t> vector2i16, vec2i16, v2i16;
    typedef vector3<int16_t> vector3i16, vec3i16, v3i16;
    typedef vector4<int16_t> vector4i16, vec4i16, v4i16;

    typedef vector2<int32_t> vector2i32, vec2i32, v2i32;
    typedef vector3<int32_t> vector3i32, vec3i32, v3i32;
    typedef vector4<int32_t> vector4i32, vec4i32, v4i32;

    typedef vector2<uint8_t> vector2u8, vec2u8, v2u8;
    typedef vector3<uint8_t> vector3u8, vec3u8, v3u8;
    typedef vector4<uint8_t> vector4u8, vec4u8, v4u8;

    typedef vector2<uint16_t> vector2u16, vec2u16, v2u16;
    typedef vector3<uint16_t> vector3u16, vec3u16, v3u16;
    typedef vector4<uint16_t> vector4u16, vec4u16, v4u16;

    typedef vector2<uint32_t> vector2u32, vec2u32, v2u32;
    typedef vector3<uint32_t> vector3u32, vec3u32, v3u32;
    typedef vector4<uint32_t> vector4u32, vec4u32, v4u32;
  }

  using namespace vector_types;
}

#ifndef RK_VECTOR_NO_GLOBAL
using Rk::vector;
using namespace Rk::vector_types;
#endif

