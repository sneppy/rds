#pragma once

#include "mat4.h"

/**
 * Vector intrinsics specialization
 */
template<typename T>
struct Mat4<T, true>
{
public:
	/// Vector intrinsics operation class
	using VecOps = Simd::Vector<T, 4>;
	using DVecOps = Simd::Vector<T, 8>;

	/// Vector intrinsics data type
	using VecT = typename VecOps::Type;
	using DVecT = typename DVecOps::Type;

	union
	{
		/// Intrinsics data
		/// @{
		VecT	vec[4];
		DVecT	dvec[2];
		/// @}

		/// Matrix buffer
		T matrix[4][4];

		/// Linear buffer
		T array[16];

		struct
		{
			/// Single components
			/// @todo Maybe unnecessary
			/// @{
			T _a, _b, _c, _d, _e, _f, _g, _h, _i, _l, _m, _n, _o;
			/// @}
		};
	};

public:
	/// Default constructor
	FORCE_INLINE Mat4() {}

	/// Vectors constructor
	FORCE_INLINE Mat4(VecT v0, VecT v1, VecT v2, VecT v3) : vec{v0, v1, v2, v3} {};

	/// Double vectors constructor
	FORCE_INLINE Mat4(DVecT upper, DVecT lower) : dvec{upper, lower} {};

	/// Elements constructor
	FORCE_INLINE Mat4(
		T __a, T __b, T __c, T __d,
		T __e, T __f, T __g, T __h,
		T __i, T __j, T __k, T __l,
		T __m, T __n, T __o, T __p
	) : array{
		__a, __b, __c, __d,
		__e, __f, __g, __h,
		__i, __j, __k, __l,
		__m, __n, __o, __p
	} {}

	/// Scalar constructor, fill matrix
	FORCE_INLINE Mat4(T s) : array{
		s, s, s, s,
		s, s, s, s,
		s, s, s, s,
		s, s, s, s
	} {}

	/// Array random-access operator
	/// @{
	FORCE_INLINE const T & operator[](uint8 i) const { return array[i]; }
	FORCE_INLINE T & operator[](uint8 i) { return array[i]; }
	/// @}

	/// Matrix random-access operator
	/// @{
	FORCE_INLINE const T & operator()(uint8 i, uint8 j) const { return matrix[i][j]; }
	FORCE_INLINE T & operator()(uint8 i, uint8 j) { return matrix[i][j]; }
	/// @}

	/// Returns transposed matrix
	FORCE_INLINE Mat4<T> getTranspose() const
	{
		DVecT
			upper = DVecOps::unpacklo(dvec[0], dvec[1]),
			lower = DVecOps::unpackhi(dvec[0], dvec[1]);
		
		return Mat4<T>(
			DVecOps::shuffle(upper, 0, 4, 1, 5, 2, 6, 3, 7),
			DVecOps::shuffle(lower, 0, 4, 1, 5, 2, 6, 3, 7)
		);
	}

	/// Transposes matrix in place
	FORCE_INLINE Mat4<T> & transpose()
	{
		// Unpack here, interleave there ...
		DVecT
			upper = DVecOps::unpacklo(dvec[0], dvec[1]),
			lower = DVecOps::unpackhi(dvec[0], dvec[1]);
		
		// Shuffle a bit ...
		dvec[0] = DVecOps::shuffle(upper, 0, 4, 1, 5, 2, 6, 3, 7);
		dvec[1] = DVecOps::shuffle(lower, 0, 4, 1, 5, 2, 6, 3, 7);

		// Voila!
		return *this;
	}

	/// Matrix-matrix dot product (multiplication)
	Mat4<T> operator*(const Mat4<T> & m)
	{
		// Transpose matrix for column access
		// ~6 latency
		const Mat4<T> mt = m.getTranspose();

		VecT
			a0 = VecOps::mul(vec[0], mt.vec[0]),
			a1 = VecOps::mul(vec[0], mt.vec[1]),
			a2 = VecOps::mul(vec[0], mt.vec[2]),
			a3 = VecOps::mul(vec[0], mt.vec[3]),

			b0 = VecOps::mul(vec[1], mt.vec[0]),
			b1 = VecOps::mul(vec[1], mt.vec[1]),
			b2 = VecOps::mul(vec[1], mt.vec[2]),
			b3 = VecOps::mul(vec[1], mt.vec[3]),

			c0 = VecOps::mul(vec[2], mt.vec[0]),
			c1 = VecOps::mul(vec[2], mt.vec[1]),
			c2 = VecOps::mul(vec[2], mt.vec[2]),
			c3 = VecOps::mul(vec[2], mt.vec[3]),

			d0 = VecOps::mul(vec[3], mt.vec[0]),
			d1 = VecOps::mul(vec[3], mt.vec[1]),
			d2 = VecOps::mul(vec[3], mt.vec[2]),
			d3 = VecOps::mul(vec[3], mt.vec[3]);
		
		a0 = VecOps::hadd(a0, a1),
		a2 = VecOps::hadd(a2, a3),

		b0 = VecOps::hadd(b0, b1),
		b2 = VecOps::hadd(b2, b3),

		c0 = VecOps::hadd(c0, c1),
		c2 = VecOps::hadd(c2, c3),

		d0 = VecOps::hadd(d0, d1),
		d2 = VecOps::hadd(d2, d3);

		return Mat4<T>(
			VecOps::hadd(a0, a2),
			VecOps::hadd(b0, b2),
			VecOps::hadd(c0, c2),
			VecOps::hadd(d0, d2)
		);
	}

	/// Dot product compound assignment
	Mat4<T> & operator*=(const Mat4<T> & m)
	{
		// Transpose matrix for column access
		// ~6 latency
		const Mat4<T> mt = m.getTranspose();

		/// With loop unrolling is slightly more efficient
		/// I'll leave this here commented in case
		/// I'll need it
		/* for (uint8 i = 0; i < 4; ++i)
		{
			VecT
				r0 = VecOps::mul(vec[0], mt.vec[0]),
				r1 = VecOps::mul(vec[0], mt.vec[1]),
				r2 = VecOps::mul(vec[0], mt.vec[2]),
				r3 = VecOps::mul(vec[0], mt.vec[3]);
			
			r0 = VecOps::hadd(r0, r1),
			r2 = VecOps::hadd(r2, r3);

			vec[i] = VecOps::hadd(r0, r2);
		}

		return *this; */

		VecT
			a0 = VecOps::mul(vec[0], mt.vec[0]),
			a1 = VecOps::mul(vec[0], mt.vec[1]),
			a2 = VecOps::mul(vec[0], mt.vec[2]),
			a3 = VecOps::mul(vec[0], mt.vec[3]),

			b0 = VecOps::mul(vec[1], mt.vec[0]),
			b1 = VecOps::mul(vec[1], mt.vec[1]),
			b2 = VecOps::mul(vec[1], mt.vec[2]),
			b3 = VecOps::mul(vec[1], mt.vec[3]),

			c0 = VecOps::mul(vec[2], mt.vec[0]),
			c1 = VecOps::mul(vec[2], mt.vec[1]),
			c2 = VecOps::mul(vec[2], mt.vec[2]),
			c3 = VecOps::mul(vec[2], mt.vec[3]),

			d0 = VecOps::mul(vec[3], mt.vec[0]),
			d1 = VecOps::mul(vec[3], mt.vec[1]),
			d2 = VecOps::mul(vec[3], mt.vec[2]),
			d3 = VecOps::mul(vec[3], mt.vec[3]);
		
		a0 = VecOps::hadd(a0, a1),
		a2 = VecOps::hadd(a2, a3),

		b0 = VecOps::hadd(b0, b1),
		b2 = VecOps::hadd(b2, b3),

		c0 = VecOps::hadd(c0, c1),
		c2 = VecOps::hadd(c2, c3),

		d0 = VecOps::hadd(d0, d1),
		d2 = VecOps::hadd(d2, d3);

		vec[0] = VecOps::hadd(a0, a2),
		vec[1] = VecOps::hadd(b0, b2),
		vec[2] = VecOps::hadd(c0, c2),
		vec[3] = VecOps::hadd(d0, d2);
		return *this;
	}

	/**
	 * Matrix-vector dot product
	 * 
	 * @param [in] v vector operand
	 * @return transformed vector
	 * @{
	 */
	FORCE_INLINE Vec4<T, true> operator*(const Vec4<T, true> & v)
	{
		VecT
			x = VecOps::mul(vec[0], v.data),
			y = VecOps::mul(vec[1], v.data),
			z = VecOps::mul(vec[2], v.data),
			w = VecOps::mul(vec[3], v.data);

		return Vec4<T, true>(VecOps::hadd(
			VecOps::hadd(x, y),
			VecOps::hadd(z, w)
		));
	}
	FORCE_INLINE Vec3<T, true> operator*(const Vec3<T, true> & v) { return (Vec3<T, true>)operator*(Vec4<T, true>(v)); }
	/// @}

protected:
	#define A dvec[0]
	#define B dvec[1]

	/// Compute upper part of the algebraic complements matrix
	FORCE_INLINE DVecT getUpperAlgebraicComplements() const
	{
		const DVecT
			a0 = DVecOps::shuffle(A, 5, 4, 4, 4, 1, 0, 0, 0),
			a1 = DVecOps::shuffle(A, 6, 6, 5, 5, 2, 2, 1, 1),
			a2 = DVecOps::shuffle(A, 7, 7, 7, 6, 3, 3, 3, 2),

			b0 = DVecOps::shuffle(B, 2, 2, 1, 1, 2, 2, 1, 1),
			b1 = DVecOps::shuffle(B, 7, 7, 7, 6, 7, 7, 7, 6),
			b2 = DVecOps::shuffle(B, 3, 3, 3, 2, 3, 3, 3, 2),
			b3 = DVecOps::shuffle(B, 6, 6, 5, 5, 6, 6, 5, 5),
			b4 = DVecOps::shuffle(B, 1, 0, 0, 0, 1, 0, 0, 0),
			b5 = DVecOps::shuffle(B, 5, 4, 4, 4, 5, 4, 4, 4);

		return DVecOps::mul(
			DVecOps::load(T(1), T(-1), T(1), T(-1), T(-1), T(1), T(-1), T(1)),
			DVecOps::add(
				DVecOps::sub(
					DVecOps::mul(
						a0,
						DVecOps::sub(
							DVecOps::mul(
								b0,
								b1
							),
							DVecOps::mul(
								b2,
								b3
							)
						)
					),
					DVecOps::mul(
						a1,
						DVecOps::sub(
							DVecOps::mul(
								b4,
								b1
							),
							DVecOps::mul(
								b2,
								b5
							)
						)
					)
				),
				DVecOps::mul(
					a2,
					DVecOps::sub(
						DVecOps::mul(
							b4,
							b3
						),
						DVecOps::mul(
							b0,
							b5
						)
					)
				)
			)
		);
	}

	/// Compute lower part of the algebraic complements matrix
	FORCE_INLINE DVecT getLowerAlgebraicComplements() const
	{
		const DVecT
			b0 = DVecOps::shuffle(B, 5, 4, 4, 4, 1, 0, 0, 0),
			b1 = DVecOps::shuffle(B, 6, 6, 5, 5, 2, 2, 1, 1),
			b2 = DVecOps::shuffle(B, 7, 7, 7, 6, 3, 3, 3, 2),

			a0 = DVecOps::shuffle(A, 2, 2, 1, 1, 2, 2, 1, 1),
			a1 = DVecOps::shuffle(A, 7, 7, 7, 6, 7, 7, 7, 6),
			a2 = DVecOps::shuffle(A, 3, 3, 3, 2, 3, 3, 3, 2),
			a3 = DVecOps::shuffle(A, 6, 6, 5, 5, 6, 6, 5, 5),
			a4 = DVecOps::shuffle(A, 1, 0, 0, 0, 1, 0, 0, 0),
			a6 = DVecOps::shuffle(A, 5, 4, 4, 4, 5, 4, 4, 4);
		
		DVecOps::mul(
			DVecOps::load(T(1), T(-1), T(1), T(-1), T(-1), T(1), T(-1), T(1)),
			DVecOps::add(
				DVecOps::sub(
					DVecOps::mul(
						b0,
						DVecOps::sub(
							DVecOps::mul(
								a0,
								a1
							),
							DVecOps::mul(
								a2,
								a3
							)
						)
					),
					DVecOps::mul(
						b1,
						DVecOps::sub(
							DVecOps::mul(
								a4,
								a1
							),
							DVecOps::mul(
								a2,
								a6
							)
						)
					)
				),
				DVecOps::mul(
					b2,
					DVecOps::sub(
						DVecOps::mul(
							a4,
							a3
						),
						DVecOps::mul(
							a0,
							a6
						)
					)
				)
			)
		);
	}

	#undef A
	#undef B

public:
	/// Calculate matrix of algebraic complements
	Mat4<T> getAlgebraicComplementsMatrix()
	{
		return Mat4<T>(
			/// Upper vector
			getUpperAlgebraicComplements(),
			/// Lower vector
			getLowerAlgebraicComplements()
		);
	}

	/// Get matrix inverse
	Mat4<T> operator!()
	{
		// Get complements matrix
		Mat4<T> c = getAlgebraicComplementsMatrix();

		// Compute determinant
		const T det = array[0] * c[0] + array[1] * c[1] + array[2] * c[2] + array[3] * c[3];

		if (LIKELY(det != T(0)))
		{
			c.dvec[0] = DVecOps::div(c.dvec[0], DVecOps::load(det)),
			c.dvec[1] = DVecOps::div(c.dvec[1], DVecOps::load(det));
			return c.transpose();
		}
		else
			return Mat4<T>(T(0));
	}

	/// Invert matrix in place
	Mat4<T> & invert()
	{
		// Get complements matrix
		Mat4<T> c = getAlgebraicComplementsMatrix();

		// Compute determinant
		const T det = array[0] * c[0] + array[1] * c[1] + array[2] * c[2] + array[3] * c[3];

		if (LIKELY(det != T(0)))
		{
			dvec[0] = DVecOps::div(c.dvec[0], DVecOps::load(det)),
			dvec[1] = DVecOps::div(c.dvec[1], DVecOps::load(det));
			transpose();
		}

		return *this;
	}

	/// Print matrix to stdout or specified fd
	FORCE_INLINE void print(FILE * out = stdout) const;

	//////////////////////////////////////////////////
	// Constructors
	//////////////////////////////////////////////////
	
	/// Create a diag matrix from single scalar value
	static FORCE_INLINE Mat4<T> eye(T s)
	{
		return Mat4<float32, true>(
			s, T(0), T(0), T(0),
			T(0), s, T(0), T(0),
			T(0), T(0), s, T(0),
			T(0), T(0), T(0), s
		);
	}

	/// Create a diag matrix with specified diagonal
	/// @{
									static FORCE_INLINE Mat4<T> diag(T a, T b, T c, T d)
									{
										return Mat4<T>(
											a, T(0), T(0), T(0),
											T(0), b, T(0), T(0),
											T(0), T(0), c, T(0),
											T(0), T(0), T(0), d
										);
									}
	template<bool bHasVectorIntrinsics>	static FORCE_INLINE Mat4<T> diag(const Vec4<T, bHasVectorIntrinsics> & v) { return diag(v.x, v.y, v.z, v.w); }
	/// @}

	/// Create a transform matrix with scaling
	/// @{
									static FORCE_INLINE Mat4<T> scaling(T x, T y, T z)
									{
										return Mat4<T>(
											x, T(0), T(0), T(0),
											T(0), y, T(0), T(0),
											T(0), T(0), z, T(0),
											T(0), T(0), T(0), T(1)
										);
									}
	template<bool bHasVectorIntrinsics>	static FORCE_INLINE Mat4<T> scaling(const Vec3<T, bHasVectorIntrinsics> & v) { return scaling(v.x, v.y, v.z); }
	/// @}

	/// Create a transform matrix with translation
	/// @{
									static FORCE_INLINE Mat4<T> translation(T x, T y, T z)
									{
										return Mat4<T>(
											 T(1),  T(0),  T(0), x,
											 T(0),  T(1),  T(0), y,
											 T(0),  T(0),  T(1), z,
											 T(0),  T(0),  T(0),  T(1)
										);
									}
	template<bool bHasVectorIntrinsics>	static FORCE_INLINE Mat4<T> translation(const Vec3<T, bHasVectorIntrinsics> & v) { return translation(v.x, v.y, v.z); }
	/// @}

	/// Create a transform matrix with rotation
	/// @{
	template<bool bHasVectorIntrinsics>	static FORCE_INLINE Mat4<T> rotation(T angle, const Vec3<T, bHasVectorIntrinsics> & axis);
	/// @}

	/// Create a complete transform matrix
	static FORCE_INLINE Mat4<T> transform(const Vec3<T> & v,/** Quat<T> rotation, */const Vec3<T> & s/*  = Vec3<T>::unit */)
	{
		/// @todo Matrix multiplication is slow, there's a better (more complicated) way
		return translation(v) * scaling(s);
	}

	/// Create a projection matrix
	static FORCE_INLINE Mat4<T> projection();
};

template<>
void Mat4<float32, true>::print(FILE * out) const
{
	fprintf(out, "m4f (%.3f, %.3f, %.3f, %.3f)\n", array[0x0], array[0x1], array[0x2], array[0x3]);
	fprintf(out, "    (%.3f, %.3f, %.3f, %.3f)\n", array[0x4], array[0x5], array[0x6], array[0x3]);
	fprintf(out, "    (%.3f, %.3f, %.3f, %.3f)\n", array[0x8], array[0x9], array[0xa], array[0xb]);
	fprintf(out, "    (%.3f, %.3f, %.3f, %.3f)\n", array[0xc], array[0xd], array[0xe], array[0xf]);
}

//////////////////////////////////////////////////
// Constructors
//////////////////////////////////////////////////