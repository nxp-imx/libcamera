/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
 *
 * Matrix and related operations
 */
#pragma once

#include <algorithm>
#include <sstream>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/span.h>

#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(Matrix)

namespace ipa {

#ifndef __DOXYGEN__
template<typename T, unsigned int Rows, unsigned int Cols,
	 std::enable_if_t<std::is_arithmetic_v<T>> * = nullptr>
#else
template<typename T, unsigned int Rows, unsigned int Cols>
#endif /* __DOXYGEN__ */
class Matrix
{
public:
	Matrix()
	{
		data_.fill(static_cast<T>(0));
	}

	Matrix(const std::vector<T> &data)
	{
		std::copy(data.begin(), data.end(), data_.begin());
	}

	static Matrix identity()
	{
		Matrix ret;
		for (size_t i = 0; i < std::min(Rows, Cols); i++)
			ret[i][i] = static_cast<T>(1);
		return ret;
	}

	~Matrix() = default;

	const std::string toString() const
	{
		std::stringstream out;

		out << "Matrix { ";
		for (unsigned int i = 0; i < Rows; i++) {
			out << "[ ";
			for (unsigned int j = 0; j < Cols; j++) {
				out << (*this)[i][j];
				out << ((j + 1 < Cols) ? ", " : " ");
			}
			out << ((i + 1 < Rows) ? "], " : "]");
		}
		out << " }";

		return out.str();
	}

	Span<const T, Cols> operator[](size_t i) const
	{
		return Span<const T, Cols>{ &data_.data()[i * Cols], Cols };
	}

	Span<T, Cols> operator[](size_t i)
	{
		return Span<T, Cols>{ &data_.data()[i * Cols], Cols };
	}

#ifndef __DOXYGEN__
	template<typename U, std::enable_if_t<std::is_arithmetic_v<U>>>
#else
	template<typename U>
#endif /* __DOXYGEN__ */
	Matrix<T, Rows, Cols> &operator*=(U d)
	{
		for (unsigned int i = 0; i < Rows * Cols; i++)
			data_[i] *= d;
		return *this;
	}

private:
	std::array<T, Rows * Cols> data_;
};

#ifndef __DOXYGEN__
template<typename T, typename U, unsigned int Rows, unsigned int Cols,
	 std::enable_if_t<std::is_arithmetic_v<T>> * = nullptr>
#else
template<typename T, typename U, unsigned int Rows, unsigned int Cols>
#endif /* __DOXYGEN__ */
Matrix<U, Rows, Cols> operator*(T d, const Matrix<U, Rows, Cols> &m)
{
	Matrix<U, Rows, Cols> result;

	for (unsigned int i = 0; i < Rows; i++) {
		for (unsigned int j = 0; j < Cols; j++)
			result[i][j] = d * m[i][j];
	}

	return result;
}

#ifndef __DOXYGEN__
template<typename T, typename U, unsigned int Rows, unsigned int Cols,
	 std::enable_if_t<std::is_arithmetic_v<T>> * = nullptr>
#else
template<typename T, typename U, unsigned int Rows, unsigned int Cols>
#endif /* __DOXYGEN__ */
Matrix<U, Rows, Cols> operator*(const Matrix<U, Rows, Cols> &m, T d)
{
	return d * m;
}

#ifndef __DOXYGEN__
template<typename T,
	 unsigned int R1, unsigned int C1,
	 unsigned int R2, unsigned int C2,
	 std::enable_if_t<C1 == R2> * = nullptr>
#else
template<typename T, unsigned int R1, unsigned int C1, unsigned int R2, unsigned in C2>
#endif /* __DOXYGEN__ */
Matrix<T, R1, C2> operator*(const Matrix<T, R1, C1> &m1, const Matrix<T, R2, C2> &m2)
{
	Matrix<T, R1, C2> result;

	for (unsigned int i = 0; i < R1; i++) {
		for (unsigned int j = 0; j < C2; j++) {
			T sum = 0;

			for (unsigned int k = 0; k < C1; k++)
				sum += m1[i][k] * m2[k][j];

			result[i][j] = sum;
		}
	}

	return result;
}

template<typename T, unsigned int Rows, unsigned int Cols>
Matrix<T, Rows, Cols> operator+(const Matrix<T, Rows, Cols> &m1, const Matrix<T, Rows, Cols> &m2)
{
	Matrix<T, Rows, Cols> result;

	for (unsigned int i = 0; i < Rows; i++) {
		for (unsigned int j = 0; j < Cols; j++)
			result[i][j] = m1[i][j] + m2[i][j];
	}

	return result;
}

#ifndef __DOXYGEN__
bool matrixValidateYaml(const YamlObject &obj, unsigned int size);
#endif /* __DOXYGEN__ */

} /* namespace ipa */

#ifndef __DOXYGEN__
template<typename T, unsigned int Rows, unsigned int Cols>
std::ostream &operator<<(std::ostream &out, const ipa::Matrix<T, Rows, Cols> &m)
{
	out << m.toString();
	return out;
}

template<typename T, unsigned int Rows, unsigned int Cols>
struct YamlObject::Getter<ipa::Matrix<T, Rows, Cols>> {
	std::optional<ipa::Matrix<T, Rows, Cols>> get(const YamlObject &obj) const
	{
		if (!ipa::matrixValidateYaml(obj, Rows * Cols))
			return std::nullopt;

		ipa::Matrix<T, Rows, Cols> matrix;
		T *data = &matrix[0][0];

		unsigned int i = 0;
		for (const YamlObject &entry : obj.asList()) {
			const auto value = entry.get<T>();
			if (!value)
				return std::nullopt;

			data[i++] = *value;
		}

		return matrix;
	}
};
#endif /* __DOXYGEN__ */

} /* namespace libcamera */
