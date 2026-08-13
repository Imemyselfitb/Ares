#pragma once

#include <iostream>

class Matrix
{
public:
	Matrix(uint8_t rows, uint8_t cols);
	Matrix(uint8_t rows, uint8_t cols, float* data)
		: rows(rows), cols(cols), data(data) {}

public:
	static Matrix Identity(uint8_t size);
	static Matrix Diagonal(uint8_t size, const float* diagonalValues);

public:
	void AssignDotProduct(const Matrix& matA, const Matrix& matB);
	void Transpose();
	bool CholeskyDecompose(); // Returns success or failure (if the matrix is not positive definite)
	void SolveCholesky(const Matrix& cholesky);

public:
	Matrix Transposed() const;

public:
	inline constexpr float& operator()(uint8_t row, uint8_t col) {
		return data[(!isTransposed) ? (row * cols + col) : (col * rows + row)];
	}
	inline constexpr const float& operator()(uint8_t row, uint8_t col) const {
		return data[(!isTransposed) ? (row * cols + col) : (col * rows + row)];
	}

	void operator+=(const Matrix& other);
	void operator-=(const Matrix& other);
	void operator*=(float scalar);
	void operator/=(float scalar);

public:
	uint8_t rows;
	uint8_t cols;
	float* data = nullptr;

private:
	bool isTransposed = false;
};

std::ostream& operator<<(std::ostream& os, const Matrix& mat);
