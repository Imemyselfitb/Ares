#include "Matrix.h"

Matrix::Matrix(uint8_t rows, uint8_t cols)
	: rows(rows), cols(cols)
{
	data = new float[rows * cols];
	memset(data, 0, rows * cols * sizeof(float));
}

Matrix Matrix::Identity(uint8_t size)
{
	Matrix mat(size, size);
	for (uint8_t i = 0; i < size; i++)
		mat.data[i * size + i] = 1.0f;

	return mat;
}

Matrix Matrix::Diagonal(uint8_t size, const float* diagonalValues)
{
	Matrix mat(size, size);
	for (uint8_t i = 0; i < size; i++)
		mat.data[i * size + i] = diagonalValues[i];

	return mat;
}

void Matrix::Transpose()
{
	std::swap(rows, cols);
	isTransposed = !isTransposed;
}

Matrix Matrix::Transposed() const
{
	Matrix copy{ rows, cols, data };
	copy.Transpose();
	return copy;
}

void Matrix::AssignDotProduct(const Matrix& matA, const Matrix& matB)
{
	_ASSERT(matA.cols == matB.rows);
	_ASSERT((!isTransposed && rows == matA.rows && cols == matB.cols) ||
		(isTransposed && cols == matA.rows && rows == matB.cols));

	isTransposed = false;
	rows = matA.rows;
	cols = matB.cols;
	for (uint8_t x = 0; x < rows; x++)
	{
		for (uint8_t y = 0; y < cols; y++)
		{
			float s = 0.0f;
			for (uint8_t i = 0; i < matA.cols; i++)
				s += matA(x, i) * matB(i, y);

			data[x * cols + y] = s;
		}
	}
}

void Matrix::operator+=(const Matrix& other)
{
	_ASSERT(rows == other.rows && cols == other.cols);

	if (!(isTransposed ^ other.isTransposed))
	{
		for (uint16_t i = 0; i < rows * cols; i++)
			data[i] += other.data[i];
		return;
	}

	for (uint8_t i = 0; i < rows; i++)
	{
		for (uint8_t j = 0; j < cols; j++)
			data[i * cols + j] += other.data[j * rows + i];
	}
}
void Matrix::operator-=(const Matrix& other)
{
	_ASSERT(rows == other.rows && cols == other.cols);

	if (!(isTransposed ^ other.isTransposed))
	{
		for (uint16_t i = 0; i < rows * cols; i++)
			data[i] -= other.data[i];
		return;
	}

	for (uint8_t i = 0; i < rows; i++)
	{
		for (uint8_t j = 0; j < cols; j++)
			data[i * cols + j] -= other.data[j * rows + i];
	}
}
void Matrix::operator*=(float scalar)
{
	for (uint16_t i = 0; i < rows * cols; i++)
		data[i] *= scalar;
}
void Matrix::operator/=(float scalar)
{
	for (uint16_t i = 0; i < rows * cols; i++)
		data[i] /= scalar;
}

bool Matrix::CholeskyDecompose()
{
	_ASSERT(rows == cols);

	if (isTransposed)
		Transpose();

	for (uint8_t i = 0; i < rows; i++)
	{
		for (uint8_t j = 0; j <= i; j++)
		{
			float sum = data[i * cols + j];
			for (uint8_t k = 0; k < j; k++)
				sum -= data[i * cols + k] * data[j * cols + k];

			if (i == j && sum <= 0.0f)
				return false;

			data[i * cols + j] = (i == j) ? std::sqrt(sum) : (sum / data[j * cols + j]);
		}

#if 0 // Removes the upper part of the matrix, but this is not necessary for the Cholesky decomposition to work (removed for performance)
		for (uint8_t j = i + 1; j < cols; j++)
			data[i * cols + j] = 0.0f;
#endif
	}

	return true;
}

void Matrix::SolveCholesky(const Matrix& cholesky)
{
	_ASSERT(cholesky.rows == cholesky.cols);
	_ASSERT(rows == cholesky.rows);

	for (uint8_t i = 0; i < rows; i++)
	{
		for (uint8_t k = 0; k < i; k++)
		{
			float choleskyIK = cholesky.data[i * cholesky.cols + k];
			for (uint8_t j = 0; j < cols; j++)
				data[i * cols + j] -= choleskyIK * data[k * cols + j];
		}

		float diag = 1.0f / cholesky.data[i * cholesky.cols + i];
		for(uint8_t j = 0; j < cols; j++)
			data[i * cols + j] *= diag;
	}

	for (int16_t i = rows - 1; i >= 0; i--)
	{
		for (uint8_t k = (uint8_t)i + 1; k < rows; k++)
		{
			float choleskyIK = cholesky.data[k * cholesky.cols + i];
			for (uint8_t j = 0; j < cols; j++)
				data[i * cols + j] -= choleskyIK * data[k * cols + j];
		}

		float diag = 1.0f / cholesky.data[i * cholesky.cols + i];
		for (uint8_t j = 0; j < cols; j++)
			data[i * cols + j] *= diag;
	}
}

std::ostream& operator<<(std::ostream& os, const Matrix& mat)
{
	os << "[\n\t[";
	for(uint8_t i = 0; i < mat.rows; i++)
	{
		for (uint8_t j = 0; j < mat.cols; j++)
		{
			os << mat(i, j);
			if (j < mat.cols - 1)
				os << ",\t";
		}
		if (i < mat.rows - 1)
			os << "],\n\t[";
	}
	os << "]\n]";
	return os;
}
