#include <cassert>
#include <stdexcept>
#include <numeric>
#include <cmath>
#include <utility>
#include <iomanip>

/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N>::Matrix(T value)
{
  for (std::size_t row = 0; row < M; ++row)
  {
    for (std::size_t column = 0; column < N; ++column)
    {
      matrix.at(row).at(column) = value;
    }
  }
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N>::Matrix(const std::initializer_list<T>& aList)
{
  // Check the arguments
  assert(aList.size() == M * N);

  auto row_iter = aList.begin();
  for (std::size_t row = 0; row < M; ++row)
  {
    for (std::size_t column = 0; column < N; ++column, ++row_iter)
    {
      matrix.at(row).at(column) = *row_iter;
    }
  }
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N>::Matrix(const std::initializer_list<std::initializer_list<T> >& aList)
{
  // Check the arguments, the static assert assures that there is at least 1 M and 1 N!
  assert(aList.size() == M && (*aList.begin()).size() == N);

  auto row_iter = aList.begin();
  for (std::size_t row = 0; row < aList.size(); ++row, ++row_iter)
  {
    auto column_iter = (*row_iter).begin();
    for (std::size_t column = 0; column < (*row_iter).size(); ++column, ++column_iter)
    {
      matrix.at(row).at(column) = *column_iter;
    }
  }
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N>::Matrix(const Matrix<T, M, N>& aMatrix) : matrix(aMatrix.matrix)
{
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
std::array<T, N>& Matrix<T, M, N>::at(std::size_t aRowIndex)
{
  return matrix.at(aRowIndex);
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
const std::array<T, N>& Matrix<T, M, N>::at(std::size_t aRowIndex) const
{
  return matrix.at(aRowIndex);
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
T& Matrix<T, M, N>::at(std::size_t aRowIndex, std::size_t aColumnIndex)
{
  return matrix.at(aRowIndex).at(aColumnIndex);
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
const T& Matrix<T, M, N>::at(std::size_t aRowIndex, std::size_t aColumnIndex) const
{
  return matrix.at(aRowIndex).at(aColumnIndex);
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
std::array<T, N>& Matrix<T, M, N>::operator[](std::size_t aRowIndex)
{
  return matrix[aRowIndex];
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
const std::array<T, N>& Matrix<T, M, N>::operator[](std::size_t aRowIndex) const
{
  return matrix[aRowIndex];
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N>& Matrix<T, M, N>::operator=(const Matrix<T, M, N>& rhs)
{
  if (this != &rhs)
  {
    matrix = rhs.matrix;
  }
  return *this;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
bool Matrix<T, M, N>::operator==(const Matrix<T, M, N>& rhs) const
{
  return matrix == rhs.matrix;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
template <class T2>
Matrix<T, M, N>& Matrix<T, M, N>::operator*=(const T2& scalar)
{
  static_assert(std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see "
                                               "http://en.cppreference.com/w/cpp/types/is_arithmetic");

  for (std::size_t row = 0; row < M; ++row)
  {
    for (std::size_t column = 0; column < N; ++column)
    {
      matrix.at(row).at(column) *= scalar;
    }
  }
  return *this;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
template <class T2>
Matrix<T, M, N> Matrix<T, M, N>::operator*(const T2& scalar) const
{
  static_assert(std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see "
                                               "http://en.cppreference.com/w/cpp/types/is_arithmetic");

  Matrix<T, M, N> result(*this);
  return result *= scalar;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
template <class T2>
Matrix<T, M, N>& Matrix<T, M, N>::operator/=(const T2& aScalar)
{
  static_assert(std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see "
                                               "http://en.cppreference.com/w/cpp/types/is_arithmetic");

  for (std::size_t row = 0; row < M; ++row)
  {
    for (std::size_t column = 0; column < N; ++column)
    {
      matrix.at(row).at(column) /= aScalar;
    }
  }
  return *this;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
template <class T2>
Matrix<T, M, N> Matrix<T, M, N>::operator/(const T2& aScalar) const
{
  static_assert(std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see "
                                               "http://en.cppreference.com/w/cpp/types/is_arithmetic");

  Matrix<T, M, N> result(*this);
  return result /= aScalar;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N>& Matrix<T, M, N>::operator+=(const Matrix<T, M, N>& rhs)
{
  for (std::size_t row = 0; row < M; ++row)
  {
    for (std::size_t column = 0; column < N; ++column)
    {
      matrix[row][column] += rhs.at(row, column);
    }
  }
  return *this;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N> Matrix<T, M, N>::operator+(const Matrix<T, M, N>& rhs) const
{
  Matrix<T, M, N> result(*this);
  return result += rhs;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N>& Matrix<T, M, N>::operator-=(const Matrix<T, M, N>& rhs)
{
  for (std::size_t row = 0; row < M; ++row)
  {
    for (std::size_t column = 0; column < N; ++column)
    {
      matrix[row][column] -= rhs.at(row, column);
    }
  }
  return *this;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N> Matrix<T, M, N>::operator-(const Matrix<T, M, N>& rhs) const
{
  Matrix<T, M, N> result(*this);
  return result -= rhs;
}
/**
 * (M, N) * (N, P) -> (M, P)
 */
template <typename T, std::size_t M, std::size_t N>
template <std::size_t columns>
Matrix<T, M, columns> Matrix<T, M, N>::operator*(const Matrix<T, N, columns>& rhs) const
{
  Matrix<T, M, columns> result;

  for (size_t i = 0; i < columns; ++i)
  {
    for (size_t lhsR = 0; lhsR < M; ++lhsR)
    {
      for (size_t col = 0; col < N; ++col)
      {
        result.at(lhsR, i) += this->at(lhsR, col) * rhs.at(col, i);
      }
    }
  }
  return result;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, N, M> Matrix<T, M, N>::transpose() const
{
  Matrix<T, N, M> result;

  for (size_t i = 0; i < M; ++i)
  {
    for (size_t j = 0; j < N; ++j)
    {
      result.at(j).at(i) = this->at(i).at(j);
    }
  }
  return result;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N> Matrix<T, M, N>::identity() const
{
  Matrix<T, N, M> result;

  for (size_t i = 0; i < N; ++i)
  {
    for (size_t j = 0; j < M; ++j)
    {
      if (j == i)
      {
        result.at(i).at(j) = 1;
      }
      else
      {
        result.at(i).at(j) = 0;
      }
    }
  }

  return result;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
T getLineariseFactor(const Matrix<T, M, N>& aMatrix, std::size_t aPivotColumn, std::size_t aCurrentRow)
{
  T lineariseFactor = 0;
  if (aMatrix[aCurrentRow][aPivotColumn] == 0.0)
  {
    // No change for a 0-row
    lineariseFactor = 0;
  }
  else if (aCurrentRow == aPivotColumn)
  {
    // use (a-1)/pivot value
    lineariseFactor = (aMatrix[aCurrentRow][aPivotColumn] - !0) / aMatrix[aPivotColumn][aPivotColumn];
  }
  else
  {
    // us a/pivot value
    lineariseFactor = aMatrix[aCurrentRow][aPivotColumn] / aMatrix[aPivotColumn][aPivotColumn];
  }
  return lineariseFactor;
}

template <class T, std::size_t M, std::size_t N>
std::size_t getMaxPivotRow(const Matrix<T, M, N>& aMatrix, size_t pivotColumn)
{
  T maxPivotValue = 0;
  std::size_t maxPivotRow = 0;

  for (std::size_t row = pivotColumn; row < M; ++row)
  {
    if (maxPivotValue < std::abs(aMatrix.at(row).at(pivotColumn)))
    {
      maxPivotValue = std::abs(aMatrix.at(row).at(pivotColumn));
      maxPivotRow = row;
    }
  }
  return maxPivotRow;
}

template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N> Matrix<T, M, N>::gauss() const
{
  Matrix<T, M, N> result(*this);

  // pivotColumn == pivotRow == diagonal
  for (std::size_t pivotColumn = 0; pivotColumn < M; ++pivotColumn)
  {
    // set the correct pivotRow
    std::size_t maxPivotRow = getMaxPivotRow(result, pivotColumn);
    std::swap(result.matrix[pivotColumn], result.matrix[maxPivotRow]);

    // Linearise the rows
    for (std::size_t currentRow = pivotColumn; currentRow < M; ++currentRow)
    {
      T lineariseFactor = getLineariseFactor(result, pivotColumn, currentRow);
      for (std::size_t column = 0; column < N; ++column)
      {
        result[currentRow][column] -= result[pivotColumn][column] * lineariseFactor;
      }
    }
  }
  return result;
}
/**
 *
 */

/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N> Matrix<T, M, N>::gaussJordan() const
{
  Matrix<T, M, N> result(*this);

  // pivotColumn == pivotRow == diagonal
  for (std::size_t pivotColumn = 0; pivotColumn < M; ++pivotColumn)
  {
    // set the correct pivotRow
    std::size_t maxPivotRow = getMaxPivotRow(result, pivotColumn);
    std::swap(result.matrix[pivotColumn], result.matrix[maxPivotRow]);

    // Linearise the rows
    for (std::size_t currentRow = 0; currentRow < M; ++currentRow)
    {
      T lineariseFactor = getLineariseFactor(result, pivotColumn, currentRow);
      for (std::size_t column = 0; column < N; ++column)
      {
        result[currentRow][column] -= result[pivotColumn][column] * lineariseFactor;
      }
    }
  }
  return result;
}
/**
 *

 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, 1> Matrix<T, M, N>::solve() const
{
  Matrix<T, M, 1> result;
  Matrix<T, M, N> gausJordan = *this;

  gausJordan = gausJordan.gaussJordan();

  for (size_t i = 0; i < M; ++i)
  {
    result.at(i, 0) = gausJordan.at(i, N - 1);
  }
  return result;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
Matrix<T, M, N> Matrix<T, M, N>::inverse() const
{
  Matrix<T, M, N> result(*this);
  Matrix<T, M, N> inverse = result.identity();

  // pivotColumn == pivotRow == diagonal
  for (std::size_t pivotColumn = 0; pivotColumn < M; ++pivotColumn)
  {
    // set the correct pivotRow
    std::size_t maxPivotRow = getMaxPivotRow(result, pivotColumn);
    std::swap(result.matrix[pivotColumn], result.matrix[maxPivotRow]);
    std::swap(inverse.matrix[pivotColumn], inverse.matrix[maxPivotRow]);

    // Linearise the rows
    for (std::size_t currentRow = 0; currentRow < M; ++currentRow)
    {
      T lineariseFactor = getLineariseFactor(result, pivotColumn, currentRow);
      for (std::size_t column = 0; column < N; ++column)
      {
        result[currentRow][column] -= result[pivotColumn][column] * lineariseFactor;
        inverse[currentRow][column] -= inverse[pivotColumn][column] * lineariseFactor;
      }
    }
  }
  return inverse;
}
/**
 *
 */
template <class T, std::size_t M, std::size_t N>
std::string Matrix<T, M, N>::to_string() const
{
  std::string result = "Matrix<" + std::to_string(N) + "," + std::to_string(M) + ">\n{\n";
  for (std::size_t i = 0; i < M; ++i)
  {
    for (std::size_t j = 0; j < N; ++j)
    {
      result += std::to_string(matrix[i][j]) + ",";
    }
    result += "\n";
  }
  result += "}";
  return result;
}
/**
 *
 */
template <typename T, const std::size_t N>
bool equals(const Matrix<T, 1, N>& lhs, const Matrix<T, 1, N>& rhs,
            const T aPrecision /*= std::numeric_limits<T>::epsilon()*/, const unsigned long aFactor /*= 1*/)
{
  for (size_t i = 0; i < N; ++i)
  {
    if (std::abs(lhs.at(0, i) - rhs.at(0, i)) > aFactor * aPrecision)
    {
      return false;
    }
  }
  return true;
}
/**
 *
 */
template <typename T, const std::size_t M>
bool equals(const Matrix<T, M, 1>& lhs, const Matrix<T, M, 1>& rhs,
            const T aPrecision /*= std::numeric_limits<T>::epsilon()*/, const unsigned long aFactor /*= 1*/)
{
  for (size_t i = 0; i < M; ++i)
  {
    if (std::abs(lhs.at(i, 0) - rhs.at(i, 0)) > (double)aFactor * aPrecision)
    {
      return false;
    }
  }
  return true;
}
/**
 *
 */
template <typename T, const std::size_t M, const std::size_t N>
bool equals(const Matrix<T, M, N>& lhs, const Matrix<T, M, N>& rhs,
            const T aPrecision /*= std::numeric_limits<T>::epsilon()*/, const unsigned long aFactor /*= 1*/)
{
  for (size_t i = 0; i < M; ++i)
  {
    for (size_t j = 0; j < N; ++j)
    {
      if (std::abs(lhs.at(i, j) - rhs.at(i, j)) > aFactor * aPrecision)
      {
        return false;
      }
    }
  }
  return true;
}
