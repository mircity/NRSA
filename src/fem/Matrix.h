#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <ostream>
#include <stdexcept>
#include <vector>

namespace nrsa::fem {

// A small, dense, column-major-free (row-major, the ordinary convention)
// matrix of doubles. This is deliberately NOT meant to scale to a whole
// building's global stiffness matrix — that is exactly what
// SparseMatrix exists for. Matrix is for the small, fixed-size, dense
// blocks a single finite element deals with: a 12x12 frame-element
// stiffness matrix, a 3x3 rotation, a 24x24 shell-element block — the
// kind of matrix a direct (non-sparse) Gaussian-elimination solve is
// entirely appropriate for.
class Matrix {
public:
    Matrix() = default;
    Matrix(std::size_t rows, std::size_t cols, double fill = 0.0)
        : rows_(rows), cols_(cols), data_(rows * cols, fill) {}

    static Matrix identity(std::size_t n) {
        Matrix m(n, n, 0.0);
        for (std::size_t i = 0; i < n; ++i) m(i, i) = 1.0;
        return m;
    }
    static Matrix zeros(std::size_t rows, std::size_t cols) { return Matrix(rows, cols, 0.0); }

    std::size_t rows() const { return rows_; }
    std::size_t cols() const { return cols_; }

    double& operator()(std::size_t r, std::size_t c) {
        assert(r < rows_ && c < cols_);
        return data_[r * cols_ + c];
    }
    double operator()(std::size_t r, std::size_t c) const {
        assert(r < rows_ && c < cols_);
        return data_[r * cols_ + c];
    }

    Matrix operator+(const Matrix& o) const {
        requireSameShape(o, "operator+");
        Matrix out(rows_, cols_);
        for (std::size_t i = 0; i < data_.size(); ++i) out.data_[i] = data_[i] + o.data_[i];
        return out;
    }
    Matrix operator-(const Matrix& o) const {
        requireSameShape(o, "operator-");
        Matrix out(rows_, cols_);
        for (std::size_t i = 0; i < data_.size(); ++i) out.data_[i] = data_[i] - o.data_[i];
        return out;
    }
    Matrix operator*(double scalar) const {
        Matrix out(rows_, cols_);
        for (std::size_t i = 0; i < data_.size(); ++i) out.data_[i] = data_[i] * scalar;
        return out;
    }
    // Matrix-matrix product — plain O(n^3) triple loop. Fine for the
    // element-sized (<= a few dozen rows/cols) matrices this class is
    // meant for; NOT the multiplication path a global assembly should
    // ever go through (that's SparseMatrix's job).
    Matrix operator*(const Matrix& o) const {
        if (cols_ != o.rows_) {
            throw std::invalid_argument(
                "Matrix::operator*: shape mismatch (" + std::to_string(rows_) + "x" +
                std::to_string(cols_) + ") * (" + std::to_string(o.rows_) + "x" +
                std::to_string(o.cols_) + ")");
        }
        Matrix out(rows_, o.cols_, 0.0);
        for (std::size_t i = 0; i < rows_; ++i) {
            for (std::size_t k = 0; k < cols_; ++k) {
                double aik = (*this)(i, k);
                if (aik == 0.0) continue;
                for (std::size_t j = 0; j < o.cols_; ++j) {
                    out(i, j) += aik * o(k, j);
                }
            }
        }
        return out;
    }

    Matrix transpose() const {
        Matrix out(cols_, rows_);
        for (std::size_t i = 0; i < rows_; ++i)
            for (std::size_t j = 0; j < cols_; ++j)
                out(j, i) = (*this)(i, j);
        return out;
    }

    // Symmetry check within a relative tolerance — every element
    // stiffness matrix this library produces should be symmetric (they
    // all come from a self-adjoint energy formulation), so this is a
    // cheap, useful sanity assertion for tests, not something used in
    // the hot assembly path.
    bool isSymmetric(double relTol = 1e-9) const {
        if (rows_ != cols_) return false;
        for (std::size_t i = 0; i < rows_; ++i) {
            for (std::size_t j = i + 1; j < cols_; ++j) {
                double a = (*this)(i, j), b = (*this)(j, i);
                double scale = std::max({1.0, std::abs(a), std::abs(b)});
                if (std::abs(a - b) > relTol * scale) return false;
            }
        }
        return true;
    }

    double frobeniusNorm() const {
        double s = 0.0;
        for (double v : data_) s += v * v;
        return std::sqrt(s);
    }

    // Gaussian elimination with partial pivoting, solving A*x = b for a
    // square A. Throws if A is singular (or numerically indistinguishable
    // from singular) to the tolerance given — a caller solving an
    // unrestrained/mechanism model (a common modeling mistake: a node
    // with no supports at all) needs that thrown, not a silently
    // garbage answer.
    static std::vector<double> solve(Matrix A, std::vector<double> b, double pivotTol = 1e-12) {
        if (A.rows_ != A.cols_) {
            throw std::invalid_argument("Matrix::solve: A must be square");
        }
        std::size_t n = A.rows_;
        if (b.size() != n) {
            throw std::invalid_argument("Matrix::solve: b size does not match A");
        }
        for (std::size_t col = 0; col < n; ++col) {
            // Partial pivot: find the largest-magnitude entry at or
            // below the diagonal in this column.
            std::size_t pivotRow = col;
            double best = std::abs(A(col, col));
            for (std::size_t r = col + 1; r < n; ++r) {
                double v = std::abs(A(r, col));
                if (v > best) { best = v; pivotRow = r; }
            }
            if (best < pivotTol) {
                throw std::runtime_error(
                    "Matrix::solve: singular (or unrestrained-mechanism) system at column " +
                    std::to_string(col) + " — check that every DOF has either a stiffness path "
                    "or a restraint.");
            }
            if (pivotRow != col) {
                for (std::size_t c = 0; c < n; ++c) std::swap(A(col, c), A(pivotRow, c));
                std::swap(b[col], b[pivotRow]);
            }
            for (std::size_t r = col + 1; r < n; ++r) {
                double factor = A(r, col) / A(col, col);
                if (factor == 0.0) continue;
                for (std::size_t c = col; c < n; ++c) A(r, c) -= factor * A(col, c);
                b[r] -= factor * b[col];
            }
        }
        // Back-substitution.
        std::vector<double> x(n, 0.0);
        for (std::size_t ii = n; ii-- > 0;) {
            double sum = b[ii];
            for (std::size_t c = ii + 1; c < n; ++c) sum -= A(ii, c) * x[c];
            x[ii] = sum / A(ii, ii);
        }
        return x;
    }

    // Cholesky factorization A = L*L^T for a symmetric POSITIVE-DEFINITE
    // A, returning the lower-triangular factor L (upper triangle left
    // zero). This is the standard, cheaper-than-Gaussian-elimination
    // route for exactly the SPD systems this engine's elastic stiffness
    // matrices are (a properly restrained linear-elastic model), and is
    // used by analysis::BucklingAnalysis to reduce the generalized
    // eigenproblem K*phi = -lambda*Kg*phi to a standard symmetric one
    // via the classic L^-1*Kg*L^-T congruence transform -- see that
    // class for why.
    //
    // Throws std::runtime_error at the first non-positive pivot
    // (A(i,i) minus the accumulated sum drops to <= tolerance), the
    // same "say why, don't return garbage" pattern as solve() above --
    // in practice this means A was not actually positive-definite (an
    // unrestrained mechanism, or a caller passing a matrix that isn't
    // truly SPD), not a numerical fluke worth silently working around.
    Matrix choleskyLower(double pivotTol = 1e-12) const {
        if (rows_ != cols_) {
            throw std::invalid_argument("Matrix::choleskyLower: matrix must be square");
        }
        std::size_t n = rows_;
        Matrix L(n, n, 0.0);
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = 0; j <= i; ++j) {
                double sum = (*this)(i, j);
                for (std::size_t k = 0; k < j; ++k) sum -= L(i, k) * L(j, k);
                if (i == j) {
                    if (sum <= pivotTol) {
                        throw std::runtime_error(
                            "Matrix::choleskyLower: matrix is not positive-definite (non-positive "
                            "pivot at row " +
                            std::to_string(i) +
                            ") -- check that the model is fully and correctly restrained.");
                    }
                    L(i, j) = std::sqrt(sum);
                } else {
                    L(i, j) = sum / L(j, j);
                }
            }
        }
        return L;
    }

    // Inverse of a lower-triangular matrix via forward substitution,
    // solving L*X = I one column of I at a time. Undefined (will divide
    // by whatever is on the diagonal) if L is not actually lower-
    // triangular with a nonzero diagonal -- callers only ever pass this
    // the L from choleskyLower(), whose diagonal is guaranteed strictly
    // positive by that method's own check.
    Matrix inverseLowerTriangular() const {
        if (rows_ != cols_) {
            throw std::invalid_argument("Matrix::inverseLowerTriangular: matrix must be square");
        }
        std::size_t n = rows_;
        Matrix inv(n, n, 0.0);
        for (std::size_t col = 0; col < n; ++col) {
            for (std::size_t i = col; i < n; ++i) {
                double sum = (i == col) ? 1.0 : 0.0;
                for (std::size_t k = col; k < i; ++k) sum -= (*this)(i, k) * inv(k, col);
                inv(i, col) = sum / (*this)(i, i);
            }
        }
        return inv;
    }

    friend std::ostream& operator<<(std::ostream& os, const Matrix& m) {
        for (std::size_t i = 0; i < m.rows_; ++i) {
            for (std::size_t j = 0; j < m.cols_; ++j) os << m(i, j) << (j + 1 < m.cols_ ? " " : "");
            os << "\n";
        }
        return os;
    }

private:
    void requireSameShape(const Matrix& o, const char* op) const {
        if (rows_ != o.rows_ || cols_ != o.cols_) {
            throw std::invalid_argument(
                std::string("Matrix::") + op + ": shape mismatch (" + std::to_string(rows_) + "x" +
                std::to_string(cols_) + ") vs (" + std::to_string(o.rows_) + "x" +
                std::to_string(o.cols_) + ")");
        }
    }

    std::size_t rows_ = 0, cols_ = 0;
    std::vector<double> data_;
};

// A thin, dedicated vector type for the common case of a 1-column
// quantity (a load vector, a displacement vector) where naming it
// "Matrix(n,1)" everywhere would be needlessly verbose at every call
// site. Deliberately minimal — anything needing real matrix machinery
// on a vector (e.g. an outer product) goes through Matrix directly.
class Vector {
public:
    Vector() = default;
    explicit Vector(std::size_t n, double fill = 0.0) : data_(n, fill) {}
    Vector(std::initializer_list<double> vals) : data_(vals) {}

    std::size_t size() const { return data_.size(); }
    double& operator()(std::size_t i) { assert(i < data_.size()); return data_[i]; }
    double operator()(std::size_t i) const { assert(i < data_.size()); return data_[i]; }
    double& operator[](std::size_t i) { return data_[i]; }
    double operator[](std::size_t i) const { return data_[i]; }

    const std::vector<double>& data() const { return data_; }
    std::vector<double>& data() { return data_; }

    Vector operator+(const Vector& o) const {
        requireSameSize(o);
        Vector out(data_.size());
        for (std::size_t i = 0; i < data_.size(); ++i) out[i] = data_[i] + o[i];
        return out;
    }
    Vector operator-(const Vector& o) const {
        requireSameSize(o);
        Vector out(data_.size());
        for (std::size_t i = 0; i < data_.size(); ++i) out[i] = data_[i] - o[i];
        return out;
    }
    Vector operator*(double scalar) const {
        Vector out(data_.size());
        for (std::size_t i = 0; i < data_.size(); ++i) out[i] = data_[i] * scalar;
        return out;
    }
    double dot(const Vector& o) const {
        requireSameSize(o);
        double s = 0.0;
        for (std::size_t i = 0; i < data_.size(); ++i) s += data_[i] * o[i];
        return s;
    }
    double norm() const { return std::sqrt(dot(*this)); }

private:
    void requireSameSize(const Vector& o) const {
        if (data_.size() != o.data_.size()) {
            throw std::invalid_argument("Vector: size mismatch (" + std::to_string(data_.size()) +
                                         " vs " + std::to_string(o.data_.size()) + ")");
        }
    }
    std::vector<double> data_;
};

inline Vector operator*(const Matrix& m, const Vector& v) {
    if (m.cols() != v.size()) {
        throw std::invalid_argument("Matrix*Vector: shape mismatch (" + std::to_string(m.rows()) +
                                     "x" + std::to_string(m.cols()) + ") * (" +
                                     std::to_string(v.size()) + ")");
    }
    Vector out(m.rows(), 0.0);
    for (std::size_t i = 0; i < m.rows(); ++i) {
        double s = 0.0;
        for (std::size_t j = 0; j < m.cols(); ++j) s += m(i, j) * v(j);
        out(i) = s;
    }
    return out;
}

}  // namespace nrsa::fem
