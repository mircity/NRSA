#pragma once

#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "fem/Matrix.h"

namespace nrsa::fem {

// A symmetric sparse matrix, stored as one hash map per row holding
// only that row's UPPER-triangle entries (column index >= row index).
// This is exactly what a structural stiffness matrix is — always
// symmetric for the linear-elastic elements this library builds — so
// storing (and later multiplying) only half the matrix is a correct,
// not just convenient, optimization, not an approximation.
//
// Row-of-maps (rather than a compressed format like CSR) is chosen
// deliberately for ASSEMBLY: a global stiffness matrix is built by
// thousands of small, randomly-ordered += operations (one per element,
// scattered into whatever global DOF indices that element happens to
// touch), and a hash map handles that access pattern in O(1) amortized
// per entry. A CSR conversion (or a proper fill-reducing sparse
// factorization) is a natural next addition if a direct sparse solve
// is later wanted instead of Solver's iterative approach — see the
// README roadmap note on why iterative was chosen first.
class SparseMatrix {
public:
    explicit SparseMatrix(std::size_t n) : n_(n), rows_(n) {}

    std::size_t size() const { return n_; }

    // Adds `value` to the symmetric matrix entry (i, j) [== (j, i)].
    // Callers assembling a symmetric local element block must only call
    // this ONCE per physical (i, j) pair (e.g. iterate the local
    // element matrix's own upper triangle, i <= j, not the full square)
    // — calling it for both (i, j) and (j, i) from the same local
    // symmetric entry would double-count that entry, since this method
    // already represents both directions from a single call.
    void add(std::size_t i, std::size_t j, double value) {
        if (i >= n_ || j >= n_) {
            throw std::out_of_range("SparseMatrix::add: index out of range");
        }
        if (i > j) std::swap(i, j);
        rows_[i][j] += value;
    }

    double get(std::size_t i, std::size_t j) const {
        if (i > j) std::swap(i, j);
        auto it = rows_[i].find(j);
        return it == rows_[i].end() ? 0.0 : it->second;
    }

    double diagonal(std::size_t i) const { return get(i, i); }

    // Matrix-vector product y = K*x, using both triangles even though
    // only the upper one is stored — each stored (i, j) with i != j
    // contributes to both y[i] (via x[j]) and y[j] (via x[i]).
    Vector multiply(const Vector& x) const {
        if (x.size() != n_) {
            throw std::invalid_argument("SparseMatrix::multiply: vector size mismatch");
        }
        Vector y(n_, 0.0);
        for (std::size_t i = 0; i < n_; ++i) {
            for (const auto& [j, val] : rows_[i]) {
                y(i) += val * x(j);
                if (j != i) y(j) += val * x(i);
            }
        }
        return y;
    }

    // Number of DISTINCT stored entries (upper triangle only, i.e. the
    // true degrees of freedom of sparsity — not doubled for the
    // symmetric mirror multiply() reconstructs on the fly).
    std::size_t nnz() const {
        std::size_t total = 0;
        for (const auto& row : rows_) total += row.size();
        return total;
    }

    // Returns a new SparseMatrix equal to this one scaled by `factor`
    // (a deep copy of the sparsity pattern with every stored value
    // multiplied by factor) — used by time-integration schemes (e.g.
    // analysis::DynamicAnalysis's Newmark-beta effective stiffness,
    // K_eff = c1*K + c2*diag(M)) that need to combine a scaled copy of
    // an already-assembled matrix with additional diagonal terms,
    // without re-deriving element-level assembly from scratch.
    SparseMatrix scaledCopy(double factor) const {
        SparseMatrix out(n_);
        for (std::size_t i = 0; i < n_; ++i) {
            for (const auto& [j, val] : rows_[i]) out.add(i, j, val * factor);
        }
        return out;
    }

    // Fraction of the full n x n matrix that is actually nonzero —
    // the number a dense O(n^2)-memory/O(n^3)-solve approach would have
    // no way to avoid paying for, and the whole reason SparseMatrix
    // exists. Reported as (upper-triangle nnz) / (upper-triangle size)
    // so it reads as "how full is the matrix", not skewed by the
    // symmetric-storage convention.
    double density() const {
        std::size_t upperSize = n_ * (n_ + 1) / 2;
        return upperSize == 0 ? 0.0 : static_cast<double>(nnz()) / static_cast<double>(upperSize);
    }

private:
    std::size_t n_;
    std::vector<std::unordered_map<std::size_t, double>> rows_;
};

}  // namespace nrsa::fem
