#pragma once

#include <vector>

#include "starsim/optim/sparse_matrix.hpp"

namespace starsim::optim {

// Fill-reducing ordering of a symmetric sparsity pattern: minimum degree on the explicit elimination
// graph (eliminating a node joins its neighbours into a clique), ties broken by the lowest index so the
// result is deterministic. The pattern may hold one or both triangles; values and the diagonal are ignored.
// Returns perm with perm[k] = original index of the k-th pivot.
std::vector<int> minimum_degree_ordering(const CscMatrix& pattern);

}  // namespace starsim::optim
