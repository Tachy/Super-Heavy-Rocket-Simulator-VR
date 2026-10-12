#include "starsim/optim/minimum_degree.hpp"

#include <algorithm>
#include <iterator>
#include <set>
#include <stdexcept>
#include <utility>

namespace starsim::optim {

std::vector<int> minimum_degree_ordering(const CscMatrix& pattern) {
    const int n = pattern.cols();
    if (pattern.rows() != n) throw std::invalid_argument("minimum_degree_ordering: matrix must be square");

    std::vector<std::vector<int>> adjacency(static_cast<std::size_t>(n));
    for (int j = 0; j < n; ++j) {
        for (int p = pattern.col_start()[j]; p < pattern.col_start()[j + 1]; ++p) {
            const int i = pattern.row_index()[p];
            if (i == j) continue;
            adjacency[i].push_back(j);
            adjacency[j].push_back(i);
        }
    }
    std::set<std::pair<int, int>> queue;  // (degree, node): begin() is the next pivot
    for (int i = 0; i < n; ++i) {
        auto& list = adjacency[i];
        std::sort(list.begin(), list.end());
        list.erase(std::unique(list.begin(), list.end()), list.end());
        queue.insert({static_cast<int>(list.size()), i});
    }

    std::vector<int> perm;
    perm.reserve(static_cast<std::size_t>(n));
    std::vector<int> merged;
    while (!queue.empty()) {
        const int pivot = queue.begin()->second;
        queue.erase(queue.begin());
        perm.push_back(pivot);
        const std::vector<int> neighbours = std::move(adjacency[pivot]);
        adjacency[pivot].clear();
        // The neighbours become a clique; the pivot leaves the graph.
        for (const int u : neighbours) {
            auto& list = adjacency[u];
            queue.erase({static_cast<int>(list.size()), u});
            merged.clear();
            std::set_union(list.begin(), list.end(), neighbours.begin(), neighbours.end(), std::back_inserter(merged));
            list.clear();
            for (const int v : merged) {
                if (v != u && v != pivot) list.push_back(v);
            }
            queue.insert({static_cast<int>(list.size()), u});
        }
    }
    return perm;
}

}  // namespace starsim::optim
