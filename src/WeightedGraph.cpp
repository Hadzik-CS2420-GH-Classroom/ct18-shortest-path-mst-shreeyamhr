// =============================================================================
// CT18: Shortest Path & MST — Implementation
// =============================================================================

#include "WeightedGraph.h"
#include <iostream>
#include <tuple>
#include <limits>
#include <queue>
#include <vector>
#include <functional>
#include <unordered_set>

// =============================================================================
// 1. Constructor
// =============================================================================

WeightedGraph::WeightedGraph() {}

// =============================================================================
// 2. add_vertex / add_edge
// =============================================================================
//
// ? SEE DIAGRAM: images/cpp_diagrams.md #1 — add_edge weighted undirected edge
//
// ! DISCUSSION: add_edge is the only change from CT17's Graph.
//   - CT17 pushed just the neighbor string into a vector<string>
//   - CT18 pushes an Edge struct carrying BOTH the destination and the cost
//   - everything else (add_vertex, has_vertex, counts) is identical
//

// Provided — identical to CT17's Graph::add_vertex.
void WeightedGraph::add_vertex(const std::string& vertex) {
    if (adj_list_.find(vertex) == adj_list_.end()) {
        adj_list_[vertex] = {};
    }
}

void WeightedGraph::add_edge(const std::string& from, const std::string& to, int weight) {
    // TODO: ensure both vertices exist, then push Edge{to, weight} into
    //       from's list AND Edge{from, weight} into to's list (undirected)
    add_vertex(from);
    add_vertex(to);

    adj_list_[from].push_back({ to, weight });
    adj_list_[to].push_back({ from, weight });
}

// =============================================================================
// 3. Queries
// =============================================================================

// Provided — identical to CT17's Graph::has_vertex.
bool WeightedGraph::has_vertex(const std::string& vertex) const {
    return adj_list_.count(vertex) > 0;
}

// Provided — identical to CT17's Graph::vertex_count.
int WeightedGraph::vertex_count() const {
    return static_cast<int>(adj_list_.size());
}

// Provided — identical to CT17's Graph::edge_count.
int WeightedGraph::edge_count() const {
    int total = 0;
    for (const auto& [vertex, edges] : adj_list_) {
        total += static_cast<int>(edges.size());
    }
    return total / 2;
}

// =============================================================================
// 4. Dijkstra's Algorithm
// =============================================================================
//
// ? SEE DIAGRAM: images/cpp_diagrams.md #2  — dijkstra implementation
// ? SEE DIAGRAM: images/cpp_diagrams.md #2a — stale-entry skip breakdown
// ? SEE DIAGRAM: images/cpp_diagrams.md #2b — code execution trace
//
// ! DISCUSSION: The relaxation step.
//   - for each neighbor v of the current vertex u:
//     if dist[u] + edge_weight < dist[v]:
//       update dist[v] and push to priority queue
//   - "relax" means: we found a shorter path, so update our records
//
// ! DISCUSSION: Priority queue stores {distance, vertex}.
//   - we use std::greater so the smallest distance is at the top (min-heap)
//   - when we pop a vertex with d > dist[u], it's a stale entry — skip it
//   - stale entries happen because std::priority_queue has no decrease-key
//

std::unordered_map<std::string, int>
WeightedGraph::dijkstra(const std::string& source) const {
    std::unordered_map<std::string, int> dist;
    // TODO: implement Dijkstra's — step numbers match the SVG rows
    //
    // 1. Declare dist (already done above)
    // 2. Initialize every vertex's dist to std::numeric_limits<int>::max()
    for (const auto& [vertex, edges] : adj_list_) {
        dist[vertex] = std::numeric_limits<int>::max();
    }

    // 3. Guard: if (!has_vertex(source)) return dist;
    if (!has_vertex(source)) return dist;

    // 4. Seed the source: dist[source] = 0;
    dist[source] = 0;

    // 5. Type alias: using Pair = std::pair<int, std::string>;
    using Pair = std::pair<int, std::string>;

    // 6. Min-heap declaration:
    //      std::priority_queue<Pair, std::vector<Pair>,
    //                          std::greater<Pair>> min_heap;
    //    (std::greater flips the default max-heap to a min-heap)
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> min_heap;

    // 7. Seed the min-heap: min_heap.push({0, source})
    min_heap.push({ 0, source });

    // 8. Main loop — while (!min_heap.empty()):
    while (!min_heap.empty()) {
        //      pop (d, u) with min_heap.top() / min_heap.pop()
        auto [d, u] = min_heap.top();
        min_heap.pop();

        // 9. Stale-entry skip: if (d > dist[u]) continue;
        if (d > dist[u]) continue;

        // 10. Relax every edge in adj_list_.at(u): compute new_dist = dist[u]
        //     + edge.weight; if new_dist < dist[edge.to], update dist[edge.to]
        //     and min_heap.push({new_dist, edge.to})
        for (const auto& edge : adj_list_.at(u)) {
            int new_dist = dist[u] + edge.weight;
            if (new_dist < dist[edge.to]) {
                dist[edge.to] = new_dist;
                min_heap.push({ new_dist, edge.to });
            }
        }
    }

    return dist;
}

// =============================================================================
// 5. Prim's Algorithm — MST
// =============================================================================
//
// ? SEE DIAGRAM: images/cpp_diagrams.md #3  — prims implementation
// ? SEE DIAGRAM: images/cpp_diagrams.md #3b — code execution trace
//
// ! DISCUSSION: Prim's grows the MST one edge at a time.
//   - start with one vertex in the MST
//   - repeatedly add the cheapest edge that connects an MST vertex to a non-MST vertex
//   - stop when all vertices are in the MST (V-1 edges)
//
// ! DISCUSSION: Tuple is (weight, from, to) — 3 fields, not 2.
//   - weight first so the min-heap orders by it
//   - we need 'from' as well as 'to' to record the MST edge in the output
//   - skip rule: if 'to' is already in the MST, adding it would create a cycle
//

// Provided for reference — CCD 2.12 (Prim's) is lecture-only.
// Walk through this in class using prims_impl_setup.svg and
// prims_impl_loop.svg, but students don't type it.
std::pair<std::vector<std::tuple<std::string, std::string, int>>, int>
WeightedGraph::prims_mst(const std::string& start) const {
    std::vector<std::tuple<std::string, std::string, int>> mst_edges;
    int total_weight = 0;

    if (!has_vertex(start)) return { mst_edges, total_weight };

    // 1. Start the MST with 'start'
    std::unordered_set<std::string> in_mst;
    in_mst.insert(start);

    // 2. Type alias for (weight, from, to) tuples
    using EdgeTuple = std::tuple<int, std::string, std::string>;

    // 3. Build a min-heap: std::greater flips default max-heap to min-heap
    std::priority_queue<EdgeTuple, std::vector<EdgeTuple>, std::greater<EdgeTuple>> min_heap;

    // 4. Seed the frontier with every edge leaving 'start'
    for (const auto& edge : adj_list_.at(start)) {
        min_heap.push({ edge.weight, start, edge.to });
    }

    // 5. Cache V so we can stop once the MST covers every vertex
    const int V = vertex_count();

    // 6. Main loop: pop the cheapest crossing edge
    while (!min_heap.empty() && static_cast<int>(in_mst.size()) < V) {
        auto [w, from, to] = min_heap.top();
        min_heap.pop();

        // 7. Skip edges that would form a cycle
        if (in_mst.count(to)) continue;

        // 8. Accept the edge (record + total + mark in MST)
        mst_edges.push_back({ from, to, w });
        total_weight += w;
        in_mst.insert(to);

        // 9. Grow the frontier with every crossing edge from 'to'
        for (const auto& edge : adj_list_.at(to)) {
            if (!in_mst.count(edge.to)) {
                min_heap.push({ edge.weight, to, edge.to });
            }
        }
    }

    return { mst_edges, total_weight };
}

// =============================================================================
// 6. print
// =============================================================================

void WeightedGraph::print() const {
    for (const auto& [vertex, edges] : adj_list_) {
        std::cout << vertex << ": ";
        for (size_t i = 0; i < edges.size(); ++i) {
            if (i > 0) std::cout << ", ";
            std::cout << edges[i].to << "(" << edges[i].weight << ")";
        }
        std::cout << std::endl;
    }
}