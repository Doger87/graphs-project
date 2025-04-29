#include "Graph.h"
#include <stack>
#include <queue>
#include <unordered_set>
#include <limits>
#include <algorithm>
#include <functional>

// ——— getNumVertices ———
int Graph::getNumVertices() const {
    return static_cast<int>(adj.size());
}

// ——— getNumEdges ———
int Graph::getNumEdges() const {
    int total = 0;
    for (auto& kv : adj)
        total += static_cast<int>(kv.second.size());
    return total / 2;  // undirected edges stored twice
}

// ——— add ———
bool Graph::add(std::string start,
                std::string end,
                int edgeWeight) {
    if (edgeWeight < 0) return false;

    // ensure both vertices exist
    if (!adj.count(start)) adj[start] = {};
    if (!adj.count(end))   adj[end]   = {};

    // avoid duplicate edge
    for (auto& p : adj[start])
        if (p.first == end)
            return false;

    adj[start].push_back({end, edgeWeight});
    adj[end].push_back({start, edgeWeight});
    return true;
}

// ——— remove ———
bool Graph::remove(std::string start,
                   std::string end) {
    auto removeEdge = [&](const std::string& u,
                          const std::string& v) {
        auto it = adj.find(u);
        if (it == adj.end()) return false;
        auto& vec = it->second;
        for (auto iter = vec.begin(); iter != vec.end(); ++iter) {
            if (iter->first == v) {
                vec.erase(iter);
                return true;
            }
        }
        return false;
    };
    if (!removeEdge(start,end) || !removeEdge(end,start))
        return false;

    if (adj[start].empty()) adj.erase(start);
    if (adj[end].empty())   adj.erase(end);
    return true;
}

// ——— getEdgeWeight ———
int Graph::getEdgeWeight(std::string start,
                         std::string end) const {
    auto it = adj.find(start);
    if (it == adj.end()) return -1;
    for (auto& p : it->second)
        if (p.first == end)
            return p.second;
    return -1;
}

// ——— DFS traversal ———
void Graph::depthFirstTraversal(std::string start,
                                void visit(std::string&)) {
    std::unordered_set<std::string> visited;
    std::function<void(const std::string&)> dfs =
    [&](const std::string& u) {
        if (visited.count(u) == 0) {
            visited.insert(u);
            std::string label = u;
            visit(label);
            for (auto& nbr : adj[u])
                dfs(nbr.first);
        }
    };
    if (adj.count(start))
        dfs(start);
}

// ——— BFS traversal ———
void Graph::breadthFirstTraversal(std::string start,
                                  void visit(std::string&)) {
    if (!adj.count(start)) return;
    std::unordered_set<std::string> visited;
    std::queue<std::string> q;
    visited.insert(start);
    q.push(start);

    while (!q.empty()) {
        auto u = q.front(); q.pop();
        std::string label = u;
        visit(label);
        for (auto& nbr : adj[u]) {
            if (!visited.count(nbr.first)) {
                visited.insert(nbr.first);
                q.push(nbr.first);
            }
        }
    }
}

// ——— Dijkstra helper ———
std::pair<std::vector<std::string>,int>
Graph::dijkstra(const std::string& start,
                const std::string& end) const {
    using State = std::pair<int,std::string>;
    std::unordered_map<std::string,int> dist;
    std::unordered_map<std::string,std::string> prev;

    for (auto& kv : adj)
        dist[kv.first] = std::numeric_limits<int>::max();

    if (!dist.count(start) || !dist.count(end))
        return {{}, -1};

    dist[start] = 0;
    std::priority_queue<State,
        std::vector<State>,
        std::greater<State>> pq;
    pq.push({0,start});

    while (!pq.empty()) {
        auto [d,u] = pq.top(); pq.pop();
        if (u == end) break;
        if (d > dist[u]) continue;

        for (auto& nbr : adj.at(u)) {
            int nd = d + nbr.second;
            if (nd < dist[nbr.first]) {
                dist[nbr.first] = nd;
                prev[nbr.first] = u;
                pq.push({nd,nbr.first});
            }
        }
    }

    if (dist[end] == std::numeric_limits<int>::max())
        return {{}, -1};

    std::vector<std::string> path;
    for (auto at = end; at != start; at = prev[at])
        path.push_back(at);
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return {path, dist[end]};
}

//shortestPath
std::vector<std::string>
Graph::shortestPath(std::string start,
                    std::string end) const {
    return dijkstra(start,end).first;
}

// kShortestPath (Yen’s algorithm)
std::vector<std::vector<std::string>>
Graph::kShortestPath(std::string start,
                     std::string end,
                     int k) const
{
    std::vector<std::vector<std::string>> A;
    std::vector<std::pair<std::vector<std::string>,int>> B;

    // 1) First shortest path
    auto [firstPath, firstCost] = dijkstra(start, end);
    if (firstPath.empty())
        return A;
    A.push_back(firstPath);

    // 2) Up to k-1 more paths
    for (int i = 1; i < k; ++i) {
        const auto &prevPath = A[i-1];
        for (int j = 0; j + 1 < (int)prevPath.size(); ++j) {
            const std::string &spurNode = prevPath[j];
            std::vector<std::string> rootPath(prevPath.begin(),
                                              prevPath.begin() + j + 1);

            Graph gCopy = *this;
            // a) remove edges that share rootPath
            for (const auto &pathA : A) {
                if ((int)pathA.size() > j &&
                    std::equal(rootPath.begin(), rootPath.end(), pathA.begin())) {
                    gCopy.remove(pathA[j], pathA[j+1]);
                }
            }
            // b) remove rootPath nodes except spurNode
            for (const auto &node : rootPath) {
                if (node == spurNode) continue;
                gCopy.adj.erase(node);
                for (auto &kv : gCopy.adj) {
                    auto &nbrs = kv.second;
                    nbrs.erase(std::remove_if(nbrs.begin(), nbrs.end(),
                        [&](auto &p){ return p.first == node; }), nbrs.end());
                }
            }

            // c) spur path from spurNode
            auto [spurPath, spurCost] = gCopy.dijkstra(spurNode, end);
            if (spurPath.empty()) continue;

            // d) total path
            std::vector<std::string> totalPath = rootPath;
            totalPath.insert(totalPath.end(), spurPath.begin() + 1, spurPath.end());

            // e) total cost
            int rootCost = 0;
            for (int t = 1; t < (int)rootPath.size(); ++t)
                rootCost += getEdgeWeight(rootPath[t-1], rootPath[t]);
            int totalCost = rootCost + spurCost;

            // f) add to B if unique
            bool exists = false;
            for (auto &cand : B) {
                if (cand.first == totalPath) { exists = true; break; }
            }
            if (!exists) B.push_back({totalPath, totalCost});
        }
        if (B.empty()) break;
        std::sort(B.begin(), B.end(),
                  [](auto &L, auto &R){ return L.second < R.second; });
        A.push_back(B.front().first);
        B.erase(B.begin());
    }
    return A;
}
