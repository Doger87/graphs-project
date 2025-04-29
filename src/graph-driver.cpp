#include <iostream>
#include <fstream>
#include <algorithm>
#include <sstream>
#include <string>

#include "Graph.h"

int main() {
	 Graph g;
    std::ifstream fin("edges.txt");
    if (!fin.is_open()) {
        std::cerr << "ERROR: could not open edges.txt\n";
        return 1;
    }

    int count = 0;
    std::string line;
    while (std::getline(fin, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string a, b, wStr;

        // since my files had commas I had to come up with this to split on commas
        if (!std::getline(ss, a, ',')) continue;
        if (!std::getline(ss, b, ',')) continue;
        if (!std::getline(ss, wStr)) continue;

        int w = std::stoi(wStr);
        if (g.add(a, b, w)) ++count;
    }

    std::cout << "Loaded " << count
              << " edges into graph.  "
              << "Vertices: " << g.getNumVertices()
              << ", Edges: " << g.getNumEdges()
              << "\n";

    // 2) it gives the Prompt
    std::string start, end;
    int k;
    std::cout << "Starting airport: ";
    std::cin  >> start;
    std::cout << "Ending airport:   ";
    std::cin  >> end;
    std::cout << "Number of paths:  ";
    std::cin  >> k;

    // 3) Gets the k shortest
    auto paths = g.kShortestPath(start, end, k);
    if (paths.empty()) {
        std::cout << "No path found from "
                  << start << " to " << end << "\n";
        return 0;
    }
    // 4) Computes and sort by distance
    std::vector<std::pair<std::vector<std::string>,int>> out;
    for (auto& p : paths) {
        int dist = 0;
        for (size_t i = 1; i < p.size(); ++i)
            dist += g.getEdgeWeight(p[i-1], p[i]);
        out.push_back({p, dist});
    }
    std::sort(out.begin(), out.end(),
              [](auto &A, auto &B){ return A.second < B.second; });

    // 5) Finally prints
    for (auto &pr : out) {
        for (size_t i = 0; i < pr.first.size(); ++i) {
            std::cout << pr.first[i]
                      << (i+1<pr.first.size()? "-" : "");
        }
        std::cout << ": " << pr.second << "\n";
    }

    return 0;
}
