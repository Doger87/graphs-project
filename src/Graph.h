#ifndef GRAPH_H
#define GRAPH_H

#include "GraphInterface.h"
#include <string>
#include <vector>
#include <unordered_map>

/** A simple undirected, weighted graph using an adjacency list.
 @file Graph.h */
class Graph : public GraphInterface<std::string>
{
public:
    Graph() = default;
    ~Graph() = default;

    /** GraphInterface::getNumVertices */
    int getNumVertices() const override;

    /** GraphInterface::getNumEdges */
    int getNumEdges() const override;

    /** GraphInterface::add */
    bool add(std::string start,
             std::string end,
             int edgeWeight) override;

    /** GraphInterface::remove */
    bool remove(std::string start,
                std::string end) override;

    /** GraphInterface::getEdgeWeight */
    int getEdgeWeight(std::string start,
                      std::string end) const override;

    /** GraphInterface::depthFirstTraversal */
    void depthFirstTraversal(std::string start,
                             void visit(std::string&)) override;

    /** GraphInterface::breadthFirstTraversal */
    void breadthFirstTraversal(std::string start,
                               void visit(std::string&)) override;

    /** Returns the shortest path (labels) between start/end via Dijkstra. */
    std::vector<std::string> shortestPath(std::string start,
                                          std::string end) const;

    /** Returns up to k shortest paths between start/end via Yen’s algorithm. */
    std::vector<std::vector<std::string>>
      kShortestPath(std::string start,
                    std::string end,
                    int k) const;

private:
    // adjacency list: vertex → list of (neighbor, weight)
    std::unordered_map<std::string,
                       std::vector<std::pair<std::string,int>>> adj;

    // Dijkstra helper: returns {path, totalDistance}
    std::pair<std::vector<std::string>,int>
    dijkstra(const std::string& start,
             const std::string& end) const;
};

#endif // GRAPH_H
