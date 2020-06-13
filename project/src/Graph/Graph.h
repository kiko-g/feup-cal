#pragma once
#ifndef GRAPH_H_
#define GRAPH_H_

#include <unordered_map>
#include <queue>
#include <list>
#include <limits>
#include <algorithm>
#include <cmath>
#include <chrono>
#include "MutablePriorityQueue.h"
#include "Vertex.h"
using namespace std;
using namespace std::chrono;

class Graph {
private:
    int offsetX;    // for Graph Viewer
    int offsetY;    // for Graph Viewer

    vector<Vertex*> vertexSet;
    unordered_map<int, Vertex*> vertexIndexes;    //search for id and return vertex (much faster)

    vector<vector<double>> minDistance;       // used for floyd Warshall algorithm
    vector<vector<Vertex*>> next;             // used for floyd Warshall algorithm
    void dfsVisit(Vertex *origin) const;      // pre processing

    const static int infinite = 99999999;

public:
    Vertex* findVertex(const int &id) const;
    bool addVertex(const int &id, const int &x, const int &y);
    void addPointOfInterest(Vertex* vertex);
    bool addEdge(const int &id, const int &origin, const int &dest);

    void setOffsetX(int x);
    void setOffsetY(int y);
    int getOffsetX();
    int getOffsetY();

    int getNumVertex() const;
    vector<Vertex*> getVertexSet() const;

    // pre processing
    bool preProcess(int origin);

    // dijkstra
    Vertex* dijkstraInitCentral(const int origin);
    Vertex* dijkstraInit(const int origin);
    Vertex* dijkstraBackwardsInit(const int dest);
    bool dijkstraOriginal(const int origin);
    bool dijkstra(const int origin, const int dest, unordered_set<int> &processedEdges);
    int getPathTo(const int dest, vector<Edge> &edges) const;
    int getPathFromCentralTo(const int dest, vector<Edge> &edges) const;

    // dijkstra related
    double heuristicDistance(Vertex *origin, Vertex *dest);
    bool dijkstraOrientedSearch(const int origin, const int dest, unordered_set<int> &processedEdges) ;
    bool dijkstraBidirectional(const int origin, const int dest, unordered_set<int> &processedEdges, unordered_set<int> &processedEdgesInv);

    // all pairs
    void floydWarshallShortestPath();
    vector<int> getfloydWarshallPath(const int origin, const int dest) const;
};

/**************** Pre processing ************/
/**
 * @brief Starts from the origin vertex and runs through the graph using depth first search
 * @param origin - Pointer to where the vertex starts
 */
void Graph::dfsVisit(Vertex *origin) const {
    // Set the vertex as visited
    origin->visited = true;

    // Iterate over all the neighbours
    for(auto edge : origin->adj)
        // If the neighbour hasn't been visited, run dfsVisit with that vertex
        if(!edge.dest->visited)
            dfsVisit(edge.dest);

    for(auto edge : origin->invAdj)
        // If the neighbour hasn't been visited, run dfsVisit with that vertex
        if(!edge.origin->visited)
            dfsVisit(edge.origin);
}

/**
 * Eliminates all nodes from the graph that do not belong to the strongly connected component
 * that the node(origin) belongs.
 * @param origin vertex do be processed
 * @return
 */
bool Graph::preProcess(int origin) {
    auto orig = findVertex(origin);
    if (orig == nullptr) return false;

    for(auto vertex : vertexSet)
        vertex->visited = false;

    dfsVisit(orig);

    set<int> removed;
    // deletes nodes
    for(auto it = vertexSet.begin(); it != vertexSet.end(); it++) {
        if(!(*it)->visited) {
            vertexIndexes.erase((*it)->getId());
            removed.insert((*it)->getId());
            it = vertexSet.erase(it);
            it--;
        }
    }

    // deletes outgoing edges of the deleted nodes (no need to delete from invAdj as Vertex is a pointer)
    for(auto vertex : vertexSet) {
        for(auto it = vertex->adj.begin(); it != vertex->adj.end(); it++)
            if(removed.find(it->getDest()->getId()) != removed.end()) {
                it = vertex->adj.erase(it);
                it--;
            }
    }

    return true;
}

/**************** Usual operations ************/

/**
 * @brief finds a vertex in the graph
 * @param id - id of the vertex that we are looking for
 * @return - a pointer to the vertex corresponding to the id given
 */
Vertex* Graph::findVertex(const int &id) const {
    auto it = vertexIndexes.find(id);
    return it == vertexIndexes.end() ? nullptr : it->second;
}

/**
 * @brief Adds a vertex to the graph
 * @param id - id of the vertex to add
 * @param x - coordinate in the x axis of the vertex
 * @param y - coordinate in the y axis of the vertex
 * @return - true if it added successfully
 */
bool Graph::addVertex(const int &id, const int &x, const int &y) {
    if (findVertex(id) != nullptr) return false;

    auto vertex = new Vertex(id, x, y);
    vertexSet.push_back(vertex);
    vertexIndexes.insert(pair<int, Vertex*>(id, vertex));

    return true;
}

/**
 * @brief Adds an edge to the graph
 * @param id - id of the edge to add
 * @param origin - id of the vertex where the edge begins
 * @param dest - id of the vertex where the edge ends
 * @return - true if it added successfully
 */
bool Graph::addEdge(const int &id, const int &origin, const int &dest) {
    auto v1 = findVertex(origin);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return false;

    v1->addEdge(id, v2, v1->pos.euclideanDistance(v2->pos));

    return true;
}

void Graph::setOffsetX(int x) {
    this->offsetX = x;
}

void Graph::setOffsetY(int y) {
    this->offsetY = y;
}

int Graph::getOffsetX() {
    return offsetX;
}

int Graph::getOffsetY() {
    return offsetY;
}

int Graph::getNumVertex() const {
    return vertexSet.size();
}

vector<Vertex*> Graph::getVertexSet() const {
    return vertexSet;
}

/**************** Dijkstra ************/

/**
 * @brief Initializes all the camps of the vertexes related to the central with the default values
 * @param origin - id of the vertex that represents the origin of the graph
 * @return a pointer to the vertex that represents the origin of the graph
 */
Vertex* Graph::dijkstraInitCentral(const int origin) {
    for(auto vertex : vertexSet) {
        vertex->visited = false;
        vertex->distCentral = infinite;
        vertex->pathCentral = NULL;
        vertex->edgePathCentral = Edge();
        vertex->queueIndex = 0;
    }

    auto start = findVertex(origin);
    start->distCentral = 0;

    return start;
}

/**
 * @brief Initializes all the camps of the vertexes with the default values
 * @param origin - id of the vertex that represents the origin of the graph
 * @return a pointer to the vertex that represents the origin of the graph
 */
Vertex* Graph::dijkstraInit(const int origin) {
    for(auto vertex : vertexSet) {
        vertex->visited = false;
        vertex->invVisited = false;
        vertex->dist = infinite;
        vertex->path = nullptr;
        vertex->edgePath = Edge();
        vertex->invDist = infinite;
        vertex->invPath = nullptr;
        vertex->invEdgePath = Edge();
        vertex->heuristicValue = infinite;
        vertex->invHeuristicValue = infinite;
        vertex->queueIndex = 0;
        vertex->invQueueIndex = 0;
        vertex->inv = false;
    }

    auto start = findVertex(origin);
    start->dist = 0;
    start->heuristicValue = 0;

    return start;
}

/**
 * @brief Initializes all the camps of the vertexes with the default values related to the invGraph
 * @param dest
 * @return
 */
Vertex* Graph::dijkstraBackwardsInit(const int dest){
    auto final = findVertex(dest);
    final->invDist = 0;
    final->invHeuristicValue = 0;

    return final;
}

/**
 * @brief Runs the dijkstra algorithm to find the shortest path to all the vertixes
 * @param origin - int representing the id of the origin of the graph
 * @return return true if it runned successfully
 */

bool Graph::dijkstraOriginal(const int origin)  {
    // Initializes the vertex variables based on the origin node
    Vertex* start = dijkstraInitCentral(origin);

    if(start == nullptr) return false;

    // Initialize the priority queue and insert the first vertex
    MutablePriorityQueue<Vertex> minQueue;
    minQueue.insert(start);

    // Iterate over the priority queue until it is empty
    while(!minQueue.empty()) {
        // From the queue extract the vertex that has the minimum distance from the origin point
        Vertex* min = minQueue.extractMin();
        min->visited = true;

        // Iterate over all the edges that start in the min vertex
        for(Edge edge : min->adj) {
            auto childVertex = edge.dest;

            // For each child of the min vertex, if the distance to the central is bigger then the
            // distance of the new path, then this is the new best path
            if(childVertex->distCentral > min->distCentral + edge.weight) {
                // The distance of the child vertex is equal to the distance from the start point to the father vertex
                // plus the distance to from the father vertex to the child (denoted as the weight of the edge that connects them)
                childVertex->distCentral = min->distCentral + edge.weight;
                childVertex->pathCentral = min;
                childVertex->edgePathCentral = edge;

                // if childVertex is not in queue, insert it, otherwise, update the queue with the new path
                if(childVertex->queueIndex == 0) minQueue.insert(childVertex);
                else minQueue.decreaseKey(childVertex);
            }
        }

        // Since our graph is bidirectional we iterate over all the edges that end in the min vertex
        for(Edge edge : min->invAdj) {
            Vertex* fatherVertex = edge.origin;

            // For each fatherVertex of the min vertex, if the distance to the central is bigger then the
            // distance of the new path, then this is the new best path
            if(fatherVertex->distCentral > min->distCentral + edge.weight) {
                fatherVertex->distCentral = min->distCentral + edge.weight;
                fatherVertex->pathCentral = min;
                fatherVertex->edgePathCentral = edge;

                // if fatherVertex is not in queue, insert it, otherwise, update the queue with the new path
                if(fatherVertex->queueIndex == 0) minQueue.insert(fatherVertex);
                else minQueue.decreaseKey(fatherVertex);
            }
        }
    }

    return true;
}

/**
 * @brief Runs dijkstra algorithm to find the best path between two points
 * @param origin - integer representing the id of starting node
 * @param dest - integer representing the id of destination node
 * @param processedEdges - set that stores the id of th edges that are processed
 * @return - true if it runs successfully
 */
bool Graph::dijkstra(const int origin, const int dest, unordered_set<int> &processedEdges)  {
    // Initialize all the vertex and find the origin and destination
    auto start = dijkstraInit(origin);
    auto final = findVertex(dest);
	processedEdges.clear();

    // If it can't find the start vertex or the final vertex then it can't execute the algorithm
    if(start == nullptr || final == nullptr) return false;

    // Initialize the priority queue and insert the first vertex
    MutablePriorityQueue<Vertex> minQueue;
    minQueue.insert(start);

    // Iterate over the priority queue until it is empty or we find the final vertex
    while(!minQueue.empty()) {
        // From the queue extract the vertex that has the minimum distance from the origin point
        Vertex* min = minQueue.extractMin();
        min->visited = true;

        // The algorithm ends when we dequeue the final vertex
        if(min == final)
            break;

        // Iterate over all the edges that start in the min vertex
        for(auto edge : min->adj) {
            auto childVertex = edge.dest;

            // If the childVertex as already been dequeued then we can skip it
            if(childVertex->visited) continue;

            // Save the edge that has been processed to be drawn later
            processedEdges.insert(edge.getId());

            // Relax the Child Vertex
            // If the distance to the start vertex is bigger then the distance of the new path,
            // then this is the new best path
            if(childVertex->dist > min->dist + edge.weight) {
                // The distance of the child vertex is equal to the distance from the start point to the father vertex
                // plus the distance from the father vertex to the child (denoted as the weight of the edge that connects them)
                childVertex->dist = min->dist + edge.weight;

                // The path is the vertex that leads to the Child Vertex by taking the edge saved in edgePath
                childVertex->path = min;
                childVertex->edgePath = edge;

                // if childVertex is not in queue, insert it, otherwise, update the queue with the new path
                if(childVertex->queueIndex == 0) minQueue.insert(childVertex);
                else minQueue.decreaseKey(childVertex);
            }
        }

        // Since our graph is bidirectional we iterate over all the edges that end in the min vertex
        for(auto edge : min->invAdj) {
            auto FatherVertex = edge.origin;
            // If the Father Vertex as already been dequeued then we can skip it
            if(FatherVertex->visited) continue;

            // Save the edge that has been processed to be drawn later
            processedEdges.insert(edge.getId());

            // Relax the Father Vertex
            // If the distance to the start vertex is bigger then the distance of the new path,
            // then this is the new best path
            if(FatherVertex->dist > min->dist + edge.weight) {
                // The distance of the father vertex is equal to the distance from the start point to the child vertex
                // plus the distance from the father vertex to the child (denoted as the weight of the edge that connects them)
                FatherVertex->dist = min->dist + edge.weight;

                // The path is the vertex that leads to the Father Vertex by taking the edge saved in edgePath
                FatherVertex->path = min;
                FatherVertex->edgePath = edge;

                // if FatherVertex is not in queue, insert it, otherwise, update the queue with the new path
                if(FatherVertex->queueIndex == 0) minQueue.insert(FatherVertex);
                else minQueue.decreaseKey(FatherVertex);
            }
        }
    }

    return true;
}

/**
 * @brief Saves the edges that make the path from the a vertex to the other into a vector
 * after running one of the dijkstra algorithms
 * @param dest - integer representing the id of the vertex where the path ends
 * @param edges - vector of Edges where we save the edges that belong to the path
 * @return integer representing the distance from the start vertex to the destination
 */
int Graph::getPathTo(const int dest, vector<Edge> &edges) const {
    // Searches for the destination vertex
    Vertex *destination = findVertex(dest);

    // If we can't find the destination vertex or if the destination has no path that goes to it
    // Then we can run the algorithm
    if(destination == nullptr || (destination->path == nullptr && destination->invPath == nullptr))
        return false;

    // The total distance of the trip is the attribute dist that we calculate in each of the algorithm
    int dist = destination->dist;

    // Iterate until the destination has no vertex that lead to it
    // This means that it is the start vertex
    while(destination->path != nullptr) {
        // Save the edge that leads to the previous vertex into edges
        edges.push_back(destination->getEdgePath());

        // Set destination equal to the vertex that leads to him
        destination = destination->path;

    }

    // Since we start from the end, rather then the beginning, we must reverse the vector so it has the correct order
    reverse(edges.begin(), edges.end());

    return dist;
}

/**
 * @brief Saves the edges that make the path from the a vertex to the central after running the dijkstraOriginal algorithms (line 255)
 * @param dest - integer representing the id of the vertex that we want to calculate the distance to the central
 * @param edges - vector of Edges where we save the edges that belong to the path to the central
 * @return integer representing the distance from the central to the destination
 */
int Graph::getPathFromCentralTo(const int dest, vector<Edge> &edges) const {
    // Searches for the destination vertex
    Vertex *destination = findVertex(dest);

    // If we can't find the destination vertex or if the destination has no path that goes to it
    // Then we can run the algorithm
    if(destination == nullptr || destination->pathCentral == nullptr)
        return false;

    // The total distance to the central equal to the attribute distCentral that we calculate in dijsktraOriginal (line 255)
    int dist = destination->distCentral;

    // Iterate until the destination has no vertex that lead to it
    // This means that it is the central vertex
    while(destination->pathCentral != nullptr) {
        // Save the edge that leads to the previous vertex into edges
        edges.push_back(destination->edgePathCentral);

        // Set destination equal to the vertex that leads to him
        destination = destination->pathCentral;
    }

    // Since we start from the end, rather then the beginning, we must reverse the vector so it has the correct order
    reverse(edges.begin(), edges.end());

    return dist;
}

/**************** Optimizing Dijkstra ************/

/**
 * @brief Calls a function to calculate the euclidean distance from one vertex to the other
 * @param origin - integer representing the id of the vertex that is the origin of the path
 * @param dest - integer representing the id of the vertex that is the destination of the path
 * @return integer representing the euclidian distance from the origin vertex to the destination vertex
 */
double Graph::heuristicDistance(Vertex *origin, Vertex *dest) {
    return origin->getPosition().euclideanDistance(dest->getPosition());
}

/**
 * @brief Optimization of the regular dijkstra algorithm by using an heuristic function to aid the search
 * @param origin - integer representing the id of starting node
 * @param dest - integer representing the id of destination node
 * @param processedEdges - set that stores the id of th edges that are processed
 * @return - true if it runs successfully
 */
bool Graph::dijkstraOrientedSearch(const int origin, const int dest, unordered_set<int> &processedEdges) 
{
    /*
     * Some notation to help the understanding of the comments of this algorithm
     * G(Vertex* v) --> distance from v to the start vertex, that is, cumulative sum of the weights of the edges
     * that go from the start vertex to v;
     * H(Vertex* v) --> heuristic distance from v to the final vertex;
     * F(Vertex* v) --> G(v) + H(v), sum of the real distance from the start to v and the speculative distance from
     * v to the final vertex. This is what it is used when selecting the minimum vertex of the queue.
     */

    // Initializes the vertex variables based on the origin node and finds the final vertex
    Vertex* start = dijkstraInit(origin);
    Vertex* final = findVertex(dest);
    processedEdges.clear();

    // If it can't find the start vertex or the final vertex then it can't execute the algorithm
    if(start == nullptr || final == nullptr) return false;

    int i = 0;
    MutablePriorityQueue<Vertex> minQueue;
    minQueue.insert(start); // Initialize the priority queue and insert the start vertex

    // Iterate over the priority queue until it is empty or we find the final vertex
    while(!minQueue.empty()) {
        // From the queue extract the vertex that has the minimum F()
        Vertex* min = minQueue.extractMin();
        min->visited = true;

        if(min->getId() == final->getId())
            break; // The algorithm ends when we dequeue the final vertex

        // Iterate over all the edges that start in the min vertex
        for(Edge edge : min->adj) {
            Vertex* childVertex = edge.dest;
            int weight = edge.weight;

            // If the childVertex as already been dequeued then we can skip it
            if(childVertex->visited) continue;

            // Save the edge that has been processed to be drawn later
            processedEdges.insert(edge.getId());

            // Relax the Child Vertex
            // If the distance to the start node is bigger then the distance of the new path,
            // then this is the new best path
            if(min->dist + weight < childVertex->dist) {
                // The path is the vertex that leads to the Child Vertex by taking the edge saved in edgePath
                childVertex->path = min;
                childVertex->edgePath = edge;

                // Recalculate G(childVertex)
                childVertex->dist = min->dist + weight;

                // Recalculate F(childVertex)
                childVertex->heuristicValue = childVertex->dist + heuristicDistance(childVertex, final);

                // if childVertex is not in queue, insert it, otherwise, update the queue with the new path
                if(childVertex->queueIndex == 0) minQueue.insert(childVertex);
                else minQueue.decreaseKey(childVertex);
            }
        }

        // Since our graph is bidirectional we iterate over all the edges that end in the min vertex
        for(Edge edge : min->invAdj) {
            Vertex* fatherVertex = edge.origin;
            int weight = edge.weight;

            // If the Father Vertex as already been dequeued then we can skip it
            if(fatherVertex->visited) continue;

            // Save the edge that has been processed to be drawn later
            processedEdges.insert(edge.getId());

            // Relax the Father Vertex
            // If the distance to the central is bigger then the distance of the new path,
            // then this is the new best path
            if(min->dist + weight  < fatherVertex->dist) {
                // The path is the vertex that leads to the Child Vertex by taking the edge saved in edgePath
                fatherVertex->path = min;
                fatherVertex->edgePath = edge;

                // Recalculate G(fatherVertex)
                fatherVertex->dist = min->dist + weight;

                // Recalculate F(fatherVertex)
                fatherVertex->heuristicValue = fatherVertex->dist + heuristicDistance(fatherVertex, final);

                // if fatherVertex is not in queue, insert it, otherwise, update the queue with the new path
                if(fatherVertex->queueIndex == 0) minQueue.insert(fatherVertex);
                else minQueue.decreaseKey(fatherVertex);
            }
        }
    }

    return true;
}

// Upgrades the optimization using a* with bidirectional search
bool Graph::dijkstraBidirectional(const int origin, const int dest, unordered_set<int> &processedEdges, unordered_set<int> &processedEdgesInv) 
{
    /*
     * Some notation to help the understanding of the comments of this algorithm
     * G(Vertex* v) --> distance from v to the start vertex (or final vertex if it is used in the backward search),
     * that is, cumulative sum of the weights of the edges that go from the start vertex to v;
     * H(Vertex* v) --> heuristic distance from v to the final vertex (or the start vertex if it is used in the backward search);
     * F(Vertex* v) --> G(v) + H(v), sum of the real distance from the start to v and the speculative distance from
     * v to the final vertex. This is what it is used when selecting the minimum vertex of the queue.
     * All the vertex attributes that have "inv" in the name represent the same thing as the ones that don't have it,
     * the only difference is that they are used as if we were travelling to the inverse graph. This lets us not have to
     * invert the graph itself.
     */

    // Initializes the vertex variables based on the origin node and finds the final vertex (while setting the correct
    // inv values to it)
    auto start = dijkstraInit(origin);
    auto final = dijkstraBackwardsInit(dest);

    // Make sure the sets don't have anything in them
    processedEdges.clear();
    processedEdgesInv.clear();

    // If it can't find the start vertex or the final vertex then it can't execute the algorithm
    if(start == nullptr || final == nullptr) return false;

    // Initialize the forward priority queue
    MutablePriorityQueue<Vertex> forwardMinQueue;
    forwardMinQueue.setInv(false); // Let the queue know that it is the forward queue
    forwardMinQueue.insert(start); // Add the start vertex to it

    MutablePriorityQueue<Vertex> backwardMinQueue; // Initialize the backward priority queue
    backwardMinQueue.setInv(true);  // Let the queue know that it is the backward queue
    backwardMinQueue.insert(final); // Add the final vertex to it

    // Vectors representing a closed list of the vertexes that have been processed in each search
    vector<int> processed;
    vector<int> backward_processed;

    // Initialize the forward search and backward search minimum vertex
    Vertex *forwardMin = nullptr;
    Vertex *backwardMin = nullptr;
    Vertex *middle_vertex = nullptr; // Initialize the vertex where both searches will meet

    // Iterate over both priority queues until one of them is empty or when they process the same vertex
    while(!forwardMinQueue.empty() && !backwardMinQueue.empty()) {
        /*
         * Forward Search
         */

        // Extract the vertex with the minimum F() from the forward queue
        forwardMin = forwardMinQueue.extractMin();
        forwardMin->visited = true;

        // Add it to the processed vector
        processed.push_back(forwardMin->id);

        // Iterate over all the edges that start in the vertex
        for(Edge edge : forwardMin->adj) {
            Vertex* childVertex = edge.dest;
            int weight = edge.weight;

            // If the childVertex as already been dequeued then we can skip it
            if(childVertex->visited) continue;

            // Save the edge that has been processed to be drawn later
            processedEdges.insert(edge.getId());

            // Relax the Child Vertex
            // If the distance to the start node is bigger then the distance of the new path,
            // then this is the new best path
            if(forwardMin->dist + weight < childVertex->dist ) {
                // The path is the vertex that leads to the Child Vertex by taking the edge saved in edgePath
                childVertex->path = forwardMin;
                childVertex->edgePath = edge;

                // Recalculate G(childVertex)
                childVertex->dist = forwardMin->dist + weight;

                // Recalculate F(childVertex)
                childVertex->heuristicValue = childVertex->dist + heuristicDistance(childVertex, final);


                // if childVertex is not in queue, insert it, otherwise, update the queue with the new path
                if(childVertex->queueIndex == 0) forwardMinQueue.insert(childVertex);
                else forwardMinQueue.decreaseKey(childVertex);
            }
        }

        // Since our graph is bidirectional we iterate over all the edges that end in the vertex
        for(Edge edge : forwardMin->invAdj) {
            Vertex* fatherVertex = edge.origin;
            int weight = edge.weight;

            // If the Father Vertex as already been dequeued then we can skip it
            if(fatherVertex->visited) continue;

            // Save the edge that has been processed to be drawn later
            processedEdges.insert(edge.getId());

            // Relax the Father Vertex
            // If the distance to the central is bigger then the distance of the new path,
            // then this is the new best path
            if(forwardMin->dist + weight < fatherVertex->dist ) {
                // The path is the vertex that leads to the Father Vertex by taking the edge saved in edgePath
                fatherVertex->path = forwardMin;
                fatherVertex->edgePath = edge;

                // Recalculate G(fatherVertex)
                fatherVertex->dist = forwardMin->dist + weight;

                // Recalculate F(fatherVertex)
                fatherVertex->heuristicValue = fatherVertex->dist + heuristicDistance(fatherVertex, final);

                // if fatherVertex is not in queue, insert it, otherwise, update the queue with the new path
                if(fatherVertex->queueIndex == 0) forwardMinQueue.insert(fatherVertex);
                else forwardMinQueue.decreaseKey(fatherVertex);
            }
        }

        // If the vertex was already processed in the backward search then save the vertex and end the search
        if(find(backward_processed.begin(), backward_processed.end(), forwardMin->id) != backward_processed.end()) {
            middle_vertex = forwardMin;
            break;
        }

        /*
         * Backward Search
         */

        // Extract the vertex with the minimum F() from the backward queue
        backwardMin = backwardMinQueue.extractMin();
        backwardMin->invVisited = true;

        // Add it to the processed vector
        backward_processed.push_back(backwardMin->id);

        // Iterate over all the edges that end in the vertex
        for(Edge edge : backwardMin->invAdj) {
            Vertex* fatherVertex = edge.origin;
            int weight = edge.weight;

            // If the Father Vertex as already been dequeued then we can skip it
            if(fatherVertex->invVisited) continue;

            // Save the edge that has been processed to be drawn later
            processedEdgesInv.insert(edge.getId());

            // Relax the Father Vertex
            // If the distance to the final node is bigger then the distance of the new path,
            // then this is the new best path
            if(backwardMin->invDist + weight < fatherVertex->invDist) {
                // The invPath is the vertex that leads to the Father Vertex by taking the edge saved in invEdgePath
                fatherVertex->invPath = backwardMin;
                fatherVertex->invEdgePath = edge;

                // Recalculate G(fatherVertex)
                fatherVertex->invDist = backwardMin->invDist + weight;

                // Recalculate F(fatherVertex)
                fatherVertex->invHeuristicValue = fatherVertex->invDist + heuristicDistance(fatherVertex, start);

                // if fatherVertex is not in queue, insert it, otherwise, update the queue with the new path
                if(fatherVertex->invQueueIndex == 0) backwardMinQueue.insert(fatherVertex);
                else backwardMinQueue.decreaseKey(fatherVertex);

                continue;
            }
        }

        // Since our graph is bidirectional we iterate over all the edges that begin in the vertex
        for(Edge edge : backwardMin->adj) {
            Vertex* childVertex = edge.dest;
            int weight = edge.weight;

            // If the childVertex as already been dequeued then we can skip it
            if(childVertex->invVisited) continue;

            // Save the edge that has been processed to be drawn later
            processedEdgesInv.insert(edge.getId());

            // Relax the Child Vertex
            // If the distance to the start node is bigger then the distance of the new path,
            // then this is the new best path
            if(backwardMin->invDist + weight < childVertex->invDist) {
                // The path is the vertex that leads to the Child Vertex by taking the edge saved in edgePath
                childVertex->invPath = backwardMin;
                childVertex->invEdgePath = edge;

                // Recalculate G(childVertex)
                childVertex->invDist = backwardMin->invDist + weight;

                // Recalculate F(childVertex)
                childVertex->invHeuristicValue = childVertex->invDist + heuristicDistance(childVertex, start);

                // if childVertex is not in queue, insert it, otherwise, update the queue with the new path
                if(childVertex->invQueueIndex == 0) backwardMinQueue.insert(childVertex);
                else backwardMinQueue.decreaseKey(childVertex);

                continue;
            }
        }

        // If the vertex was already processed in the forward search then save the vertex and end the search
        if(find(processed.begin(), processed.end(), backwardMin->id) != processed.end()){
            middle_vertex = backwardMin;
            break;
        }
    }

    // Save the distance from the start vertex to the final vertex to the middle vertex
    int min_dist = middle_vertex->heuristicValue + middle_vertex->invHeuristicValue;

    // Search the forward queue to find a vertex that as a smaller distance from the start vertex
    // to the final vertex
    while(!forwardMinQueue.empty()) {
        // Extract a vertex from the queue
        forwardMin = forwardMinQueue.extractMin();

        // If the new vertex has a smaller distance then set it has the middle vertex
        if(forwardMin->heuristicValue + forwardMin->invHeuristicValue < min_dist) {
            min_dist = forwardMin->heuristicValue + forwardMin->invHeuristicValue;
            middle_vertex = forwardMin;
        }
    }

    // Search the backward queue to find a vertex that as a smaller distance from the start vertex
    // to the final vertex
    while(!backwardMinQueue.empty()) {
        // Extract a vertex from the queue
        backwardMin = backwardMinQueue.extractMin();

        // If the new vertex has a smaller distance then set it has the middle vertex
        if(backwardMin->heuristicValue+ backwardMin->invHeuristicValue < min_dist){
            min_dist = backwardMin->heuristicValue+ backwardMin->invHeuristicValue;
            middle_vertex = backwardMin;
        }
    }

    // Convert the invPath and invEdgePath to path and edgePath to be used in getPathTo (line 404)
    while(middle_vertex->invPath != nullptr) {
        // The invPath is the vertex that leads to midle_vertex from the backward search
        // So we set path, edgePath, and dist of that vertex with the middle_vertex values
        middle_vertex->invPath->path = middle_vertex;
        middle_vertex->invPath->edgePath = middle_vertex->invEdgePath;
        middle_vertex->invPath->dist = middle_vertex->dist + middle_vertex->invEdgePath.getWeight();
        middle_vertex = middle_vertex->invPath;
    }

    return true;
}

/**************** All Pairs Shortest Path  ***************/

void Graph::floydWarshallShortestPath() {
    int vertSize = this->vertexSet.size();

    this->minDistance.resize(vertSize);
    for(auto &elem : this->minDistance) elem.resize(vertSize);

    this->next.resize(vertSize);
    for(auto &elem : next) elem.resize(vertSize);

    // this->minDistance -> <vector<vector<double>> that stores distance between i and j
    // this->next        -> <vector<vector<Vertex<T>*>> that stores the path between i and j
    for(int i = 0; i < vertSize; i++) {
        for(int j = 0; j < vertSize; j++) {
            this->minDistance.at(i).at(j) = infinite;
            this->next.at(i).at(j) = nullptr;
        }
    }

    for(Vertex *vertex : this->vertexSet) {
        for(Edge &edge : vertex->adj) {
            int u = -1;
            int v = -1;

            for(int k = 0; k < vertSize; k++) {
                if(this->vertexSet.at(k)->id == vertex->id) u = k;
                if(this->vertexSet.at(k)->id == edge.dest->id) v = k;
            }

            // for each edge dist[u][v] = weight(u, v)
            this->minDistance.at(u).at(v) = edge.weight;
            this->next.at(u).at(v) = edge.dest;
        }
    }

    // for each vertex dist[v][v] = 0
    for(int i = 0; i < vertSize; i++) {
        this->minDistance.at(i).at(i) = 0;
        this->next.at(i).at(i) = this->vertexSet.at(i);
    }

    for(int k = 0; k < vertSize; k++) {
        for (int i = 0; i < vertSize; i++) {
            for(int j = 0; j < vertSize; j++) {
                if(this->minDistance.at(i).at(j) > this->minDistance.at(i).at(k) + this->minDistance.at(k).at(j)) {
                    this->minDistance.at(i).at(j) = this->minDistance.at(i).at(k) + this->minDistance.at(k).at(j);
                    this->next.at(i).at(j) = this->next.at(i).at(k);
                }
            }
        }
    }
}

vector<int> Graph::getfloydWarshallPath(const int orig, const int dest) const {
    vector<int> res;
    int vertSize = this->vertexSet.size();

    Vertex *init, *final;
    init = this->findVertex(orig);
    final = this->findVertex(dest);
    if(init == nullptr || final == nullptr) return res;

    int initIndex, finalIndex;
    for(int i = 0; i < vertSize; i++)
        if(this->vertexSet.at(i)->id == init->id) initIndex = i;

    for(int i = 0; i < vertSize; i++)
        if(this->vertexSet.at(i)->id == final->id) finalIndex = i;

    if(this->next.at(initIndex).at(finalIndex) == nullptr) return res;

    res.push_back(init->id);
    while(init->id != final->id) {
        init = next.at(initIndex).at(finalIndex);
        res.push_back(init->id);
        for(int i = 0; i < vertSize; i++)
            if(this->vertexSet.at(i)->id == init->id) initIndex = i;
    }

    return res;
}

#endif