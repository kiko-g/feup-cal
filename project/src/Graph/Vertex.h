#pragma once
#ifndef VERTEX_H_
#define VERTEX_H_

#include "../Position/Position.h"
#include "Edge.h"

using namespace std;

class Vertex {
public:
    enum Tag {CENTRAL, INTEREST_POINT, PICKUP, DROPOFF, DEFAULT};

private:
    int id;                            // identifier of the vertex
    Position pos;			           // content of the vertex
    vector<Edge> adj;		           // outgoing edges
    vector<Edge> invAdj;               // ingoing edges
    Tag tag = DEFAULT;                 // vertex Tag
    double dist = infinite;
    double distCentral = infinite;
    double invDist = infinite;
    // central path
    Vertex *pathCentral = nullptr;
    Edge edgePathCentral;
    // others path
    Vertex *path = nullptr;
    Vertex *invPath = nullptr;
    Edge edgePath;
    Edge invEdgePath;
    bool inv = false;

    int queueIndex = 0; 		    // required by MutablePriorityQueue
    double heuristicValue = 0;      // oriented search optimization (a*)
    double invHeuristicValue = 0;
    int invQueueIndex = 0;
    bool invVisited = false;
    bool visited = false;		      // auxiliary field

    void addEdge(const int &id, Vertex *dest, const double &weight);
    const static int infinite = 99999999;

public:
    Vertex(const int &id, const int &x, const int &y) {
        this->id = id;
        this->pos = Position(x, y);
    }

    /* get methods */
    int getId() const;
    Position getPosition() const;
    vector<Edge> getAdj() const;
    vector<Edge> getInvAdj() const;
    double getDist() const;
    Vertex *getPath() const;
    Edge getEdgePath() const;
    bool getVisited() const;
    Tag getTag() const;
    void setTag(Vertex::Tag tag);

    bool operator<(Vertex &vertex) const; //required by MutablePriorityQueue
    friend class Graph;
    friend class MutablePriorityQueue<Vertex>;
};

/**
 * Auxiliary function to add an outgoing edge to a vertex (this),
 * with a given destination vertex (dest) and edge weight (weight).
 */
void Vertex::addEdge(const int &id, Vertex *dest, const double &weight) {
    Edge edge = Edge(id, dest, this, weight);
    adj.push_back(edge);
    dest->invAdj.push_back(edge);
}

int Vertex::getId() const {
    return this->id;
}

Position Vertex::getPosition() const {
    return this->pos;
}

vector<Edge> Vertex::getAdj() const {
    return this->adj;
}

vector<Edge> Vertex::getInvAdj() const {
    return this->invAdj;
}

double Vertex::getDist() const {
    return this->dist;
}

Vertex* Vertex::getPath() const {
    return this->path;
}

Edge Vertex::getEdgePath() const {
    return this->edgePath;
}

bool Vertex::getVisited() const {
    return this->visited;
}

Vertex::Tag Vertex::getTag() const {
    return this->tag;
}

void Vertex::setTag(Vertex::Tag tag) {
    this->tag = tag;
}

bool Vertex::operator<(Vertex &vertex) const {

    if(heuristicValue != infinite)
        return this->heuristicValue < vertex.heuristicValue;

    if(invHeuristicValue != infinite)
        return this->invHeuristicValue < vertex.invHeuristicValue;

    if(invDist != infinite)
        return this->invDist < vertex.invDist;

    return this->dist < vertex.dist;
}

#endif