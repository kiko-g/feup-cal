#pragma once
#ifndef EDGE_H_
#define EDGE_H_

class Vertex;

class Edge {
    private:
        int id;
        Vertex *origin;           // origin vertex
        Vertex *dest;             // destination vertex
        double weight;            // edge weight
    
    public:
        Edge() {
            this->id = -1;
            this->origin = nullptr;
            this->dest = nullptr;
            this->weight = 0;
        }

        Edge(const int &id, Vertex *dest, Vertex *origin, const double &weight) {
            this->id = id;
            this->dest = dest;
            this->origin = origin;
            this->weight = weight;
        }

        /* get methods */
        int getId() const;
        Vertex* getDest() const;
        double getWeight() const;
        static vector<int> getIds(vector<Edge> edges);

        friend class Graph;
        friend class Vertex;
};

int Edge::getId() const {
	return this->id;
}

Vertex* Edge::getDest() const {
	return this->dest;
}

double Edge::getWeight() const {
	return this->weight;
}

vector<int> Edge::getIds(vector<Edge> edges)  {
    vector<int> ids;
    for(Edge edge : edges) ids.push_back(edge.id);
    return ids;
}

#endif