#pragma once
#ifndef MEAT_WAGONS_GRAPHVISUALIZER_H
#define MEAT_WAGONS_GRAPHVISUALIZER_H

#include <thread>
#include "../Graph/Graph.h"
#include "GraphViewer/cpp/graphviewer.h"

class GraphVisualizer {
private:
    bool isActive = true;
    int width, height;
    GraphViewer *gv;

public:
    GraphVisualizer(const int width, const int height) {
        this->width = width;
        this->height = height;
        this->gv = new GraphViewer(1, 1, false);
    }

    ~GraphVisualizer() {
        delete gv;
    }
	
    bool getStatus();
    void setStatus(bool status);
    GraphViewer* getViewer() const;
    void drawFromThread(Graph *graph);
    void draw(Graph *graph);
    void setPath(const vector<int> &edges, const string &edgeColor, const bool isShortestPath = false);
    void setNode(const int id, const int size, const string color, const string label);
    void drawShortestPathFromThread(const unordered_set<int> &processedEdges, const unordered_set<int> &processedEdgesInv, const vector<Edge> &edges, Graph *graph);
    void drawShortestPath(const unordered_set<int> &processedEdges, const unordered_set<int> &processedEdgesInv, const vector<Edge> &edges, Graph *graph);
    void newGv();
};

bool GraphVisualizer::getStatus() {
    return isActive;
}

void GraphVisualizer::setStatus(bool status) {
    isActive = status;
}


GraphViewer* GraphVisualizer::getViewer() const {
    return gv;
}

void GraphVisualizer::drawFromThread(Graph *graph) {
	if(!isActive) return;
    newGv();
    thread threadProcess(&GraphVisualizer::draw, this, graph);
    threadProcess.detach();
}

void GraphVisualizer::draw(Graph *graph) {
    //vertexes settings
    this->gv->defineVertexColor("black");
    //edges settings
    this->gv->defineEdgeColor("gray");
    this->gv->defineEdgeCurved(false);

    vector<Vertex*> vertexSet = graph->getVertexSet();

    for(Vertex *origin : vertexSet)
        this->gv->addNode(origin->getId(), origin->getPosition().getX() - graph->getOffsetX(), origin->getPosition().getY() - graph->getOffsetY());

    for(Vertex *origin : vertexSet) {
        if(origin->getTag() == Vertex::Tag::CENTRAL) {
            this->gv->setVertexColor(origin->getId(), "red");
            this->gv->setVertexLabel(origin->getId(), "Meat Wagons Central");
            this->gv->setVertexSize(origin->getId(), 40);
        }

        else if(origin->getTag() == Vertex::Tag::INTEREST_POINT) {
            this->gv->setVertexColor(origin->getId(), "yellow");
            this->gv->setVertexLabel(origin->getId(), "Point of interest");
            this->gv->setVertexSize(origin->getId(), 20);
        }

        else if(origin->getTag() == Vertex::Tag::PICKUP) {
            this->gv->setVertexColor(origin->getId(), "green");
            this->gv->setVertexSize(origin->getId(), 30);
        }

        else if(origin->getTag() == Vertex::Tag::DROPOFF) {
            this->gv->setVertexColor(origin->getId(), "magenta");
            this->gv->setVertexLabel(origin->getId(), "DROPOFF");
            this->gv->setVertexSize(origin->getId(), 30);
        }

        else {
            this->gv->setVertexSize(origin->getId(), 5);
        }

        for(Edge e : origin->getAdj()) {
            this->gv->addEdge(e.getId(), origin->getId(), e.getDest()->getId(), EdgeType::UNDIRECTED);
        }
    }
    this->gv->rearrange();
}

void GraphVisualizer::setPath(const vector<int> &edges, const string &edgeColor, const bool isShortestPath) {
    for(int id : edges) {
        this->gv->setEdgeColor(id, edgeColor);
        if(isShortestPath)
            this->gv->setEdgeThickness(id, 8);
        else
            gv->setEdgeThickness(id, 3);
    }
}

void GraphVisualizer::setNode(const int id, const int size, const string color, const string label="") {
    this->gv->setVertexSize(id, size);
    this->gv->setVertexColor(id, color);
    this->gv->setVertexLabel(id, label);
}

void GraphVisualizer::drawShortestPathFromThread(const unordered_set<int> &processedEdges, const unordered_set<int> &processedEdgesInv, const vector<Edge> &edges, Graph *graph) {
    newGv();
    thread threadProcess(&GraphVisualizer::drawShortestPath, this, processedEdges, processedEdgesInv, edges, graph);
    threadProcess.detach();
}

void GraphVisualizer::drawShortestPath(const unordered_set<int> &processedEdges, const unordered_set<int> &processedEdgesInv, const vector<Edge> &edges, Graph *graph) {
    // get processed path
    vector<int> edgesProcessed(processedEdges.begin(), processedEdges.end());
    this->setPath(edgesProcessed, "orange", false);

    if(processedEdgesInv.size() != 0) {
        vector<int> edgesProcessedInv(processedEdgesInv.begin(), processedEdgesInv.end());
        this->setPath(edgesProcessedInv, "magenta", false);
    }

    vector<int> edgesIds = Edge::getIds(edges);
    // draw shortest path
    this->setPath(edgesIds, "blue", true);
    this->draw(graph);
}

void GraphVisualizer::newGv() {
    this->gv = new GraphViewer(600, 600, false);
    this->gv->createWindow(width, height);
}

#endif //MEAT_WAGONS_GRAPHVISUALIZER_H