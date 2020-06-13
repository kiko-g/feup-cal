#pragma once
#ifndef MEAT_WAGONS_READER_H
#define MEAT_WAGONS_READER_H

#include <fstream>
#include "Graph.h"

class Reader {
    private:
        string path;

    public:
        Reader(const string &path) : path(path) {}

        bool readGraph(Graph *graph, int &central);
        bool readRequests(multiset<Request*> &requestVector);
        bool setTags(Graph *graph);
        bool setCentral(Graph *graph, int &central);
};

bool Reader::readGraph(Graph *graph, int &central) {
    ifstream nodesStream(path + "/nodes.txt");
    ifstream edgesStream(path + "/edges.txt");

    if(!nodesStream.is_open() || !edgesStream.is_open()) return false;

    int id, origin, dest;
    int numNodes, numEdges;
    double x, y;
    char c;

    int minX = numeric_limits<int>::max(), minY = numeric_limits<int>::max(), maxX = 0, maxY = 0;

    nodesStream >> numNodes;
    for (int i = 1; i <= numNodes; i++) {
        nodesStream >> c >> id >> c >> x >> c >> y >> c;
        graph->addVertex(id, x, y);
        x > maxX ? maxX = x : maxX;
        y > maxY ? maxY = y : maxY;
        x < minX ? minX = x : minX;
        y < minY ? minY = y : minY;
    }

    edgesStream >> numEdges;
    for (int i = 1; i <= numEdges; i++) {
        edgesStream >> c >> origin >> c >> dest >> c;
        graph->addEdge(i, origin, dest);
    }

    nodesStream.close();
    edgesStream.close();

    setTags(graph);
    setCentral(graph, central);

    return true;
}

bool Reader::readRequests(multiset<Request*> &requestVector) {
    ifstream requests(path + "/requests.txt");

    if(!requests.is_open()) return false;

    string name;
    int dest, priority;
    int hour, min, sec;

    while(requests >> name) {
        requests >> dest >> priority >> hour >> min >> sec;
        Time arrival(hour, min, sec);
        Request * request = new Request(name, dest, priority, arrival);
        requestVector.insert(request);
    }
    return true;
}

bool Reader::setTags(Graph *graph) {
    ifstream tagsStream(path + "/tags.txt");

    if(!tagsStream.is_open()) return false;

    int id, trash, numTags;
    string tagName;

    tagsStream >> trash;
    tagsStream >> tagName >> numTags;
    for(int j = 0; j < numTags; j++) {
        tagsStream >> id;
        Vertex *vertex = graph->findVertex(id);
        if(vertex == nullptr) return false;
        vertex->setTag(Vertex::INTEREST_POINT);
    }
    return true;
}

bool Reader::setCentral(Graph *graph, int &central) {
    int pos = path.find_last_of('/');
    string city = path.substr(pos + 1);
    Vertex *centralVertex;

    if(city == "Porto") {
        centralVertex = graph->findVertex(90379359);
        centralVertex->setTag(Vertex::CENTRAL);
        central = 90379359;
    }
    else if(city == "Aveiro") {
        centralVertex = graph->findVertex(330341307);
        centralVertex->setTag(Vertex::CENTRAL);
        central = 330341307;
    }
    else if(city == "Braga") {
        centralVertex = graph->findVertex(914277393);
        centralVertex->setTag(Vertex::CENTRAL);
        central = 914277393;
    }
    else if(city == "Coimbra") {
        centralVertex = graph->findVertex(26062543);
        centralVertex->setTag(Vertex::CENTRAL);
        central = 26062543;
    }
    else if(city == "Ermesinde") {
        centralVertex = graph->findVertex(269567665);
        centralVertex->setTag(Vertex::CENTRAL);
        central = 269567665;
    }
    else if(city == "Fafe") {
        centralVertex = graph->findVertex(25264987);
        centralVertex->setTag(Vertex::CENTRAL);
        central = 25264987;
    }
    else if(city == "Gondomar") {
        centralVertex = graph->findVertex(275217973);
        centralVertex->setTag(Vertex::CENTRAL);
        central = 275217973;
    }
    else if(city == "Lisboa") {
        centralVertex = graph->findVertex(389941187);
        centralVertex->setTag(Vertex::CENTRAL);
        central = 389941187;
    }
    else if(city == "Maia") {
        centralVertex = graph->findVertex(264117399);
        centralVertex->setTag(Vertex::CENTRAL);
        central = 264117399;
    }
    else if(city == "Viseu") {
        centralVertex = graph->findVertex(27114564);
        centralVertex->setTag(Vertex::CENTRAL);
        central = 27114564;
    }
   /*else if(city == "Portugal") {
        centralVertex = graph->findVertex();
        centralVertex->setTag(Vertex::CENTRAL);
        central = ;
    }*/

    graph->setOffsetX(centralVertex->getPosition().getX() - 300);
    graph->setOffsetY(centralVertex->getPosition().getY() - 250);

    return true;
}

#endif //MEAT_WAGONS_READER_H
