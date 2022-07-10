#pragma once
#ifndef MEAT_WAGONS_MEATWAGONS_H
#define MEAT_WAGONS_MEATWAGONS_H

#include <unordered_set>
#include <set>
#include <time.h>
#include <algorithm>
#include "Request.h"
#include "../Graph/Reader.h"
#include "../GraphViewer/GraphVisualizer.h"
#include "Wagon.h"

bool compareRequests(Request* r1, Request* r2){
    return r1->getArrival() < r2->getArrival();
}

class MeatWagons {
    private:
        int central;
        Graph *graph = nullptr;
        vector<Vertex*> pointsOfInterest;
        string graphName;
        multiset<Wagon> wagons;
        multiset<Request*> requests;
        multiset<Request*> constantRequests;
        int zoneMaxDist;
        bool processed = false;
        const static int averageVelocity = 9;

    public:
        GraphVisualizer *viewer = new GraphVisualizer(600, 600);
        MeatWagons(const int wagons) {
            for(int i = 0; i < wagons; i++){
                this->wagons.insert(Wagon(i, 5));
            }
            this->zoneMaxDist = 2000;
        }

        int getCentral() const;
        bool setCentral(const int &id);
        string getGraphName() const;
        int getMaxDist() const;
        void setMaxDist(const int max);
        void setWagons(const int n, const int capacity);
        Graph* getGraph() const;
        multiset<Wagon> getWagons() const;
        void addWagon(const int capacity);
        void removeWagon(const int id);
        multiset<Request*> getRequests() const;
        multiset<Request*> getConstantRequests() const;

        bool setGraph(const string path);
        bool preProcess(const int node, const bool draw);
        bool shortestPath(const int option, const int origin, const int dest);
        multiset<Wagon>::iterator getWagon();

        bool deliver(const int iteration);
        int chooseDropOff(const vector<Vertex*> &pickupNodes);
        vector<Request*> groupRequests(const int capacity);
        Vertex* getNearestNeighbour(Vertex *node,  const vector<Vertex*> &neighbours);
        int tspPath(vector<Vertex*> &tspNodes, vector<Request *> reqs, vector<Edge> &tspPath, int dropOffNode, Time& startTime);
        Delivery* drawDeliveriesFromThread(int wagonIndex, int deliveryIndex);
        bool drawDeliveries(int wagonIndex, int deliveryIndex);
        Request* findRequest(Vertex * vert, vector<Request *> requests);
        bool firstIteration();
        bool secondIteration();
        bool thirdIteration();
        int objectiveFunction();

        //auxiliar functions
        bool getViewerStatus(string &status);
        void setViewerStatus(bool status);
};

int MeatWagons::getCentral() const {
    return this->central;
}

bool MeatWagons::setCentral(const int &id) {
    if(this->graph->findVertex(id) == nullptr) return false;
    this->central = id;
    return true;
}

string MeatWagons::getGraphName() const {
    return this->graphName;
}

int MeatWagons::getMaxDist() const {
    return this->zoneMaxDist;
}

void MeatWagons::setMaxDist(const int max) {
    this->zoneMaxDist = max;
}

Graph* MeatWagons::getGraph() const {
    return this->graph;
}

multiset<Wagon> MeatWagons::getWagons() const {
    return this->wagons;
}

void MeatWagons::setWagons(const int n, const int capacity) {
    this->wagons.clear();
    for(int i=0; i < n; ++i) {
        this->wagons.insert(Wagon(i, capacity));
    }
}

void MeatWagons::addWagon(const int capacity) {
    wagons.insert(Wagon(wagons.size(), capacity));
}

void MeatWagons::removeWagon(const int id) {
    for (auto wit = wagons.begin(); wit != wagons.end(); wit++)
        if (id == wit->getId()) {
            wagons.erase(Wagon(id, wit->getCapacity()));
            return;
        }
}

multiset<Request*> MeatWagons::getRequests() const {
    return this->requests;
}

multiset<Request*> MeatWagons::getConstantRequests() const {
    return this->constantRequests;
}

/**
 * @param graphPath path to graph
 * @return true upon success
 */
bool MeatWagons::setGraph(const string graphPath) {
    Reader graphReader = Reader(graphPath);
    Graph* graphRead = new Graph();

    if(!graphReader.readGraph(graphRead, central))
        return false;
    if(!graphReader.readRequests(requests))
        return false;
    
    this->constantRequests = requests;
    this->processed = false;
    this->graph = graphRead;
    this->graphName = graphPath.substr(graphPath.find_last_of('/') + 1);
    this->viewer->drawFromThread(this->graph);
    this->processed = false;

    return true;
}

/**
 * Pre processes graph and eliminates all the requests in which the destination is a node that was removed
 * after the pre processement of the graph
 * @param node to be processed
 * @return
 */
bool MeatWagons::preProcess(const int node, const bool draw) {
    if(this->graph == nullptr) return false;
    if(!this->graph->preProcess(node)) return false;
    if(!this->graph->dijkstraOriginal(central)) return false;


    for(int i = 0; i < this->requests.size(); i++) {
        auto it = next(this->requests.begin(), i);
        Request *r = *it;
        Vertex *vert = this->graph->findVertex((r)->getDest());
        if(vert == nullptr) {
            this->requests.erase(it);
            i--;
        } else {
            this->pointsOfInterest.push_back(vert);
        }
    }

    this->processed = true;
    if(draw) this->viewer->drawFromThread(this->graph);
    return true;
}

/**
 * @brief Calculates the shortest path from one point to another with different algorithms
 * @param option - integer representing the algorithm to be used
 * @param origin - start point to calculate the distance
 * @param dest - destination of the path
 */
bool MeatWagons::shortestPath(const int option, const int origin, const int dest) {
    if(this->graph == nullptr) return false;

    unordered_set<int> processedEdges, processedEdgesInv;
    switch (option) {
        case 1: if (!this->graph->dijkstra(origin, dest, processedEdges)) return false; break;
        case 2: if (!this->graph->dijkstraOrientedSearch(origin, dest, processedEdges)) return false; break;
        case 3: if (!this->graph->dijkstraBidirectional(origin, dest, processedEdges, processedEdgesInv)) return false; break;
    }

    vector<Edge> edges;
    this->graph->getPathTo(dest, edges);
    this->viewer->drawShortestPathFromThread(processedEdges, processedEdgesInv, edges, this->graph);

    return true;
}

/**
 * Controls which iteration is to be used
 * @param iteration - number of the iteration to use
 */
bool MeatWagons::deliver(int iteration) {
    if(!this->processed) this->preProcess(central, false);
    if(this->constantRequests.size() == 0) return false;
    if(this->requests.size() == 0) {
        this->requests = this->constantRequests;
        for (int i = 0; i < this->requests.size(); i++) {
            auto it = next(this->requests.begin(), i);
            Request *r = *it;
            Vertex *vert = this->graph->findVertex((r)->getDest());
            if (vert == nullptr) {
                this->requests.erase(it);
                i--;
            }
            else this->pointsOfInterest.push_back(vert);
        }
    }

    switch (iteration) {
        case 1: return this->firstIteration();
        case 2: return this->secondIteration();
        case 3: return this->thirdIteration();
        default: return false;
    }
}

/**
 * @brief Generates a random dropOff
 * @param pickupNodes - nodes that cannot be used as drop off point
 * @return an integer that represents the id of the drop off vertex
 */
int MeatWagons::chooseDropOff(const vector<Vertex*> &pickupNodes) {
    srand((unsigned) time(0));
    int id, randomId;

    while(true){
        randomId = (rand() % this->requests.size());
        id = this->pointsOfInterest.at(randomId)->getId();
        if(id == central) continue;

        auto it = find_if(begin(pickupNodes), end(pickupNodes), [=](const Vertex* v) {
            return v->getId() == id && (find(begin(pickupNodes), end(pickupNodes), v) != pickupNodes.end());
        });

        if(it != pickupNodes.end()) continue;
        return id;
    }
}

/**
 * @brief Groups a number of requests together, based on their distance to one another
 * @param capacity - number of requests to be grouped
 * @return a vector containing pointers to the requests that were grouped
 */
vector<Request *> MeatWagons::groupRequests(const int capacity){
    // Initialize the vector where we will put the grouped requests
    vector <Request *> group;
    int max_dist = 0, dist, pos = 0, max_dist_request_pos = 0;
    auto it = requests.begin();
    // We start with the first request since they are ordered by the arrival
    Vertex* initial_vert = this->graph->findVertex((*it)->getDest());
    group.push_back((*it));
    it++;
    // Iterate over the requests to find the nearest to the first one
    while(it != requests.end()) {
        // Get the vertex of the pick up node related to the request
        Vertex *vert = this->graph->findVertex((*it)->getDest());

        // Calculate its distance to the first request
        dist = vert->getPosition().euclideanDistance(initial_vert->getPosition());

        if(dist >= this->zoneMaxDist) { it++; continue; };

        // If the vector as less requests then the capacity we can just add it to the vector
        if(group.size() < capacity) {
            group.push_back(*it);
            pos++;

            // Save the one that as the biggest distance to the first request
            if(dist > max_dist) {
                max_dist = dist;
                max_dist_request_pos = pos;
            }
        }

        else if(dist < max_dist) {
            // Remove the request that has the biggest distance to the first request
            // Save the requests from the first position to the position of the request that has the biggest distance
            vector <Request *> groupInit;
            groupInit.assign(group.begin(), group.begin() + max_dist_request_pos);
            // Save the requests from (not including) the request that has the biggest distance to the last one
            vector <Request *> groupFinal;
            groupFinal.assign(group.begin() + max_dist_request_pos + 1, group.end());

            // Clear the grouped requests and add the all the requests to it except the one that has the biggest distance
            // It has to be done this way because we were working with pointer and using group.erase() would delete the request itself
            group.clear();
            group.insert(group.end(), groupInit.begin(), groupInit.end());
            group.insert(group.end(), groupFinal.begin(), groupFinal.end());

            // Assume the new request has the biggest distance to the first request
            max_dist = dist;
            max_dist_request_pos = pos;

            // Check if there is a request with a bigger distance then the new request
            for(auto itr = 0; itr < group.size(); itr++) {
                auto *vertex = this->graph->findVertex(group[0]->getDest());
                dist = vertex->getPosition().euclideanDistance(initial_vert->getPosition());

                if(dist > max_dist) {
                    max_dist = dist;
                    max_dist_request_pos = itr;
                }
            }

            // Put the new request into the vector
            group.push_back(*it);
        }

        it++;
    }

    return group;
}

/**
 * @brief finds the closest node to a specific node
 * @param node - vertex use as a reference to calculate de distance
 * @param neighbours - vector of vertex containing other nodes
 * @return a pointer to the vertex closest to the given node
 */
Vertex* MeatWagons::getNearestNeighbour(Vertex *node,  const vector<Vertex*> &neighbours) {
    double nearestDistance = this->graph->findVertex(node->getId())->getPosition().euclideanDistance((*neighbours.begin())->getPosition());
    auto nearestNeighbour = *neighbours.begin();

    for(auto it = ++neighbours.begin(); it != neighbours.end(); it++) {
        auto currDistance = this->graph->findVertex(node->getId())->getPosition().euclideanDistance((*it)->getPosition());
        if(currDistance < nearestDistance) {
            nearestDistance = currDistance;
            nearestNeighbour = *it;
        }
    }

    return nearestNeighbour;
}

/**
 * @brief finds the request of a specific destination
 * @param vert - Vertex * of de destination
 * @param reqs - vector containing all the requests
 * @return a pointer to the requests with vert as its destination, nullptr if it doens't exist any request
 */
Request *MeatWagons::findRequest(Vertex* vert, vector<Request*> reqs) {
    for (auto it = reqs.begin(); it != reqs.end(); it++) {
        if ((*it)->getDest() == vert->getId()) {
            return *it;
        }
    }
    return nullptr;
}

/**
 * @brief Calculates the shortest path passing through various points using dijkstra bidirectional
 * @param tspNodes - vector of all the vertex that the wagon must pass by
 * @param reqs - vector of all the reqs that are used in this ride
 * @param tspPath - vector of edges that are passed through
 * @param dropOffNode - integer representing the id of the drop off vertex
 * @param startTime - represents the time that the wagon leaves the central
 * @return - integer representing the distance from the central to the drop off node passing through tspNodes
 */
int MeatWagons::tspPath(vector<Vertex*> &tspNodes, vector<Request *> reqs, vector<Edge> &tspPath, int dropOffNode, Time& startTime){
    unordered_set <int> processedEdges, processedInvEdges;
    int totalDist = 0;

    // Because tspNodes is a vector ordered by dist -> the first element is the closest to the central
    Vertex *closest = *tspNodes.begin();

    // Calculate the distance from the central to the first point
    totalDist += this->graph->getPathFromCentralTo(closest->getId(), tspPath);
    tspNodes.erase(tspNodes.begin());

    // Get the first request
    Request* r = findRequest(closest, reqs);

    // Set the real arrival time for the first request
    if(r != nullptr){
        // If the startTime is equal to the arrival it means the wagon is ready to leave before the arrival
        if(startTime == r->getArrival())
            startTime = startTime - Time(0, 0, totalDist / averageVelocity);

        r->setRealArrival(startTime + Time(0, 0, totalDist / averageVelocity));
    }

    while(!tspNodes.empty()) {
        // Get the nearest point
        Vertex *next = getNearestNeighbour(closest, tspNodes);

        //Calculate the distance from the previous point to the new one using Dijkstra Bidirectional
        this->graph->dijkstraBidirectional(closest->getId(), next->getId(), processedEdges, processedInvEdges);
        totalDist += this->graph->getPathTo(next->getId(), tspPath);

        // Set the real arrival time of the request belonging to that vertex
        r = findRequest(next, reqs);
        if(r != nullptr)
            r->setRealArrival(startTime + Time(0, 0, totalDist / averageVelocity));

        tspNodes.erase(std::find(tspNodes.begin(), tspNodes.end(), next));
        closest = next;
    }

    // Calculate the distance from the last point to the drop off point
    this->graph->dijkstraBidirectional(closest->getId(), dropOffNode, processedEdges, processedInvEdges);
    totalDist += this->graph->getPathTo(dropOffNode, tspPath);

    // Set the real deliver time all the reqs (they are all delivered at the same point so it will be equal to everyone)
    for(Request* req : reqs){
        req->setRealDeliver(startTime + Time(0, 0, totalDist / averageVelocity));
        this->requests.erase(req);
    }

    return totalDist;
}

Delivery* MeatWagons::drawDeliveriesFromThread(int wagonIndex, int deliveryIndex) {
    thread threadProcess(&MeatWagons::drawDeliveries, this, wagonIndex, deliveryIndex);
    threadProcess.detach();
    return next(this->wagons.begin(), wagonIndex)->getDeliveries().at(deliveryIndex);
}

bool MeatWagons::drawDeliveries(int wagonIndex, int deliveryIndex) {
    if(wagonIndex > this->wagons.size()) return false;

    this->viewer->newGv();
    Delivery * delivery = next(this->wagons.begin(), wagonIndex)->getDeliveries().at(deliveryIndex);

    for(auto request : delivery->getRequests()) {
        stringstream stream;
        stream << request->getRealArrival();
        this->graph->findVertex(request->getDest())->setTag(Vertex::PICKUP);
        this->viewer->getViewer()->setVertexLabel(request->getDest(), request->getPrisoner() + " arrival at: " + stream.str());
        this->graph->findVertex(delivery->getDropOff())->setTag(Vertex::DROPOFF);
    }

    vector<int> path = Edge::getIds(delivery->getForwardPath());
    this->viewer->setPath(path, "blue", true);
    this->viewer->draw(this->graph);

    for(auto request : delivery->getRequests()) {
        this->graph->findVertex(request->getDest())->setTag(Vertex::INTEREST_POINT);
        this->graph->findVertex(delivery->getDropOff())->setTag(Vertex::INTEREST_POINT);
    }

    return true;
}




/**
 * This iteration 1 wagon with capacity = 1, this is, it only delivers one prisioner at a time
 * It uses dijkstra bidirectional to calculate the shortest path from the central to the drop off node passing through
 * the pick up node and back to the central again.
 */
bool MeatWagons::firstIteration() {
    if(wagons.size() != 1)  return false;
    if(wagons.begin()->getCapacity() != 1)  return false;

    unordered_set<int> processedEdges, processedInvEdges;

    // Initialize the wagons that will be used
    for(Wagon wagon : this->wagons) wagon.init();

    // Iterate until all the requests are processed
    while(!requests.empty()) {
        // requests are ordered by pickup time
        Request *request = *requests.begin();

        // Get the wagon that has the maximum capacity (the wagons are ordered)
        auto wagonIt = --this->wagons.end();
        auto wagon = *wagonIt;
        this->wagons.erase(wagonIt);

        vector<Edge> edgesForwardTrip;

        // Calculates the path from the central to the prisioner
        int distToPrisoner = this->graph->getPathFromCentralTo(request->getDest(), edgesForwardTrip);

        // Choose a drop off node
        int dropOffNode = chooseDropOff({this->graph->findVertex(request->getDest())});

        /* Calculate the distance from the prisioner node to the drop off node */
        this->graph->dijkstraOrientedSearch(request->getDest(), dropOffNode, processedEdges);
        int dropOffDist = graph->getPathTo(dropOffNode, edgesForwardTrip);
        int totalDist = dropOffDist + distToPrisoner;

        /* Calculate the distance from the drop off node back to the central */
        this->graph->dijkstraOrientedSearch(dropOffNode, central, processedEdges);
        totalDist += this->graph->getPathTo(central, edgesForwardTrip);

        // The wagon leaves either when it returns from a trip or when it as time to travel to the first pick up node
        Time startTime = wagon.getDeliveries().size() > 0 ? wagon.getNextAvailableTime() : request->getArrival() - Time(0, 0, distToPrisoner / averageVelocity);

        // Set the real arrival and deliver based on the startTime
        request->setRealArrival(startTime + Time(0, 0 , distToPrisoner / averageVelocity));
        request->setRealDeliver(request->getRealArrival() + Time(0, 0, dropOffDist / averageVelocity));
        wagon.setNextAvailableTime(startTime + Time(0, 0, totalDist / averageVelocity));

        vector<Request *> vr;
        vr.push_back(request);

        // Create the delivery and add it to the wagon
        Delivery *delivery = new Delivery(startTime, vr, edgesForwardTrip, totalDist / averageVelocity, dropOffNode, totalDist);
        wagon.addDelivery(delivery);

        // wagon now is back at the central
        this->wagons.insert(wagon);
        requests.erase(request);
    }

    return true;
}

/**
 * This iteration 1 wagon with capacity > 1 to deliver the prisioners.
 * It uses dijkstra bidirectional to calculate the shortest path through all the nodes that it needs to pass
 * from the central to the drop Off node and back to the central.
 */
bool MeatWagons::secondIteration() {
    if(wagons.size() != 1)  return false;
    if(wagons.begin()->getCapacity() <= 1)  return false;

    unordered_set<int> processedEdges, processedInvEdges;

    // Initialize the wagons that will be used
    for(Wagon wagon : this->wagons) wagon.init();

    while(!requests.empty()) {
        // Get the wagon that has the maximum capacity (the wagons are ordered)
        auto wagonIt = --this->wagons.end();
        auto wagon = *wagonIt;
        this->wagons.erase(wagonIt);

        // Groupes all the nearest requests into a vector and sorts it by arrival time
        vector<Request *> groupedRequests = groupRequests(wagon.getCapacity());
        sort(groupedRequests.begin(), groupedRequests.end(), compareRequests);

        // Populates the tspNodes with the vertex where the prisioners are
        vector<Vertex*> tspNodes;
        for(auto r : groupedRequests) {
            Vertex *tspNode = this->graph->findVertex(r->getDest());
            tspNodes.push_back(tspNode);
        }

        // The wagon leaves either when it returns from a trip or when it as time to travel to the first pick up node
        // The startTime will be changed in tspPath function if the wagon as time to travel to the first pick up node
        Time startTime = wagon.getDeliveries().size() > 0 ?wagon.getNextAvailableTime() : groupedRequests[0]->getArrival();

        // Choose a drop off node
        int dropOffNode = chooseDropOff(tspNodes);
        vector<Edge> tspPath;

        // Calculate the distance using dijkstra Bidirectional
        int totalDist = this->tspPath(tspNodes, groupedRequests, tspPath, dropOffNode, startTime);

        // Calculate the distance from the drop off node back to the central
        this->graph->dijkstraOrientedSearch(dropOffNode, central, processedEdges);
        totalDist += this->graph->getPathTo(central, tspPath);
        wagon.setNextAvailableTime(startTime + Time(0, 0, totalDist / averageVelocity));

        // Add the delivery to the wagon
        Delivery *delivery = new Delivery(startTime, groupedRequests, tspPath, totalDist / averageVelocity, dropOffNode, totalDist);
        wagon.addDelivery(delivery);

        // wagon now is back at the central
        this->wagons.insert(wagon);
    }

    return true;
}

/**
 * This iteration using more than 1 wagon with different capacity to deliver the prisioners.
 * It uses dijkstra bidirectional to calculate the shortest path through all the nodes that it needs to pass
 * from the central to the drop Off node and back to the central.
 */
bool MeatWagons::thirdIteration() {
    if(wagons.size() <= 1)  return false;

    unordered_set<int> processedEdges, processedInvEdges;
    for(Wagon wagon : this->wagons) wagon.init();

    while(!requests.empty()) {
        // get wagon with max capacity and the sooner available
        auto wagonIt = getWagon();
        auto wagon = *wagonIt;
        this->wagons.erase(wagonIt);

        // Groups all the nearest requests into a vector and sorts it by arrival time
        vector<Request *> groupedRequests = groupRequests(wagon.getCapacity());
        sort(groupedRequests.begin(), groupedRequests.end(), compareRequests);

        // Populates the tspNodes with the vertex where the prisioners are
        vector<Vertex*> tspNodes;
        for(auto r : groupedRequests) {
            Vertex *tspNode = this->graph->findVertex(r->getDest());
            tspNodes.push_back(tspNode);
        }

        // The wagon leaves either when it returns from a trip or when it as time to travel to the first pick up node
        // The startTime will be changed in tspPath function if the wagon as time to travel to the first pick up node
        Time startTime = wagon.getDeliveries().size() > 0 ? wagon.getNextAvailableTime() : groupedRequests[0]->getArrival();

        // Choose a drop off node
        int dropOffNode = chooseDropOff(tspNodes);
        vector<Edge> tspPath;

        // Calculate the distance using dijkstra Bidirectional
        int totalDist = this->tspPath(tspNodes, groupedRequests, tspPath, dropOffNode, startTime);

        // Calculate the distance from the drop off node back to the central
        this->graph->dijkstraOrientedSearch(dropOffNode, central, processedEdges);
        totalDist += this->graph->getPathTo(central, tspPath);
        wagon.setNextAvailableTime(startTime + Time(0, 0, totalDist / averageVelocity));

        // Add the delivery to the wagon
        Delivery *delivery = new Delivery(startTime, groupedRequests, tspPath, totalDist / averageVelocity, dropOffNode, totalDist);
        wagon.addDelivery(delivery);

        // wagon now is back at the central
        this->wagons.insert(wagon);
    }

    return true;
}

/**
 * @brief Searches for the wagon that has de biggest capacity and is available sooner
 * @return An iterator pointing to a wagon
 */
multiset<Wagon>::iterator MeatWagons::getWagon(){
    multiset<Wagon>::iterator itr = --this->wagons.end(), it = itr;
    Time min_time = itr->getNextAvailableTime();

    for(itr; itr != wagons.begin(); itr--){
        if(itr->getNextAvailableTime() == Time()){
            return itr;
        }

        if(itr->getNextAvailableTime() < min_time){
            it = itr;
            min_time = itr->getNextAvailableTime();
        }
    }

    if(wagons.begin()->getNextAvailableTime() <= Time()){
        return wagons.begin();
    }

    if(wagons.begin()->getNextAvailableTime() < min_time){
        return wagons.begin();
    }

    return it;
}

/**
 * @brief Calculates the score of the grouping function based on the distance the wagon travels and the empty spaces
 * @return
 */
int MeatWagons::objectiveFunction() {
    int sum = 0;

    for(Wagon w : this->wagons)
        for(Delivery *d: w.getDeliveries())
            sum += d->getTotalDist() + w.getSpaceLeft();

    return sum;
}

bool MeatWagons::getViewerStatus(string &status) {
	if(viewer->getStatus()) status = "ON";
	else status = "OFF";

	return viewer->getStatus();
}

void MeatWagons::setViewerStatus(bool status) {
    viewer->setStatus(status);
}

#endif //MEAT_WAGONS_MEATWAGONS_H