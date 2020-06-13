#pragma once
#ifndef MEAT_WAGONS_DELIVERY_H
#define MEAT_WAGONS_DELIVERY_H

class Delivery {
    private:
        Time start, end;
        vector<Request*> requests;
        vector<Edge> forwardPath;
        int dropOff, totalDist;

    public:
        Delivery(Time &start, vector<Request*> &requests, vector<Edge> &forwardPath, int weight, int dropOff, int totalDist = 0) {
            this->start = start;
            this->requests = requests;
            this->forwardPath = forwardPath;
            this->end = start + Time(0, 0, weight);
            this->dropOff = dropOff;
            this->totalDist = totalDist;

        }

        Time getStart() const;
        Time getEnd() const;
        vector<Request*> getRequests() const;
        vector<Edge> getForwardPath() const;
        int getDropOff() const;
        int getTotalDist() const;
};

Time Delivery::getStart() const {
    return this->start;
}

Time Delivery::getEnd() const {
    return this->end;
}

int Delivery::getTotalDist() const {
    return this->totalDist;
}

vector<Request*> Delivery::getRequests() const {
    return this->requests;
}

vector<Edge> Delivery::getForwardPath() const {
    return this->forwardPath;
}

int Delivery::getDropOff() const {
    return dropOff;
}

#endif //MEAT_WAGONS_DELIVERY_H