#pragma once
#ifndef MEAT_WAGONS_REQUEST_H
#define MEAT_WAGONS_REQUEST_H

#include "Time.h"

using namespace std;

class Request {
    private:
        string prisoner;
        int dest, priority;
        Time arrival, deliver;
        bool assigned = false;
        Time realArrival, realDeliver;
        bool processed;

    public:
        Request() {}
        Request(const string &prisoner, const int dest, const int priority, const Time &arrival) :
            prisoner(prisoner), dest(dest), priority(priority), arrival(arrival) {
            processed = false;
        }

        string getPrisoner() const;
        int getDest() const;
        int getPriority() const;
        Time getArrival() const;
        Time getRealArrival() const;
        Time getRealDeliver() const;

        void setRealArrival(const Time &time);
        void setRealDeliver(const Time &time);

        bool operator<(const Request &request) const;
};

string Request::getPrisoner() const {
    return this->prisoner;
}

int Request::getDest() const {
    return this->dest;
}

int Request::getPriority() const {
    return this->priority;
}

Time Request::getArrival() const {
    return this->arrival;
}

Time Request::getRealArrival() const {
    return this->realArrival;
}

Time Request::getRealDeliver() const {
    return this->realDeliver;
}

void Request::setRealArrival(const Time &time){
    this->realArrival = time;
}

void Request::setRealDeliver(const Time &time){
    this->realDeliver = time;
}

bool Request::operator<(const Request &request) const {
    return this->arrival < request.arrival;
}

#endif //MEAT_WAGONS_REQUEST_H