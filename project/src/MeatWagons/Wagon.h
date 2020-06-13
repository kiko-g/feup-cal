#pragma once
#ifndef WAGON_H_
#define WAGON_H_

#include "Delivery.h"

class Wagon {
    private:
        int id, capacity;
        Time nextAvailableTime;
        vector<Delivery*> deliveries;

    public:
        Wagon(const int id, const int capacity) : id(id), capacity(capacity) {}

        int getId() const;
        int  getCapacity() const;
        Time getNextAvailableTime() const;
        void setNextAvailableTime(const Time &time);
        const vector<Delivery*> getDeliveries() const;
        void addDelivery(Delivery* delivery);
        int getSpaceLeft() const;

        void init();
        bool operator<(const Wagon &wagon) const;
};

int Wagon::getId() const {
    return this->id;
}

int  Wagon::getCapacity() const {
    return this->capacity;
}

Time Wagon::getNextAvailableTime() const {
    return this->nextAvailableTime;
}

void Wagon::setNextAvailableTime(const Time &time) {
    this->nextAvailableTime = time;
}

const vector<Delivery*> Wagon::getDeliveries() const {
    return this->deliveries;
}

void Wagon::addDelivery(Delivery *delivery) {
    this->deliveries.push_back(delivery);
}

int Wagon::getSpaceLeft() const {
    return this->capacity - this->deliveries.size();
}

void Wagon::init() {
    this->deliveries.clear();
    this->nextAvailableTime = Time();
}

bool Wagon::operator<(const Wagon &wagon) const {
    if(this->capacity == wagon.capacity) return this->id < wagon.id;
    return this->capacity < wagon.capacity;
}

#endif