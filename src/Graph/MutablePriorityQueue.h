#pragma once
#ifndef SRC_MUTABLEPRIORITYQUEUE_H_
#define SRC_MUTABLEPRIORITYQUEUE_H_

#include <vector>


using namespace std;

/**
 * class T must have: (i) accessible field int queueIndex; (ii) operator< defined.
 */

template <class T>
class MutablePriorityQueue {
    vector<T *> H;
    void heapifyUp(unsigned i);
    void heapifyDown(unsigned i);
    inline void set(unsigned i, T * x);

public:
    MutablePriorityQueue();
    void insert(T * x);
    T * extractMin();
    void decreaseKey(T * x);
    bool empty();
    bool inv = false;

    void setInv(bool t);
};

// Index calculations
#define parent(i) ((i) / 2)
#define leftChild(i) ((i) * 2)

template<class T>
void MutablePriorityQueue<T>::setInv(bool t) {
    this->inv = t;
}

template <class T>
MutablePriorityQueue<T>::MutablePriorityQueue() {
    H.push_back(nullptr);
    // indices will be used starting in 1
    // to facilitate parent/child calculations
}

template <class T>
bool MutablePriorityQueue<T>::empty() {
    return H.size() == 1;
}

template <class T>
T* MutablePriorityQueue<T>::extractMin() {
    auto x = H[1];
    H[1] = H.back();
    H.pop_back();
    heapifyDown(1);
    if(inv){
        x->invQueueIndex = 0;
    }
    else{
        x->queueIndex = 0;
    }
    return x;
}

template <class T>
void MutablePriorityQueue<T>::insert(T *x) {
    H.push_back(x);
    heapifyUp(H.size()-1);
}

template <class T>
void MutablePriorityQueue<T>::decreaseKey(T *x) {
    if(inv){
        heapifyUp(x->invQueueIndex);

    }
    else{
        heapifyUp(x->queueIndex);
    }
}

template <class T>
void MutablePriorityQueue<T>::heapifyUp(unsigned i) {
    auto x = H[i];
    while (i > 1 && *x < *H[parent(i)]) {
        set(i, H[parent(i)]);
        i = parent(i);
    }
    set(i, x);
}

template <class T>
void MutablePriorityQueue<T>::heapifyDown(unsigned i) {
    auto x = H[i];
    while (true) {
        unsigned k = leftChild(i);
        if (k >= H.size())
            break;
        if (k+1 < H.size() && *H[k+1] < *H[k])
            ++k; // right child of i
        if ( ! (*H[k] < *x) )
            break;
        set(i, H[k]);
        i = k;
    }
    set(i, x);
}

template <class T>
void MutablePriorityQueue<T>::set(unsigned i, T * x) {
    H[i] = x;
    if(inv){
        x->invQueueIndex = i;
    }
    else{
        x->queueIndex = i;
    }
}

#endif