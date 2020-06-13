#ifndef UTILS_H
#define UTILS_H
#include <sstream>
#include <fstream>
#include "MeatWagons/MeatWagons.h"

#ifdef _WIN32
#define clearScreen() system("cls");
#else
#define clearScreen() system("clear");
#endif

void readline(string &str) {
    str.clear();
    cin.clear();
    fflush(stdin);
    getline(cin, str);
    while (str.empty()) getline(cin, str);
}

int stoint(const string &str, int &value) {
    // wrapping stoi because it may throw an exception
    try {
      value = stoi(str, nullptr, 10);
      return 0;
    }
    catch (const invalid_argument &ia) { return -1; }
    catch (const out_of_range &oor) { return -2; }
    catch (const exception &e) { return -3; }
}

#endif //UTILS_H
