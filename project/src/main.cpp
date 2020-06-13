#include <iostream>
#include "Application/Application.h"

using namespace std;

int main(int argc, char* argv[]) {
    Application application = Application();
    while(true) application.run();
}
