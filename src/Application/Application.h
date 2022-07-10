#pragma once
#ifndef MEAT_WAGONS_APPLICATION_H
#define MEAT_WAGONS_APPLICATION_H
#include <sstream>
#include <fstream>
#include <iomanip>
#include "../utils.h"
#include "../MeatWagons/MeatWagons.h"

class Application {
    private:
		string input;
        MeatWagons *controller = new MeatWagons(2);
    public:
        void run();
        void displayMenu();
        void readGraph(); 		//option 1
        void preProcess();  	//option 2
        void shortestPath(); 	//option 3
        void deliver();			//option 4
		void setCentral();		//option 5
		void wagonOperation();	//option 6
		void listRequests();	//option 7
		void toggleShowGraph();
};

void Application::displayMenu()
{
    cout << "\n---------------------------------------------------------------------" << endl;
    cout << "Menu Options: [Type 'back' to go back in any menu]" << endl;
    cout << "1 - Read Graph" << endl;
    if(controller->getGraph() != nullptr) {
		cout << "2 - Pre Process" << endl;
		cout << "3 - Shortest Path" << endl;
		cout << "4 - Deliver" << endl;
		cout << "5 - Set Central" << endl;
		cout << "6 - Wagon Operation" << endl;
		cout << "7 - List Requests" << endl;
	}
	cout << "8 - Toggle Show Graph (On/Off)" << endl;
    cout << "0 - Exit" << endl << endl;

    string status;
    controller->getViewerStatus(status);

    if(controller->getGraph() == nullptr) cout << "Graph not read yet!" << endl;
    else cout << "Graph read for '" << controller->getGraphName() << "' " 
              << "(central node ID = " << controller->getCentral() << ")" << endl;
        
    cout << "Show Graph option status: " << status << endl;

    cout << "---------------------------------------------------------------------" << endl;
    cout << "\bInput: > ";
}

void Application::readGraph() {
	cout << endl << "--- Read Graph ---";
	cout << endl << "Provide city name [Example: 'Porto']";
	cout << endl << "\bInput: > ";
	
	int tries = 0;
	while(true) {
		readline(input);
		tries++;
		if(input == "back") break;
		else if (input == "Aveiro" || input == "Braga" || input == "Coimbra" ||
		input == "Ermesinde" || input == "Fafe" || input == "Gondomar" ||
		input == "Lisboa" || input == "Maia" || input == "Porto" ||
		input == "Viseu" || input == "Portugal") {
			controller->setGraph("maps/PortugalMaps/" + input);
			break;
		}
		else if(input == "4x4" || input == "8x8" || input == "16x16") {
			controller->setGraph("maps/GridGraphs/" + input);
			break;
		}
		else if(tries < 3) cout << "\bInput: > ";
		else {
			controller->setGraph("input");
			break;
		}
	}
	
}
void Application::preProcess() {
	cout << endl << "--- Processing node ---";
	cout << endl << "Provide <node id> [Example: 'central']>";
	cout << endl << "\bInput: > ";
	
	int node;
	while(true) {
		readline(input);
		if(input == "back") break;
		else if(input == "central") {
			controller->preProcess(controller->getCentral(), true);
			break;
		}
		else if(stoint(input, node) == 0) {
			if(!controller->preProcess(node, true)) cout << "\bInput: > ";
			else break;
		}
		else cout << "\bInput: > ";
	}
	return;
}
void Application::shortestPath() {
	bool back;
	while(true) {
		int choice;
		cout << endl << "--- Shortest Path ---";
		cout << endl << "1 - Classic Dijkstra";
		cout << endl << "2 - Oriented Dijkstra (A*)";
		cout << endl << "3 - Bidirectional Dijkstra";
		cout << endl << "\bInput: > ";
		readline(input);
		
		if(input == "back") break;
		else if(stoint(input, choice) == 0 && choice >= 1 && choice <= 3) {
			int origin, dest;
			cout << endl << "\nProvide <origin node> <destination node> [Example: 90379359 411018963]";
			cout << endl << "\bInput: > ";
			while(true) {
				string secondInput;
				readline(secondInput);
				if(secondInput == "back") {
					back = false;
					break;
				}
				
				stringstream line(secondInput);
				if (line >> origin && line >> dest && controller->shortestPath(choice, origin, dest)) {
					back = true;
					break;
				}
				else cout << "\bInput: > ";
			}
		}
		else {
			cout << "\bInput: > ";
			continue;
		}
		if(back) break;
	}
	return;
}
void Application::deliver() {
	while(true) { //option 1 and 2 dont require configuration 
		cout << endl << "--- Delivering --- [Number of Wagons = " << controller->getWagons().size() << "]" ;
		cout << endl << "1 - Single Wagon with capacity 1 [Restrictions: 1 wagon with capacity 1]";
		cout << endl << "2 - Single Wagon that groups requests [Restrictions: 1 wagon with capacity > 1]";
		cout << endl << "3 - Multiple Wagons that groups requests [Restrictions: > 1 wagon]";
		cout << endl << "4 - Set Maximum Distance between Deliveries [Current ZoneMaxDist = " << controller->getMaxDist() << "]" << endl;
		cout << "\n\t--- Current Wagons List ---" << endl;
		multiset<Wagon> wagons = this->controller->getWagons();
		for(const auto & wagon : wagons) {
			cout << "\t[Wagon " << wagon.getId() << "] with capacity " << wagon.getCapacity() << endl;
		}
		cout << endl;
		
        cout << endl << "\bInput: > ";
		readline(input);

		int choice = -1;
		if(input == "back") break;
		else if(stoint(input, choice) == 0 && choice >= 0 && choice <= 4) {
			if (choice != 4) {
				if(choice == 1) controller->setWagons(1, 1);
				if(choice == 2) {
					bool brk = false;
					int capacity = -1;
					string capacity_str;
					cout << "\nProvide <wagon capacity>" << endl;
					while(true) {
						cout << "\bInput: > ";
						readline(capacity_str);
						if(capacity_str == "back") {
							brk = true;
							break;
						}
						if(stoint(capacity_str, capacity) == 0 && capacity > 0) break;
					}
					if(brk) break;
					controller->setWagons(1, capacity);
				}

				if(!controller->deliver(choice)) { cout << "Wrong iteration configuration"  << endl; continue; }
                // LIST WAGONS
                wagons = this->controller->getWagons();
                cout << "\n\n\t--- Current Wagons List ---" << endl;
				for(const auto & wagon : wagons) {
					cout << "\t[Wagon " << wagon.getId() << "] with capacity " << wagon.getCapacity() << endl;
				}
				cout << endl;

                int wagonID;
				while (true) {
					cout << endl << "--- Choose Wagon --- [0, " << wagons.size()-1 << "]" << endl;
					cout << "Wagon Index > ";
					
					string wagonInput;
					readline(wagonInput);
					if (wagonInput == "back") break;
					else if (stoint(wagonInput, wagonID) == 0 && wagonID >= 0 & wagonID <= wagons.size()-1) {
						int delivery;
						while (true) {
							cout << endl << "--- Choose a Delivery done by Wagon #" << wagonID << " --- ";
                            int delivIndexMax;
                            for(auto w : wagons)
                                if(wagonID == w.getId()) {
                                    delivIndexMax = w.getDeliveries().size()-1;
                                }
                            cout << "[0, " << delivIndexMax << "]" << endl;
							cout << "Delivery Index > ";
							
							string deliveryInput;
							readline(deliveryInput);
							if (deliveryInput == "back") break;
							else if (stoint(deliveryInput, delivery) == 0 && delivery >= 0 && delivery <= delivIndexMax) {
								Delivery *deliveryChosen = controller->drawDeliveriesFromThread(wagonID, delivery);
                                cout << endl;
								cout << "Wagon[" << wagonID << "] leaves central at: " << deliveryChosen->getStart() << endl;
								cout << "Wagon[" << wagonID << "] returns to central at: " << deliveryChosen->getEnd() << endl;
								cout << "\nRequests done:" << endl;
								for (const Request *r : deliveryChosen->getRequests()) {
									cout << "| Prisoner: " << setfill(' ') << left << setw(11) << r->getPrisoner();
                                    cout << " | Priority: " << left << setw(2) << r->getPriority();
                                    cout << " | Arrives at: " << left << setw(10) << r->getRealArrival();
                                    cout << " | Delivered at: " << left << setw(10) << r->getRealDeliver() << " |" << endl;
                                }
							}
							else cout << endl;
						}
					} else cout << endl;
				}
			} else {
				int newMaxDist;
				cout << "\n--- Setting new max distance ---\nInput > ";
				while (true) {
					readline(input);
					if (input == "back") break;
					else if (stoint(input, newMaxDist) == 0 || newMaxDist >= 1)
						controller->setMaxDist(newMaxDist);
					else cout << endl << "\bInput: > ";
					readline(input);
				}
				break;
			}
		}
		else return;
	}
	
	return;
}
void Application::setCentral() {
	cout << endl << "--- Change Central Node ---";
	cout << endl <<"Current Central Node ID: " << controller->getCentral();
	cout << endl << "Provide <node id>" << endl;
	
	int centralID;
	while(true) {
		cout << "\bInput: > ";
		readline(input);
		if(input == "back") break;
		else if(stoint(input, centralID) == 0 && controller->setCentral(centralID)) break;
		else continue;
	}
	
	return;
}
void Application::wagonOperation() {
	while(true) {
		cout << endl << "--- Wagon Operation --- ";
		cout << endl << "1 - List Wagons";
		cout << endl << "2 - Add Wagons";
		cout << endl << "3 - Remove Wagons";
		cout << endl << "\bInput: > ";
		
		int choice;
		readline(input);
		if(input == "back") break;
		else if(stoint(input, choice) || choice < 1 || choice > 3) continue;

        if(choice == 1) {
            multiset<Wagon> wagons = this->controller->getWagons();
            if(!wagons.empty()) {
                cout << "\n--- Listing Wagons ---" << endl;
                
                for(const auto & wagon : wagons) {
                    cout << "[Wagon " << wagon.getId() << "] Capacity " << wagon.getCapacity() << " available at "
                        << wagon.getNextAvailableTime() << endl;
                }
                cout << endl;
            }
            else cout << "\nNo wagons added!" << endl;
		}
		else if(choice == 2) {
			while(true) {
                cout << endl << "--- Adding a Wagon ---";
                cout << endl << "Provide <capacity>";
				cout << endl << "\bInput: > ";
				
				string secondInput;
				readline(secondInput);
				if(secondInput == "back") break;
				int capacity;
				if(stoint(secondInput, capacity) == 0 && capacity > 0) controller->addWagon(capacity);
				else continue;
			}
		}
		else if(choice == 3) {
			while(true) {
                cout << endl << "--- Removing a Wagon ---";
                cout << endl << "Provide <id>";
				cout << endl << "\bInput: > ";
				
                string secondInput;
				readline(secondInput);
				if(secondInput == "back") break;
				int id;
				stringstream line(secondInput);
				if (line >> id) controller->removeWagon(id);
                else continue;
			}
		}
	}
	return;
}
void Application::listRequests() {
	multiset<Request*> requests = controller->getConstantRequests();
    if(!requests.empty()) {
        cout << "\n--- List Requests ---" << endl;
        for (auto request : requests) {
            cout << "Prisoner " << request->getPrisoner() << " to be received in node " << request->getDest() << " with priority "
                << request->getPriority() << " at " << request->getArrival() << endl;
        }
    }
    else cout << "\nNo requests available!" << endl;
	
	return;
}
void Application::toggleShowGraph() {
    string aux;
	controller->setViewerStatus(!controller->getViewerStatus(aux));
}

void Application::run() 
{
    int option;

    displayMenu();
    while(true) {
        readline(input);
        if(stoint(input, option) == 0 && controller->getGraph() == nullptr && (option == 0 || option == 1 || option == 8)) break;
        else if(stoint(input, option) == 0 && controller->getGraph() != nullptr && option >= 0 && option <= 8) break;
        else cout << "\bInput: > ";
    }

    switch (option) {
        case 0: exit(0);
        case 1: readGraph(); break;
        case 2: preProcess(); break;
        case 3: shortestPath(); break;
        case 4: deliver(); break;
        case 5: setCentral(); break;
        case 6: wagonOperation(); break;
        case 7: listRequests(); break;
        case 8: toggleShowGraph(); break;
        default: break;
    }
}

#endif //MEAT_WAGONS_APPLICATION_H
