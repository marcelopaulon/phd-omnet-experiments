#include <omnetpp.h>
#include "MobileNode.h"
#include "stdlib.h"
#include <time.h>
#include "GroundStation.h"

#include "cvrp/Genetic.h"
#include "cvrp/LocalSearch.h"
#include "cvrp/Split.h"
#include "cvrp/InstanceCVRPLIB.h"

#include <inet/mobility/static/StationaryMobility.h>
#include <inet/mobility/contract/IMobility.h>
#include <inet/mobility/base/MobilityBase.h>

#include <iostream>
#include <fstream>

namespace projeto {

//Register_Class(GroundStation);
Define_Module(GroundStation);

class MyVisitor : public cVisitor {
public:
    std::vector<double> x_coords;
    std::vector<double> y_coords;
    int numDrones;

    void visit(cObject *obj) override {
        EV << "Object name: " << obj->getFullName() << endl;

        if (strcmp(obj->getName(), "sensors") == 0) {
            cModule *module = check_and_cast<cModule*>(obj);
            cObject *mobilityObj = module->findObject("mobility");
            StationaryMobility *mobility = check_and_cast<StationaryMobility*>(mobilityObj);
            Coord position = mobility->getCurrentPosition();

            double x = position.x;
            double y = position.y;
            double z = position.z;

            EV << "Sensor position: (" << x << ", " << y << ", " << z << ")" << endl;
            x_coords.push_back(x);
            y_coords.push_back(y);
        } else if (strcmp(obj->getName(), "quads") == 0) {
            numDrones++;
            EV << "Drone loaded" << endl;
        }
    }

    void saveTempFile(std::string tempFileName) {
        std::ofstream outputFile(tempFileName); // Open the file for writing

        if (outputFile.is_open()) {
            outputFile << "NAME : OMNETRUN" << std::endl;
            outputFile << "COMMENT : 524.61" << std::endl;
            outputFile << "TYPE : CVRP" << std::endl;
            int nSensors = x_coords.size();
            outputFile << "DIMENSION : " << nSensors << std::endl;
            outputFile << "EDGE_WEIGHT_TYPE : EUC_2D" << std::endl;
            int capacity = nSensors;
            outputFile << "CAPACITY : " << capacity << std::endl;
            outputFile << "NODE_COORD_SECTION" << std::endl;
            for (int i = 0; i < nSensors; i++) {
                outputFile << (i + 1) << " " << x_coords[i] << " " << y_coords[i] << std::endl;
            }
            outputFile << "DEMAND_SECTION" << std::endl;
            for (int i = 0; i < nSensors; i++) {
                outputFile << (i + 1) << " 1" << std::endl;
            }
            outputFile << "DEPOT_SECTION" << std::endl;
            outputFile << "1" << std::endl;
            outputFile << "-1" << std::endl;
            outputFile << "EOF" << std::endl;

            outputFile.close(); // Close the file
            std::cout << "Data written to the file successfully." << std::endl;
        } else {
            std::cerr << "Error opening the file." << std::endl;
        }
    }
};

void GroundStation::initialize(){
    // TO-DO: these ID shaw be UUID as in network
    internalMobNodeId = this->getId() -  par("simulationIndexOfFirstNode").intValue() + 1;
    par("internalMobNodeId").setIntValue(internalMobNodeId);

    //this->myType = static_cast<mobileNodeType>(par("nodeType").intValue());

    // Schedule a self-message to be delivered after the initialization process is complete
    cMessage *postInitMsg = new cMessage("PostInitializationMessage");
    scheduleAt(simTime(), postInitMsg); // Schedule the message at current simulation time


    std::cout << "UAV initialization of internalMobNodeId: " << internalMobNodeId << " Class " << this->getClassName() << "." << endl;
}
int GroundStation::processMessage(inet::Packet *msg) {

    // O getname � o payload
    std::cout  << "GROUND-STATION-" << internalMobNodeId << " received: " << msg->getName() << endl;

    return 1;

}

string GroundStation::generateNextPacketToSend(){

    std::ostringstream payload;

    payload << "Hi from " << "GROUND-STATION-" <<  internalMobNodeId << "{" << ++sentMsgs << "}" << endl;

    return payload.str().c_str();
};

void GroundStation::handleMessage(cMessage *msg) {

    std::cout  << " GroundStation::handleMessage: " << msg << endl;
    if (msg->isSelfMessage()) {
        if (strcmp(msg->getName(), "PostInitializationMessage") == 0) {
            doPostInitializationTasks();
        }
        delete msg;
    }
}

void GroundStation::doPostInitializationTasks() {
    try
    {
        cModule *topModule = getModuleByPath("<root>");  // Get the top-level module

        MyVisitor visitor;
        topModule->forEachChild(&visitor);

        std::string tempFileName = "instanceTemp.vrp";
        visitor.saveTempFile(tempFileName);

        InstanceCVRPLIB cvrp(tempFileName, true);
        int nbVeh = visitor.numDrones;
        bool verbose = true;
        AlgorithmParameters ap;
        Params params(cvrp.x_coords,cvrp.y_coords,cvrp.dist_mtx,cvrp.service_time,cvrp.demands,
                      cvrp.vehicleCapacity,cvrp.durationLimit,nbVeh,cvrp.isDurationConstraint,verbose,ap);

        // Running HGS
        Genetic solver(params);
        solver.run();

        // Exporting the best solution
        if (solver.population.getBestFound() != NULL)
        {
            //if (params.verbose) std::cout << "----- WRITING BEST SOLUTION IN : " << commandline.pathSolution << std::endl;
            //solver.population.exportCVRPLibFormat(*solver.population.getBestFound(),commandline.pathSolution);
            //solver.population.exportSearchProgress(commandline.pathSolution + ".PG.csv", commandline.pathInstance);
        }
    }
    catch (const string& e) { std::cout << "EXCEPTION | " << e << std::endl; }
    catch (const std::exception& e) { std::cout << "EXCEPTION | " << e.what() << std::endl; }

}

} //namespace
