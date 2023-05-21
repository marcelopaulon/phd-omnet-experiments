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
#include "inet/common/geometry/common/GeographicCoordinateSystem.h"

#include <iostream>
#include <fstream>

namespace projeto {

//Register_Class(GroundStation);
Define_Module(GroundStation);

class MyVisitor : public cVisitor {
public:
    std::vector<double> x_coords;
    std::vector<double> y_coords;
    int numDrones = 0;

    Coord groundStationPosition;

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
        } else if (strcmp(obj->getName(), "groundStation") == 0) {
            cModule *module = check_and_cast<cModule*>(obj);
            cObject *mobilityObj = module->findObject("mobility");
            StationaryMobility *mobility = check_and_cast<StationaryMobility*>(mobilityObj);
            groundStationPosition = mobility->getCurrentPosition();
        }
    }

    void saveTempFile(std::string tempFileName) {
        std::ofstream outputFile(tempFileName); // Open the file for writing

        if (outputFile.is_open()) {
            outputFile << "NAME : OMNETRUN" << std::endl;
            outputFile << "COMMENT : 524.61" << std::endl;
            outputFile << "TYPE : CVRP" << std::endl;
            int nSensors = x_coords.size();
            outputFile << "DIMENSION : " << (nSensors + 1) << std::endl; // dimension = nSensors + nGroundStations
            outputFile << "EDGE_WEIGHT_TYPE : EUC_2D" << std::endl;
            int capacity = nSensors + 1; // add 1 for the ground station
            outputFile << "CAPACITY : " << capacity << std::endl;
            outputFile << "NODE_COORD_SECTION" << std::endl;

            outputFile << "1 " << groundStationPosition.x << " " << groundStationPosition.y << std::endl;
            for (int i = 0; i < nSensors; i++) {
                outputFile << (i + 2) << " " << x_coords[i] << " " << y_coords[i] << std::endl;
            }
            outputFile << "DEMAND_SECTION" << std::endl;
            outputFile << "1 0" << std::endl; // Ground station has demand=0
            for (int i = 0; i < nSensors; i++) {
                outputFile << (i + 2) << " 1" << std::endl;
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

        cObject *mobilityObj = this->findObject("mobility");
        StationaryMobility *mobility = check_and_cast<StationaryMobility*>(mobilityObj);

        auto coordinateSystem = getModuleFromPar<IGeographicCoordinateSystem>(mobility->par("coordinateSystemModule"), mobility, false);

        std::string tempFileName = "instanceTemp.vrp";
        visitor.saveTempFile(tempFileName);

        InstanceCVRPLIB cvrp(tempFileName, true);
        int nbVeh = visitor.numDrones;
        bool verbose = true;
        AlgorithmParameters ap = default_algorithm_parameters();
        xRouteCoords = cvrp.x_coords;
        yRouteCoords = cvrp.y_coords;
        Params params(xRouteCoords,yRouteCoords,cvrp.dist_mtx,cvrp.service_time,cvrp.demands,
                      cvrp.vehicleCapacity,cvrp.durationLimit,nbVeh,cvrp.isDurationConstraint,verbose,ap);

        // Running HGS
        Genetic solver(params);
        solver.run();

        // Exporting the best solution
        if (solver.population.getBestFound() != NULL)
        {
            auto indiv = *solver.population.getBestFound();

            solver.population.exportCVRPLibFormat(indiv, tempFileName + "-result");

            routes.clear();
            for (int k = 0; k < (int)indiv.chromR.size(); k++)
            {
                std::string outputFileName = "paths/tempAutoRouteUav" + std::to_string(k) + ".waypoints";
                std::ofstream outputFile(outputFileName); // Open the file for writing

                if (!indiv.chromR[k].empty())
                {
                    std::vector<int> route;
                    for (int i : indiv.chromR[k]) route.push_back(i);

                    if (outputFile.is_open()) {
                        outputFile << "QGC WPL 110" << std::endl;
                        double gsx = coordinateSystem->computeGeographicCoordinate(visitor.groundStationPosition).latitude.get();
                        double gsy = coordinateSystem->computeGeographicCoordinate(visitor.groundStationPosition).longitude.get();
                        outputFile << "0\t0\t0\t16\t0\t0\t0\t0\t" << gsx  << "\t" << gsy << "\t0\t1" << std::endl;
                        outputFile << "1\t0\t0\t22\t0\t0\t0\t0\t0\t0\t0\t1" << std::endl; // TAKE-OFF
                        outputFile << "2\t0\t0\t16\t0\t0\t0\t0\t" << gsx  << "\t" << gsy << "\t20\t1" << std::endl;
                        int nextQGC = 3;
                        for (int i : route) {
                            double scx = xRouteCoords[i];
                            double scy = yRouteCoords[i];
                            auto geoCoord = coordinateSystem->computeGeographicCoordinate(Coord(scx, scy, 0.0));
                            double cx = geoCoord.latitude.get();
                            double cy = geoCoord.longitude.get();

                            outputFile << nextQGC << "\t0\t0\t16\t0\t0\t0\t0\t" << cx  << "\t" << cy << "\t20\t1" << std::endl;
                            nextQGC++;
                        }

                        outputFile << nextQGC << "\t0\t3\t120\t0\t0\t0\t0\t0\t0\t0\t1" << std::endl; // Return to home

                        outputFile.close(); // Close the file
                        std::cout << "Data written to the file successfully." << std::endl;
                    } else {
                        std::cerr << "Error opening the file." << std::endl;
                    }

                    routes.push_back(route);
                } else {
                    if (outputFile.is_open()) {
                        outputFile << "QGC WPL 110" << std::endl;
                        double gsx = coordinateSystem->computeGeographicCoordinate(visitor.groundStationPosition).latitude.get();
                        double gsy = coordinateSystem->computeGeographicCoordinate(visitor.groundStationPosition).longitude.get();
                        outputFile << "0\t0\t0\t16\t0\t0\t0\t0\t" << gsx  << "\t" << gsy << "\t0\t1" << std::endl;
                        outputFile.close(); // Close the file
                        std::cout << "Data written to the file successfully." << std::endl;
                    } else {
                        std::cerr << "Error opening the file." << std::endl;
                    }
                }
            }

            routeCost = indiv.eval.penalizedCost;
        }
    }
    catch (const string& e) { std::cout << "EXCEPTION | " << e << std::endl; }
    catch (const std::exception& e) { std::cout << "EXCEPTION | " << e.what() << std::endl; }
}

} //namespace
