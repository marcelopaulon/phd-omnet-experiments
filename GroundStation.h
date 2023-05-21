#include "inet/common/INETDefs.h"


#ifndef __GROUNDSTATION_H_
#define __GROUNDSTATION_H_

#include <omnetpp.h>
//#include <cModule.h>
#include "inet/common/ModuleAccess.h"
#include "inet/common/packet/Packet.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/transportlayer/contract/udp/UdpControlInfo_m.h"
#include "inet/applications/base/ApplicationBase.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"
#include "inet/applications/base/ApplicationBase.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"
#include <iostream>
#include <algorithm>
#include <iterator>

using std::cout; using std::endl;
using std::string; using std::reverse;


using namespace omnetpp;
using namespace inet;

namespace projeto {

//enum mobileNodeType { sensor = 1, uav = 2, baseStation = 3, missing = 171 };

class GroundStation : public cSimpleModule  {
  private:
    std::vector<double> xRouteCoords;
    std::vector<double> yRouteCoords;
    std::vector<std::vector<int>> routes;
    double routeCost = 0.0;
    void doPostInitializationTasks();
  protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg);
  public:
    int processMessage(inet::Packet *msg);
    string generateNextPacketToSend();
    int internalMobNodeId = -1;
    long sentMsgs = -1;
  //  mobileNodeType myType = missing;

};

} //namespace
#endif
