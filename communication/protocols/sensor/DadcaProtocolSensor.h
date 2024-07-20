//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef __PROJETO_DadcaProtocolSensor_H_
#define __PROJETO_DadcaProtocolSensor_H_

#include <omnetpp.h>
#include "../base/CommunicationProtocolBase.h"
#include "../../messages/network/DadcaMessage_m.h"
#include "inet/common/geometry/common/Coord.h"

#include <random>
#include <iomanip>
#include <iostream>

using namespace omnetpp;

namespace projeto {

/*
 * DadcaProtocol implements a protocol that recieves and sends DadcaMessages to simulate a
 * drone collecting data from sensors and sharing it with other drones. This protocol implements
 * the DADCA protocol.
 */
class DadcaProtocolSensor : public CommunicationProtocolBase
{
    protected:
        // Current target
        int tentativeTarget = -1;
        // Name of the current target (for addressing purposes)
        std::string tentativeTargetName;

        std::string curMessageIds = "";

        DadcaMessage lastPayload = DadcaMessage();

        int Messages = 0;

    protected:
        virtual void initialize(int stage) override;

        // Sensor does not recieve telemetry
        virtual void handleTelemetry(projeto::Telemetry *telemetry) override { return; };
        // Reacts to message recieved and updates payload accordingly
        virtual void handlePacket(Packet *pk) override;

        virtual void handleMessage(cMessage *msg) override;
    private:
        void sendSelfGenPacket();

        virtual void sendMessage(const char *target);

        static std::random_device rd;
        static std::mt19937 gen;
        static std::uniform_int_distribution<> dis;

        std::string generateUUID();

        DadcaMessage *everySecondControlPacket = nullptr;
    public:
        simsignal_t dataLoadSignalID;
        DadcaProtocolSensor();
};

// Define static members outside the class
std::random_device DadcaProtocolSensor::rd;
std::mt19937 DadcaProtocolSensor::gen(DadcaProtocolSensor::rd());
std::uniform_int_distribution<> DadcaProtocolSensor::dis(0, 15);


} //namespace

#endif
