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

#ifndef __PROJETO_DadcaAckUAVProtocol_H_
#define __PROJETO_DadcaAckUAVProtocol_H_

#include <unordered_map>
#include <omnetpp.h>
#include "../json.hpp"
#include "../base/CommunicationProtocolBase.h"
#include "../../messages/network/DadcaAckMessage_m.h"
#include "inet/common/geometry/common/Coord.h"

using namespace omnetpp;

namespace projeto {

enum CommunicationStatus { FREE=0, REQUESTING=1, PAIRED=2, COLLECTING=3, PAIRED_FINISHED=4 };

/*
 * DadcaProtocol implements a protocol that recieves and sends DadcaAckMessages to simulate a
 * drone collecting data from sensors and sharing it with other drones. This protocol implements
 * the DADCA protocol.
 */
class DadcaAckUAVProtocol : public CommunicationProtocolBase
{
    protected:
        simtime_t timeoutDuration;
        int maxBufferSize;

        // DADCA variables
        // Current tour recieved from telemetry
        std::vector<Coord> tour;

        int leftNeighbours = 0;
        int rightNeighbours = 0;

        // Communication status variable
        CommunicationStatus communicationStatus = FREE;


        // Current target
        int tentativeTarget = -1;
        // Previous target
        int lastTarget = -1;
        // Name of the current target (for addressing purposes)
        std::string tentativeTargetName;

        // Current imaginary data being carried
        int currentDataLoad=0;

        // Current imaginary buffer load
        int currentBufferLoad=0;

        // Stable data load to prevent data loss during pairing
        int stableDataLoad=currentDataLoad;

        // Last telemetry package recieved
        Telemetry currentTelemetry = Telemetry();
        Telemetry lastStableTelemetry = Telemetry();

        DadcaAckMessage lastPayload = DadcaAckMessage();


        std::unordered_map<std::string, long> acks; //Map<SensorId, LastAcked> acks;
        std::unordered_map<std::string, long> messages; //Map<SensorId, Messages> messages;
        std::unordered_map<std::string, std::pair<long, long>> messageRanges; //Map<SensorId, Range> messageRanges;

        double baseStationX = -1;
        double baseStationY = -1;
    protected:
        virtual void initialize(int stage) override;

        // Saves telemetry recieved by mobility
        virtual void handleTelemetry(projeto::Telemetry *telemetry) override;
        // Reacts to message recieved and updates payload accordingly
        virtual void handlePacket(Packet *pk) override;
        virtual void handleMessage(cMessage *msg) override;
        // Checks if timeout has finished and resets parameters if it has
        virtual bool isTimedout() override;
        // Resets parameters
        virtual void resetParameters();
        void record1sStatistics();
    private:
        virtual void updateAcks(const char *incomingAcks);
        virtual void updateRanges(const char *incomingRanges);

        // Sends sequence of orders that defines a rendevouz point, navigates
        // to it and reverses
        virtual void rendevouz();

        // Updates payload that communication will send
        virtual void updatePayload();
        virtual void setTarget(const char *target);

        cMessage *record1sStatisticsEvent;
        cOutVector baseDistanceVector;
        cOutVector bufferSizeVector;
    public:
        simsignal_t dataLoadSignalID;
        simsignal_t bufferLoadSignalID;
};

} //namespace

#endif
