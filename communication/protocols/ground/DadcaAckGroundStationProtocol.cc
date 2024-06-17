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

#include "DadcaAckGroundStationProtocol.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/TagBase_m.h"
#include "inet/common/TimeTag_m.h"
#include "inet/common/lifecycle/ModuleOperations.h"
#include "inet/common/packet/Packet.h"
#include "inet/networklayer/common/FragmentationTag_m.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/transportlayer/contract/udp/UdpControlInfo_m.h"
#include "inet/applications/base/ApplicationPacket_m.h"
#include "../../../applications/mamapp/BMeshPacket_m.h"

namespace projeto {

Define_Module(DadcaAckGroundStationProtocol);

void DadcaAckGroundStationProtocol::initialize(int stage)
{
    CommunicationProtocolBase::initialize(stage);

    if(stage == INITSTAGE_LOCAL) {
        timeoutDuration = par("timeoutDuration");

        //int duration = timeoutDuration.inUnit(SimTimeUnit::SIMTIME_S);
        // Signal that carries current data load and is emitted every time it is updated
        dataLoadSignalID = registerSignal("dataLoad");
        emit(dataLoadSignalID, currentDataLoad);

        updatePayload();

        WATCH(leftNeighbours);
        WATCH(rightNeighbours);
        WATCH(communicationStatus);
        WATCH(tentativeTarget);
        WATCH(lastTarget);
        WATCH(currentDataLoad);
    }
}

void DadcaAckGroundStationProtocol::finish() {
    ackRefresh();
    CommunicationProtocolBase::finish();
}

void DadcaAckGroundStationProtocol::handleTelemetry(projeto::Telemetry *telemetry) {
    // Starts a timeout right after the drone has completed a command (Rendevouz)
    if(currentTelemetry.getCurrentCommand() != -1 && telemetry->getCurrentCommand() == -1) {
        resetParameters();
        initiateTimeout(timeoutDuration);
    }

    // Records if the drone has reached an edge and erases the neighbours after that edge
    if(currentTelemetry.getDroneActivity() != REACHED_EDGE && telemetry->getDroneActivity() == REACHED_EDGE) {
        if(telemetry->isReversed()) {
            rightNeighbours = 0;
        } else {
            leftNeighbours = 0;
        }
    }

    // Erases neighbours when drone is recharging or shutdown
    if((currentTelemetry.getDroneActivity() != RECHARGING && telemetry->getDroneActivity() == RECHARGING) ||
            (currentTelemetry.getDroneActivity() != SHUTDOWN && telemetry->getDroneActivity() == SHUTDOWN)) {
        leftNeighbours = rightNeighbours = 0;
    }

    currentTelemetry = *telemetry;

    if(telemetry->hasObject("tourCoords")) {
        tour = *(std::vector<Coord>*) telemetry->par("tourCoords").pointerValue();
    }

    // Telemetry during timeout is not stable, it may contain information that
    // has changed after message exchange started
    if(!isTimedout()) {
        lastStableTelemetry = *telemetry;
    }
    updatePayload();
}

void DadcaAckGroundStationProtocol::handlePacket(Packet *pk) {
    auto payload = dynamicPtrCast<const DadcaAckMessage>(pk->peekAtBack());


    if(payload != nullptr) {
        bool destinationIsGroundstation = payload->getNextWaypointID() == -1;

        switch(payload->getMessageType()) {
            case DadcaAckMessageType::UAV_PING_HEARTBEAT:
            {
                // No communication from other drones matters while the drone is executing
                // or if the drone is recharging/shutdown
                if(currentTelemetry.getCurrentCommand() != -1 && !destinationIsGroundstation) {
                    return;
                }

                if(isTimedout() && lastTarget != payload->getSourceID() && tentativeTarget != payload->getSourceID()) {
                    resetParameters();
                }

                // If the drone is collecting data, prefer to pair with other drone
                if(communicationStatus == COLLECTING) {
                    resetParameters();
                }

                if(!isTimedout()) {
                    // Only accepts requests if you are going to the same waypoint as drone or you are going to the waypoint it came from
                    // or if the drone is stationary
                    if(lastStableTelemetry.getNextWaypointID() == payload->getNextWaypointID() ||
                            lastStableTelemetry.getNextWaypointID() == payload->getLastWaypointID() ||
                            lastStableTelemetry.getNextWaypointID() == -1 ||
                            destinationIsGroundstation) {
                        tentativeTarget = payload->getSourceID();
                        tentativeTargetName = pk->getName();
                        setTarget(tentativeTargetName.c_str());
                        initiateTimeout(timeoutDuration);
                        communicationStatus = REQUESTING;

                        EV_DETAIL << this->getParentModule()->getId() << " received UAV_PING_HEARTBEAT from " << tentativeTarget << endl;
                    }
                }
                break;
            }
            case DadcaAckMessageType::PAIR_CONFIRM: // equivalent to UAV_MESSAGES ?
            {
                // No communication form other drones matters while the drone is executing
                if(currentTelemetry.getCurrentCommand() != -1 && !destinationIsGroundstation) {
                    break;
                }

                if(payload->getSourceID() == tentativeTarget &&
                   payload->getDestinationID() == this->getParentModule()->getId()) {
                    EV_DETAIL << payload->getDestinationID() << " received a pair confirmation from  " << payload->getSourceID() << endl;
                    if(communicationStatus != PAIRED_FINISHED) {
                        // If both drones are travelling in the same direction, the pairing is canceled
                        // Doesn't apply if one drone is the groundStation
                        if((lastStableTelemetry.isReversed() != payload->getReversed()) || (tour.size() == 0 || destinationIsGroundstation)) {
                            // Exchanging imaginary data to the drone closest to the start of the mission
                            if(lastStableTelemetry.getLastWaypointID() < payload->getLastWaypointID()) {
                                // Drone closest to the start gets the data
                                //currentDataLoad = currentDataLoad + payload->getDataLength();

                                updateAcks(payload->getMessageRanges());
                            }

                            // Updating data load
                            ackRefresh();
                            communicationStatus = PAIRED_FINISHED;

                        }
                    }
               }
               break;
            }
            case DadcaAckMessageType::BEARER: // equivalent to UAV_MESSAGES ?
            {
                if(!isTimedout() && communicationStatus == FREE) {
                    EV_DETAIL << this->getParentModule()->getId() << " received bearer request from  " << pk->getName() << endl;
                    //currentDataLoad = currentDataLoad + payload->getDataLength();

                    updateAcks(payload->getMessageRanges());
                    initiateTimeout(timeoutDuration);

                    communicationStatus = COLLECTING;
                }
                break;
            }
            case DadcaAckMessageType::UAV_MESSAGES:
            {

                break;
            }
            default:
                EV_DETAIL << std::endl << "[DadcaAckGroundStation] Ignoring received message " << payload->getMessageType() << std::endl;
                break;
        }
        updatePayload();
    }

    auto mamPayload = dynamicPtrCast<const BMeshPacket>(pk->peekAtBack());
    if(mamPayload != nullptr) {
        if(!isTimedout() && communicationStatus == FREE) {
            currentDataLoad++;
            emit(dataLoadSignalID, currentDataLoad);
        }
    }
}

void DadcaAckGroundStationProtocol::updateAcks(const char *incomingMessageRanges) {
    if (strcmp(incomingMessageRanges, "") == 0) {
        return;
    }

    nlohmann::json jsonMap = nlohmann::json::parse(incomingMessageRanges);
    std::unordered_map<std::string, std::pair<long,long>> receivedMessageRanges = jsonMap.get<std::unordered_map<std::string, std::pair<long,long>>>();

    // BEGIN UPDATE ACKS
    for (const auto& pair : receivedMessageRanges) {
        const std::string& key = pair.first;
        const std::pair<long, long>& range = pair.second;
        long lastReceivedMessageSeq = range.second;

        auto iter = acks.find(key);
        if (iter != acks.end()) {
            iter->second = lastReceivedMessageSeq;
        } else {
            acks[key] = lastReceivedMessageSeq;
        }
    }
    // END UPDATE ACKS

    ackRefresh();
}

void DadcaAckGroundStationProtocol::ackRefresh() {
    // Set data load according to acked data
    int tempDataLoad = 0;
    for (const auto& pair : acks) {
        tempDataLoad += pair.second;
    }
    currentDataLoad = tempDataLoad;
    emit(dataLoadSignalID, currentDataLoad);
}

void DadcaAckGroundStationProtocol::updatePayload() {
    DadcaAckMessage *payload = new DadcaAckMessage();
    payload->addTag<CreationTimeTag>()->setCreationTime(simTime());

    // Sets the reverse flag on the payload
    payload->setReversed(lastStableTelemetry.isReversed());
    payload->setNextWaypointID(lastStableTelemetry.getNextWaypointID());
    payload->setLastWaypointID(lastStableTelemetry.getLastWaypointID());

    payload->setLeftNeighbours(leftNeighbours);
    payload->setRightNeighbours(rightNeighbours);

    payload->setSourceID(this->getParentModule()->getId());

    if(!isTimedout() && communicationStatus != FREE) {
        communicationStatus = FREE;
    }

    switch(communicationStatus) {
        case FREE:
        {
            payload->setMessageType(DadcaAckMessageType::UAV_PING_HEARTBEAT);
            EV_DETAIL << payload->getSourceID() << " set to UAV_PING_HEARTBEAT" << endl;
            break;
        }
        case REQUESTING:
        {
            payload->setMessageType(DadcaAckMessageType::PAIR_REQUEST_BASE_PING);
            payload->setDestinationID(tentativeTarget);
            EV_DETAIL << payload->getSourceID() << " set to pair request to " << payload->getDestinationID() << endl;
            break;
        }
        case PAIRED:
        case PAIRED_FINISHED:
        {
            payload->setMessageType(DadcaAckMessageType::PAIR_CONFIRM);
            payload->setDestinationID(tentativeTarget);
            payload->setDataLength(currentDataLoad);

            EV_DETAIL << payload->getSourceID() << " set to pair confirmation to " << payload->getDestinationID() << endl;
            break;
        }
        case COLLECTING:
            break;
    }

    // Only send the update command if the payload has actually changed
    if(payload->getMessageType() != lastPayload.getMessageType() ||
            payload->getSourceID() != lastPayload.getSourceID() ||
            payload->getDestinationID() != lastPayload.getDestinationID() ||
            payload->getNextWaypointID() != lastPayload.getNextWaypointID() ||
            payload->getLastWaypointID() != lastPayload.getLastWaypointID() ||
            payload->getReversed() != lastPayload.getReversed()) {
        lastPayload = *payload;

        nlohmann::json jsonMap = acks;

        // Set acks
        payload->setAcks(jsonMap.dump().c_str());

        CommunicationCommand *command = new CommunicationCommand();
        command->setCommandType(SET_PAYLOAD);
        command->setPayloadTemplate(payload);
        sendCommand(command);
    } else {
        delete payload;
    }
}

void DadcaAckGroundStationProtocol::setTarget(const char *target) {
    CommunicationCommand *command = new CommunicationCommand();
    command->setCommandType(SET_TARGET);
    command->setTarget(target);
    sendCommand(command);
}

bool DadcaAckGroundStationProtocol::isTimedout() {
    // Blocks the timeout if the drone is currently executing a command
    if(currentTelemetry.getCurrentCommand() != -1) {
        return true;
    }

    bool oldValue = timeoutSet;
    bool value = CommunicationProtocolBase::isTimedout();
    if(!value && oldValue) {
        resetParameters();
    }
    return value;
}

void DadcaAckGroundStationProtocol::resetParameters() {
    timeoutSet = false;
    lastTarget = tentativeTarget;
    tentativeTarget = -1;
    tentativeTargetName = "";
    setTarget("");
    communicationStatus = FREE;

    lastStableTelemetry = currentTelemetry;

    updatePayload();
}
} //namespace
