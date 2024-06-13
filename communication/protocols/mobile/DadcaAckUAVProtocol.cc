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

#include "DadcaAckUAVProtocol.h"
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

#include <inet/mobility/static/StationaryMobility.h>
#include "../../../mobility/DroneMobility.h"

namespace projeto {

Define_Module(DadcaAckUAVProtocol);

void DadcaAckUAVProtocol::initialize(int stage)
{
    CommunicationProtocolBase::initialize(stage);

    bufferSizeVector.setName("bufferSizeVector");
    baseDistanceVector.setName("baseDistanceVector");

    record1sStatisticsEvent = new cMessage("record1sStatisticsEvent");
    scheduleAt(simTime() + 1.0, record1sStatisticsEvent); // Schedule the first event after 1 second

    if(stage == INITSTAGE_LOCAL) {
        timeoutDuration = par("timeoutDuration");
        maxBufferSize = par("maxBufferSize");

        //int duration = timeoutDuration.inUnit(SimTimeUnit::SIMTIME_S);
        // Signal that carries current data load and is emitted every time it is updated
        dataLoadSignalID = registerSignal("dataLoad");
        emit(dataLoadSignalID, currentDataLoad);

        bufferLoadSignalID = registerSignal("bufferLoad");
        emit(bufferLoadSignalID, currentBufferLoad);

        updatePayload();

        WATCH(leftNeighbours);
        WATCH(rightNeighbours);
        WATCH(communicationStatus);
        WATCH(tentativeTarget);
        WATCH(lastTarget);
        WATCH(currentDataLoad);
        WATCH(currentBufferLoad);
    }
}

void DadcaAckUAVProtocol::handleMessage(cMessage *msg)
{
    CommunicationCommand *command = dynamic_cast<CommunicationCommand *>(msg);

    if (command != nullptr) {
        switch (command->getCommandType()) {
        case FAIL_COMMS:
            failedComms = true;
            break;
        case FAIL_STORAGE:
            failedStorage = true;
            currentDataLoad = 0;
            currentBufferLoad = 0;
            acks.clear();
            messages.clear();
            messageRanges.clear();
            stableDataLoad = 0;
            break;
        case FAIL_END:
            failedComms = false;
            failedStorage = false;
            break;
        default:
            break;
        }
    } else if (msg == record1sStatisticsEvent) {
        record1sStatistics();
        scheduleAt(simTime() + 1.0, record1sStatisticsEvent); // Schedule the next event after 1 second
    } else {
        CommunicationProtocolBase::handleMessage(msg);
    }
}

class FindGSPositionVisitor : public cVisitor {
public:
    FindGSPositionVisitor() {

    }

    Coord groundStationPosition;

    void visit(cObject *obj) override {
        if (strcmp(obj->getName(), "groundStation") == 0) {
            cModule *module = check_and_cast<cModule*>(obj);
            cObject *mobilityObj = module->findObject("mobility");
            StationaryMobility *mobility = check_and_cast<StationaryMobility*>(mobilityObj);
            groundStationPosition = mobility->getCurrentPosition(); // .x and .y --> coords
        }
    }
};

void DadcaAckUAVProtocol::record1sStatistics()
{
    simtime_t now = simTime();

    bufferSizeVector.recordWithTimestamp(now, currentBufferLoad);

    if (baseStationX == -1 && baseStationY == -1) {
        // Find ground station
        cModule *topModule = getModuleByPath("<root>");  // Get the top-level module

        FindGSPositionVisitor visitor;
        topModule->forEachChild(&visitor);

        baseStationX = visitor.groundStationPosition.x;
        baseStationY = visitor.groundStationPosition.y;
    }

    cObject *mobilityObj = this->getParentModule()->findObject("mobility");
    DroneMobility *mobility = check_and_cast<DroneMobility*>(mobilityObj);

    Coord curPosition = mobility->getCurrentPosition();
    double xDelta = baseStationX - curPosition.x;
    double yDelta = baseStationY - curPosition.y;
    double distanceToBase = sqrt(xDelta*xDelta + yDelta*yDelta);
    baseDistanceVector.recordWithTimestamp(now, distanceToBase);
}

void DadcaAckUAVProtocol::handleTelemetry(projeto::Telemetry *telemetry) {
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

void DadcaAckUAVProtocol::handlePacket(Packet *pk) {
    if (failedComms) {
        return;
    }

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

                        EV_DETAIL << this->getParentModule()->getId() << " recieved UAV_PING_HEARTBEAT from " << tentativeTarget << endl;
                    }
                }
                break;
            }
            case DadcaAckMessageType::PAIR_REQUEST_BASE_PING:
            {
                // No communication form other drones matters while the drone is executing
                if(currentTelemetry.getCurrentCommand() != -1 && !destinationIsGroundstation) {
                    break;
                }

                if(payload->getDestinationID() != this->getParentModule()->getId()) {
                    break;
                }

                // If the drone is collecting data, prefer to pair with other drone
                if(communicationStatus == COLLECTING) {
                    resetParameters();
                }

                if(isTimedout()) {
                    if(payload->getSourceID() == tentativeTarget) {
                        EV_DETAIL << payload->getDestinationID() << " received a pair request while timed out from " << payload->getSourceID() << endl;
                        communicationStatus = PAIRED;
                    }
                } else {
                    EV_DETAIL << payload->getDestinationID() << " received a pair request while not timed out from  " << payload->getSourceID() << endl;
                    tentativeTarget = payload->getSourceID();
                    tentativeTargetName = pk->getName();
                    initiateTimeout(timeoutDuration);

                    communicationStatus = PAIRED;
                }

                break;
            }
            case DadcaAckMessageType::PAIR_CONFIRM: // partially equivalent to UAV_MESSAGES?
            {
                // No communication form other drones matters while the drone is executing
                if(currentTelemetry.getCurrentCommand() != -1 && !destinationIsGroundstation) {
                    break;
                }

                updateAcks(payload->getAcks());

                if(payload->getSourceID() == tentativeTarget &&
                   payload->getDestinationID() == this->getParentModule()->getId()) {
                    EV_DETAIL << payload->getDestinationID() << " received a pair confirmation from  " << payload->getSourceID() << endl;
                    if(communicationStatus != PAIRED_FINISHED) {
                        // If both drones are traveling in the same direction, the pairing is canceled
                        // Doesn't apply if one drone is the groundStation
                        if((lastStableTelemetry.isReversed() != payload->getReversed()) || (tour.size() == 0 || destinationIsGroundstation)) {
                            updateRanges(payload->getMessageRanges());

                            // Exchanging imaginary data to the drone closest to the start of the mission - equivalent to UAV_MESSAGES behavior?
                            if(lastStableTelemetry.getLastWaypointID() < payload->getLastWaypointID()) {
                                // Drone closest to the start gets the data
                                //currentDataLoad = currentDataLoad + payload->getDataLength();

                                // Doesn't update neighbours if the drone has no waypoints
                                // This prevents counting the groundStation as a drone
                                if(!destinationIsGroundstation) {
                                    // Drone closest to the start updates right neighbours
                                    rightNeighbours = payload->getRightNeighbours() + 1;
                                }
                            } else {
                                // Drone farthest away loses data
                                //currentDataLoad = 0;

                                // Doesn't update neighbours if the drone has no waypoints
                                // This prevents counting the groundStation as a drone
                                if(!destinationIsGroundstation) {
                                    // Drone farthest away updates left neighbours
                                    leftNeighbours = payload->getLeftNeighbours() + 1;
                                }
                            }

                            // Only completes redevouz if tour has been recieved or the paired drone has no waypoints
                            // This prevents rendevouz with the groundStation
                            if(tour.size() > 0 && !destinationIsGroundstation) {
                                rendevouz();
                            }
                            // Updating data load
                            emit(dataLoadSignalID, currentDataLoad);
                            communicationStatus = PAIRED_FINISHED;

                        }
                    }
               }
                break;
            }
            case DadcaAckMessageType::BEARER:
            {
                if(!isTimedout() && communicationStatus == FREE) {
                    EV_DETAIL << this->getParentModule()->getId() << " received bearer request from  " << pk->getName() << endl;
                    EV_DETAIL << payload->getMessageRanges() << endl;
                    //currentDataLoad = currentDataLoad + payload->getDataLength();

                    updateRanges(payload->getMessageRanges());

                    stableDataLoad = currentDataLoad;
                    emit(dataLoadSignalID, currentDataLoad);
                    initiateTimeout(timeoutDuration);

                    communicationStatus = COLLECTING;
                }
                break;
            }

            default:
                EV_DETAIL << std::endl << "[DadcaAckUAV] Ignoring received message " << payload->getMessageType() << std::endl;
                break;
        }
        updatePayload();
    }

    auto mamPayload = dynamicPtrCast<const BMeshPacket>(pk->peekAtBack());
    if(mamPayload != nullptr) {
        if(!isTimedout() && communicationStatus == FREE) {
            //currentDataLoad++; what?
            stableDataLoad = currentDataLoad;
            emit(dataLoadSignalID, currentDataLoad);
        }
    }
}

void DadcaAckUAVProtocol::updateAcks(const char *incomingAcks) {
    if (strcmp(incomingAcks, "") == 0) {
        return;
    }

    nlohmann::json jsonMap = nlohmann::json::parse(incomingAcks);
    std::unordered_map<std::string, long> incomingAcksMap = jsonMap.get<std::unordered_map<std::string, long>>();

    for (const auto& pair : incomingAcksMap) {
        const std::string& key = pair.first;
        long incomingAck = pair.second;

        auto iter = acks.find(key);
        if (iter != acks.end()) {
            iter->second = std::max(iter->second, incomingAck);
        } else {
            acks[key] = incomingAck;
        }

        // Discard acked messages
        auto iterRanges = messageRanges.find(key);
        if (iterRanges != messageRanges.end()) {
            std::pair<long, long>& messageRange = iterRanges->second;
            if (messageRange.first < incomingAck) {
                messageRange.first = incomingAck;
            }

            if (messageRange.second < incomingAck) {
                messageRange.second = incomingAck;
            }
        }
    }

    ///// UPDATE BUFFER STATS
    long newBufferLoad = 0;
    for (const auto& pair : messageRanges) {
        //const std::string& key = pair.first;
        const std::pair<long, long>& range = pair.second;
        newBufferLoad += (range.second - range.first);
    }
    currentBufferLoad = newBufferLoad;  // TODO: check for int overflow?
    emit(bufferLoadSignalID, currentBufferLoad);
    ///// UPDATE BUFFER STATS
}

void DadcaAckUAVProtocol::updateRanges(const char *incomingRanges) {
    if (strcmp(incomingRanges, "") == 0) {
        return;
    }

    nlohmann::json jsonMap = nlohmann::json::parse(incomingRanges);
    std::unordered_map<std::string, std::pair<long,long>> receivedRanges = jsonMap.get<std::unordered_map<std::string, std::pair<long,long>>>();

    assert(currentBufferLoad <= maxBufferSize);
    long newBufferLoadTemp = currentBufferLoad;

    ///// UPDATE RANGES START
    for (const auto& pair : receivedRanges) {
        const std::string& key = pair.first;
        const std::pair<long, long>& receivedRange = pair.second;

        auto iter = messageRanges.find(key);
        if (iter != messageRanges.end()) {
            std::pair<long, long>& messageRange = iter->second;
            long originalLoad = (messageRange.second - messageRange.first);
            messageRange.first = std::min(messageRange.first, receivedRange.first);
            messageRange.second = std::max(messageRange.second, receivedRange.second);
            long newLoad = (messageRange.second - messageRange.first);
            if (newLoad != originalLoad) {
                if (newBufferLoadTemp + newLoad > maxBufferSize) {
                    // Remove ranges that would make the buffer size exceed maxBufferSize
                    messageRange.second -= (newBufferLoadTemp + newLoad - maxBufferSize);
                    newLoad = (messageRange.second - messageRange.first);
                }
                newBufferLoadTemp += (newLoad - originalLoad);
            }
        } else {
            std::pair<long, long> newRange = receivedRange;
            long newLoad = (newRange.second - newRange.first);
            if (newBufferLoadTemp + newLoad > maxBufferSize) {
                // Remove ranges that would make the buffer size exceed maxBufferSize
                newRange.second -= (newBufferLoadTemp + newLoad - maxBufferSize);
                newLoad = (newRange.second - newRange.first);
            }
            newBufferLoadTemp += newLoad;
            messageRanges[key] = newRange;
        }
    }
    ///// UPDATE RANGES END

    ///// UPDATE BUFFER STATS
    long newBufferLoad = 0;
    for (const auto& pair : messageRanges) {
        //const std::string& key = pair.first;
        const std::pair<long, long>& range = pair.second;
        newBufferLoad += (range.second - range.first);
    }
    EV_DETAIL << currentBufferLoad << std::endl << newBufferLoadTemp << std::endl << newBufferLoad;
    assert(newBufferLoadTemp == newBufferLoad); // Sanity check
    currentBufferLoad = newBufferLoad;  // TODO: check for int overflow?
    emit(bufferLoadSignalID, currentBufferLoad);
    ///// UPDATE BUFFER STATS

    //
    currentDataLoad = currentBufferLoad;
    emit(dataLoadSignalID, currentDataLoad);
}

void DadcaAckUAVProtocol::rendevouz() {
    // Drone is the left or right one in the pair
    bool isLeft = !lastStableTelemetry.isReversed();

    // Calculates rally point
    std::vector<double> cumulativeDistances;
    double totalDistance = 0;
    Coord lastCoord;
    for(int i = 0; i < tour.size(); i++) {
        Coord coord = tour[i];

        if(i > 0) {
            totalDistance += lastCoord.distance(coord);
        }
        cumulativeDistances.push_back(totalDistance);

        lastCoord = coord;
    }
    int totalNeighbours = leftNeighbours + rightNeighbours;

    // Shared border where the paired drones will meet
    double sharedBorder = (double) leftNeighbours / (totalNeighbours + 1);
    // If the drone is not inverted it will start it's trip from the end
    // of it's segment and them invert
    if(isLeft) {
        sharedBorder += 1.0/(totalNeighbours + 1);
    }
    double sharedBorderDistance = totalDistance * sharedBorder;

    // Determines the waypoint index closest to the shared border
    int waypointIndex;
    for(waypointIndex=0; waypointIndex < cumulativeDistances.size() ; waypointIndex++) {
        if(cumulativeDistances[waypointIndex] > sharedBorderDistance) {
            // Waypoint before the acumulated distance is higher than the
            // shared border distance
            waypointIndex--;
            break;
        }
    }

    // Fraction that devides the waypoint before the shared border and the one after it
    // at the shared border
    double localBorderFraction = 1 - (cumulativeDistances[waypointIndex + 1] - sharedBorderDistance) / (cumulativeDistances[waypointIndex + 1] - cumulativeDistances[waypointIndex]);

    Coord coordsBefore = tour[waypointIndex];
    Coord coordsAfter = tour[waypointIndex + 1];
    Coord sharedBorderCoords = ((coordsAfter - coordsBefore) * localBorderFraction) + coordsBefore;

    bool isAhead = false;

    if(!isLeft) {
        if(lastStableTelemetry.getNextWaypointID() > waypointIndex) {
            isAhead = true;
        }
    } else {
        if(lastStableTelemetry.getLastWaypointID() > waypointIndex) {
            isAhead = true;
        }
    }

    // The rendevouz is divided in three steps

    // First the drones navigate to the waypoint closest to the shared border
    // If the drones are already coming from/going to the waypoint closest they don't need this command
    if((!isLeft || lastStableTelemetry.getLastWaypointID() != waypointIndex) &&
            (isLeft || lastStableTelemetry.getNextWaypointID() != waypointIndex)) {
        MobilityCommand *firstCommand = new MobilityCommand();
        firstCommand->setCommandType(GOTO_WAYPOINT);

        // If the drone is ahead of the shared border navigate to the next waypoint after it
        if(isAhead) {
            firstCommand->setParam1(waypointIndex + 1);
        } else {
            firstCommand->setParam1(waypointIndex);
        }

        sendCommand(firstCommand);
    }

    // Them the drones meet at the shared border
    MobilityCommand *secondCommand = new MobilityCommand();
    secondCommand->setCommandType(GOTO_COORDS);
    secondCommand->setParam1(sharedBorderCoords.x);
    secondCommand->setParam2(sharedBorderCoords.y);
    secondCommand->setParam3(sharedBorderCoords.z);

    // After the drones reach the coords they will be oriented
    // going from waypointIndex to waypointIndex + 1
    secondCommand->setParam4(waypointIndex + 1);
    secondCommand->setParam5(waypointIndex);
    sendCommand(secondCommand);


    // Them the drones reverses
    // Only the drone on the left needs to reverse
    // since both drones are oriented unreversed
    if(isLeft) {
        MobilityCommand *thirdCommand = new MobilityCommand();
        thirdCommand->setCommandType(REVERSE);
        sendCommand(thirdCommand);
    }
}

void DadcaAckUAVProtocol::updatePayload() {
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

        if (payload->getMessageType() == DadcaAckMessageType::BEARER
                || payload->getMessageType() == DadcaAckMessageType::PAIR_CONFIRM) { // UAV_MESSAGES equivalent?
            nlohmann::json jsonMap = messageRanges;

            // Set message ranges
            payload->setMessageRanges(jsonMap.dump().c_str());
            EV_DETAIL << "UAV forwarding message ranges: " << payload->getMessageRanges() << std::endl;
        }

        CommunicationCommand *command = new CommunicationCommand();
        command->setCommandType(SET_PAYLOAD);
        command->setPayloadTemplate(payload);
        sendCommand(command);
    } else {
        delete payload;
    }
}

void DadcaAckUAVProtocol::setTarget(const char *target) {
    CommunicationCommand *command = new CommunicationCommand();
    command->setCommandType(SET_TARGET);
    command->setTarget(target);
    sendCommand(command);
}

bool DadcaAckUAVProtocol::isTimedout() {
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

void DadcaAckUAVProtocol::resetParameters() {
    timeoutSet = false;
    lastTarget = tentativeTarget;
    tentativeTarget = -1;
    tentativeTargetName = "";
    setTarget("");
    communicationStatus = FREE;

    lastStableTelemetry = currentTelemetry;
    stableDataLoad = currentDataLoad;

    updatePayload();
}
} //namespace
