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

#include "DadcaAckProtocolSensor.h"

#include "inet/common/ModuleAccess.h"
#include "inet/common/TagBase_m.h"
#include "inet/common/TimeTag_m.h"
#include "inet/common/lifecycle/ModuleOperations.h"
#include "inet/common/packet/Packet.h"
#include "inet/networklayer/common/FragmentationTag_m.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/transportlayer/contract/udp/UdpControlInfo_m.h"
#include "inet/applications/base/ApplicationPacket_m.h"

namespace projeto {

Define_Module(DadcaAckProtocolSensor);

void DadcaAckProtocolSensor::initialize(int stage)
{
    CommunicationProtocolBase::initialize(stage);

    if(stage == INITSTAGE_LOCAL) {
        updatePayload();

        sendSelfGenPacket();
    }
}

void DadcaAckProtocolSensor::handleMessage(cMessage *msg) {
    auto message = static_cast<CommunicationCommand *>(msg);

    bool handled = false;

    if (message != nullptr && message->getPayloadTemplate() != nullptr && message->isSelfMessage()) {
        auto payload = static_cast<const DadcaAckMessage *>(message->getPayloadTemplate());

        if (payload != nullptr && payload->getMessageType() == DadcaAckMessageType::INT_GEN_MESSAGE) {
            Messages += 1; // Generate a new message
            cancelAndDelete(msg);

            // Schedule again for next simulated second (INT_GEN_MESSAGE)
            sendSelfGenPacket();

            handled = true;

            return;
        }
    }

    if (!handled) {
        CommunicationProtocolBase::handleMessage(msg);
    }
}

void DadcaAckProtocolSensor::handlePacket(Packet *pk) {
    auto payload = pk->peekAtBack<DadcaAckMessage>(B(34), 1);

    if(payload != nullptr) {
        if(payload->getMessageType() == DadcaAckMessageType::UAV_PING_HEARTBEAT)
        {
            std::cout << this->getParentModule()->getFullName() << " received UAV_PING_HEARTBEAT from " << tentativeTarget << endl;
            tentativeTarget = payload->getSourceID();
            tentativeTargetName = pk->getName();
            setTarget(tentativeTargetName.c_str());

            nlohmann::json jsonMap = nlohmann::json::parse(payload->getAcks());
            std::unordered_map<std::string, long> receivedAcks = jsonMap.get<std::unordered_map<std::string, long>>();

            const char *myId = this->getParentModule()->getFullName();

            auto iter = receivedAcks.find(myId);
            if (iter != receivedAcks.end()) {
                std::cout << "Key found. Value: " << iter->second << std::endl;
                long lastReceivedAck = iter->second;
                // if m->acks[MyId] > LastAckedMessage:
                if (lastReceivedAck > LastAckedMessage) {
                    //   LastAckedMessage = m->acks[MyId]
                    LastAckedMessage = lastReceivedAck;
                    //   clear acked messages
                    // TODO
                }
            } else {
                std::cout << "Key not found. My ID: " << myId << std::endl << "Acks received map: " << payload->getAcks() << std::endl;
            }

            // send $SENSOR\_MESSAGES$
            //  - Messages

            updatePayload();
        } else {
            std::cout << std::endl << "[DadcaAckSensor] Ignoring received message " << payload->getMessageType() << std::endl;
        }
    }
}

void DadcaAckProtocolSensor::sendSelfGenPacket() {
    if (everySecondControlPacket != nullptr) {
        delete everySecondControlPacket;
    }

    everySecondControlPacket = new DadcaAckMessage();
    everySecondControlPacket->addTag<CreationTimeTag>()->setCreationTime(simTime());

    everySecondControlPacket->setMessageType(DadcaAckMessageType::INT_GEN_MESSAGE);
    everySecondControlPacket->setSourceID(this->getParentModule()->getId());
    everySecondControlPacket->setDestinationID(this->getParentModule()->getId());

    CommunicationCommand *command = new CommunicationCommand();
    command->setCommandType(SET_PAYLOAD);
    command->setPayloadTemplate(everySecondControlPacket);
    // TODO try with cMessage

    scheduleAt(simTime() + 1.0, command);
}

void DadcaAckProtocolSensor::updatePayload() {
    DadcaAckMessage *payload = new DadcaAckMessage();
    payload->addTag<CreationTimeTag>()->setCreationTime(simTime());

    payload->setMessageType(DadcaAckMessageType::BEARER);
    payload->setSourceID(this->getParentModule()->getId());
    payload->setDestinationID(tentativeTarget);

    // Send last Message index
    std::unordered_map<std::string, std::pair<long, long>> ranges;
    std::pair<long, long> r(1, Messages);
    ranges[this->getParentModule()->getFullName()] = r;

    nlohmann::json jsonMap = ranges;

    // Set Message Ranges field
    payload->setMessageRanges(jsonMap.dump().c_str());

    std::cout << payload->getSourceID() << " sending bearer to " << tentativeTarget  << endl;

    lastPayload = *payload;

    CommunicationCommand *command = new CommunicationCommand();
    command->setCommandType(SET_PAYLOAD);
    command->setPayloadTemplate(payload);
    sendCommand(command);
}

void DadcaAckProtocolSensor::setTarget(const char *target) {
    CommunicationCommand *command = new CommunicationCommand();
    command->setCommandType(SET_TARGET);
    command->setTarget(target);
    sendCommand(command);
}

} //namespace
