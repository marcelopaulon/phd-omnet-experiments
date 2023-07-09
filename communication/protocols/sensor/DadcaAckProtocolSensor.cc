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
    }
}

void DadcaAckProtocolSensor::handlePacket(Packet *pk) {
    auto payload = pk->peekAtBack<DadcaAckMessage>(B(34), 1);


    if(payload != nullptr) {
        if(payload->getMessageType() == DadcaAckMessageType::HEARTBEAT)
        {
            std::cout << this->getParentModule()->getFullName() << " recieved heartbeat from " << tentativeTarget << endl;
            tentativeTarget = payload->getSourceID();
            tentativeTargetName = pk->getName();
            setTarget(tentativeTargetName.c_str());
            updatePayload();
        }
    }
}

void DadcaAckProtocolSensor::updatePayload() {
    DadcaAckMessage *payload = new DadcaAckMessage();
    payload->addTag<CreationTimeTag>()->setCreationTime(simTime());

    payload->setMessageType(DadcaAckMessageType::BEARER);
    payload->setSourceID(this->getParentModule()->getId());
    payload->setDestinationID(tentativeTarget);
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
