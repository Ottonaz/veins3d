#include <veins/modules/analogueModel/GarageModelSelector.h>
#include <veins/modules/analogueModel/RiceRayleighFading.h>
#include "veins/modules/utility/NBHeightMapper.h"
#include "veins/base/modules/BaseWorldUtility.h"
#include "veins/modules/obstacle/ObstacleControl.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/base/connectionManager/ChannelAccess.h"

using Veins::AirFrame;
using Veins::ObstacleControlAccess;
using Veins::TraCIScenarioManagerAccess;
using Veins::TraCICommandInterface;
using Veins::TraCIMobility;

GarageModelSelector::GarageModelSelector(double carrierFrequency, std::vector<std::string> demFiles, bool isRasterType,
        bool considerDEM, double diffSpacing, double demCellSize, bool considerVehicles, int numPaths, double kFactor,
        simtime_t_cref interval)
{
    this->carrierFrequency = carrierFrequency;
    floorCtrl = FloorControlAccess().getIfExists();
    obstCtrl = ObstacleControlAccess().getIfExists();
    traciCI = TraCIScenarioManagerAccess().get()->getCommandInterface();

    RiceRayleighFading* riceRayleighModel = new RiceRayleighFading(carrierFrequency, numPaths, kFactor, interval);
    diffModel = new EnvironmentalDiffraction(carrierFrequency, considerDEM, demFiles, isRasterType, demCellSize,
            diffSpacing, considerVehicles);
    garageModel = new GarageModels(carrierFrequency, floorCtrl, obstCtrl, diffModel, riceRayleighModel);
}

void GarageModelSelector::filterSignal(AirFrame *frame, const Coord& senderPos, const Coord& receiverPos)
{
    // check if both vehicles inside parking garage by checking road name
    // alternative check might be based on bounding box
    ChannelMobilityPtrType senderMob =
            dynamic_cast<ChannelAccess* const >(frame->getSenderModule())->getMobilityModule();
    TraCIMobility* senderTraci = dynamic_cast<TraCIMobility*>(senderMob);
    ChannelMobilityPtrType receiverMob =
            dynamic_cast<ChannelAccess* const >(frame->getArrivalModule())->getMobilityModule();
    TraCIMobility* receiverTraci = dynamic_cast<TraCIMobility*>(receiverMob);

    std::string senderRoad = traciCI->vehicle(senderTraci->getExternalId()).getRoadId();
    std::string receiverRoad = traciCI->vehicle(receiverTraci->getExternalId()).getRoadId();

    if (senderRoad.find("_GRG") != std::string::npos && receiverRoad.find("_GRG") != std::string::npos) {
        garageModel->filterSignal(frame, senderPos, receiverPos);
        return;
    }

    // at least one vehicle is outside, thus apply normal models
    double obstFactor = obstCtrl->calculateAttenuation(senderPos, receiverPos).factor;
    double diffFactor = diffModel->calcAttenuation(frame, senderPos, receiverPos);
    double floorFactor = floorCtrl->calculateAttenuation(senderPos, receiverPos);

    double dist = (receiverPos - senderPos).length();
    double freespace = BaseWorldUtility::speedOfLight() / (4 * M_PI * dist * carrierFrequency);
    freespace = freespace * freespace;

    Signal& s = frame->getSignal();

    // add Mapping of constant attenuation factors
    double constFactors = obstFactor * diffFactor * floorFactor * freespace;
    bool hasFrequency = s.getTransmissionPower()->getDimensionSet().hasDimension(Dimension::frequency());
    const DimensionSet& domain = hasFrequency ? DimensionSet::timeFreqDomain() : DimensionSet::timeDomain();
    ConstantSimpleConstMapping* constAttMapping = new ConstantSimpleConstMapping(domain, constFactors);
    s.addAttenuation(constAttMapping);
}

