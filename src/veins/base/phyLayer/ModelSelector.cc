#include <veins/base/phyLayer/ModelSelector.h>
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/base/connectionManager/ChannelAccess.h"
#include <typeinfo>

#include <iostream>
#include <fstream>
#include "veins/SignalStats.h"

using Veins::TraCIScenarioManagerAccess;
using Veins::TraCICommandInterface;
using Veins::TraCIMobility;

std::map<std::string, double> ModelSelector::streetWidths;

ModelSelector::ModelSelector(AnalogueModelList& modelList) :
        analogueModels(modelList), rsam(nullptr), simplePathloss(nullptr), twoRayModel(nullptr), nRayModel(nullptr), environDiff(
                nullptr), obstShadowing(nullptr), floorModel(nullptr), rrFading(nullptr)
{
    for (AnalogueModelList::const_iterator it = analogueModels.begin(); it != analogueModels.end(); it++) {
        if (dynamic_cast<RadioStateAnalogueModel*>(*it) != nullptr)
            rsam = dynamic_cast<RadioStateAnalogueModel*>(*it);
        else if (dynamic_cast<SimplePathlossModel*>(*it) != nullptr)
            simplePathloss = dynamic_cast<SimplePathlossModel*>(*it);
        else if (dynamic_cast<TwoRayInterferenceModel*>(*it) != nullptr)
            twoRayModel = dynamic_cast<TwoRayInterferenceModel*>(*it);
        else if (dynamic_cast<NRayGroundInterference*>(*it) != nullptr)
            nRayModel = dynamic_cast<NRayGroundInterference*>(*it);
        else if (dynamic_cast<EnvironmentalDiffraction*>(*it) != nullptr)
            environDiff = dynamic_cast<EnvironmentalDiffraction*>(*it);
        else if (dynamic_cast<SimpleObstacleShadowing*>(*it) != nullptr)
            obstShadowing = dynamic_cast<SimpleObstacleShadowing*>(*it);
        else if (dynamic_cast<FloorAttenuation*>(*it) != nullptr)
            floorModel = dynamic_cast<FloorAttenuation*>(*it);
        else if (dynamic_cast<RiceRayleighFading*>(*it) != nullptr)
            rrFading = dynamic_cast<RiceRayleighFading*>(*it);
        else
            throw cRuntimeError(
                    "The AnalogueModel %s is not supported by the ModelSelector. If you still want to use the ModelSelector, please add it accordingly.",
                    typeid(**it).name());

        tunnelControl = TunnelControlAccess().getIfExists();

        traciCI = TraCIScenarioManagerAccess().get()->getCommandInterface();
    }
}

ModelSelector::~ModelSelector()
{
    // analogue models are destroyed by the BasePhyLayer, so nothing to do here
}

void ModelSelector::filterSignal(AirFrame* frame, const Coord& senderPos, const Coord& receiverPos)
{
    rsam->filterSignal(frame, senderPos, receiverPos);

    ChannelMobilityPtrType senderMob =
            dynamic_cast<ChannelAccess* const >(frame->getSenderModule())->getMobilityModule();
    TraCIMobility* senderTraci = dynamic_cast<TraCIMobility*>(senderMob);
    ChannelMobilityPtrType receiverMob =
            dynamic_cast<ChannelAccess* const >(frame->getArrivalModule())->getMobilityModule();
    TraCIMobility* receiverTraci = dynamic_cast<TraCIMobility*>(receiverMob);

    std::string senderRoad = senderTraci ? senderTraci->getRoadId() : "";
    std::string receiverRoad = senderTraci ? senderTraci->getRoadId() : "";

    // first, check if we are in a special environment, i.e., garage or tunnel
    // TODO cache the results of these queries?
    std::string sndGarage = "";
    if(senderRoad != "") traciCI->road(senderRoad).getParameter("garage", sndGarage);
    if (sndGarage != "") {
        std::string rcvGarage = "";
        if(receiverRoad != "") traciCI->road(receiverRoad).getParameter("garage", rcvGarage);
        if (sndGarage == rcvGarage) {
            applyGarageModels(frame, senderPos, receiverPos);
            return;
        }
    }
    std::string sndTunnel = "", rcvTunnel = "";
    if(senderRoad != "") traciCI->road(senderRoad).getParameter("tunnel", sndTunnel);
    if(receiverRoad != "") traciCI->road(receiverRoad).getParameter("tunnel", rcvTunnel);
    if (sndTunnel != "" || rcvTunnel != "") {
        applyTunnelModels(frame, senderPos, receiverPos, sndTunnel, rcvTunnel);
        return;
    }

    // we are in a "normal" outdoor environment
    bool isNLOS = applyNLOSModels(frame, senderPos, receiverPos); // FloorAtt, EnvDiff, SimpleObstacle
    if (isNLOS && simplePathloss) {
        simplePathloss->filterSignal(frame, senderPos, receiverPos);
    }
    else if (twoRayModel || nRayModel) {
        applyGroundReflModel(frame, senderPos, receiverPos, senderRoad, receiverRoad); // 2-ray or N-ray, also considering street width
    }

}

void ModelSelector::applyGarageModels(AirFrame *frame, const Coord& senderPos, const Coord& receiverPos)
{
    Signal& s = frame->getSignal();

    simplePathloss->filterSignal(frame, senderPos, receiverPos);

    // apply FloorAttenuation
    double floorFactor = floorModel->calcAttenuation(senderPos, receiverPos);

    double wallFactor = 1.0;
    double diffFactor = 1.0;

    // if FloorAttenuation is 1.0, we have same-floor communication
    if (FWMath::close(floorFactor, 1.0)) {
        // apply DiffractionModel for vehicles in LOS
        if (environDiff)
            diffFactor = environDiff->calcAttenuation(frame, senderPos, receiverPos, true);

        if (obstShadowing)
            wallFactor = obstShadowing->calcAttenuation(senderPos, receiverPos, true).factor;

        // if no wall and no vehicle in LOS, apply Rice fading (K factor from papers)
        // otherwise, apply Rayleigh fading (or Rice model with small K factor, in general)
        if (FWMath::close(wallFactor, 1.0) && FWMath::close(diffFactor, 1.0))
            rrFading->filterSignal(frame, senderPos, receiverPos);
        else
            rrFading->filterSignal(frame, senderPos, receiverPos, 1.74);
    }

    // add Mapping of constant attenuation factors
    double constFactors = floorFactor * wallFactor * diffFactor;
    bool hasFrequency = s.getTransmissionPower()->getDimensionSet().hasDimension(Dimension::frequency());
    const DimensionSet& domain = hasFrequency ? DimensionSet::timeFreqDomain() : DimensionSet::timeDomain();
    ConstantSimpleConstMapping* constAttMapping = new ConstantSimpleConstMapping(domain, constFactors);
    s.addAttenuation(constAttMapping);
}

void ModelSelector::applyTunnelModels(AirFrame *frame, const Coord& senderPos, const Coord& receiverPos,
        std::string sndTunnel, std::string rcvTunnel)
{
    // very simple assumption, to be improved in future versions: as tunnels can be seen as waveguides, we don't consider
    // any obstacles, walls, etc., and just add distance-dependent free-space path loss if both cars are in the same tunnel
    // (many studies even suggest a path loss exponent smaller than 2)
    // otherwise, we use TunnelControl to determine an attenuation (at the moment, we assume full obstruction if the LOS
    // intersects with tunnel walls, floor or ceiling

    simplePathloss->filterSignal(frame, senderPos, receiverPos); // if both cars are in the same tunnel, that's all we do

    if (sndTunnel != rcvTunnel) {
        double tunnelFactor = tunnelControl->calculateAttenuation(senderPos, receiverPos, sndTunnel, rcvTunnel);
        if (FWMath::close(tunnelFactor, 1.0))
            return;
        Signal& s = frame->getSignal();
        bool hasFrequency = s.getTransmissionPower()->getDimensionSet().hasDimension(Dimension::frequency());
        const DimensionSet& domain = hasFrequency ? DimensionSet::timeFreqDomain() : DimensionSet::timeDomain();
        ConstantSimpleConstMapping* constAttMapping = new ConstantSimpleConstMapping(domain, tunnelFactor);
        s.addAttenuation(constAttMapping);
    }
}

bool ModelSelector::applyNLOSModels(AirFrame* frame, const Coord& senderPos, const Coord& receiverPos)
{
    Signal& s = frame->getSignal();

    double floorFactor = 1.0;
    double diffFactor = 1.0;
    double obstFactor = 1.0;
    double vegFactor = 1.0;

    if (floorModel)
        floorFactor = floorModel->calcAttenuation(senderPos, receiverPos);

    // if FloorAttenuation is 1.0, we have same-floor communication
    if (FWMath::close(floorFactor, 1.0)) {
        if (environDiff)
            diffFactor = environDiff->calcAttenuation(frame, senderPos, receiverPos);
        if (obstShadowing){
            SignalStats ss = obstShadowing->calcAttenuation(senderPos, receiverPos);
            obstFactor = ss.factor;
            s.setStats(ss);
        }
    }

    // add Mapping of constant attenuation factors
    double constFactors = floorFactor * diffFactor * obstFactor * vegFactor;
    bool hasFrequency = s.getTransmissionPower()->getDimensionSet().hasDimension(Dimension::frequency());
    const DimensionSet& domain = hasFrequency ? DimensionSet::timeFreqDomain() : DimensionSet::timeDomain();
    ConstantSimpleConstMapping* constAttMapping = new ConstantSimpleConstMapping(domain, constFactors);
    s.addAttenuation(constAttMapping);

    return FWMath::close(constFactors, 1.0) ? false : true;
}

void ModelSelector::applyGroundReflModel(AirFrame* frame, const Coord& senderPos, const Coord& receiverPos, const std::string& senderRoad, const std::string& receiverRoad)
{
    if (floorModel) {
        // if there are floors in between, the ground reflection model would produce wrong results, thus just apply free space pathloss
        bool floorsBetween = floorModel->getFloorControl().groundFloorsBetween(senderPos, receiverPos);
        if (floorsBetween) {
            simplePathloss->filterSignal(frame, senderPos, receiverPos);
            return;
        }
    }
    // scale the impact of the ground reflection model based on the street width
    double scaling = getGroundReflScaling(frame, senderRoad, receiverRoad);
    if (FWMath::close(scaling, 0.0)) { // a scaling factor of 0.0 corresponds to a narrow street, thus ignore ground reflections (due to too many other reflections)
        simplePathloss->filterSignal(frame, senderPos, receiverPos);
        return;
    }

    if (nRayModel) { // && isOn3dTerrain()) {
        nRayModel->filterSignal(frame, senderPos, receiverPos, scaling); // scaling needs to be considered
    } else if (twoRayModel) {
        twoRayModel->filterSignal(frame, senderPos, receiverPos, scaling); // scaling needs to be considered
    }
}

double ModelSelector::getGroundReflScaling(AirFrame* frame, const std::string& senderRoad, const std::string& receiverRoad)
{
    double txStreetWidth = 0.0;
    double rxStreetWidth = 0.0;

    auto txStrWidthIt = streetWidths.find(senderRoad);
    if (txStrWidthIt != streetWidths.end()) {
        txStreetWidth = txStrWidthIt->second;
    } else {
        if(senderRoad != "") traciCI->road(senderRoad).getParameter("streetWidth", txStreetWidth);
        if (txStreetWidth == 0.0) {
            EV << "No street width assigned to edge " << senderRoad << ". Assuming a 120m wide street." << std::endl;
            txStreetWidth = 120.0;
        }
        streetWidths[senderRoad] = txStreetWidth;
    }

    auto rxStrWidthIt = streetWidths.find(receiverRoad);
    if (rxStrWidthIt != streetWidths.end()) {
        rxStreetWidth = rxStrWidthIt->second;
    }
    else {
        if(receiverRoad != "") traciCI->road(receiverRoad).getParameter("streetWidth", rxStreetWidth);
        if (rxStreetWidth == 0.0) {
            EV << "No street width assigned to edge " << receiverRoad << ". Assuming a 120m wide street." << std::endl;
            rxStreetWidth = 120.0;
        }
        streetWidths[receiverRoad] = rxStreetWidth;
    }

    double streetWidth = std::min(txStreetWidth, rxStreetWidth);
    // based on "On the impact of street width on 5.9 GHz radio signal propagation in vehicular networks", the 2-ray model does not
    // make sense for street widths below 18m, for 28m it's still reasonable.
    // Thus, we employ the simple approach of scaling the relative permittivity between 18m and 38m (quadratically).
    // Further measurement studies to this end would be desirable.
    double scaling = 1.0;
    if (streetWidth > 18.0 && streetWidth < 38.0)
        scaling = pow((streetWidth - 18.0) / 20.0, 2);
    else if (streetWidth <= 18.0)
        scaling = 0.0;

    return scaling;
}

