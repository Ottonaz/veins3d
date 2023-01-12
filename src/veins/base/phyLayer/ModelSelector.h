/*
 * ModelSelector.h
 *
 *  Created on: Jan 24, 2022
 *      Author: brummer
 */

#ifndef MODELSELECTOR_H_
#define MODELSELECTOR_H_

#include <vector>
#include <map>
#include "veins/base/phyLayer/AnalogueModel.h"
#include "veins/base/utils/Coord.h"
#include "veins/base/phyLayer/PhyUtils.h"
#include "veins/modules/analogueModel/SimplePathlossModel.h"
#include "veins/modules/analogueModel/TwoRayInterferenceModel.h"
#include "veins/modules/analogueModel/NRayGroundInterference.h"
#include "veins/modules/analogueModel/EnvironmentalDiffraction.h"
#include "veins/modules/analogueModel/SimpleObstacleShadowing.h"
#include "veins/modules/analogueModel/FloorAttenuation.h"
#include "veins/modules/analogueModel/RiceRayleighFading.h"
#include "veins/modules/tunnel/TunnelControl.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

using Veins::TraCICommandInterface;

namespace Veins {
class AirFrame;
}
using Veins::AirFrame;

/**
 * @brief This class is used by the BasePhyLayer if the corresponding flag ("useModelSelector") is set there.
 * In this case, the BasePhyLayer does not apply all the analogue models itself. Instead, it calls the
 * ModelSelector, which selects the models that should be used depending on the environment of the current
 * transmission.
 *
 * @author Alexander Brummer
 *
 * @ingroup phyLayer
 */
class ModelSelector
{
private:
    typedef std::vector<AnalogueModel*> AnalogueModelList;
    AnalogueModelList& analogueModels;
    RadioStateAnalogueModel* rsam;
    SimplePathlossModel* simplePathloss;
    TwoRayInterferenceModel* twoRayModel;
    NRayGroundInterference* nRayModel;
    EnvironmentalDiffraction* environDiff;
    SimpleObstacleShadowing* obstShadowing;
    FloorAttenuation* floorModel;
    RiceRayleighFading* rrFading;
    TunnelControl* tunnelControl;

    TraCICommandInterface* traciCI;
    static std::map<std::string, double> streetWidths;

    void applyGarageModels(AirFrame *frame, const Coord& senderPos, const Coord& receiverPos);
    void applyTunnelModels(AirFrame *frame, const Coord& senderPos, const Coord& receiverPos, std::string sndTunnel, std::string rcvTunnel);
    bool applyNLOSModels(AirFrame *frame, const Coord& senderPos, const Coord& receiverPos);
    void applyGroundReflModel(AirFrame *frame, const Coord& senderPos, const Coord& receiverPos, const std::string& senderRoad, const std::string& receiverRoad);
    double getGroundReflScaling(AirFrame* frame, const std::string& senderRoad, const std::string& receiverRoad);

public:
    ModelSelector(AnalogueModelList&);
    virtual ~ModelSelector();

    void filterSignal(AirFrame *frame, const Coord& senderPos, const Coord& receiverPos);

};

#endif /* MODELSELECTOR_H_ */
