//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

/*
 * Based on PhyLayer.cc from Karl Wessel
 * and modifications by Christopher Saloman
 */

#include "veins/modules/phy/PhyLayer80211p.h"

#include "veins/modules/phy/Decider80211p.h"
#include "veins/modules/analogueModel/SimplePathlossModel.h"
#include "veins/modules/analogueModel/BreakpointPathlossModel.h"
#include "veins/modules/analogueModel/LogNormalShadowing.h"
#include "veins/modules/analogueModel/JakesFading.h"
#include "veins/modules/analogueModel/PERModel.h"
#include "veins/modules/analogueModel/SimpleObstacleShadowing.h"
#include "veins/modules/analogueModel/TwoRayInterferenceModel.h"
#include "veins/modules/analogueModel/NakagamiFading.h"
#include "veins/modules/analogueModel/DiffAndGround.h"
#include <veins/modules/analogueModel/FloorAttenuation.h>
#include "veins/modules/analogueModel/RiceRayleighFading.h"
#include "veins/modules/analogueModel/GarageModels.h"
#include "veins/modules/analogueModel/GarageModelSelector.h"
#include <veins/modules/floor/FloorControl.h>
#include "veins/base/connectionManager/BaseConnectionManager.h"
#include "veins/modules/utility/Consts80211p.h"
#include "veins/modules/messages/AirFrame11p_m.h"
#include "veins/base/phyLayer/MacToPhyControlInfo.h"

using Veins::ObstacleControlAccess;

Define_Module(PhyLayer80211p);

/** This is needed to circumvent a bug in MiXiM that allows different header length interpretations for receiving and sending airframes*/
void PhyLayer80211p::initialize(int stage)
{
    if (stage == 0) {
        //get ccaThreshold before calling BasePhyLayer::initialize() which instantiates the deciders
        ccaThreshold = pow(10, par("ccaThreshold").doubleValue() / 10);
        allowTxDuringRx = par("allowTxDuringRx").boolValue();
        collectCollisionStatistics = par("collectCollisionStatistics").boolValue();
    }
    BasePhyLayer::initialize(stage);
    if (stage == 0) {
        if (par("headerLength").longValue() != PHY_HDR_TOTAL_LENGTH) {
            throw cRuntimeError(
                    "The header length of the 802.11p standard is 46bit, please change your omnetpp.ini accordingly by either setting it to 46bit or removing the entry");
        }
        //erase the RadioStateAnalogueModel
        analogueModels.erase(analogueModels.begin());
    }
}

AnalogueModel* PhyLayer80211p::getAnalogueModelFromName(std::string name, ParameterMap& params)
{

    if (name == "SimplePathlossModel") {
        return initializeSimplePathlossModel(params);
    }
    else if (name == "LogNormalShadowing") {
        return initializeLogNormalShadowing(params);
    }
    else if (name == "JakesFading") {
        return initializeJakesFading(params);
    }
    else if (name == "BreakpointPathlossModel") {
        return initializeBreakpointPathlossModel(params);
    }
    else if (name == "PERModel") {
        return initializePERModel(params);
    }
    else if (name == "SimpleObstacleShadowing") {
        return initializeSimpleObstacleShadowing(params);
    }
    else if (name == "TwoRayInterferenceModel") {
        if (world->use2D())
            error(
                    "The TwoRayInterferenceModel uses nodes' z-position as the antenna height over ground. Refusing to work in a 2D world");
        return initializeTwoRayInterferenceModel(params);
    }
    else if (name == "NakagamiFading") {
        return initializeNakagamiFading(params);
    }
    else if (name == "EnvironmentalDiffraction") {
        if (world->use2D())
            error(
                    "The EnvironmentalDiffraction model uses nodes' z-positions and only makes sense in a 3D environment. Refusing to work in a 2D world");
        return initializeEnvironmentalDiffraction(params);
    }
    else if (name == "NRayGroundInterference") {
        if (world->use2D())
            error(
                    "The NRayGroundInterference model uses nodes' z-positions and only makes sense in a 3D environment. Refusing to work in a 2D world");
        return initializeNRayGroundInterference(params);
    }
    else if (name == "DiffAndGround") {
        if (world->use2D())
            error(
                    "The DiffAndGround model uses nodes' z-positions and only makes sense in a 3D environment. Refusing to work in a 2D world");
        return initializeDiffAndGround(params);
    }
    else if (name == "FloorAttenuation") {
        return initializeFloorAttenuation(params);
    }
    else if (name == "RiceRayleighFading") {
        return initializeRiceRayleighFading(params);
    }
    else if (name == "GarageModels") {
        return initializeGarageModels(params);
    }
    else if (name == "GarageModelSelector") {
        return initializeGarageModelSelector(params);
    }
    return BasePhyLayer::getAnalogueModelFromName(name, params);
}

AnalogueModel* PhyLayer80211p::initializeLogNormalShadowing(ParameterMap& params)
{
    double mean = params["mean"].doubleValue();
    double stdDev = params["stdDev"].doubleValue();
    simtime_t interval = params["interval"].doubleValue();

    return new LogNormalShadowing(mean, stdDev, interval);
}

AnalogueModel* PhyLayer80211p::initializeJakesFading(ParameterMap& params)
{
    int fadingPaths = params["fadingPaths"].longValue();
    simtime_t delayRMS = params["delayRMS"].doubleValue();
    simtime_t interval = params["interval"].doubleValue();

    double carrierFrequency = getCarrierFrequency(params, __FUNCTION__);

    return new JakesFading(fadingPaths, delayRMS, carrierFrequency, interval);
}

AnalogueModel* PhyLayer80211p::initializeBreakpointPathlossModel(ParameterMap& params)
{
    double alpha1 = -1, alpha2 = -1, breakpointDistance = -1;
    double L01 = -1, L02 = -1;
    double carrierFrequency;
    bool useTorus = world->useTorus();
    const Coord& playgroundSize = *(world->getPgs());
    ParameterMap::iterator it;

    it = params.find("alpha1");
    if (it != params.end()) // parameter alpha1 has been specified in config.xml
            {
        // set alpha1
        alpha1 = it->second.doubleValue();
        coreEV << "initializeBreakpointPathlossModel(): alpha1 set from config.xml to " << alpha1 << endl;
        // check whether alpha is not smaller than specified in ConnectionManager
        if (cc->hasPar("alpha") && alpha1 < cc->par("alpha").doubleValue()) {
            // throw error
            throw cRuntimeError(
                    "initializeBreakpointPathlossModel(): alpha can't be smaller than specified in \
	               ConnectionManager. Please adjust your config.xml file accordingly");
        }
    }
    it = params.find("L01");
    if (it != params.end()) {
        L01 = it->second.doubleValue();
    }
    it = params.find("L02");
    if (it != params.end()) {
        L02 = it->second.doubleValue();
    }

    it = params.find("alpha2");
    if (it != params.end()) // parameter alpha1 has been specified in config.xml
            {
        // set alpha2
        alpha2 = it->second.doubleValue();
        coreEV << "initializeBreakpointPathlossModel(): alpha2 set from config.xml to " << alpha2 << endl;
        // check whether alpha is not smaller than specified in ConnectionManager
        if (cc->hasPar("alpha") && alpha2 < cc->par("alpha").doubleValue()) {
            // throw error
            throw cRuntimeError(
                    "initializeBreakpointPathlossModel(): alpha can't be smaller than specified in \
	               ConnectionManager. Please adjust your config.xml file accordingly");
        }
    }
    it = params.find("breakpointDistance");
    if (it != params.end()) // parameter alpha1 has been specified in config.xml
            {
        breakpointDistance = it->second.doubleValue();
        coreEV << "initializeBreakpointPathlossModel(): breakpointDistance set from config.xml to " << alpha2 << endl;
        // check whether alpha is not smaller than specified in ConnectionManager
    }

    carrierFrequency = getCarrierFrequency(params, __FUNCTION__);

    if (alpha1 == -1 || alpha2 == -1 || breakpointDistance == -1 || L01 == -1 || L02 == -1) {
        throw cRuntimeError("Undefined parameters for breakpointPathlossModel. Please check your configuration.");
    }

    return new BreakpointPathlossModel(L01, L02, alpha1, alpha2, breakpointDistance, carrierFrequency, useTorus,
            playgroundSize, coreDebug);
}

AnalogueModel* PhyLayer80211p::initializeTwoRayInterferenceModel(ParameterMap& params)
{
    ASSERT(params.count("DielectricConstant") == 1);

    double dielectricConstant = params["DielectricConstant"].doubleValue();

    return new TwoRayInterferenceModel(dielectricConstant, coreDebug);
}

AnalogueModel* PhyLayer80211p::initializeNakagamiFading(ParameterMap& params)
{
    bool constM = params["constM"].boolValue();
    double m = 0;
    if (constM) {
        m = params["m"].doubleValue();
    }
    return new NakagamiFading(constM, m, coreDebug);
}

AnalogueModel* PhyLayer80211p::initializeSimplePathlossModel(ParameterMap& params)
{
    // init with default value
    double alpha = 2.0;
    double carrierFrequency;
    bool useTorus = world->useTorus();
    const Coord& playgroundSize = *(world->getPgs());

    // get alpha-coefficient from config
    ParameterMap::iterator it = params.find("alpha");

    if (it != params.end()) // parameter alpha has been specified in config.xml
            {
        // set alpha
        alpha = it->second.doubleValue();
        coreEV << "initializeSimplePathlossModel(): alpha set from config.xml to " << alpha << endl;

        // check whether alpha is not smaller than specified in ConnectionManager
        if (cc->hasPar("alpha") && alpha < cc->par("alpha").doubleValue()) {
            // throw error
            throw cRuntimeError(
                    "initializeSimplePathlossModel(): alpha can't be smaller than specified in \
	               ConnectionManager. Please adjust your config.xml file accordingly");
        }
    }
    else // alpha has not been specified in config.xml
    {
        if (cc->hasPar("alpha")) // parameter alpha has been specified in ConnectionManager
                {
            // set alpha according to ConnectionManager
            alpha = cc->par("alpha").doubleValue();
            coreEV << "initializeSimplePathlossModel(): alpha set from ConnectionManager to " << alpha << endl;
        }
        else // alpha has not been specified in ConnectionManager
        {
            // keep alpha at default value
            coreEV << "initializeSimplePathlossModel(): alpha set from default value to " << alpha << endl;
        }
    }

    carrierFrequency = getCarrierFrequency(params, __FUNCTION__);

    return new SimplePathlossModel(alpha, carrierFrequency, useTorus, playgroundSize, coreDebug);
}

AnalogueModel* PhyLayer80211p::initializePERModel(ParameterMap& params)
{
    double per = params["packetErrorRate"].doubleValue();
    return new PERModel(per);
}

Decider* PhyLayer80211p::getDeciderFromName(std::string name, ParameterMap& params)
{
    if (name == "Decider80211p") {
        protocolId = IEEE_80211;
        return initializeDecider80211p(params);
    }
    return BasePhyLayer::getDeciderFromName(name, params);
}

AnalogueModel* PhyLayer80211p::initializeSimpleObstacleShadowing(ParameterMap& params)
{
    double carrierFrequency;
    bool useTorus = world->useTorus();
    const Coord& playgroundSize = *(world->getPgs());

    ParameterMap::iterator it;

    carrierFrequency = getCarrierFrequency(params, __FUNCTION__);

    ObstacleControl* obstacleControlP = ObstacleControlAccess().getIfExists();
    if (!obstacleControlP)
        throw cRuntimeError("initializeSimpleObstacleShadowing(): cannot find ObstacleControl module");
    return new SimpleObstacleShadowing(*obstacleControlP, carrierFrequency, useTorus, playgroundSize, coreDebug);
}

namespace {
template<typename Out>
void split(const std::string &s, char delim, Out result)
{
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim)
{
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}
}

AnalogueModel* PhyLayer80211p::initializeEnvironmentalDiffraction(ParameterMap& params)
{
    double carrierFrequency;
    std::vector<std::string> demFiles;
    bool isRasterType;
    double spacing;
    double demCellSize;
    bool considerDEM = false;
    bool considerVehicles = false;

    ParameterMap::iterator it;

    carrierFrequency = getCarrierFrequency(params, __FUNCTION__);

    it = params.find("considerDEM");
    if (it != params.end() && it->second.boolValue()) {
        considerDEM = true;
        it = params.find("demFiles");
        if (it == params.end()) {
            throw cRuntimeError("initializeEnvironmentalDiffraction(): No DEM file(s) provided in config.xml");
        }
        else {
            demFiles = split(it->second.stringValue(), ',');
        }
        it = params.find("isRasterType");
        if (it == params.end()) {
            throw cRuntimeError(
                    "initializeEnvironmentalDiffraction(): Not specified whether DEM is raster type or vector type (parameter isRasterType)");
        }
        else {
            isRasterType = it->second.boolValue();
        }
        it = params.find("spacing");
        if (it == params.end()) {
            throw cRuntimeError(
                    "initializeEnvironmentalDiffraction(): No spacing of distance between height profile points specified");
        }
        else {
            spacing = it->second.doubleValue();
        }
        it = params.find("demCellSize");
        if (it == params.end()) {
            demCellSize = 0.5;
        }
        else {
            demCellSize = it->second.doubleValue();
        }
    }

    it = params.find("considerVehicles");
    if (it != params.end() && it->second.boolValue()) {
        considerVehicles = true;
    }

    if (!considerDEM && !considerVehicles) {
        throw cRuntimeError(
                "initializeEnvironmentalDiffraction(): At least one of considerDEM and considerVehicles must be set to true");
    }

    return new EnvironmentalDiffraction(carrierFrequency, considerDEM, demFiles, isRasterType, demCellSize, spacing,
            considerVehicles);
}

AnalogueModel* PhyLayer80211p::initializeNRayGroundInterference(ParameterMap& params)
{
    double carrierFrequency;
    std::vector<std::string> demFiles;
    bool isRasterType;
    double spacing;
    double epsilonR;

    ParameterMap::iterator it;

    carrierFrequency = getCarrierFrequency(params, __FUNCTION__);

    it = params.find("epsilonR");
    if (it == params.end()) {
        throw cRuntimeError(
                "initializeNRayGroundInterference(): No epsilonR (relative permittivity of the ground) specified");
    }
    else {
        epsilonR = it->second.doubleValue();
    }

    it = params.find("demFiles");
    if (it == params.end()) {
        throw cRuntimeError("initializeNRayGroundInterference(): No DEM file(s) provided in config.xml");
    }
    else {
        demFiles = split(it->second.stringValue(), ',');
    }
    it = params.find("isRasterType");
    if (it == params.end()) {
        throw cRuntimeError(
                "initializeNRayGroundInterference(): Not specified whether DEM is raster type or vector type (parameter isRasterType)");
    }
    else {
        isRasterType = it->second.boolValue();
    }
    it = params.find("spacing");
    if (it == params.end()) {
        throw cRuntimeError(
                "initializeNRayGroundInterference(): No spacing of distance between height profile points specified");
    }
    else {
        spacing = it->second.doubleValue();
    }
    //		it = params.find("demCellSize");
    //		if (it == params.end()) {
    //			demCellSize = 0.5;
    //		} else {
    //			demCellSize = it->second.doubleValue();
    //		}

    return new NRayGroundInterference(carrierFrequency, epsilonR, demFiles, isRasterType, spacing);
}

AnalogueModel* PhyLayer80211p::initializeDiffAndGround(ParameterMap& params)
{
    double carrierFrequency;
    bool considerDEM = false;
    bool considerVehicles = false;
    double diffSpacing;
    double demCellSize;
    std::vector<std::string> demFiles;
    bool isRasterType;
    double groundSpacing;
    double epsilonR;

    ParameterMap::iterator it;

    carrierFrequency = getCarrierFrequency(params, __FUNCTION__);

    it = params.find("epsilonR");
    if (it == params.end()) {
        throw cRuntimeError("initializeDiffAndGround(): No epsilonR (relative permittivity of the ground) specified");
    }
    else {
        epsilonR = it->second.doubleValue();
    }

    it = params.find("demFiles");
    if (it == params.end()) {
        throw cRuntimeError("initializeDiffAndGround(): No DEM file(s) provided in config.xml");
    }
    else {
        demFiles = split(it->second.stringValue(), ',');
    }
    it = params.find("isRasterType");
    if (it == params.end()) {
        throw cRuntimeError(
                "initializeDiffAndGround(): Not specified whether DEM is raster type or vector type (parameter isRasterType)");
    }
    else {
        isRasterType = it->second.boolValue();
    }
    it = params.find("groundSpacing");
    if (it == params.end()) {
        throw cRuntimeError(
                "initializeDiffAndGround(): No spacing of distance between nray height profile points specified");
    }
    else {
        groundSpacing = it->second.doubleValue();
    }

    it = params.find("considerDEM");
    if (it != params.end() && it->second.boolValue()) {
        considerDEM = true;
        it = params.find("diffSpacing");
        if (it == params.end()) {
            throw cRuntimeError(
                    "initializeDiffAndGround(): No spacing of distance between diff height profile points specified");
        }
        else {
            diffSpacing = it->second.doubleValue();
        }
        it = params.find("demCellSize");
        if (it == params.end()) {
            demCellSize = 0.5;
        }
        else {
            demCellSize = it->second.doubleValue();
        }
    }

    it = params.find("considerVehicles");
    if (it != params.end() && it->second.boolValue()) {
        considerVehicles = true;
    }

    if (!considerDEM && !considerVehicles) {
        throw cRuntimeError(
                "initializeDiffAndGround(): At least one of considerDEM and considerVehicles must be set to true");
    }

    return new DiffAndGround(demFiles, isRasterType, carrierFrequency, epsilonR, groundSpacing, considerDEM,
            diffSpacing, demCellSize, considerVehicles);
}

AnalogueModel* PhyLayer80211p::initializeFloorAttenuation(ParameterMap& params)
{
    double carrierFrequency;

    ParameterMap::iterator it;

    carrierFrequency = getCarrierFrequency(params, __FUNCTION__);

    FloorControl* floorControlP = FloorControlAccess().getIfExists();
    if (!floorControlP)
        throw cRuntimeError("initializeFloorAttenuation(): cannot find FloorControl module");
    return new FloorAttenuation(*floorControlP, carrierFrequency);
}

AnalogueModel* PhyLayer80211p::initializeRiceRayleighFading(ParameterMap& params)
{
    double carrierFrequency;
    int numPaths;
    double kFactor;
    simtime_t interval;

    ParameterMap::iterator it;

    carrierFrequency = getCarrierFrequency(params, __FUNCTION__);

    it = params.find("numPaths");
    if (it == params.end()) {
        throw cRuntimeError(
                "initializeRiceRayleighFading(): Not specified how many paths should be considered (parameter numPaths, recommended: 8)");
    }
    else {
        numPaths = it->second.longValue();
    }

    it = params.find("kFactor");
    if (it == params.end()) {
        throw cRuntimeError(
                "initializeRiceRayleighFading(): Rician K factor not specified (parameter kFactor in linear units (NOT dB), for Rayleigh fading use 0)");
    }
    else {
        kFactor = it->second.doubleValue();
    }

    it = params.find("interval");
    if (it == params.end()) {
        throw cRuntimeError("initializeRiceRayleighFading(): interval not specified");
    }
    else {
        interval = it->second.doubleValue();
    }

    return new RiceRayleighFading(carrierFrequency, numPaths, kFactor, interval);
}

AnalogueModel* PhyLayer80211p::initializeGarageModels(ParameterMap& params)
{
    double carrierFrequency;
    int numPaths;
    double kFactor;
    simtime_t interval;

    ParameterMap::iterator it;

    carrierFrequency = getCarrierFrequency(params, __FUNCTION__);

    it = params.find("numPaths");
    if (it == params.end()) {
        throw cRuntimeError(
                "initializeGarageModels(): Not specified how many paths should be considered (parameter numPaths, recommended: 8)");
    }
    else {
        numPaths = it->second.longValue();
    }

    it = params.find("kFactor");
    if (it == params.end()) {
        throw cRuntimeError(
                "initializeGarageModels(): Rician K factor not specified (parameter kFactor in linear units (NOT dB), for Rayleigh fading use 0)");
    }
    else {
        kFactor = it->second.doubleValue();
    }

    it = params.find("interval");
    if (it == params.end()) {
        throw cRuntimeError("initializeGarageModels(): interval not specified");
    }
    else {
        interval = it->second.doubleValue();
    }

    return new GarageModels(carrierFrequency, numPaths, kFactor, interval);
}

AnalogueModel* PhyLayer80211p::initializeGarageModelSelector(ParameterMap& params)
{
    std::vector<std::string> demFiles;
    bool isRasterType;
    double carrierFrequency;
    bool considerDEM = false;
    double diffSpacing;
    double demCellSize;
    bool considerVehicles = false;
    int numPaths;
    double kFactor;
    simtime_t interval;

    ParameterMap::iterator it;

    carrierFrequency = getCarrierFrequency(params, __FUNCTION__);

    it = params.find("considerDEM");
    if (it != params.end() && it->second.boolValue()) {
        considerDEM = true;
        it = params.find("demFiles");
        if (it == params.end()) {
            throw cRuntimeError("initializeGarageModelSelector(): No DEM file(s) provided in config.xml");
        }
        else {
            demFiles = split(it->second.stringValue(), ',');
        }
        it = params.find("isRasterType");
        if (it == params.end()) {
            throw cRuntimeError(
                    "initializeGarageModelSelector(): Not specified whether DEM is raster type or vector type (parameter isRasterType)");
        }
        else {
            isRasterType = it->second.boolValue();
        }
        it = params.find("diffSpacing");
        if (it == params.end()) {
            throw cRuntimeError(
                    "initializeGarageModelSelector(): No spacing of distance between height profile points specified");
        }
        else {
            diffSpacing = it->second.doubleValue();
        }
        it = params.find("demCellSize");
        if (it == params.end()) {
            demCellSize = 0.5;
        }
        else {
            demCellSize = it->second.doubleValue();
        }
    }

    it = params.find("considerVehicles");
    if (it != params.end() && it->second.boolValue()) {
        considerVehicles = true;
    }

    if (!considerDEM && !considerVehicles) {
        throw cRuntimeError(
                "initializeGarageModelSelector(): At least one of considerDEM and considerVehicles must be set to true");
    }

    it = params.find("numPaths");
    if (it == params.end()) {
        throw cRuntimeError(
                "initializeGarageModelSelector(): Not specified how many paths should be considered (parameter numPaths, recommended: 8)");
    }
    else {
        numPaths = it->second.longValue();
    }

    it = params.find("kFactor");
    if (it == params.end()) {
        throw cRuntimeError(
                "initializeGarageModelSelector(): Rician K factor not specified (parameter kFactor in linear units (NOT dB), for Rayleigh fading use 0)");
    }
    else {
        kFactor = it->second.doubleValue();
    }

    it = params.find("interval");
    if (it == params.end()) {
        throw cRuntimeError("initializeGarageModelSelector(): interval not specified");
    }
    else {
        interval = it->second.doubleValue();
    }

    return new GarageModelSelector(carrierFrequency, demFiles, isRasterType, considerDEM, diffSpacing, demCellSize,
            considerVehicles, numPaths, kFactor, interval);
}

double PhyLayer80211p::getCarrierFrequency(ParameterMap& params, std::string caller, double defaultFreq)
{
    double carrierFrequency = defaultFreq;

    ParameterMap::iterator it;
    // get carrierFrequency from config
    it = params.find("carrierFrequency");

    // parameter carrierFrequency has been specified in config.xml
    if (it != params.end()) {
        // set carrierFrequency
        carrierFrequency = it->second.doubleValue();
        coreEV << caller << "(): carrierFrequency set from config.xml to " << carrierFrequency << endl;

        // check whether carrierFrequency is not smaller than specified in ConnectionManager
        if (cc->hasPar("carrierFrequency") && carrierFrequency < cc->par("carrierFrequency").doubleValue()) {
            // throw error
            throw cRuntimeError(
                    (caller
                            + "(): carrierFrequency can't be smaller than specified in ConnectionManager. Please adjust your config.xml file accordingly").c_str());
        }
    }
    else // carrierFrequency has not been specified in config.xml
    {
        // parameter carrierFrequency has been specified in ConnectionManager
        if (cc->hasPar("carrierFrequency")) {
            // set carrierFrequency according to ConnectionManager
            carrierFrequency = cc->par("carrierFrequency").doubleValue();
            coreEV << caller << "(): carrierFrequency set from ConnectionManager to " << carrierFrequency << endl;
        }
        else // carrierFrequency has not been specified in ConnectionManager
        {
            // keep carrierFrequency at default value
            coreEV << caller << "(): carrierFrequency set from default value to " << carrierFrequency << endl;
        }
    }

    return carrierFrequency;
}

Decider* PhyLayer80211p::initializeDecider80211p(ParameterMap& params)
{
    double centerFreq = params["centerFrequency"];
    Decider80211p* dec = new Decider80211p(this, sensitivity, ccaThreshold, allowTxDuringRx, centerFreq,
            findHost()->getIndex(), collectCollisionStatistics, coreDebug);
    dec->setPath(getParentModule()->getFullPath());
    return dec;
}

void PhyLayer80211p::changeListeningFrequency(double freq)
{
    Decider80211p* dec = dynamic_cast<Decider80211p*>(decider);
    assert(dec);
    dec->changeFrequency(freq);
}

void PhyLayer80211p::handleSelfMessage(cMessage* msg)
{

    switch (msg->getKind()) {
        //transmission overBasePhyLayer::
        case TX_OVER: {
            assert(msg == txOverTimer);
            sendControlMsgToMac(new cMessage("Transmission over", TX_OVER));
            //check if there is another packet on the chan, and change the chan-state to idle
            Decider80211p* dec = dynamic_cast<Decider80211p*>(decider);
            assert(dec);
            if (dec->cca(simTime(), NULL)) {
                //chan is idle
                DBG << "Channel idle after transmit!\n";
                dec->setChannelIdleStatus(true);

            }
            else {
                DBG << "Channel not yet idle after transmit!\n";
            }
            break;
        }
            //radio switch over
        case RADIO_SWITCHING_OVER:
            assert(msg == radioSwitchingOverTimer);
            BasePhyLayer::finishRadioSwitching();
            break;

            //AirFrame
        case AIR_FRAME:
            BasePhyLayer::handleAirFrame(static_cast<AirFrame*>(msg));
            break;

            //ChannelSenseRequest
        case CHANNEL_SENSE_REQUEST:
            BasePhyLayer::handleChannelSenseRequest(msg);
            break;

        default:
            break;
    }
}

AirFrame *PhyLayer80211p::encapsMsg(cPacket *macPkt)
{
    // the cMessage passed must be a MacPacket... but no cast needed here
    // MacPkt* pkt = static_cast<MacPkt*>(msg);

    // ...and must always have a ControlInfo attached (contains Signal)
    cObject* ctrlInfo = macPkt->removeControlInfo();
    assert(ctrlInfo);

    // create the new AirFrame
    AirFrame* frame = new AirFrame11p(macPkt->getName(), AIR_FRAME);

    // Retrieve the pointer to the Signal-instance from the ControlInfo-instance.
    // We are now the new owner of this instance.
    Signal* s = MacToPhyControlInfo::getSignalFromControlInfo(ctrlInfo);
    // make sure we really obtained a pointer to an instance
    assert(s);

    // set the members
    assert(s->getDuration() > 0);
    frame->setDuration(s->getDuration());
    // copy the signal into the AirFrame
    frame->setSignal(*s);
    //set priority of AirFrames above the normal priority to ensure
    //channel consistency (before any thing else happens at a time
    //point t make sure that the channel has removed every AirFrame
    //ended at t and added every AirFrame started at t)
    frame->setSchedulingPriority(airFramePriority());
    frame->setProtocolId(myProtocolId());
    frame->setBitLength(headerLength);
    frame->setId(world->getUniqueAirFrameId());
    frame->setChannel(radio->getCurrentChannel());

    // pointer and Signal not needed anymore
    delete s;
    s = 0;

    // delete the Control info
    delete ctrlInfo;
    ctrlInfo = 0;

    frame->encapsulate(macPkt);

    // --- from here on, the AirFrame is the owner of the MacPacket ---
    macPkt = 0;
    coreEV << "AirFrame encapsulated, length: " << frame->getBitLength() << "\n";

    return frame;
}

int PhyLayer80211p::getRadioState()
{
    return BasePhyLayer::getRadioState();
}


simtime_t PhyLayer80211p::setRadioState(int rs)
{
    if (rs == Radio::TX)
        decider->switchToTx();
    return BasePhyLayer::setRadioState(rs);
}

void PhyLayer80211p::setCCAThreshold(double ccaThreshold_dBm)
{
    ccaThreshold = pow(10, ccaThreshold_dBm / 10);
    ((Decider80211p *) decider)->setCCAThreshold(ccaThreshold_dBm);
}
double PhyLayer80211p::getCCAThreshold()
{
    return 10 * log10(ccaThreshold);
}

void PhyLayer80211p::notifyMacAboutRxStart(bool enable)
{
    ((Decider80211p *) decider)->setNotifyRxStart(enable);
}

void PhyLayer80211p::requestChannelStatusIfIdle()
{
    Enter_Method_Silent();
    Decider80211p* dec = (Decider80211p*)decider;
    if (dec->cca(simTime(),NULL)) {
        //chan is idle
        DBG << "Request channel status: channel idle!\n";
        dec->setChannelIdleStatus(true);
    }
}
