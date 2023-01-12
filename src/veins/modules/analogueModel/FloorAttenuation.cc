#include <veins/modules/analogueModel/FloorAttenuation.h>
#include <veins/modules/floor/FloorControl.h>
#include "veins/base/utils/FindModule.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/base/phyLayer/Signal_.h"

using Veins::TraCIScenarioManager;

#define debugEV EV << "PhyLayer(FloorAttenuation): "


FloorAttenuation::FloorAttenuation(FloorControl& floorControl, double carrierFrequency):
        floorControl(floorControl),
        carrierFreq(carrierFrequency) {

    TraCIScenarioManager* traciManager = FindModule<TraCIScenarioManager*>::findGlobalModule();
    if (traciManager == NULL) {
        throw cRuntimeError("Could not find TraCIScenarioManager module");
    }

}

void FloorAttenuation::filterSignal(AirFrame* frame, const Coord& senderPos, const Coord& receiverPos) {
    Signal& s = frame->getSignal();
    bool hasFrequency = s.getTransmissionPower()->getDimensionSet().hasDimension(Dimension::frequency());
    const DimensionSet& domain = hasFrequency ? DimensionSet::timeFreqDomain() : DimensionSet::timeDomain();

    ConstantSimpleConstMapping* attMapping = new ConstantSimpleConstMapping(domain,
            calcAttenuation(senderPos, receiverPos));
    s.addAttenuation(attMapping);
}

double FloorAttenuation::calcAttenuation(const Coord& senderPos, const Coord& receiverPos) {
    double factor = floorControl.calculateAttenuation(senderPos, receiverPos);
    debugEV << "value is: " << factor << endl;
    return factor;
}

FloorControl& FloorAttenuation::getFloorControl() const {
    return floorControl;
}

