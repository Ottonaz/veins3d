#include <veins/modules/analogueModel/GarageModels.h>
#include "veins/modules/utility/NBHeightMapper.h"
#include "veins/base/modules/BaseWorldUtility.h"
#include "veins/modules/obstacle/ObstacleControl.h"

using Veins::AirFrame;
using Veins::ObstacleControlAccess;

GarageModels::GarageModels(double carrierFrequency, int numPaths, double kFactor, simtime_t_cref interval)
{
    this->carrierFrequency = carrierFrequency;
    floorCtrl = FloorControlAccess().getIfExists();
    obstCtrl = ObstacleControlAccess().getIfExists();

    diffModel = new EnvironmentalDiffraction(carrierFrequency, false, 0.0, 0.0, true);
    riceRayleighModel = new RiceRayleighFading(carrierFrequency, numPaths, kFactor, interval);
}

void GarageModels::filterSignal(AirFrame *frame, const Coord& senderPos, const Coord& receiverPos)
{
    Signal& s = frame->getSignal();

    double dist = (receiverPos - senderPos).length();
    double freespace = BaseWorldUtility::speedOfLight() / (4 * M_PI * dist * carrierFrequency);
    freespace = freespace * freespace;

    // apply FloorAttenuation
    double floorFactor = floorCtrl->calculateAttenuation(senderPos, receiverPos);
    double wallFactor = 1.0;
    double diffFactor = 1.0;

    // if FloorAttenuation is 1.0, we have same-floor communication
    if (FWMath::close(floorFactor, 1.0)) {
        // apply DiffractionModel for vehicles in LOS
        diffFactor = diffModel->calcAttenuation(frame, senderPos, receiverPos, true);

        wallFactor = obstCtrl->calculateAttenuation(senderPos, receiverPos, true).factor;

        // if no wall and no vehicle in LOS, apply Rice fading (K factor from papers)
        // otherwise, apply Rayleigh fading (or Rice model with small K factor, in general)
        if (FWMath::close(wallFactor, 1.0) && FWMath::close(diffFactor, 1.0))
            riceRayleighModel->filterSignal(frame, senderPos, receiverPos);
        else
            riceRayleighModel->filterSignal(frame, senderPos, receiverPos, 0.0);
    }

    // add Mapping of constant attenuation factors
    double constFactors = freespace * floorFactor * wallFactor * diffFactor;
    bool hasFrequency = s.getTransmissionPower()->getDimensionSet().hasDimension(Dimension::frequency());
    const DimensionSet& domain = hasFrequency ? DimensionSet::timeFreqDomain() : DimensionSet::timeDomain();
    ConstantSimpleConstMapping* constAttMapping = new ConstantSimpleConstMapping(domain, constFactors);
    s.addAttenuation(constAttMapping);
}

