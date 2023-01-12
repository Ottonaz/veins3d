#ifndef GARAGEMODELSELECTOR_H_
#define GARAGEMODELSELECTOR_H_

#include <veins/base/phyLayer/AnalogueModel.h>
#include "veins/base/messages/AirFrame_m.h"
#include "veins/modules/floor/FloorControl.h"
#include "veins/modules/analogueModel/EnvironmentalDiffraction.h"
#include "veins/modules/obstacle/ObstacleControl.h"
#include "veins/modules/analogueModel/GarageModels.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

using Veins::ObstacleControl;
using Veins::TraCICommandInterface;

/**
 * @brief Obsolete AnalogueModel which distinguishes between garage and normal environment.
 * Now, the ModelSelector integrated in the BasePhyLayer should be used instead.
 *
 *
 * An example config.xml for this AnalogueModel can be the following (includes the parameters of
 * all models, might be changed in upcoming versions; for a description of them, please check the
 * respective models):
 * @verbatim
     <AnalogueModel type="GarageModelSelector">
         <parameter name="carrierFrequency" type="double" value="5.890e+9"/>
         <parameter name="considerDEM" type="bool" value="false"/>
         <parameter name="demFiles" type="string" value="DTMconverted.tif"/>
         <parameter name="isRasterType" type="bool" value="true"/>
         <parameter name="diffSpacing" type="double" value="5"/>
         <parameter name="demCellSize" type="double" value="1"/>
         <parameter name="considerVehicles" type="bool" value="true"/>
         <parameter name="numPaths" type="long" value="8"/>
         <parameter name="kFactor" type="double" value="9.55"/>
         <parameter name="interval" type="double" value="0.0004"/>
         <obstacles>
             <type id="building" db-per-cut="9" db-per-meter="0.4" />
             <type id="innerWall" db-per-cut="5" db-per-meter="0" />
         </obstacles>
         <floors>
             <type id="general" att-factor="0" />
             <floorSeg id="floor_0" color="cyan" type="general" fill="0" layer="128.00" shape="8.75,15.50,1.38 8.75,26.71,1.38 78.75,26.71,1.38 78.75,15.50,1.38"/>
         </floors>
     </AnalogueModel>
 @endverbatim
 *
 * @author Alexander Brummer
 *
 * @ingroup analogueModels
 */
class GarageModelSelector : public AnalogueModel
{
public:

    /**
     * @brief Initializes the analogue model.
     *
     * @param carrierFrequency the carrier frequency
     * @param demFiles the filename(s) of the DEM
     * @param isRasterType whether the DEM is in raster format (GeoTiff)
     * @param considerDEM states whether the terrain characteristics should be considered for the diffraction model
     * @param diffSpacing spacing of queried height profile points for the diffraction model
     * @param demCellSize cell size for the DEM cache
     * @param considerVehicles states whether other vehicles should be considered
     * @param numPaths number of paths to be considered for Rice/Rayleigh fading
     * @param kFactor Rician K factor
     * @param interval interval at which fading samples are generated
     */
    GarageModelSelector(double carrierFrequency, std::vector<std::string> demFiles, bool isRasterType, bool considerDEM,
            double diffSpacing, double demCellSize, bool considerVehicles, int numPaths, double kFactor,
            simtime_t_cref interval);

    /**
     * @brief Destructor.
     * Delete the models/control objects.
     */
    virtual ~GarageModelSelector()
    {
        delete garageModel;
    }

    /**
     * @brief Filters a specified AirFrame's Signal by adding the correct attenuation.
     *
     * @param frame the Airframe in question
     * @param senderPos the sender's position
     * @param receiverPos the receiver's position
     */
    virtual void filterSignal(AirFrame *frame, const Coord& senderPos, const Coord& receiverPos);

protected:
    /** @brief Carrier frequency. */
    double carrierFrequency;

    /** @brief FloorControl object */
    FloorControl* floorCtrl;

    /** @brief ObstacleControl object */
    ObstacleControl* obstCtrl;

    /** @brief TraCICommandInterface object */
    TraCICommandInterface* traciCI;

    /** @brief EnvironmentalDiffraction model object */
    EnvironmentalDiffraction* diffModel;

    /** @brief GarageModels object */
    GarageModels* garageModel;
};

#endif /* GARAGEMODELSELECTOR_H_ */
