#ifndef FLOORATTENUATION_H_
#define FLOORATTENUATION_H_

#include <veins/base/phyLayer/AnalogueModel.h>
#include <veins/modules/floor/FloorControl.h>
#include "veins/base/utils/Coord.h"
#include "veins/base/messages/AirFrame_m.h"

/**
 * @brief This model can be used in three-dimensional scenarios in order to take into account the
 * impact of "vertical" communication through floors (if multiple road levels are present). The floors
 * are represented by Floor objects and managed by the FloorControl class.
 *
 * A detailed explanation of this model in the context of 3D VANET simulation and simulation results
 * can be found in:
 * Alexander Brummer, Reinhard German and Anatoli Djanatliev, "Modeling V2X Communications Across
 * Multiple Road Levels," 90th IEEE Vehicular Technology Conference (VTC2019-Fall), 2nd IEEE Connected
 * and Automated Vehicles Symposium (CAVS 2019), Honolulu, HI, USA, September 2019
 *
 * An example config.xml for this AnalogueModel can be the following:
 * @verbatim
    <AnalogueModel type="FloorAttenuation">
    	<!-- The signal frequency -->
		<parameter name="carrierFrequency" type="double" value="5.890e+9"/>
		<floors>
			<!-- floor type definition, not used at the moment -->
			<type id="general" att-factor="0" />
			<!-- manual definition of floors using the poly tag can be used
			     here as well -->
		</floors>
	</AnalogueModel>
   @endverbatim
 *
 * @author Alexander Brummer
 *
 * @ingroup analogueModels
 */
class FloorAttenuation : public AnalogueModel
{
public:
	/**
	 * @brief Initializes the analogue model.
	 *
	 * @param floorControl reference to the FloorControl object
	 * @param carrierFrequency the carrier frequency
	 */
    FloorAttenuation(FloorControl& floorControl, double carrierFrequency);

    /**
     * @brief Filters a specified AirFrame's Signal by adding the resulting attenuation.
     * @param frame the Airframe in question
     * @param senderPos the sender's position
     * @param receiverPos the receiver's position
     */
    virtual void filterSignal(AirFrame* frame, const Coord& senderPos, const Coord& receiverPos);

    /**
     * @brief Calculates the attenuation due to floor attenuation.
     *
     * @param senderPos the sender's position
     * @param receiverPos the receiver's position
     * @return value of attenuation in linear units (non-dB)
     */
    double calcAttenuation(const Coord& senderPos, const Coord& receiverPos);

    /**
     * @brief Return a reference to the FloorControl module.
     *
     * @return reference to the FloorControl module
     */
    FloorControl& getFloorControl() const;

private:
    /** @brief Reference to the FloorControl object */
    FloorControl& floorControl;

    /** @brief Carrier frequency */
    double carrierFreq;
};

#endif /* FLOORATTENUATION_H_ */
