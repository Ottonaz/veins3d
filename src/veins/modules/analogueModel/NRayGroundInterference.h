#ifndef NRAYGROUNDINTERFERENCE_H_
#define NRAYGROUNDINTERFERENCE_H_

#include <veins/base/phyLayer/AnalogueModel.h>
#include "veins/base/messages/AirFrame_m.h"

/**
 * @brief Implementation of the n-ray ground interference model. This model is an extension of the two-ray
 * interference model for three-dimensional scenarios. In the 3D case, there can be an arbitrary number of
 * ground reflections depending on the ground shape. This model generates a height profile between sender
 * and receiver and checks each segment for a potential reflection.
 *
 * A detailed explanation of this model in the context of 3D VANET simulation and simulation results
 * can be found in:
 * Alexander Brummer, Lorenz Ammon and Anatoli Djanatliev, "N-Ray Ground Interference: Extending the
 * Two-Ray Interference Model for 3D Terrain Shapes," 11th IEEE Vehicular Networking Conference
 * (VNC 2019), Los Angeles, CA, USA, December 2019
 * Alexander Brummer, Moritz Guetlein, Matthias Schaefer, Reinhard German and Anatoli Djanatliev,
 * "Experimental Evaluation of the N-Ray Ground Interference Model," 12th IEEE Vehicular Networking
 * Conference (VNC 2020), Ulm, Germany (Virtual Event), December 2020
 *
 * An example config.xml for this AnalogueModel can be the following:
 * @verbatim
    <AnalogueModel type="NRayGroundInterference">
        <!-- The signal frequency -->
        <parameter name="carrierFrequency" type="double" value="5.890e+9"/>
		<!-- Relative permittivity -->
		<parameter name="epsilonR" type="double" value="1.025"/>
		<!-- File name of the DEM to query the terrain's elevation -->
		<parameter name="demFiles" type="string" value="merged1.tif"/>
		<!-- True if the DEM is in raster format (GeoTiff) -->
		<parameter name="isRasterType" type="bool" value="true"/>
		<!-- The spacing of queried height profile points of the terrain along the LOS -->
		<parameter name="spacing" type="double" value="14"/>
    </AnalogueModel>
   @endverbatim
 *
 * @author Alexander Brummer
 *
 * @ingroup analogueModels
 */
class NRayGroundInterference : public AnalogueModel
{
public:
	/**
	 * @brief Initializes the analogue model.
	 *
	 * @param carrierFrequency the carrier frequency
	 * @param epsilonR relative permittivity
	 * @param demFiles the filename(s) of the DEM
	 * @param isRasterType whether the DEM is in raster format (GeoTiff)
	 * @param spacing the spacing of queried height profile points of the terrain along the LOS
	 */
	NRayGroundInterference(double carrierFrequency, double epsilonR, std::vector<std::string> demFiles,
			bool isRasterType, double spacing);

	/**
	 * @brief Alternative constructor for cases in which the DEM has already been initialized.
	 *
	 * @param carrierFrequency the carrier frequency
	 * @param epsilonR relative permittivity
	 * @param spacing the spacing of queried height profile points of the terrain along the LOS
	 */
	NRayGroundInterference(double carrierFrequency, double epsilonR, double spacing);

	/**
	 * @brief Filters a specified AirFrame's Signal by adding the resulting attenuation.
	 * @param frame the Airframe in question
	 * @param senderPos the sender's position
	 * @param receiverPos the receiver's position
	 */
	virtual void filterSignal(AirFrame *frame, const Coord& senderPos, const Coord& receiverPos);

	/**
	 * @brief Filters a specified AirFrame's Signal by adding the resulting attenuation (including scaling of epsilonR).
	 * @param frame the Airframe in question
	 * @param senderPos the sender's position
	 * @param receiverPos the receiver's position
	 * @param scaling optional scaling of epsilonR
	 */
	virtual void filterSignal(AirFrame *frame, const Coord& senderPos, const Coord& receiverPos, double scaling);

	/**
	 * @brief calculates the attenuation due to n-ray ground interference.
	 *
	 * @param senderPos the sender's position
	 * @param receiverPos the receiver's position
	 * @param scaling optional scaling of epsilonR
	 * @return value of attenuation in linear units (non-dB)
	 */
	double calcAttenuation(const Coord& senderPos, const Coord& receiverPos, double scaling = 1.0);

protected:
	/** @brief Carrier frequency. */
	double carrierFrequency;

	/** @brief Relative permittivity. */
	double epsilonR;

	/** @brief Spacing of height profile points between sender and receiver. */
	double spacing;
};

#endif /* NRAYGROUNDINTERFERENCE_H_ */
