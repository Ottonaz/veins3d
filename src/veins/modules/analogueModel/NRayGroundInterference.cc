#include <veins/modules/analogueModel/NRayGroundInterference.h>
#include "veins/modules/utility/NBHeightMapper.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

using Veins::AirFrame;
using Veins::TraCIScenarioManager;
using Veins::TraCICommandInterface;

NRayGroundInterference::NRayGroundInterference(double carrierFrequency, double epsilonR, std::vector<std::string> demFiles,
        bool isRasterType, double spacing)
{
	this->carrierFrequency = carrierFrequency;
	this->epsilonR = epsilonR;
	this->spacing = spacing;

	const NBHeightMapper& hm = NBHeightMapper::get();
	if (!hm.ready()) NBHeightMapper::loadHM(demFiles, isRasterType);
}

NRayGroundInterference::NRayGroundInterference(double carrierFrequency, double epsilonR, double spacing)
{
	this->carrierFrequency = carrierFrequency;
	this->epsilonR = epsilonR;
	this->spacing = spacing;

	const NBHeightMapper& hm = NBHeightMapper::get();
	if (!hm.ready()) throw cRuntimeError("No height map for n-ray model");
}

void NRayGroundInterference::filterSignal(AirFrame *frame, const Coord& senderPos, const Coord& receiverPos) {
    filterSignal(frame, senderPos, receiverPos, 1.0);
}

void NRayGroundInterference::filterSignal(AirFrame *frame, const Coord& senderPos, const Coord& receiverPos, double scaling) {
	Signal& s = frame->getSignal();

	double factor = calcAttenuation(senderPos, receiverPos, scaling);
	EV << "Attenuation by NRayGroundInterference is: " << factor << endl;

	bool hasFrequency = s.getTransmissionPower()->getDimensionSet().hasDimension(Dimension::frequency());
	const DimensionSet& domain = hasFrequency ? DimensionSet::timeFreqDomain() : DimensionSet::timeDomain();
	ConstantSimpleConstMapping* attMapping = new ConstantSimpleConstMapping(domain, factor);
	s.addAttenuation(attMapping);
}

double NRayGroundInterference::calcAttenuation(const Coord& senderPos, const Coord& receiverPos, double scaling) {
	double scaledEpsilonR = 1.0 + (epsilonR - 1.0) * scaling;
	TraCIScenarioManager* traciManager = FindModule<TraCIScenarioManager*>::findGlobalModule();
	TraCICommandInterface* traciCI = traciManager->getCommandInterface();
	const NBHeightMapper& hm = NBHeightMapper::get();
	if (!hm.ready()) throw cRuntimeError("No height map for n-ray ground interference model");
	std::vector<Coord> profile;
	// get 2D positions and horizontal distance
	Coord snd2D(senderPos.x, senderPos.y);
	Coord rcv2D(receiverPos.x, receiverPos.y);
	Coord vHor = rcv2D - snd2D;
	double dHor = vHor.length();
	vHor = vHor/dHor;

	// sender and receiver positions in the profile view
	Coord sndProfile(0.0, senderPos.z);
	Coord rcvProfile(dHor, receiverPos.z);

	// compute first and last profile point
	double lon, lat;
	std::tie(lon, lat) = traciCI->getLonLat(snd2D);
	profile.push_back(Coord(0.0, hm.getZ(Position(lon, lat))));

	// compute remaining profile points with given spacing in between
	for (double d = spacing; d < dHor; d += spacing) {
		std::tie(lon, lat) = traciCI->getLonLat(snd2D + vHor*d);
		profile.push_back(Coord(d, hm.getZ(Position(lon, lat))));
//		profile.push_back(Coord(d, 0.0));
	}
	std::tie(lon, lat) = traciCI->getLonLat(rcv2D);
	profile.push_back(Coord(dHor, hm.getZ(Position(lon, lat))));

	// compute length of line of sight
	double dLOS = sqrt(pow(dHor, 2) + pow(senderPos.z - receiverPos.z, 2));
	double lambda = BaseWorldUtility::speedOfLight()/carrierFrequency;

	// initialize the sums of real and imaginary part of the resulting phasor with the LOS ray
	double realSum = 1/dLOS;
	double imagSum = 0;

	int numRefl = 0;

	// check each segment of the profile for potential reflection
	Coord prev = profile[0];
	for (int i = 1; i < profile.size(); ++i) {
		const Coord& p1 = prev;
		const Coord& p2 = profile[i];
		Coord seg = p2 - p1;
		//std::cout << atan2(seg.y, seg.x) << std::endl;
		// vector from beginning of segment to sender/receiver position
		Coord p1S = sndProfile - p1;
		Coord p1R = rcvProfile - p1;

		// compute projections of sender/receiver on the line spanned by the segment
		Coord sndOnSeg = p1 + seg*(p1S.dot(seg)/seg.dot(seg));
		Coord rcvOnSeg = p1 + seg*(p1R.dot(seg)/seg.dot(seg));
		Coord dOnSeg = rcvOnSeg - sndOnSeg;

		// check if point of reflection is actually lying on the segment
		Coord hT = sndProfile - sndOnSeg;
		Coord hR = rcvProfile - rcvOnSeg;
		Coord reflPoint = sndOnSeg + (dOnSeg)*(hT.length()/(hT.length() + hR.length()));
		Coord p1ToRefl = reflPoint - p1;
		double dotProduct = seg.dot(p1ToRefl);
		// if dot product is not in [0; 1], reflection point is not on the segment, so continue with next segment
		if (dotProduct < 0 || p1ToRefl.length() > seg.length()) {
			prev = p2;
			continue;
		}

		// check whether reflected ray hits the ground (if so, continue as there is no reflection)
		Coord sRefl = reflPoint - sndProfile;
		Coord reflR = rcvProfile - reflPoint;
		bool cont = false;
		for (int j = 1; j < profile.size() - 1; ++j) {
			const Coord& profilePoint = profile[j];
			double dist = profilePoint.x;

			double check;
			if (dist < reflPoint.x)
				check = sndProfile.y + (dist/sRefl.x)*sRefl.y;
			else if (dist > reflPoint.x)
				check = reflPoint.y + ((dist - reflPoint.x)/(rcvProfile.x - reflPoint.x))*reflR.y;

			if (check <= profilePoint.y) {
				cont = true;
				break;
			}
		}
		if (cont)
			continue;

		// finally compute the parameters of this reflected ray
		double dRef = sqrt(pow(dOnSeg.length(), 2) + pow(hT.length() + hR.length(), 2));
		double deltaPhi = 2*M_PI*(dRef - dLOS)/lambda;
		double sinTheta = (hT.length() + hR.length())/dRef;
		double cosTheta = dOnSeg.length()/dRef;
		double gamma = (sinTheta - sqrt(scaledEpsilonR - pow(cosTheta, 2)))/(sinTheta + sqrt(scaledEpsilonR - pow(cosTheta, 2)));

		// add the real and imaginary part of the phasor of this ray to the sums
		realSum += gamma*cos(deltaPhi)/dRef;
		imagSum += gamma*sin(deltaPhi)/dRef;

		prev = p2;
		++numRefl;
	}

	double att = pow(lambda/(4*M_PI), 2)*(pow(realSum, 2) + pow(imagSum, 2));

	return att;
}
