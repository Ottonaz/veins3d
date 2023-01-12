#ifndef FLOORCONTROL_H_
#define FLOORCONTROL_H_

#include <omnetpp.h>
#include "veins/modules/utility/RTree.h"
#include "veins/base/utils/Coord.h"
#include "veins/modules/floor/FloorSegment.h"
#include "veins/modules/world/annotations/AnnotationManager.h"
#include "veins/modules/mobility/traci/TraCIConnection.h"

#define MY_RTREE_QUAL RTree<FloorSegment*, FloorSegment, double, 3, FloorSegment::Result>

using namespace omnetpp;

/**
 * @brief Models and manages floors as required by the Floor Attenuation model. Its purpose is similar to
 * the ObstacleControl class (which manages Obstacles such as buildings). However, the floors (FloorSegment)
 * are located in three-dimensional space and are stored in an R-tree.
 * For further information about the model, see the FloorAttenuation class.
 *
 * @author Alexander Brummer
 */
class FloorControl : public cSimpleModule
{
public:
	FloorControl();
	virtual void initialize();
	virtual void handleMessage(cMessage *msg);
	virtual void handleSelfMsg(cMessage *msg);
	void addXmlSegments(Veins::TraCIConnection* tc);
	void addLaneFromTypeAndShape(std::string id, std::string typeId, std::vector<Coord> shape, double laneWidth);
	void addJunctionFromTypeAndShape(std::string id, std::string typeId, std::vector<Coord> shape);
	double getFactorByType(std::string type);
	double calculateAttenuation(const Coord& senderPos, const Coord& receiverPos) /*const*/;
	bool groundFloorsBetween(const Coord& senderPos, const Coord& receiverPos);
protected:
	Veins::AnnotationManager* annotations;
	Veins::TraCIConnection* traciConn;
	std::map<std::string, double> attFactors;
	cNormal* randGen;

private:
	void addFromXml(cXMLElement* xml);
	std::vector<std::vector<Coord>> constructSegments(std::vector<Coord> shape, double laneWidth);

	MY_RTREE_QUAL rTree;
};

class FloorControlAccess
{
public:
	FloorControlAccess()
	{
	}

	FloorControl* getIfExists()
	{
		return dynamic_cast<FloorControl*>(getSimulation()->getModuleByPath("floorControl"));
	}
};

#endif
