#ifndef TUNNELCONTROL_H_
#define TUNNELCONTROL_H_

#include <omnetpp.h>
#include "veins/base/utils/Coord.h"
#include "veins/modules/tunnel/Tunnel.h"
#include "veins/modules/world/annotations/AnnotationManager.h"
#include "veins/modules/mobility/traci/TraCIConnection.h"


using namespace omnetpp;
using namespace Veins;

/**
 * @brief Manages tunnels. They are represented by their walls, floor and ceiling.
 * Used by the ModelSelector.
 *
 * @author Alexander Brummer
 */
class TunnelControl : public cSimpleModule
{
public:
	TunnelControl();
	virtual void initialize();
	virtual void handleMessage(cMessage *msg);
	virtual void handleSelfMsg(cMessage *msg);
	void addFromNetXml(TraCIConnection* traciConn);
	double calculateAttenuation(const Coord& senderPos, const Coord& receiverPos, std::string sndTunnel, std::string rcvTunnel) /*const*/;

protected:
	Veins::AnnotationManager* annotations;
	std::map<std::string, Tunnel*> tunnels;

};

class TunnelControlAccess
{
public:
    TunnelControlAccess()
	{
	}

	TunnelControl* getIfExists()
	{
		return dynamic_cast<TunnelControl*>(getSimulation()->getModuleByPath("tunnelControl"));
	}
};

#endif
