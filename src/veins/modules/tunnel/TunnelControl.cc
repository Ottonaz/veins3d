#include <veins/modules/tunnel/TunnelControl.h>
#include <algorithm>

Define_Module(TunnelControl);

TunnelControl::TunnelControl() {

}

void TunnelControl::initialize()
{
    annotations = Veins::AnnotationManagerAccess().getIfExists();
}

void TunnelControl::handleMessage(cMessage *msg) {
	if (msg->isSelfMessage()) {
		handleSelfMsg(msg);
		return;
	}
	error("TunnelControl doesn't handle messages from other modules");
}

void TunnelControl::handleSelfMsg(cMessage *msg) {
	error("TunnelControl doesn't handle self-messages");
}

void TunnelControl::addFromNetXml(TraCIConnection* traciConn) {
    cXMLElement* netXml = par("netXml").xmlValue();
    cXMLElementList edges = netXml->getElementsByTagName("edge");
    for (const auto edge : edges) {
        cXMLElement* tunnelParam = edge->getFirstChildWithAttribute("param", "key", "tunnel");
        if (!tunnelParam)
            continue;
        std::string tunnelId = tunnelParam->getAttribute("value");
        if (tunnels.count(tunnelId) != 0)
            continue;


        std::vector<Coord> sh;

        // add from node
        cXMLElement* from = netXml->getElementById(edge->getAttribute("from"));
        double fromX = std::stod(from->getAttribute("x"));
        double fromY = std::stod(from->getAttribute("y"));
        double fromZ = 0.0;
        const char* fromZc = from->getAttribute("z");
        if (fromZc)
            fromZ = std::stod(fromZc);
        sh.push_back(traciConn->traci2omnet(Veins::TraCICoord(fromX, fromY, fromZ)));

        // add shape if given
        const char* shape = edge->getAttribute("shape");
        if (shape != nullptr) {
            cStringTokenizer tok(shape);
            while (tok.hasMoreTokens()) {
                std::string xyz = tok.nextToken();
                std::vector<double> xyzVec = cStringTokenizer(xyz.c_str(), ",").asDoubleVector();
                sh.push_back(
                        traciConn->traci2omnet(
                                Veins::TraCICoord(xyzVec[0], xyzVec[1], xyzVec.size() == 3 ? xyzVec[2] : 0.0)));
            }
        }

        // add to node
        cXMLElement* to = netXml->getElementById(edge->getAttribute("to"));
        double toX = std::stod(to->getAttribute("x"));
        double toY = std::stod(to->getAttribute("y"));
        double toZ = 0.0;
        const char* toZc = to->getAttribute("z");
        if (toZc)
            toZ = std::stod(toZc);
        sh.push_back(traciConn->traci2omnet(Veins::TraCICoord(toX, toY, toZ)));

        // check for and remove duplicates at front and back
        if (sh[0] == sh[1])
            sh.erase(sh.begin());
        if (sh.back() == sh[sh.size() - 2])
            sh.pop_back();

        const char* spreadType = edge->getAttribute("spreadType");
        if (!spreadType) spreadType = "right";

        double width = 0.0;
        cXMLElementList lanes = edge->getChildrenByTagName("lane");
        for (const auto l : lanes) {
            const char* widthC = l->getAttribute("width");
            if (widthC)
                width += std::stod(widthC);
            else
                width += 3.2;
        }

        tunnels[tunnelId] = new Tunnel(tunnelId, spreadType, sh, width);
    }

}

double TunnelControl::calculateAttenuation(const Coord& senderPos, const Coord& receiverPos, std::string sndTunnel, std::string rcvTunnel) /*const*/ {
    if (sndTunnel != "" && tunnels[sndTunnel]->intersectsWith(senderPos, receiverPos))
        return 0.0;
    if (rcvTunnel != "" && tunnels[rcvTunnel]->intersectsWith(senderPos, receiverPos))
        return 0.0;

    return 1.0;
}
