#include <veins/modules/floor/FloorControl.h>
#include <algorithm>

Define_Module(FloorControl);

FloorControl::FloorControl(): rTree(&FloorSegment::addSelf) {

}

void FloorControl::initialize()
{
    annotations = Veins::AnnotationManagerAccess().getIfExists();
    randGen = new cNormal(this->getRNG(0), 0, par("stdDev").doubleValue());
}

void FloorControl::handleMessage(cMessage *msg) {
	if (msg->isSelfMessage()) {
		handleSelfMsg(msg);
		return;
	}
	error("FloorControl doesn't handle messages from other modules");
}

void FloorControl::handleSelfMsg(cMessage *msg) {
	error("FloorControl doesn't handle self-messages");
}

void FloorControl::addLaneFromTypeAndShape(std::string id, std::string typeId, std::vector<Coord> shape, double laneWidth) {
    std::vector<std::vector<Coord>> segments = constructSegments(shape, laneWidth);
    if (segments.empty()) return;

    for (std::vector<Coord> corners: segments) {
        FloorSegment* fs = new FloorSegment(id, typeId, getFactorByType(typeId));
        fs->setShape(corners);
        if (annotations) {
            annotations->drawPolygon(corners, "blue");
        }
        const double cmin[3] = {fs->getBboxP1().x, fs->getBboxP1().y, fs->getBboxP1().z};
        const double cmax[3] = {fs->getBboxP2().x, fs->getBboxP2().y, fs->getBboxP2().z};
        rTree.Insert(cmin, cmax, fs);
    }
}

void FloorControl::addJunctionFromTypeAndShape(std::string id, std::string typeId, std::vector<Coord> shape) {
    if (shape.size() < 3) return;

    FloorSegment* fs = new FloorSegment(id, typeId, getFactorByType(typeId));
    fs->setShape(shape);
    if (annotations) {
        annotations->drawPolygon(shape, "blue");
    }
    const double cmin[3] = {fs->getBboxP1().x, fs->getBboxP1().y, fs->getBboxP1().z};
    const double cmax[3] = {fs->getBboxP2().x, fs->getBboxP2().y, fs->getBboxP2().z};
    rTree.Insert(cmin, cmax, fs);
}

double FloorControl::getFactorByType(std::string type) {
    if (attFactors.find(type) != attFactors.end()) return attFactors[type];
    else if (attFactors.find("general") != attFactors.end()) return attFactors["general"];
    else {
        error("Floor type %s unknown and no general factor defined", type.c_str());
        return -1;
    }
}

void FloorControl::addXmlSegments(Veins::TraCIConnection* tc) {
    traciConn = tc;
    addFromXml(par("floorDescr").xmlValue());
}

void FloorControl::addFromXml(cXMLElement* xml) {
    std::string rootTag = xml->getTagName();
    if (rootTag != "floors") {
        throw cRuntimeError("Floor definition root tag was \"%s\", but expected \"floors\"", rootTag.c_str());
    }

    cXMLElementList list = xml->getChildren();
    for (cXMLElementList::const_iterator i = list.begin(); i != list.end(); ++i) {
        cXMLElement* e = *i;

        std::string tag = e->getTagName();

        if (tag == "type") {
            // <type id="concrete" att-factor="9" />

            ASSERT(e->getAttribute("id"));
            std::string id = e->getAttribute("id");
            ASSERT(e->getAttribute("att-factor"));
            std::string attFactorParStr = e->getAttribute("att-factor");
            double attFactorPar = strtod(attFactorParStr.c_str(), 0);

            attFactors[id] = attFactorPar;
        }
        else if (tag == "floorSeg") {
            // <poly id="bridge7" type="concrete" shape="2,3.5,12 19.1,3.5,12.8 19.1,6.7,12.8, 2,6.7,12" />

            ASSERT(e->getAttribute("id"));
            std::string id = e->getAttribute("id");
            ASSERT(e->getAttribute("type"));
            std::string type = e->getAttribute("type");
            ASSERT(e->getAttribute("shape"));
            std::string shape = e->getAttribute("shape");

            FloorSegment* fs = new FloorSegment(id, type, getFactorByType(type));
            std::vector<Coord> sh;
            cStringTokenizer st(shape.c_str());
            while (st.hasMoreTokens()) {
                std::string xyz = st.nextToken();
                std::vector<double> xyzVec = cStringTokenizer(xyz.c_str(), ",").asDoubleVector();
                ASSERT(xyzVec.size() == 3);
                sh.push_back(traciConn->traci2omnet(Veins::TraCICoord(xyzVec[0], xyzVec[1], xyzVec[2])));
            }
            fs->setShape(sh);
            const double cmin[3] = {fs->getBboxP1().x, fs->getBboxP1().y, fs->getBboxP1().z};
            const double cmax[3] = {fs->getBboxP2().x, fs->getBboxP2().y, fs->getBboxP2().z};
            rTree.Insert(cmin, cmax, fs);
        }
        else {
            throw cRuntimeError("Found unknown tag in floor definition: \"%s\"", tag.c_str());
        }


    }
}

std::vector<std::vector<Coord>> FloorControl::constructSegments(std::vector<Coord> shape, double laneWidth) {
    ASSERT(shape.size() >= 2);
    std::vector<std::vector<Coord>> segments;
    std::vector<Coord> corners;

    Coord vbefore;

    // generation of convex polygons only (achieved by allowing max. 4 vertices)

    // scale lane width (e.g., to match the real width of a bridge
    // alternatively, the desired width could also be hardcoded
    laneWidth*=2;

    for (unsigned int i = 0; i < shape.size() - 1; ++i) {
        Coord vi = shape[i + 1] - shape[i];

        if (corners.empty()) {
            if (vi == Coord::ZERO) continue;
            vi.normalize();

            Coord ni(vi.y, -vi.x, 0);
            ni.normalize();

            corners.push_back(shape[i] + ni*(0.5*laneWidth));
            corners.push_back(shape[i] - ni*(0.5*laneWidth));

            vbefore = vi;
        } else {
            if (vi == Coord::ZERO) continue;
            vi.normalize();

            Coord di = vi - vbefore;

            if (di != Coord::ZERO) {
                Coord vi2d(vi.x, vi.y, 0);
                vi2d.normalize();
                Coord vbefore2d(vbefore.x, vbefore.y, 0);
                vbefore2d.normalize();
                Coord di2d = vi2d - vbefore2d;

                Coord c1;
                Coord c2;

                if (di2d == Coord::ZERO) {
                    di2d.x = vi.y;
                    di2d.y = -vi.x;

                    di2d.normalize();
                    c1 = shape[i] + di2d*(0.5*laneWidth);
                    c2 = shape[i] - di2d*(0.5*laneWidth);
                } else {
                    double angle = atan2(vi.y, vi.x) - atan2(-vbefore.y, -vbefore.x);
                    if (angle < 0) angle += 2 * M_PI;
                    double factor = sin(angle/2);
                    if (angle >= M_PI) factor *= -1;

                    di2d.normalize();
                    c1 = shape[i] + di2d*(0.5*laneWidth/factor);
                    c2 = shape[i] - di2d*(0.5*laneWidth/factor);
                }

                int pos = static_cast<int>(corners.size()/2);
                corners.insert(corners.begin() + pos, c1);
                corners.insert(corners.begin() + pos + 1, c2);

                // push back current polygon and start next one
                segments.push_back(corners);
                corners.clear();
                corners.push_back(c1);
                corners.push_back(c2);

                vbefore = vi;
            }
        }

        if (!corners.empty() && i == shape.size() - 2) {
            if (vi == Coord::ZERO) vi = vbefore;

            Coord ni(vi.y, -vi.x, 0);
            ni.normalize();

            int pos = static_cast<int>(corners.size()/2);
            corners.insert(corners.begin() + pos, shape[i + 1] + ni*(0.5*laneWidth));
            corners.insert(corners.begin() + pos + 1, shape[i + 1] - ni*(0.5*laneWidth));
        }
    }

    if (!corners.empty()) segments.push_back(corners);

    return segments;
}

double FloorControl::calculateAttenuation(const Coord& senderPos, const Coord& receiverPos) /*const*/ {
    // calculate bounding box of transmission
    double minB[] = {std::min(senderPos.x, receiverPos.x), std::min(senderPos.y, receiverPos.y), std::min(senderPos.z, receiverPos.z)};
    double maxB[] = {std::max(senderPos.x, receiverPos.x), std::max(senderPos.y, receiverPos.y), std::max(senderPos.z, receiverPos.z)};

    FloorSegment::Result result;
    if (rTree.Search(minB, maxB, result) == 0) {
        return 1.0;
    }

    std::map<double, const FloorSegment*> intersecs;
    for (const FloorSegment* fs: result.floorSegs) {
        Coord intersection;
        if (fs->intersectsWith(senderPos, receiverPos, intersection)) {
            double dist = senderPos.distance(intersection);
            bool isNew = true;
            for (std::map<double, const FloorSegment*>::iterator it = intersecs.begin(); it != intersecs.end(); ++it) {
                if (dist < (it->first + 1.0) && dist > (it->first - 1.0)) {
                    isNew = false;
                    break;
                }
            }
            if (isNew) intersecs[dist] = fs;
        }
    }

    double dbFactor;

    //    // TODO: consideration of individual intersected floors including types possible
    //    for (const FloorSegment* fs: intersecs) {
    //
    //    }

    // simple case: just consider the number of intersected floors
    // attenuation factor model "914 MHz path loss prediction models for indoor wireless communications in multifloored buildings" (Seidel, Rappaport)
    // factors 1-3 chosen here are based on "5-GHz V2V Channel Characteristics for Parking Garages" (Sun, Matolak, Liu) and averaged from Table III
    // factors 4-6 extrapolated using 4PL (4 parameter logistic regression)
    switch (intersecs.size()) {
        case 0:
            dbFactor = 0.0;
            break;
        case 1:
            dbFactor = -25.8;
            break;
        case 2:
            dbFactor = -37.5;
            break;
        case 3:
            dbFactor = -43.8;
            break;
        case 4:
            dbFactor = -47.7;
            break;
        case 5:
            dbFactor = -50.3;
            break;
        case 6:
            dbFactor = -52.1;
            break;
        default:
            dbFactor = -52.1;
    }


    if (dbFactor != 0.0) dbFactor += randGen->draw();
    return FWMath::dBm2mW(dbFactor);
}

bool FloorControl::groundFloorsBetween(const Coord& senderPos, const Coord& receiverPos)
{
    // calculate bounding box of transmission
    double minB[] = { std::min(senderPos.x, receiverPos.x), std::min(senderPos.y, receiverPos.y), 0.0 };
    double maxB[] = { std::max(senderPos.x, receiverPos.x), std::max(senderPos.y, receiverPos.y), std::max(senderPos.z,
            receiverPos.z) };

    FloorSegment::Result result;
    if (rTree.Search(minB, maxB, result) == 0) {
        return false;
    }

    // check floors for 2D intersections
    for (const FloorSegment* fs : result.floorSegs) {
        if (fs->intersectsWith2D(senderPos, receiverPos))
            return true;
    }

    // check sndPos and rcvPos (are they in 2D shape of floors?)
    for (const FloorSegment* fs : result.floorSegs) {
        if (fs->isPointInObstacle(senderPos) || fs->isPointInObstacle(receiverPos))
            return true;
    }

    return false;
}
