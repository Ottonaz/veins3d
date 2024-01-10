//
// ObstacleControl - models obstacles that block radio transmissions
// Copyright (C) 2010 Christoph Sommer <christoph.sommer@informatik.uni-erlangen.de>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//

#include <sstream>
#include <map>
#include <set>

#include "veins/modules/obstacle/ObstacleControl.h"

using Veins::ObstacleControl;

Define_Module(Veins::ObstacleControl);

ObstacleControl::~ObstacleControl()
{

}

void ObstacleControl::initialize(int stage)
{
    if (stage == 1) {
        debug = par("debug");

        obstacles.clear();
        innerWalls.clear();
        cacheEntries.clear();

        annotations = AnnotationManagerAccess().getIfExists();
        if (annotations)
            annotationGroup = annotations->createGroup("obstacles");

        obstaclesXml = par("obstacles");

        addFromXml(obstaclesXml);
    }
}

void ObstacleControl::finish()
{
    for (Obstacles::iterator i = obstacles.begin(); i != obstacles.end(); ++i) {
        for (ObstacleGridRow::iterator j = i->begin(); j != i->end(); ++j) {
            while (j->begin() != j->end())
                erase(*j->begin());
        }
    }
    obstacles.clear();

    for (Obstacles::iterator i = innerWalls.begin(); i != innerWalls.end(); ++i) {
        for (ObstacleGridRow::iterator j = i->begin(); j != i->end(); ++j) {
            while (j->begin() != j->end())
                erase(*j->begin(), true);
        }
    }
    innerWalls.clear();
}

void ObstacleControl::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        handleSelfMsg(msg);
        return;
    }
    error("ObstacleControl doesn't handle messages from other modules");
}

void ObstacleControl::handleSelfMsg(cMessage *msg)
{
    error("ObstacleControl doesn't handle self-messages");
}

void ObstacleControl::addFromXml(cXMLElement* xml)
{
    std::string rootTag = xml->getTagName();
    if (rootTag != "obstacles") {
        throw cRuntimeError("Obstacle definition root tag was \"%s\", but expected \"obstacles\"", rootTag.c_str());
    }

    cXMLElementList list = xml->getChildren();
    for (cXMLElementList::const_iterator i = list.begin(); i != list.end(); ++i) {
        cXMLElement* e = *i;

        std::string tag = e->getTagName();

        if (tag == "type") {
            // <type id="building" db-per-cut="9" db-per-meter="0.4" />

            ASSERT(e->getAttribute("id"));
            std::string id = e->getAttribute("id");
            ASSERT(e->getAttribute("db-per-cut"));
            std::string perCutParS = e->getAttribute("db-per-cut");
            double perCutPar = strtod(perCutParS.c_str(), 0);
            ASSERT(e->getAttribute("db-per-meter"));
            std::string perMeterParS = e->getAttribute("db-per-meter");
            double perMeterPar = strtod(perMeterParS.c_str(), 0);

            perCut[id] = perCutPar;
            perMeter[id] = perMeterPar;

        }
        else if (tag == "poly") {

            // <poly id="building#0" type="building" color="#F00" shape="16,0 8,13.8564 -8,13.8564 -16,0 -8,-13.8564 8,-13.8564" />
            ASSERT(e->getAttribute("id"));
            std::string id = e->getAttribute("id");
            ASSERT(e->getAttribute("type"));
            std::string type = e->getAttribute("type");
            ASSERT(e->getAttribute("color"));
            std::string color = e->getAttribute("color");
            ASSERT(e->getAttribute("shape"));
            std::string shape = e->getAttribute("shape");

            double height = e->getAttribute("height") ?  std::stof(e->getAttribute("height")) : 0;

            Obstacle obs(id, type, getAttenuationPerCut(type), getAttenuationPerMeter(type));
            std::vector<Coord> sh;
            cStringTokenizer st(shape.c_str());
            while (st.hasMoreTokens()) {
                std::string xy = st.nextToken();
                std::vector<double> xya = cStringTokenizer(xy.c_str(), ",").asDoubleVector();
                ASSERT(xya.size() == 2);
                sh.push_back(Coord(xya[0], xya[1]));
            }
            obs.setShape(sh,height);
            if (type == "innerWall") {
                add(obs, true);
            }
            else {
                add(obs);
            }
        }
        else {
            throw cRuntimeError("Found unknown tag in obstacle definition: \"%s\"", tag.c_str());
        }

    }

}

void ObstacleControl::addFromTypeAndShape(std::string id, std::string typeId, std::vector<Coord> shape, double height)
{
    if (!isTypeSupported(typeId)) {
        throw cRuntimeError("Unsupported obstacle type: \"%s\"", typeId.c_str());
    }
    Obstacle obs(id, typeId, getAttenuationPerCut(typeId), getAttenuationPerMeter(typeId));
    obs.setShape(shape);
    
    if(par("enable3d"))
        obs.setShape(shape, height);
    else
        obs.setShape(shape, 0);

    if (typeId == "innerWall") {
        add(obs, true);
    }
    else {
        add(obs);
    }
}

void ObstacleControl::add(Obstacle obstacle, bool inner)
{
    Obstacle* o = new Obstacle(obstacle);

    size_t fromRow = std::max(0, int(o->getBboxP1().x / GRIDCELL_SIZE));
    size_t toRow = std::max(0, int(o->getBboxP2().x / GRIDCELL_SIZE));
    size_t fromCol = std::max(0, int(o->getBboxP1().y / GRIDCELL_SIZE));
    size_t toCol = std::max(0, int(o->getBboxP2().y / GRIDCELL_SIZE));

    Obstacles& obstCollection = inner ? innerWalls : obstacles;

    for (size_t row = fromRow; row <= toRow; ++row) {
        for (size_t col = fromCol; col <= toCol; ++col) {
            if (obstCollection.size() < col + 1)
                obstCollection.resize(col + 1);
            if (obstCollection[col].size() < row + 1)
                obstCollection[col].resize(row + 1);
            (obstCollection[col])[row].push_back(o);
        }
    }

    // visualize using AnnotationManager
    if (annotations)
        o->visualRepresentation = annotations->drawPolygon(o->getShape(), "red", annotationGroup);

    // clear the cache in case of a "real" obstacle (inner walls are not cached)
    if (!inner)
        cacheEntries.clear();
}

void ObstacleControl::erase(const Obstacle* obstacle, bool inner)
{
    Obstacles& obstCollection = inner ? innerWalls : obstacles;

    for (Obstacles::iterator i = obstCollection.begin(); i != obstCollection.end(); ++i) {
        for (ObstacleGridRow::iterator j = i->begin(); j != i->end(); ++j) {
            for (ObstacleGridCell::iterator k = j->begin(); k != j->end();) {
                Obstacle* o = *k;
                if (o == obstacle) {
                    k = j->erase(k);
                }
                else {
                    ++k;
                }
            }
        }
    }

    if (annotations && obstacle->visualRepresentation)
        annotations->erase(obstacle->visualRepresentation);
    delete obstacle;

    // clear the cache in case of a "real" obstacle (inner walls are not cached)
    if (!inner)
        cacheEntries.clear();
}

SignalStats ObstacleControl::calculateAttenuation(const Coord& senderPos, const Coord& receiverPos, bool inner) /*const*/
{
    Enter_Method_Silent();

    if ((perCut.size() == 0) || (perMeter.size() == 0)) {
        throw cRuntimeError("Unable to use SimpleObstacleShadowing: No obstacle types have been configured");
    }
    if (obstacles.size() == 0 && innerWalls.size() == 0) {
        throw cRuntimeError("Unable to use SimpleObstacleShadowing: No obstacles have been added");
    }

    // return cached result, if available
    CacheKey cacheKey(senderPos, receiverPos);
    if (!inner) {
        CacheEntries::const_iterator cacheEntryIter = cacheEntries.find(cacheKey);
        if (cacheEntryIter != cacheEntries.end()) {
            //return *cacheEntryIter;
        }
    }

    // calculate bounding box of transmission
    Coord bboxP1 = Coord(std::min(senderPos.x, receiverPos.x), std::min(senderPos.y, receiverPos.y));
    Coord bboxP2 = Coord(std::max(senderPos.x, receiverPos.x), std::max(senderPos.y, receiverPos.y));

    size_t fromRow = std::max(0, int(bboxP1.x / GRIDCELL_SIZE));
    size_t toRow = std::max(0, int(bboxP2.x / GRIDCELL_SIZE));
    size_t fromCol = std::max(0, int(bboxP1.y / GRIDCELL_SIZE));
    size_t toCol = std::max(0, int(bboxP2.y / GRIDCELL_SIZE));

    Obstacles& obstCollection = inner ? innerWalls : obstacles;
    std::set<Obstacle*> processedObstacles;
    double totalDistance = senderPos.distance(receiverPos);
    double factor = 1;
    double totalCuts = 0;
    double totalFractionInObstacle = 0;
    for (size_t col = fromCol; col <= toCol; ++col) {
        if (col >= obstCollection.size()) break;
        for (size_t row = fromRow; row <= toRow; ++row) {
            if (row >= obstCollection[col].size()) break;
            const ObstacleGridCell& cell = (obstCollection[col])[row];
            for (ObstacleGridCell::const_iterator k = cell.begin(); k != cell.end(); ++k) {

                Obstacle* o = *k;

                if (processedObstacles.find(o) != processedObstacles.end()) continue;
                processedObstacles.insert(o);

                // bail if bounding boxes cannot overlap
                if (o->getBboxP2().x < bboxP1.x) continue;
                if (o->getBboxP1().x > bboxP2.x) continue;
                if (o->getBboxP2().y < bboxP1.y) continue;
                if (o->getBboxP1().y > bboxP2.y) continue;

                double factorOld = factor;

                factor *= o->calculateAttenuation(senderPos, receiverPos, &totalCuts, &totalFractionInObstacle);

                // draw a "hit!" bubble
                if (annotations && (factor != factorOld)) annotations->drawBubble(o->getBboxP1(), "hit");

                // bail if attenuation is already extremely high
                if (factor < 1e-30) break;

            }
        }
    }
    SignalStats ss = SignalStats(totalCuts,totalDistance,totalFractionInObstacle,factor);

    // cache result
    if (!inner) {
        if (cacheEntries.size() >= 1000) cacheEntries.clear();
        //cacheEntries[cacheKey] = ss;
    }

    return ss;
}

double ObstacleControl::getAttenuationPerCut(std::string type)
{
    if (perCut.find(type) != perCut.end())
        return perCut[type];
    else {
        error("Obstacle type %s unknown", type.c_str());
        return -1;
    }
}

double ObstacleControl::getAttenuationPerMeter(std::string type)
{
    if (perMeter.find(type) != perMeter.end())
        return perMeter[type];
    else {
        error("Obstacle type %s unknown", type.c_str());
        return -1;
    }
}

bool ObstacleControl::isTypeSupported(std::string type)
{
    //the type of obstacle is supported if there are attenuation values for borders and interior
    return (perCut.find(type) != perCut.end()) && (perMeter.find(type) != perMeter.end());
}
