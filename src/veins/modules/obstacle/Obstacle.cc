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

#include <set>
#include "veins/modules/obstacle/Obstacle.h"

using Veins::Obstacle;

Obstacle::Obstacle(std::string id, std::string type, double attenuationPerCut, double attenuationPerMeter) :
        visualRepresentation(0), id(id), type(type), attenuationPerCut(attenuationPerCut), attenuationPerMeter(
                attenuationPerMeter)
{
}

void Obstacle::setShape(Coords shape)
{
    setShape(shape, 0);
}

void Obstacle::setShape(Coords shape, double height )
{
    coords = shape;
    this-> height = height;
    mesh = {};
    bboxP1 = Coord(1e7, 1e7);
    bboxP2 = Coord(-1e7, -1e7);
    Coords::const_iterator i = coords.begin();
    Coords::const_iterator j = i + 1;
    for (; i != coords.end(); j = ++i + 1) {
        bboxP1.x = std::min(i->x, bboxP1.x);
        bboxP1.y = std::min(i->y, bboxP1.y);
        bboxP2.x = std::max(i->x, bboxP2.x);
        bboxP2.y = std::max(i->y, bboxP2.y);

        if(height > 0 && j != coords.end())
        {
            // generating mesh for the walls
            mesh.push_back({ Coord(i->x,i->y,0), Coord(i->x,i->y,height), Coord(j->x,j->y,height)});
            mesh.push_back({ Coord(j->x,j->y,0), Coord(j->x,j->y,height), Coord(i->x,i->y,0)});

        }
    }
    if(height > 0)
    {
        Coords shapeWithoutLast(shape.begin(), shape.end() - 1);
        for(Coords monotonePolygon : partitionPolygonIntoMonotone(shapeWithoutLast))
        {
            for(Triangle triangle : triangulateMonotonePolygon(monotonePolygon))
            {
                triangle.p1.z = 0;triangle.p2.z = 0;triangle.p3.z = 0;
            mesh.push_back(triangle);
            triangle.p1.z = height;triangle.p2.z = height;triangle.p3.z = height;
            mesh.push_back(triangle);
            }
        }
    }
}

const Obstacle::Coords& Obstacle::getShape() const
{
    return coords;
}

const Obstacle::Mesh& Obstacle::getMesh() const
{
    return mesh;
}

const double Obstacle::getHeight() const
{
    return height;
}

const Coord Obstacle::getBboxP1() const
{
    return bboxP1;
}

const Coord Obstacle::getBboxP2() const
{
    return bboxP2;
}

namespace {

bool isPointInObstacle(Coord point, const Obstacle& o)
{
    bool isInside = false;
    if(point.z < 0 || (point.z > o.getHeight() && o.getHeight() > 0)) return false;
    const Obstacle::Coords& shape = o.getShape();
    Obstacle::Coords::const_iterator i = shape.begin();
    Obstacle::Coords::const_iterator j = (shape.rbegin() + 1).base();
    for (; i != shape.end(); j = i++) {
        bool inYRangeUp = (point.y >= i->y) && (point.y < j->y);
        bool inYRangeDown = (point.y >= j->y) && (point.y < i->y);
        bool inYRange = inYRangeUp || inYRangeDown;
        if (!inYRange)
            continue;
        bool intersects = point.x < (i->x + ((point.y - i->y) * (j->x - i->x) / (j->y - i->y)));
        if (!intersects)
            continue;
        isInside = !isInside;
    }
    return isInside;
}

double segmentsIntersectSegment(const Coord& p1From, const Coord& p1To, const Coord& p2From, const Coord& p2To)
{
    Coord p1Vec = p1To - p1From;
    Coord p2Vec = p2To - p2From;
    Coord p1p2 = p1From - p2From;

    double D = (p1Vec.x * p2Vec.y - p1Vec.y * p2Vec.x);

    double p1Frac = (p2Vec.x * p1p2.y - p2Vec.y * p1p2.x) / D;
    if (p1Frac < 0 || p1Frac > 1)
        return -1;

    double p2Frac = (p1Vec.x * p1p2.y - p1Vec.y * p1p2.x) / D;
    if (p2Frac < 0 || p2Frac > 1)
        return -1;

    return p1Frac;
}

double segmentsIntersectTriangle(const Veins::Segment& segment, const Veins::Triangle& triangle) 
{
        Coord edge1, edge2, segmentVector, h, s, q;
    double a, f, u, v, t;

    edge1.x = triangle.p2.x - triangle.p1.x;
    edge1.y = triangle.p2.y - triangle.p1.y;
    edge1.z = triangle.p2.z - triangle.p1.z;
    edge2.x = triangle.p3.x - triangle.p1.x;
    edge2.y = triangle.p3.y - triangle.p1.y;
    edge2.z = triangle.p3.z - triangle.p1.z;
    segmentVector.x = segment.p2.x - segment.p1.x;
    segmentVector.y = segment.p2.y - segment.p1.y;
    segmentVector.z = segment.p2.z - segment.p1.z;

    h = segmentVector.crossProduct(edge2);
    a = edge1.dotProduct(h);

    if (a > -0.00001 && a < 0.00001) {
        return -1;
    }

    f = 1 / a;
    s.x = segment.p1.x - triangle.p1.x;
    s.y = segment.p1.y - triangle.p1.y;
    s.z = segment.p1.z - triangle.p1.z;

    u = f * s.dotProduct(h);

    if (u < 0 || u > 1) {
        return -1;
    }

    q = s.crossProduct(edge1);
    v = f * segmentVector.dotProduct(q);

    if (v < 0 || u + v > 1) {
        return -1;
    }

    t = f * edge2.dotProduct(q);

    if (t > 0 && t < 1) {
        return t;
    }

    return -1;
}


}

double Obstacle::calculateAttenuation(const Coord& senderPos, const Coord& receiverPos, double *totalCuts, double *totalFractionInObstacle ) const
{

    // if obstacles has neither borders nor matter: bail.
    if (getShape().size() < 2)
        return 1;

    // if sender is directly above receiver or vice versa: bail
    Coord srVec = receiverPos - senderPos;
    if (!FWMath::close(srVec.z, 0.0) && FWMath::close(srVec.x, 0.0) && FWMath::close(srVec.y, 0.0))
        return 1;

    // get a list of points (in [0, 1]) along the line between sender and receiver where the beam intersects with this obstacle
    std::multiset<double> intersectAt;
    bool doesIntersect = false;
    const Obstacle::Coords& shape = getShape();
    if(getHeight() <= 0 )
    {
        Obstacle::Coords::const_iterator i = shape.begin();
        Obstacle::Coords::const_iterator j = (shape.rbegin() + 1).base();
        for (; i != shape.end(); j = i++) {
            Coord c1 = *i;
            Coord c2 = *j;

            double i = segmentsIntersectSegment(senderPos, receiverPos, c1, c2);
            if (i != -1) {
                doesIntersect = true;
                intersectAt.insert(i);
            }

            if (shape.size() == 2) // it's just a wall, so only one segment to check
                break;
        }
    }
    else
    {
        const Obstacle::Mesh& mesh = getMesh();
        Obstacle::Mesh::const_iterator i = mesh.begin();
        for (; i != mesh.end(); i++) {

            double res = segmentsIntersectTriangle({senderPos, receiverPos}, *i);
            if (res != -1) {
                intersectAt.insert(res);
                doesIntersect = true;
            }
        }
    }

    // if beam interacts with neither borders nor matter: bail.
    bool senderInside = isPointInObstacle(senderPos, *this);
    bool receiverInside = isPointInObstacle(receiverPos, *this);
    if (!doesIntersect && !senderInside && !receiverInside)
        return 1;

    // remember number of cuts before messing with intersection points
    double numCuts = intersectAt.size();
    *totalCuts += numCuts;

    // if obstacle is not just a wall, consider distance in obstacle
    double fractionInObstacle = 0;
    if (shape.size() > 2) {
        // for distance calculation, make sure every other pair of points marks transition through matter and void, respectively.
        if (senderInside)
            intersectAt.insert(0);
        if (receiverInside)
            intersectAt.insert(1);
        
        //ASSERT((intersectAt.size() % 2) == 0);
        if((intersectAt.size() % 2) == 0){           // temporary solution

            // sum up distances in matter.
            for (std::multiset<double>::const_iterator i = intersectAt.begin(); i != intersectAt.end();) {
                double p1 = *(i++);
                double p2 = *(i++);
                fractionInObstacle += (p2 - p1);
            }
            *totalFractionInObstacle += fractionInObstacle;
        }
    }

    // calculate attenuation
    double totalDistance = senderPos.distance(receiverPos);
    double attenuation = (attenuationPerCut * numCuts) + (attenuationPerMeter * fractionInObstacle * totalDistance);
    return pow(10.0, -attenuation / 10.0);
}

std::string Obstacle::getType() const
{
    return type;
}

std::string Obstacle::getId() const
{
    return id;
}
