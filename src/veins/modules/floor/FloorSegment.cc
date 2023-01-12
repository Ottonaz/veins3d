#include <veins/modules/floor/FloorSegment.h>

FloorSegment::FloorSegment(std::string id, std::string type, double factor) :
id(id),
type(type),
factor(factor) {

}

FloorSegment::~FloorSegment() {

}

void FloorSegment::setShape(Coords shape) {
    corners = shape;
    bboxP1 = Coord(1e7, 1e7, 1e7);
    bboxP2 = Coord(-1e7, -1e7, -1e7);
    for (Coords::const_iterator i = corners.begin(); i != corners.end(); ++i) {
        bboxP1.x = std::min(i->x, bboxP1.x);
        bboxP1.y = std::min(i->y, bboxP1.y);
        bboxP1.z = std::min(i->z, bboxP1.z);
        bboxP2.x = std::max(i->x, bboxP2.x);
        bboxP2.y = std::max(i->y, bboxP2.y);
        bboxP2.z = std::max(i->z, bboxP2.z);
    }
}

std::string FloorSegment::getId() const {
    return id;
}

std::string FloorSegment::getType() const {
    return type;
}

const Coords& FloorSegment::getShape() const {
    return corners;
}

const Coord FloorSegment::getBboxP1() const {
    return bboxP1;
}

const Coord FloorSegment::getBboxP2() const {
    return bboxP2;
}


bool FloorSegment::intersectsWith(const Coord& senderPos, const Coord& receiverPos, Coord& intersection) const {
    Coord e1 = corners[1] - corners[0];
    Coord e2 = corners.back() - corners[0];
    Coord n = e1*e2; // cross product
    // plane equation: n*P + d = 0
    double d = -n.x*corners[0].x - n.y*corners[0].y - n.z*corners[0].z; // get d of plane equation by plugging in point

    Coord direction = receiverPos - senderPos; // line equation = senderPos + l*direction

    // set plane equation = line equation and solve for l
    // l = -(n*senderPos + d)/(n*direction)

    double denom = n.x*direction.x + n.y*direction.y + n.z*direction.z;
    if (denom == 0) return false; // plane and line are parallel

    double l = -(n.x*senderPos.x + n.y*senderPos.y + n.z*senderPos.z + d)/denom;
    if (l < 0 || l > 1) return false; // segment (i.e. LOS between sender and receiver) does not intersect plane

    intersection = senderPos + direction*l;

    // check if intersection point is inside floor segment
    // 2D (X,Y) projection is sufficient
    // thus, check if point is on the same side of all polygon edges (only valid for convex polygons)
    int prevSign = 0;
    for (unsigned int i = 0; i < corners.size(); ++i) {
        unsigned int first = (i + 1)%corners.size();
        double res = (corners[first].x - corners[i].x)*(intersection.y - corners[i].y) - (corners[first].y - corners[i].y)*(intersection.x - corners[i].x);
        int sign = (res > 0) - (res < 0); // signum
        if ((prevSign > 0 && sign < 0) || (prevSign < 0 && sign > 0)) return false; // point on different sides of edges, thus cannot be inside polygon
        if (sign != 0) prevSign = sign;
    }

    return true;
}

void FloorSegment::addSelf(const Result& queryResult) const {
    queryResult.floorSegs.push_back(this);
}


bool FloorSegment::intersectsWith2D(const Coord& senderPos, const Coord& receiverPos) const
{
    Coord v_sr = receiverPos - senderPos;
    double d2d = sqrt(v_sr.x * v_sr.x + v_sr.y * v_sr.y);

    Coords::const_iterator i = corners.begin();
    Coords::const_iterator j = (corners.rbegin() + 1).base();
    bool intersects = false;
    for (; i != corners.end(); j = i++) {
        Coord c1 = *i;
        Coord c2 = *j;

        double relDist = segmentsIntersectAt(senderPos, receiverPos, c1, c2);
        if (relDist != -1) {
            double relIntersection = segmentsIntersectAt(c1, c2, senderPos, receiverPos);
            double segmentZ = c1.z + (c2.z - c1.z)*relIntersection;
            double losZ = senderPos.z + (receiverPos.z - senderPos.z)*relDist;
            // if segment is above LOS, there can be no GROUND reflection
            if (segmentZ > losZ)
                return false;
            Coord intersecProfile(d2d * relDist, segmentZ);
            intersects = true;
        }
    }

    return intersects;
}

bool FloorSegment::isPointInObstacle(Coord point) const
{
    bool isInside = false;
    Coords::const_iterator i = corners.begin();
    Coords::const_iterator j = (corners.rbegin() + 1).base();
    for (; i != corners.end(); j = i++) {
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

double FloorSegment::segmentsIntersectAt(Coord p1From, Coord p1To, Coord p2From, Coord p2To) const
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
