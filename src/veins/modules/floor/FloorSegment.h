#ifndef FLOORSEGMENT_H_
#define FLOORSEGMENT_H_

#include "veins/base/utils/Coord.h"

typedef std::vector<Coord> Coords;

/**
 * @brief Represents (part of) a floor, required for the FloorAttenuation model. All segments are managed
 * by the FloorControl class. For simpler calculations, the represented polygon must not have more than
 * four corners (to make sure that the polygon is convex).
 *
 * @author Alexander Brummer
 */
class FloorSegment
{
public:

    FloorSegment(std::string id, std::string type, double factor);
    virtual ~FloorSegment();

    void setShape(Coords shape);

    std::string getType() const;
    std::string getId() const;

    const Coords& getShape() const;
    const Coord getBboxP1() const;
    const Coord getBboxP2() const;

    bool intersectsWith(const Coord& senderPos, const Coord& receiverPos, Coord& intersection) const;
    bool intersectsWith2D(const Coord& senderPos, const Coord& receiverPos) const;
    bool isPointInObstacle(Coord point) const;

    typedef std::vector<const FloorSegment*> FloorSegs;

    /**
     * @brief class for circumventing the const-restriction of RTree::Search-context
     */
    class Result {
    public:
        Result() {};
        ~Result() {};
        // @brief method not really const
        void add(FloorSegment* floorSeg) const {
            floorSegs.push_back(floorSeg);
        };
        mutable FloorSegs floorSegs;
    };

    void addSelf(const Result& queryResult) const;

protected:
    std::string id;
    std::string type;
    double factor;
    Coords corners;
    Coord bboxP1;
    Coord bboxP2;

    double segmentsIntersectAt(Coord p1From, Coord p1To, Coord p2From, Coord p2To) const;
};

#endif /* FLOORSEGMENT_H_ */
