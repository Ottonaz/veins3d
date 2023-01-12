#ifndef TUNNEL_H_
#define TUNNEL_H_

#include "veins/modules/obstacle/Obstacle.h"
#include "veins/modules/floor/FloorSegment.h"
#include "veins/modules/world/annotations/AnnotationManager.h"

typedef std::vector<Coord> Coords;

/**
 * @brief A simple representation of a tunnel based on its walls, floor and ceiling.
 *
 * @author Alexander Brummer
 */
namespace Veins{
class Tunnel
{
public:

    Tunnel(std::string id, std::string spreadType, std::vector<Coord> shape, double width);
    virtual ~Tunnel();

    typedef std::vector<const FloorSegment*> FloorSegs;
    typedef std::vector<Obstacle*> Obstacles;

    std::string getId() const;

    bool intersectsWith(const Coord& senderPos, const Coord& receiverPos) const;


protected:
    Veins::AnnotationManager* annotations;
    std::string id;
    FloorSegs floorAndCeiling;
    Obstacles walls;

    std::vector<std::vector<Coord>> constructSegments(std::vector<Coord> shape, double width);
};
}

#endif /* TUNNEL_H_ */
