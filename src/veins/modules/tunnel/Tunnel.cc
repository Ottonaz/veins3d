#include <veins/modules/tunnel/Tunnel.h>
#include <limits>

using Veins::Tunnel;

Tunnel::Tunnel(std::string id, std::string spreadType, std::vector<Coord> shape, double width) :
        id(id)
{
    annotations = Veins::AnnotationManagerAccess().getIfExists();

    if (spreadType == "right")
        width *= 2;
    std::vector<std::vector<Coord>> segments = constructSegments(shape, width);
    ASSERT(!segments.empty());

    for (std::vector<Coord> corners : segments) {
        // add FloorSegments for floor
        FloorSegment* fs = new FloorSegment(id, "tunnel", 0.0);
        fs->setShape(corners);
//        if (annotations) {
//            annotations->drawPolygon(corners, "orange");
//        }
        floorAndCeiling.push_back(fs);

        // add FloorSegments for ceiling
        for (auto& c : corners) {
            c.z += 5.0;
        }
        fs = new FloorSegment(id, "tunnel", 0.0);
        fs->setShape(corners);
        floorAndCeiling.push_back(fs);

        // add Obstacles for the walls
        Obstacle* w1 = new Obstacle(id, "tunnel", std::numeric_limits<int>::max(), 0.0);
        w1->setShape( { corners[0], corners[1] });
        Obstacle* w2 = new Obstacle(id, "tunnel", std::numeric_limits<int>::max(), 0.0);
        w2->setShape( { corners[2], corners[3] });
        walls.push_back(w1);
        walls.push_back(w2);
    }
}

Tunnel::~Tunnel()
{

}

std::vector<std::vector<Coord>> Tunnel::constructSegments(std::vector<Coord> shape, double width) {
    ASSERT(shape.size() >= 2);

    std::vector<std::vector<Coord>> segments;
    std::vector<Coord> corners;

    Coord vbefore;

    // generation of convex polygons only (achieved by allowing max. 4 vertices)

    // scale width (e.g., to match the real width of a tunnel
    // alternatively, the desired width could also be hardcoded
    width*=1.2;

    for (unsigned int i = 0; i < shape.size() - 1; ++i) {
        Coord vi = shape[i + 1] - shape[i];

        if (corners.empty()) {
            if (vi == Coord::ZERO) continue;
            vi.normalize();

            Coord ni(vi.y, -vi.x, 0);
            ni.normalize();

            corners.push_back(shape[i] + ni*(0.5*width));
            corners.push_back(shape[i] - ni*(0.5*width));

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
                    c1 = shape[i] + di2d*(0.5*width);
                    c2 = shape[i] - di2d*(0.5*width);
                } else {
                    double angle = atan2(vi.y, vi.x) - atan2(-vbefore.y, -vbefore.x);
                    if (angle < 0) angle += 2 * M_PI;
                    double factor = sin(angle/2);
                    if (angle >= M_PI) factor *= -1;

                    di2d.normalize();
                    c1 = shape[i] + di2d*(0.5*width/factor);
                    c2 = shape[i] - di2d*(0.5*width/factor);
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
            corners.insert(corners.begin() + pos, shape[i + 1] + ni*(0.5*width));
            corners.insert(corners.begin() + pos + 1, shape[i + 1] - ni*(0.5*width));
        }
    }

    if (!corners.empty()) segments.push_back(corners);

    return segments;
}

std::string Tunnel::getId() const
{
    return id;
}

bool Tunnel::intersectsWith(const Coord& senderPos, const Coord& receiverPos) const
{
    double a = 0;
    double b = 0;
    for (const auto& o: walls) {
        double obstAtt = o->calculateAttenuation(senderPos, receiverPos, &a, &b);
        if (!FWMath::close(obstAtt, 1.0))
            return true;
    }

    for (const auto& f: floorAndCeiling) {
        Coord intersection;
        if (f->intersectsWith(senderPos, receiverPos, intersection))
            return true;
    }

    return false;
}

