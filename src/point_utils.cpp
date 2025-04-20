// point_utils.h
#include <cmath>
struct Point { double x, y; };

class PointUtils {
public:
    // Compute Euclidean distance
    static double distance(const Point &a, const Point &b) {
        double dx = a.x - b.x, dy = a.y - b.y;
        return std::sqrt(dx*dx + dy*dy);
    }
};
