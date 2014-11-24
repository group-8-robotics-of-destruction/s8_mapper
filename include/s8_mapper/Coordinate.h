#ifndef __COORDINATE_H
#define __COORDINATE_H

#include <s8_utils/math.h>

using namespace s8::utils::math;

namespace s8 {
    struct Coordinate {
        double x;
        double y;

        Coordinate() : x(0.0), y(0.0) {}
        Coordinate(double x, double y) : x(x), y(y) {}

        bool operator== (Coordinate coordinate) {
            return s8::utils::math::is_zero(x - coordinate.x) && s8::utils::math::is_zero(y - coordinate.y);
        }
    };

    std::string to_string(Coordinate coordinate) {
        return "(" + std::to_string(coordinate.x) + ", " + std::to_string(coordinate.y) + ")";
    }
}

#endif
