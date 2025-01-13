#ifndef ROBOT4WS_MAPPING_UTILITIES_HPP
#define ROBOT4WS_MAPPING_UTILITIES_HPP

#include <grid_map_core/GridMap.hpp>  // Per grid_map::Index
#include <functional>                // Per std::hash
#include <cv_bridge/cv_bridge.h>
#include <map>

namespace robot4ws_mapping {

    // Functor per calcolare l'hash di un grid_map::Index
    struct IndexHash{
        size_t operator()(const grid_map::Index& index) const {
            return std::hash<int>()(index(0)) ^ std::hash<int>()(index(1));
        }
    };

    // Functor per confrontare due grid_map::Index
    struct IndexEqual{
        bool operator()(const grid_map::Index& lhs, const grid_map::Index& rhs) const {
            return lhs(0) == rhs(0) && lhs(1) == rhs(1);
        }
    };

    // Enum per rappresentare gli ID dei colori
    enum ColorId : int {
        Unknown = -1,
        Red = 0,
        Green = 1,
        Blue = 2,
        Yellow = 3,
        Orange = 4,
        Cyan = 5,
        Purple = 6,
        Last = 7 
    };

    static const int NUM_COLORS = static_cast<int>(ColorId::Last);

    inline cv::Vec3b colorIdToBGR(ColorId colorId){
        static const std::map<ColorId, cv::Vec3b> color_map = {
            {ColorId::Red,    cv::Vec3b(0, 0, 255)},
            {ColorId::Green,  cv::Vec3b(0, 255, 0)},
            {ColorId::Blue,   cv::Vec3b(255, 0, 0)},
            {ColorId::Yellow, cv::Vec3b(0, 255, 255)},
            {ColorId::Orange, cv::Vec3b(0, 165, 255)},
            {ColorId::Cyan,   cv::Vec3b(255, 255, 0)},
            {ColorId::Purple, cv::Vec3b(128, 0, 128)},
            {ColorId::Unknown, cv::Vec3b(0, 0, 0)}
        };

        auto it = color_map.find(colorId);
        return it != color_map.end() ? it->second : cv::Vec3b(0, 0, 0);
    }
} // namespace robot4ws_mapping

#endif // ROBOT4WS_MAPPING_UTILITIES_HPP
