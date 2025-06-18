#pragma once

#include <string>
#include <unordered_map>

enum class BimElementType {
    WALL,
    ROOF,
    FLOOR,
    WINDOW,
    DOOR,
    OTHER,
    GROUND,
    NOT_ASSIGNED
};

inline std::string toString(BimElementType type) {
    switch (type) {
        case BimElementType::WALL: return "Wall";
        case BimElementType::ROOF: return "Roof";
        case BimElementType::FLOOR: return "Floor";
        case BimElementType::WINDOW: return "Window";
        case BimElementType::DOOR: return "Door";
        case BimElementType::OTHER: return "Other";
        case BimElementType::GROUND: return "Ground";
        default: return "Not asssigned";
    }
}

inline BimElementType fromString(const std::string& str) {
    static const std::unordered_map<std::string, BimElementType> map = {
        {"Wall", BimElementType::WALL},
        {"Roof", BimElementType::ROOF},
        {"Floor", BimElementType::FLOOR},
        {"Window", BimElementType::WINDOW},
        {"Door", BimElementType::DOOR},
        {"Other", BimElementType::OTHER},
        {"Ground", BimElementType::GROUND},
        {"Not assigned", BimElementType::NOT_ASSIGNED}
    };

    auto it = map.find(str);
    return it != map.end() ? it->second : BimElementType::NOT_ASSIGNED;
}


