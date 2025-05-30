#ifndef BIM_ELEMENT_TYPE_H
#define BIM_ELEMENT_TYPE_H

#include <string>
#include <unordered_map>

enum class BimElementType {
    Unknown,
    Wall,
    Roof,
    Floor,
    Window,
    Door,
    Other
};

inline std::string toString(BimElementType type) {
    switch (type) {
        case BimElementType::Wall: return "Wall";
        case BimElementType::Roof: return "Roof";
        case BimElementType::Floor: return "Floor";
        case BimElementType::Window: return "Window";
        case BimElementType::Door: return "Door";
        case BimElementType::Other: return "Other";
        default: return "Unknown";
    }
}

inline BimElementType fromString(const std::string& str) {
    static const std::unordered_map<std::string, BimElementType> map = {
        {"Wall", BimElementType::Wall},
        {"Roof", BimElementType::Roof},
        {"Floor", BimElementType::Floor},
        {"Window", BimElementType::Window},
        {"Door", BimElementType::Door},
        {"Other", BimElementType::Other},
        {"Unknown", BimElementType::Unknown}
    };

    auto it = map.find(str);
    return it != map.end() ? it->second : BimElementType::Unknown;
}

#endif // BIM_ELEMENT_TYPE_H

