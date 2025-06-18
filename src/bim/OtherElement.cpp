#include "OtherElement.h"

OtherElement::OtherElement(const geom::GeometryData& dataWithoutPlane)
    : BimElement(dataWithoutPlane, BimElementType::OTHER) {
    // Pas besoin de vérifier l'absence de plan — c'est toléré.
}

