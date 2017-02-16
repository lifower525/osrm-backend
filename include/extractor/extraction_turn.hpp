#ifndef OSRM_EXTRACTION_TURN_HPP
#define OSRM_EXTRACTION_TURN_HPP

#include <boost/numeric/conversion/cast.hpp>

#include <extractor/guidance/intersection.hpp>

#include <cstdint>

namespace osrm
{
namespace extractor
{

struct ExtractionTurn
{
    ExtractionTurn(const guidance::ConnectedRoad &turn, bool has_traffic_light)
        : angle(180. - turn.angle), turn_type(turn.instruction.type),
          direction_modifier(turn.instruction.direction_modifier),
          has_traffic_light(has_traffic_light), weight(0.), duration(0.),
          source_local_access_only(false), target_local_access_only(false)
    {
    }

    const double angle;
    const guidance::TurnType::Enum turn_type;
    const guidance::DirectionModifier::Enum direction_modifier;
    const bool has_traffic_light;
    double weight;
    double duration;
    bool source_local_access_only;
    bool target_local_access_only;
};
}
}

#endif
