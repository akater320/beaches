#pragma once
#ifndef BEACHES_WAY_EDGES_HANDLER_H
#define BEACHES_WAY_EDGES_HANDLER_H

#include <vector>
#include "osmium/handler.hpp"
#include "osmium/osm/way.hpp"
#include "beaches/edge.h"
#include "beaches/water_polygon_relation_manager.h"

namespace beaches {
    class WayEdgesHandler : public osmium::handler::Handler {
    public:
        explicit WayEdgesHandler(std::vector<beaches::Edge>& edges, const WaterPolygonRelationManager& relationManager) 
            : m_edges{ edges }, m_relationManager{relationManager}
        {	}

        void way(const osmium::Way& way) noexcept;
    private:
        std::vector<beaches::Edge>& m_edges;
        const WaterPolygonRelationManager& m_relationManager;
    };

    inline void beaches::WayEdgesHandler::way(const osmium::Way & way) noexcept
    {
        if (way.nodes().size() < 2)
            return; //No edges.

        EdgeFlags wayFlags = m_relationManager.getFlags(way.id());

        if (way.is_closed() &&
            (way.tags().has_tag("natural", "water") ||
            way.tags().has_tag("waterway", "riverbank")) && 
            !(way.tags().has_tag("water", "intermittent") || way.tags().has_tag("itermittent", "yes"))) {

            wayFlags |= EdgeFlags::OuterRing;
        }

        bool isCoastline = way.tags().has_tag("natural", "coastline");

        //TODO: Check for degenerate polygons.
        if (wayFlags != EdgeFlags::None || isCoastline) {

            const auto& nodes = way.nodes();
            auto iter = nodes.begin();
            osmium::object_id_type prevId = iter->ref();
            iter++;
            for (; iter != nodes.end(); iter++) {
                osmium::object_id_type curId = iter->ref();

                EdgeFlags flags = wayFlags;
                if (isCoastline) {
                    flags |= (prevId < curId) ? EdgeFlags::WaterOnRight : EdgeFlags::WaterOnLeft;
                }

                m_edges.emplace_back(
                    std::min(prevId, curId),
                    std::max(prevId, curId),
                    flags
                );

                prevId = curId;
            }
        }

    }
}

#endif // !BEACHES_WAY_EDGES_HANDLER_H

