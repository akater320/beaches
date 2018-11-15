#pragma once
#ifndef BEACHES_REACHABLE_WAYS_HANDLER_H
#define BEACHES_REACHABLE_WAYS_HANDLER_H

#include "osmium/handler.hpp"
#include "osmium/osm/way.hpp"

#include "beaches/water_polygon_relation_manager.h"
#include "beaches/edge.h"

namespace beaches {
    class ReachableWaysHandler : public osmium::handler::Handler {
    public:
        ReachableWaysHandler(WaterPolygonRelationManager& relations,
            const std::vector<Edge>& edges) :
            m_relations{ relations }, m_edges{ edges }
        {

        }

        //redefine
        void way(const osmium::Way& way) {
            EdgeFlags flags = m_relations.getFlags(way.id());

            if ((flags & EdgeFlags::OuterRing) == EdgeFlags::OuterRing) {
                //This edge is the outer boundary of a water polygon.
                //Determine whether any constituent edge is tagged as reachable.

                const auto& nodes = way.nodes();
                auto iter = nodes.begin();
                osmium::object_id_type prevId = iter->ref();
                iter++;
                bool isReachable = false;
                for (; iter != nodes.end() && !isReachable; iter++) {
                    osmium::object_id_type curId = iter->ref();

                    isReachable = edgeIsReachable(prevId, curId);

                    prevId = curId;
                }

                if (isReachable) {
                    m_relations.setReachable(way.id(), true);
                }
            }
        }

        bool edgeIsReachable(osmium::object_id_type node1, osmium::object_id_type node2) {
            auto lowerId = std::min(node1, node2);
            auto higherId = std::max(node1, node2);

            auto iter = std::lower_bound(m_edges.begin(), m_edges.end(), lowerId,
                [](const Edge& pEdge, const osmium::object_id_type pId) {
                return pEdge.Node1() < pId;
            });

            //All nodes have a low degree. ~2
            for (; iter != m_edges.end() && iter->Node1() == lowerId; iter++) {
                if (iter->Node2() == higherId) {
                    return iter->hasFlag(EdgeFlags::ReachableFromSea);
                }
            }

            return false;
        }

    private:
        WaterPolygonRelationManager& m_relations;
        const std::vector<Edge>& m_edges;
    };
}

#endif // !BEACHES_REACHABLE_WAYS_HANDLER_H

