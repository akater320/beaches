#pragma once
#ifndef BEACHES_ISLANDS_HANDLER_H
#define BEACHES_ISLANDS_HANDLER_H

#include "osmium/handler.hpp"
#include "osmium/osm/way.hpp"

#include "beaches/water_polygon_relation_manager.h"
#include "beaches/edge.h"

namespace beaches {
    class IslandsHandler : public osmium::handler::Handler {
    public:
        explicit IslandsHandler(WaterPolygonRelationManager& relations,
            std::vector<Edge>& edges) :
            m_relations{ relations }, m_edges{ edges }
        {

        }

        //redefine
        void way(const osmium::Way& way) const noexcept 
        {
            bool isReachable = m_relations.IsReachableFromSea(way.id());

            if (isReachable) {
                EdgeFlags flags = m_relations.getFlags(way.id());

                if ((flags&EdgeFlags::InnerRing) == EdgeFlags::InnerRing) {
                    //This way is an inner ring in a water polygon that is reachable.
                    //Mark every edge as "coastline outer ring".

                    const auto& nodes = way.nodes();
                    auto iter = nodes.begin();
                    osmium::object_id_type prevId = iter->ref();
                    iter++;
                    bool isReachable = false;
                    for (; iter != nodes.end() && !isReachable; iter++) {
                        osmium::object_id_type curId = iter->ref();

                        getEdge(prevId, curId).setFlag(EdgeFlags::CoastOuterRing, true);

                        prevId = curId;
                    }
                }
            }
        }

        Edge& getEdge(const osmium::object_id_type node1, const osmium::object_id_type node2) const {
            auto lowerId = std::min(node1, node2);
            auto higherId = std::max(node1, node2);

            auto iter = std::lower_bound(m_edges.begin(), m_edges.end(), lowerId,
                [](const Edge& pEdge, const osmium::object_id_type pId) {
                return pEdge.Node1() < pId;
            });

            //Nodes have low degree. Usually 2. So, cache-friendly forward-search.
            for (; iter != m_edges.end() && iter->Node1() == lowerId; iter++) {
                if (iter->Node2() == higherId) {
                    return *iter;
                }
            }

            std::cout << "Could not find edge.\n";
            assert(false);
            std::exit(-1);
        }

    private:
        WaterPolygonRelationManager& m_relations;
        std::vector<Edge>& m_edges;
    };
}

#endif // !BEACHES_ISLANDS_HANDLER_H

