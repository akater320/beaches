#pragma once
#ifndef BEACHES_WATER_POLYGON_RELATION_MANAGER_H
#define BEACHES_WATER_POLYGON_RELATION_MANAGER_H

#include <vector>
#include <algorithm>
#include <iostream>
//#include <optional>

#include "osmium/handler.hpp"
#include "osmium/osm/relation.hpp"
#include "beaches/relation_member.h"

namespace beaches {
    class WaterPolygonRelationManager : public osmium::handler::Handler {
    public:
        explicit WaterPolygonRelationManager()
        {

        }

        //ugly, move out the vector instead?
        void sortMembersForLookup() {
            //Sort the relations by member id. This allows quick searches.
            std::sort(m_members.begin(), m_members.end(),
                [](const RelationMember& left, const RelationMember& right) {
                return left.Member() < right.Member();
            });
        }

        void updateReachableFlags() {
            //If any outer way is marked as "reachable" then propagate that to inner rings.
            std::sort(m_members.begin(), m_members.end(),
                [](const RelationMember& left, const RelationMember& right) {
                return left.Relation() < right.Relation();
            });

            auto iter1 = m_members.begin();
            auto iter2 = m_members.begin();

            while (iter1 != m_members.end()) {
                auto relationId = iter1->Relation();
                iter2 = std::upper_bound(iter1, m_members.end(), relationId,
                    [](const osmium::object_id_type pId, const RelationMember& pWay) {
                    return pId < pWay.Relation();
                });

                bool isReachable = std::any_of(iter1, iter2, 
                    [](const RelationMember& mem) { return mem.IsReachableFromSea(); });
                if (isReachable) {
                    std::for_each(iter1, iter2,
                        [](RelationMember& mem) { mem.SetReachableFromSea(true); });
                }

                iter1 = iter2;
            }

            //Return the sort order for quick lookups by way(member) id.
            sortMembersForLookup();
        }

        EdgeFlags getFlags(osmium::object_id_type wayId) const {
            //NOTE: The vector must be sorted by member id first!
            auto iter = std::lower_bound(m_members.begin(), m_members.end(), wayId,
                [](const RelationMember& pMember, const osmium::object_id_type pId) {
                return pMember.Member() < pId;
            }
            );

            //Ways may belong to multiple relations.
            EdgeFlags flags = EdgeFlags::None;
            for (; iter != m_members.end() && iter->Member() == wayId; iter++) {
                flags |= iter->Flags();
            }

            return flags;
        }

        bool IsReachableFromSea(osmium::object_id_type wayId) const {
            //NOTE: The vector must be sorted by member id first!
            auto iter = std::lower_bound(m_members.begin(), m_members.end(), wayId,
                [](const RelationMember& pMember, const osmium::object_id_type pId) {
                return pMember.Member() < pId;
            }
            );

            //Ways may belong to multiple relations.
            for (; iter != m_members.end() && iter->Member() == wayId; iter++) {
                if (iter->IsReachableFromSea()) {
                    return true;
                }
            }

            return false;
        }

        //The handler method.
        void relation(const osmium::Relation& relation) noexcept;

        const std::vector<RelationMember>& Members() const noexcept {
            return m_members;
        }

        void setReachable(osmium::object_id_type wayId, bool isReachable) {
            auto iter = std::lower_bound(m_members.begin(), m_members.end(), wayId,
                [](const RelationMember& pMember, const osmium::object_id_type pWayId) {
                return pMember.Member() < pWayId;
            });

            //The way may belong to multiple relations. Mark all of them. 
            for (; iter != m_members.end() && iter->Member() == wayId; iter++) {
                iter->SetReachableFromSea(isReachable);
            }
        }

    private:
        std::vector<RelationMember> m_members;
    };

    inline void beaches::WaterPolygonRelationManager::relation(const osmium::Relation & relation) noexcept
    {
        const auto& tags = relation.tags();
        if (tags.has_tag("type", "multipolygon") && (
            tags.has_tag("waterway", "riverbank") ||
            tags.has_tag("natural", "water")) &&
            !(tags.has_tag("water", "intermittent") || tags.has_tag("intermittent", "yes"))) {

            for (const auto& mem : relation.members()) {
                EdgeFlags flags = EdgeFlags::None;

                if (strcmp("outer", mem.role()) == 0) {
                    flags |= EdgeFlags::OuterRing;
                }

                if (strcmp("inner", mem.role()) == 0) {
                    flags |= EdgeFlags::InnerRing;
                }

                m_members.emplace_back(relation.id(), mem.ref(), flags);
            }
        }
    }
}

#endif // !BEACHES_WATER_POLYGON_RELATION_MANAGER_H

