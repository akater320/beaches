#pragma once
#ifndef BEACHES_NODE_WITH_LOCATION_H
#define BEACHES_NODE_WITH_LOCATION_H

#include "osmium/osm/types.hpp"

namespace beaches {
    class NodeWithLocation {
    public:
        explicit NodeWithLocation(osmium::object_id_type id) :
            m_id{id} {

        }

        void setLat(float val) noexcept {
            m_lat = val;
        }
        void setLon(float val) noexcept {
            m_lon = val;
        }
        float Lat() const noexcept {
            return m_lat;
        }
        float Lon() const noexcept {
            return m_lon;
        }
        osmium::object_id_type Id() const noexcept {
            return m_id;
        }

    private:
        osmium::object_id_type m_id; //8 bytes
        float m_lat; //4 bytes
        float m_lon; //4 bytes

        //16 bytes, 8 byte alignment.
    };
}

#endif // !BEACHES_NODE_WITH_LOCATION_H

