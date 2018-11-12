#pragma once
#ifndef BEACHES_EDGE_H
#define BEACHES_EDGE_H

#include "osmium/osm/types.hpp"
#include "beaches/util.h"
#include "beaches/edge_flags.h"

namespace beaches {
	class Edge {
	public:
		explicit Edge(osmium::object_id_type node1Id, osmium::object_id_type node2Id, EdgeFlags flags)
			: m_flags{flags} {
			beaches::PackIntoBytes<5, osmium::object_id_type>(m_node1IdBytes, node1Id);
			beaches::PackIntoBytes<5, osmium::object_id_type>(m_node2IdBytes, node2Id);
		}

		osmium::object_id_type Node1() const noexcept {
			return beaches::UnpackFromBytes<5, osmium::object_id_type>(m_node1IdBytes);
		}

		osmium::object_id_type Node2() const noexcept {
			return beaches::UnpackFromBytes<5, osmium::object_id_type>(m_node2IdBytes);
		}

		void setFlag(EdgeFlags flag, bool value) {
			if (value) {
				m_flags |= flag;
			}
			else {
				m_flags &= ~flag;
			}
		}

		void unionFlags(EdgeFlags others) {
			m_flags |= others;
		}

		bool hasFlag(EdgeFlags flag) const noexcept {
			return (m_flags & flag) == flag;
		}

		EdgeFlags Flags() const noexcept {
			return m_flags;
		}

	private:
		uint8_t m_node1IdBytes[5];
		uint8_t m_node2IdBytes[5];
		EdgeFlags m_flags;
	};
}

#endif // !BEACHES_EDGE_H

