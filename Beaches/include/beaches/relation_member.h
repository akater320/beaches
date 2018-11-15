#pragma once
#ifndef BEACHES_RELATION_MEMBER_H
#define BEACHES_RELATION_MEMBER_H

#include "osmium/osm/types.hpp"
#include "beaches/edge_flags.h"
#include "beaches/util.h"

namespace beaches {
	class RelationMember {
	public:
		explicit RelationMember(osmium::object_id_type relationId, osmium::object_id_type memberId, EdgeFlags flags)
			: m_flags{flags} {
			PackIntoBytes<5, osmium::object_id_type>(m_relationIdBytes, relationId);
			PackIntoBytes<5, osmium::object_id_type>(m_memberIdBytes, memberId);
		}

		osmium::object_id_type Relation() const noexcept {
			return UnpackFromBytes<5, osmium::object_id_type>(m_relationIdBytes);
		}

		osmium::object_id_type Member() const noexcept {
			return UnpackFromBytes<5, osmium::object_id_type>(m_memberIdBytes);
		}

		EdgeFlags Flags() const noexcept {
			return m_flags;
		}

		bool IsReachableFromSea() const noexcept {
			return m_reachableFromSea;
		}

		void SetReachableFromSea(bool value) noexcept {
			m_reachableFromSea = value;
		}

	private:
		uint8_t m_relationIdBytes[5];
		uint8_t m_memberIdBytes[5];
		EdgeFlags m_flags; //1 byte
		bool m_reachableFromSea = false; //1-4 bytes? //TODO: make atomic?
		//16 bytes after padding. 1 byte alignment.
	};
}

#endif // !BEACHES_RELATION_MEMBER_H

