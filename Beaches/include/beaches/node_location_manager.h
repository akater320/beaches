#pragma once
#ifndef BEACHES_NODE_LOCATION_MANAGER_H
#define BEACHES_NODE_LOCATION_MANAGER_H

#include "osmium/handler.hpp"
#include "osmium/osm/node.hpp"
#include "beaches/util.h"
#include "beaches/node_with_location.h"

namespace beaches {
	class NodeLocationManger : public osmium::handler::Handler {
	public:
		explicit NodeLocationManger() {}

		void trackNode(osmium::object_id_type nodeId) {
			const auto iter = std::lower_bound(m_nodes.begin(), m_nodes.begin() + m_sortedIndex, nodeId,
				[](const NodeWithLocation& pNode, const osmium::object_id_type pId) {
				return pNode.Id() < pId; });
			if (iter != m_nodes.end() && iter->Id() == nodeId) {
				return;
			}

			m_nodes.emplace_back(nodeId);
		}

		void clear() {
			this->m_nodes.clear();
			m_sortedIndex = 0;
		}

		void prepareHandler() {
			std::cout << "Tracker has " << m_nodes.size() << " nodes.\n";

			//Sort the nodes and remove duplicates.
			std::sort(m_nodes.begin(), m_nodes.end(),
				[](const NodeWithLocation& left, const NodeWithLocation& right) {
				return left.Id() < right.Id();
			});

			//Remove the duplicates.
			m_nodes.erase(std::unique(m_nodes.begin(), m_nodes.end(),
				[](const NodeWithLocation& left, const NodeWithLocation& right) { return left.Id() == right.Id(); }),
				m_nodes.end());
			m_nodes.shrink_to_fit();

			m_sortedIndex = m_nodes.size();
			std::cout << "Tracker now has " << m_nodes.size() << " nodes.\n";
		}

		void node(const osmium::Node& node) noexcept;

		const NodeWithLocation& GetNode(osmium::object_id_type id) const noexcept {
			auto iter = std::lower_bound(m_nodes.begin(), m_nodes.end(), id,
				[](const NodeWithLocation& pNode, const osmium::object_id_type pId) { return pNode.Id() < pId; });
			if (iter != m_nodes.end() && iter->Id() == id) {
				return *iter;
			}
			assert(false);
			std::exit(-1);
		}
	private:
		std::vector<NodeWithLocation> m_nodes;
		long m_sortedIndex = -1; //past the end
	};

	inline void beaches::NodeLocationManger::node(const osmium::Node & node) noexcept
	{
		auto iter = std::lower_bound(m_nodes.begin(), m_nodes.end(), node.id(),
			[](const NodeWithLocation& pNode, const osmium::object_id_type pId) { return pNode.Id() < pId; });
		if (iter != m_nodes.end() && iter->Id() == node.id()) {
			const auto loc = node.location();
			iter->setLat(loc.lat());
			iter->setLon(loc.lon());
		}
	}
}

#endif // !BEACHES_NODE_LOCATION_MANAGER_H

