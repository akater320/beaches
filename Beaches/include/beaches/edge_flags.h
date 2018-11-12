#pragma once
#ifndef BEACHES_EDGE_FLAGS_H
#define BEACHES_EDGE_FLAGS_H

namespace beaches {
	enum class EdgeFlags : uint8_t {
		None = 0,
		OuterRing = 1 << 0,
		InnerRing = 1 << 1,
		WaterOnLeft = 1 << 2,
		WaterOnRight = 1 << 3,
		Erased = 1 << 4,
		CoastOuterRing = 1<< 5,
		ReachableFromSea = 1 << 6, 
		Visited = 1 << 7
	};

	inline EdgeFlags operator|(const EdgeFlags& left, const EdgeFlags& right) {
		return static_cast<EdgeFlags>((static_cast<uint8_t>(left) | static_cast<uint8_t>(right)));
	}

	inline EdgeFlags operator&(const EdgeFlags& left, const EdgeFlags& right) {
		return static_cast<EdgeFlags>((static_cast<uint8_t>(left) & static_cast<uint8_t>(right)));
	}

	inline EdgeFlags operator~(const EdgeFlags& left) {
		return static_cast<EdgeFlags>(~static_cast<uint8_t>(left));
	}

	inline static EdgeFlags& operator|=(EdgeFlags& left, const EdgeFlags& right) {
		left = left | right;
		return left;
	}

	inline static EdgeFlags& operator&=(EdgeFlags& left, const EdgeFlags& right) {
		left = left & right;
		return left;
	}
}

#endif // !BEACHES_EDGE_FLAGS_H

