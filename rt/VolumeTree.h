#pragma once

#include <vector>

struct sphere;
struct ray;

enum class Axis
{
	X = 0,
	Y,
	Z
};

class AABBox
{
public:
	AABBox();

	bool init(std::vector<sphere const*> const& spheres, int left, int right, bool as_leaf);
	bool intersect(ray const& r, float& t) const;

	Axis get_longest_axis() const;

	// If is_leaf - true then spheres_index and spheres_num 
	// point to spheres range in VolumeTree::m_spheres vector
	bool is_leaf = false;
	int spheres_index;
	int spheres_num;

	// Bounds
	float minx, maxx;
	float miny, maxy;
	float minz, maxz;
};

// This is a binary tree used to represent an AABBoxes hierarchy
class VolumeTree
{
public:
	std::vector<AABBox> build(sphere* spheres, std::uint32_t num_spheres);

	// Finds an intersection with the closest sphere for O(log n)
	bool get_intersection(ray& r, sphere const* & closest);

	static const int s_MaxSpheresPerNode = 2;

private:

	struct IntersectionStackEntry
	{
		int node_index;
		float t;
	};
	using IntersectionStack = std::stack<IntersectionStackEntry>;

	void _build_recursive(int left, int right, int current_node);
	bool _find_next_nodes(IntersectionStack& intersected_nodes, ray const& r, int& current_index);

private:

	// Binary tree of AABBoxes stored as array
	std::vector<AABBox> m_nodes;
	std::vector<sphere const*> m_spheres;
};
