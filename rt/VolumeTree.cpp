#include <algorithm>
#include <stack>

#include "VolumeTree.h"
#include "rt_structs.h"

AABBox::AABBox()
	: spheres_index(-1)
	, spheres_num(0)
{}

Axis AABBox::get_longest_axis() const
{
	float absx = std::abs(minx - maxx);
	float absy = std::abs(miny - maxy);

	float max = std::max(absx, absy);

	if (max == absx)
	{
		return Axis::X;
	}
	if (max == absy)
	{
		return Axis::Y;
	}

	return Axis::X;
}

bool AABBox::init(std::vector<sphere const*> const& spheres, int left, int right, bool as_leaf)
{
	is_leaf = as_leaf;
	if (is_leaf)
	{
		auto size = right - left;
		if (size > VolumeTree::s_MaxSpheresPerNode)
		{
			return false;
		}

		spheres_index = left;
		spheres_num = size;
	}

	auto leftIt = spheres.begin() + left;
	auto rightIt = spheres.begin() + right;

	auto sphere_min_x = std::min_element(leftIt, rightIt, [](sphere const* s1, sphere const* s2)
	{
		return s1->cx - s1->radius < s2->cx - s2->radius;
	});
	auto sphere_max_x = std::max_element(leftIt, rightIt, [](sphere const* s1, sphere const* s2)
	{
		return s1->cx + s1->radius < s2->cx + s2->radius;
	});
	auto sphere_min_y = std::min_element(leftIt, rightIt, [](sphere const* s1, sphere const* s2)
	{
		return s1->cy - s1->radius < s2->cy - s2->radius;
	});
	auto sphere_max_y = std::max_element(leftIt, rightIt, [](sphere const* s1, sphere const* s2)
	{
		return s1->cy + s1->radius < s2->cy + s2->radius;
	});
	auto sphere_min_z = std::min_element(leftIt, rightIt, [](sphere const* s1, sphere const* s2)
	{
		return s1->cz - s1->radius < s2->cz - s2->radius;
	});
	auto sphere_max_z = std::max_element(leftIt, rightIt, [](sphere const* s1, sphere const* s2)
	{
		return s1->cz + s1->radius < s2->cz + s2->radius;
	});

	minx = (*sphere_min_x)->cx - (*sphere_min_x)->radius;
	maxx = (*sphere_max_x)->cx + (*sphere_max_x)->radius;
	miny = (*sphere_min_y)->cy - (*sphere_min_y)->radius;
	maxy = (*sphere_max_y)->cy + (*sphere_max_y)->radius;
	minz = (*sphere_min_z)->cz - (*sphere_min_z)->radius;
	maxz = (*sphere_max_z)->cz + (*sphere_max_z)->radius;

	return true;
}

bool AABBox::intersect(ray const& r, float& t) const
{
	float tmin = (minx - r.ox) / r.dx;
	float tmax = (maxx - r.ox) / r.dx;

	if (tmin > tmax) std::swap(tmin, tmax);

	float tymin = (miny - r.oy) / r.dy;
	float tymax = (maxy - r.oy) / r.dy;

	if (tymin > tymax) std::swap(tymin, tymax);

	if ((tmin > tymax) || (tymin > tmax))
		return false;

	if (tymin > tmin)
		tmin = tymin;

	if (tymax < tmax)
		tmax = tymax;

	float tzmin = (minz - r.oz) / r.dz;
	float tzmax = (maxz - r.oz) / r.dz;

	if (tzmin > tzmax) std::swap(tzmin, tzmax);

	if ((tmin > tzmax) || (tzmin > tmax))
		return false;

	if (tzmin > tmin)
		tmin = tzmin;

	if (tzmax < tmax)
		tmax = tzmax;

	t = tmin;
	return true;
}

std::vector<AABBox> VolumeTree::build(sphere* spheres, std::uint32_t num_spheres)
{
	m_nodes.resize(2 * num_spheres);

	m_spheres.resize(num_spheres);
	for (auto i = 0U; i < num_spheres; ++i)
	{
		m_spheres[i] = &spheres[i];
	}

	m_nodes[0].init(m_spheres, 0, static_cast<int>(m_spheres.size()), false);
	_build_recursive(0, num_spheres, 0);

	return m_nodes;
}

bool VolumeTree::get_intersection(ray& r, sphere const *& closest)
{
	IntersectionStack intersected_nodes;
	int current_index = 0;
	float t;

	closest = nullptr;

	if (!m_nodes[0].intersect(r, t))
	{
		return false;
	}

	// Traverse an AABBoxes tree.
	// This loop will always interrupt because the current_node will ultimately reach a leaf node and proceed
	// to the stack check code which is also will ultimately end up being empty 
	// because every new node in the stack is closer and closer to leaves.
	while (true)
	{
		AABBox const& current_node = m_nodes[current_index];

		// Perform an intersection check on inner spheres in this case
		if (current_node.is_leaf)
		{
			int idx = -1;
			int left = current_node.spheres_index;
			int right = left + current_node.spheres_num;

			for (int i = left; i < right; ++i)
			{
				if (intersect_sphere(*m_spheres[i], r))
				{
					idx = i;
				}
			}

			if (idx != -1)
			{
				// Save this sphere for now as we can find a closer one.
				closest = m_spheres[idx];
			}
		}
		// If we found a new intersection with childs then we continue to traverse current sub-tree.
		else if (_find_next_nodes(intersected_nodes, r, current_index))
		{
			continue;
		}

		IntersectionStackEntry entry = { -1, std::numeric_limits<float>::max() };

		// We are only interested in nodes that can contain spheres 
		// that are potentially closer than the current closest found.
		while (entry.t > r.maxt)
		{
			if (intersected_nodes.empty())
			{
				return closest ? true : false;
			}

			entry = intersected_nodes.top();
			intersected_nodes.pop();
		}

		current_index = entry.node_index;
	}
}

void VolumeTree::_build_recursive(int left, int right, int current_node)
{
	if (right - left <= s_MaxSpheresPerNode)
	{
		m_nodes[current_node].init(m_spheres, left, right, true);
		return;
	}

	AABBox const& currentBbox = m_nodes[current_node];
	Axis longest = currentBbox.get_longest_axis();

	std::sort(
		m_spheres.begin() + left,
		m_spheres.begin() + right,
		[longest](sphere const* s1, sphere const* s2)
	{
		switch (longest)
		{
		case Axis::X:
			return s1->cx < s2->cx;
		case Axis::Y:
			return s1->cy < s2->cy;
		default:
			return s1->cx < s2->cx;
		}
	}
	);

	int split_index = (left + right) / 2;

	int left_child_index = current_node * 2 + 1;
	int right_child_index = current_node * 2 + 2;

	m_nodes[left_child_index].init(m_spheres, left, split_index, false);
	m_nodes[right_child_index].init(m_spheres, split_index, right, false);

	_build_recursive(left, split_index, left_child_index);
	_build_recursive(split_index, right, right_child_index);
}

bool VolumeTree::_find_next_nodes(IntersectionStack& intersected_nodes, ray const& r, int& current_index)
{
	int left_child_index = current_index * 2 + 1;
	int right_child_index = current_index * 2 + 2;

	float left_t, right_t;
	bool left_intersection = m_nodes[left_child_index].intersect(r, left_t);
	bool right_intersection = m_nodes[right_child_index].intersect(r, right_t);

	// Here we continue to traverse the sub-tree with closer intersection
	// but store the second node in case there won't be any intersections inside the current sub-tree.
	if (left_intersection && right_intersection)
	{
		if (left_t < right_t)
		{
			current_index = left_child_index;
			intersected_nodes.push({ right_child_index, right_t });
		}
		else
		{
			current_index = right_child_index;
			intersected_nodes.push({ left_child_index, left_t });
		}
		return true;
	}
	else if (left_intersection)
	{
		current_index = left_child_index;
		return true;
	}
	else if (right_intersection)
	{
		current_index = right_child_index;
		return true;
	}

	return false;
}
