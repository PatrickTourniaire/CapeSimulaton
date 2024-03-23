#pragma once

#include "cgp/cgp.hpp"
#include "../cloth/cloth.hpp"

// Parameters of the colliding sphere (center, radius)
struct sphere_parameter {
	cgp::vec3 center;
	float radius;
};

// Parameter attached to a fixed vertex (ku,kv) coordinates + 3D position
struct position_contraint {
	int ku;
	int kv;
	cgp::vec3 position;
};

// Used for cylindrical approximations of bone segments
struct cylinder_parameter {
  cgp::vec3 positionStart;
  cgp::vec3 positionEnd;
  float radius;
};


struct constraint_structure
{
	float ground_y = 0.0f;    // Height of the flood

	std::map<size_t, position_contraint> fixed_sample; // Storage of all fixed position of the cloth

  std::vector<sphere_parameter> spherical_constraints;

  std::vector<cylinder_parameter> cylindrical_constraints;

	// Add a new fixed position
	void add_fixed_position(int ku, int kv, vec3 const& position);
	// Remove a fixed position
	void remove_fixed_position(int ku, int kv);
};
