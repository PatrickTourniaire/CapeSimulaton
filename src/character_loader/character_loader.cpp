#include "character_loader.hpp"
#include "../environment.hpp"

using namespace cgp;

character_structure load_character_lola() {
	filename_loader_structure loader_param;
	loader_param.set_skeleton(project::path+"assets/lola/skeleton/");
	loader_param.add_rigged_mesh("body", project::path+"assets/lola/mesh-lola/", project::path+"assets/lola/mesh-lola/texture.jpg");
	loader_param.add_animation("Idle", project::path+"assets/lola/animation/idle/");
	loader_param.add_animation("Walk", project::path+"assets/lola/animation/walk/");

	character_structure character;
	character.load_and_initialize(loader_param, affine_rts().set_scaling(0.01f));

	return character;
}