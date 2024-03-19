#include "scene.hpp"

#include "cgp/02_numarray/numarray/numarray.hpp"
#include "cgp/02_numarray/numarray_stack/special_types/special_types.hpp"
#include "cgp/16_drawable/mesh_drawable/mesh_drawable.hpp"
#include "character_loader/character_loader.hpp"
#include "constraint/constraint.hpp"


using namespace cgp;


void initialize_ground(mesh_drawable& ground);

void scene_structure::initialize()
{
	camera_control.initialize(inputs, window); // Give access to the inputs and window global state to the camera controler
	camera_control.set_rotation_axis_y();
	camera_control.look_at({ 4.0f, 3.0f, 3.0f }, {0,0,0}, {0,0,1});
	global_frame.initialize_data_on_gpu(mesh_primitive_frame());
	

	sphere_ik.initialize_data_on_gpu(mesh_primitive_sphere());
	sphere_ik.model.scaling = 0.05f;
	sphere_ik.material.color = {1,0,0};

	initialize_ground(ground);

	std::cout<<"- Load Lola character"<<std::endl;
	characters["Lola"] = load_character_lola();

	current_active_character = "Lola";

	for(auto& entry : characters)
		entry.second.timer.start();
  
  // Load the texture for the cape
	cloth_texture.load_and_initialize_texture_2d_on_gpu(project::path + "assets/cloth.jpg");
	initialize_cloth(gui.N_sample_edge);

  
}

void scene_structure::display_frame()
{
	// Set the light to the current position of the camera
	environment.light = camera_control.camera_model.position();
	
	if (gui.display_frame)
		draw(global_frame, environment);

	if(gui.display_ground) 
		draw(ground, environment);
  

  

	// Update the local time for each character
	for(auto& entry: characters) {
		entry.second.timer.update();
	}

	// ************************************************* //
	// Update the current skeleton of each character
	// ************************************************* //
	for(auto& entry : characters) {
		std::string character_name = entry.first;
		character_structure& character = entry.second;
		effect_transition_structure& transition = effect_transition[character_name];

		// Default animation reading a standard animation cycle
		if(transition.active==false) {
			character.animated_model.set_skeleton_from_animation(character.current_animation_name, character.timer.t_periodic);
		}

		// Currently with an active transition between two animations
		else {
			effect_transition_compute(transition, character);
			effect_transition_stop_if_completed(transition, character);
		}
	}

	// ********************************** //
	// Apply effects on the skeleton
	// ********************************** //

	// Apply the walk effect if activated
	if(effect_walk.active) {
		effect_walking(effect_walk, characters[current_active_character], inputs, effect_transition[current_active_character]);
	}


	// ********************************** //
	// Compute Skinning deformation
	// ********************************** //
	for(auto& entry_character : characters) {
		animated_model_structure& animated_model = entry_character.second.animated_model;
		for(auto& rigged_mesh_entry : animated_model.rigged_mesh) {
			std::string mesh_name = rigged_mesh_entry.first;
			animated_model.skinning_lbs(mesh_name);
		}
	}

  // UPDATE POSITION CONSTRAINT FOR CAPE
  character_structure ch = characters[current_active_character];
  
  /*
   * Joint of index 11 name: mixamorig_LeftShoulder
   * Joint of index 12 name: mixamorig_RightShoulder
  */
  cgp::numarray<mat4> joint_frames = ch.animated_model.skeleton.joint_matrix_global;
  constraint.fixed_sample.clear();
  constraint.add_fixed_position(0, 0, joint_frames[17].get_block_translation());
  //constraint.add_fixed_position(0, (int) (gui.N_sample_edge/2), joint_frames[10].get_block_translation());
  constraint.add_fixed_position(0, gui.N_sample_edge - 1, joint_frames[16].get_block_translation());
  

  numarray<int> joint_spheres = {0, 11, 12, 23, 24, 2, 3, 5, 6};
  numarray<float> joint_radiuses = {0.20, 0.05, 0.05, 0.05, 0.05, 0.15, 0.15, 0.10, 0.10};

  for (int i = 0; i < joint_spheres.size(); i++) {
    vec3 joint_position = joint_frames[joint_spheres[i]].get_block_translation();
    float radius = joint_radiuses[i];

    if (i >= constraint.spherical_constraints.size()) {
      constraint.spherical_constraints.push_back({joint_position, radius}); 
      continue;
    }

    constraint.spherical_constraints[i] = {joint_position, radius};
  }


  numarray<numarray<int>> cylinder_connections = {
    {11, 23},
    {12, 24},
    {2, 5},
    {3, 6},
  };
  numarray<float> cylinder_radiuses = {
    0.08,
    0.08,
    0.10,
    0.10,
  };

  for (int i = 0; i < cylinder_connections.size(); i++) {
    vec3 start = joint_frames[cylinder_connections[i][0]].get_block_translation();
    vec3 end = joint_frames[cylinder_connections[i][1]].get_block_translation();
    float radius = cylinder_radiuses[i];

    if (i >= constraint.cylindrical_constraints.size()) {
      constraint.cylindrical_constraints.push_back({start, end, radius}); 
      continue;
    }
    
    constraint.cylindrical_constraints[i] = {start, end, radius};
  }


  /*
   *Joint at index (0) with name: mixamorig_Hips
    Joint at index (1) with name: mixamorig_Spine
  Joint at index (2) with name: mixamorig_LeftUpLeg
  Joint at index (3) with name: mixamorig_RightUpLeg
   Joint at index (4) with name: mixamorig_Spine1
  Joint at index (5) with name: mixamorig_LeftLeg
  Joint at index (6) with name: mixamorig_RightLeg
  Joint at index (7) with name: mixamorig_Spine2
  Joint at index (8) with name: mixamorig_LeftFoot
  Joint at index (9) with name: mixamorig_RightFoot
  Joint at index (10) with name: mixamorig_Neck
  Joint at index (11) with name: mixamorig_LeftShoulder
  Joint at index (12) with name: mixamorig_RightShoulder
  Joint at index (13) with name: mixamorig_LeftToeBase
  Joint at index (14) with name: mixamorig_RightToeBase
  Joint at index (15) with name: mixamorig_Head
  Joint at index (16) with name: mixamorig_LeftArm
  Joint at index (17) with name: mixamorig_RightArm
  Joint at index (18) with name: mixamorig_LeftToe_End
  Joint at index (19) with name: mixamorig_RightToe_End
  Joint at index (20) with name: mixamorig_HeadTop_End
  Joint at index (21) with name: mixamorig_LeftEye
  Joint at index (22) with name: mixamorig_RightEye
  Joint at index (23) with name: mixamorig_LeftForeArm
  Joint at index (24) with name: mixamorig_RightForeArm
  Joint at index (25) with name: mixamorig_LeftHand
  Joint at index (26) with name: mixamorig_RightHan
  */
  /*
   * circle with center at mixamorig_Hips
   * cylinder from mixamorig_LeftUpLeg to mixamorig_LeftLeg
   * -"- for right mixamorig_RightLeg
   *  large cylinder for spine to spine 2
   *  cylinder for shoulder left to shoulder right
   *  Smaller cylinders for the arms and fore arms.
   */
  // for (int i = 0; i < ch.animated_model.skeleton.joint_name.size(); i++) {
  //  std::cout << "Joint at index (" << i << ") with name: " << ch.animated_model.skeleton.joint_name[i] << std::endl;
  //}

  if (constraint.spherical_constraints.size() != obstacle_spheres.size()) {
    for (int i = 0; i < constraint.spherical_constraints.size(); i ++) {
      cgp::mesh_drawable obstacle_sphere;

      obstacle_sphere.initialize_data_on_gpu(mesh_primitive_sphere());
	    obstacle_sphere.model.translation = constraint.spherical_constraints[i].center;
	    obstacle_sphere.model.scaling = constraint.spherical_constraints[i].radius;
	    obstacle_sphere.material.color = { 1,0,0 };
     
      obstacle_spheres.push_back(obstacle_sphere);
    }
  }

  // Draw cylindrical constraints
  if (obstacle_cylinders.empty()) {
    for (const auto& cylinder_constraint : constraint.cylindrical_constraints) {
        vec3 start = cylinder_constraint.positionStart;
        vec3 end = cylinder_constraint.positionEnd;
        float radius = cylinder_constraint.radius;

        // Create cylinder mesh_drawable
        mesh_drawable cylinder_mesh;
        cylinder_mesh.initialize_data_on_gpu(cgp::mesh_primitive_cylinder(radius, start, end));
        cylinder_mesh.material.color = {0, 1, 0}; // Set color to green

        obstacle_cylinders.push_back(cylinder_mesh);
    }
  }
  
  // Update sphere centers and draw
  for (int i = 0; i < obstacle_spheres.size(); i++) {
    obstacle_spheres[i].model.translation = constraint.spherical_constraints[i].center; 
    draw(obstacle_spheres[i], environment);
  }

  // Update cylinder centers and draw
  //for (int i = 0; i < obstacle_cylinders.size(); i++) {
  //  draw(obstacle_cylinders[i], environment); 
  //}

  // Simulation of the cloth
	// ***************************************** //
	int const N_step = 1; // Adapt here the number of intermediate simulation steps (ex. 5 intermediate steps per frame)
	for (int k_step = 0; k_step < N_step; ++k_step)
	{
		// Update the forces on each particle
		simulation_compute_force(cloth, parameters);

		// One step of numerical integration
		simulation_numerical_integration(cloth, parameters, parameters.dt);

		// Apply the positional (and velocity) constraints
		simulation_apply_constraints(cloth, constraint);

		// Check if the simulation has not diverged - otherwise stop it
		bool const simulation_diverged = simulation_detect_divergence(cloth);
		if (simulation_diverged) {
			std::cout << "\n *** Simulation has diverged ***" << std::endl;
			std::cout << " > The simulation is stoped" << std::endl;
		}
	}


	// Cloth display
	// ***************************************** //

	// Prepare to display the updated cloth
	cloth.update_normal();        // compute the new normals
	cloth_drawable.update(cloth); // update the positions on the GPU

	// Display the cloth
	draw(cloth_drawable, environment);

	// ************************************** //
	// Display the surface and the skeletons
	// ************************************** //
	for(auto& entry_character : characters) {
		character_structure& character = entry_character.second;
		animated_model_structure& animated_model = entry_character.second.animated_model;

		// Display meshes
		for(auto& rigged_mesh_entry : animated_model.rigged_mesh) {
			std::string mesh_name = rigged_mesh_entry.first;
			rigged_mesh_structure& rigged_mesh = rigged_mesh_entry.second;
			
			mesh_drawable& drawable = character.drawable[mesh_name];
			drawable.vbo_position.update(rigged_mesh.mesh_deformed.position);
			drawable.vbo_normal.update(rigged_mesh.mesh_deformed.normal);

			if(gui.display_surface) {
				drawable.material.texture_settings.active = gui.display_texture;
				draw(drawable, environment);
			}
			if(gui.display_wireframe) {
				draw_wireframe(drawable, environment);
			}
		}

		// Display skeleton
		if(gui.display_skeleton) {
			character.sk_drawable.update(animated_model.skeleton);
			character.sk_drawable.display_joint_frame = gui.display_skeleton_joint_frame;
			character.sk_drawable.display_joint_sphere = gui.display_skeleton_joint_sphere;
			character.sk_drawable.display_segments = gui.display_skeleton_bone;
			draw(character.sk_drawable, environment);
		}
	}

}


void scene_structure::display_gui()
{
	// General display functions
	ImGui::Checkbox("Global Frame", &gui.display_frame); ImGui::SameLine();
	ImGui::Checkbox("Ground", &gui.display_ground);
	ImGui::Checkbox("Wireframe", &gui.display_wireframe); 
	ImGui::Checkbox("Display surface", &gui.display_surface); ImGui::SameLine();
	ImGui::Checkbox("Texture", &gui.display_texture);
	ImGui::Checkbox("Skeleton", &gui.display_skeleton);

	ImGui::Indent();
	ImGui::Checkbox("Bone", &gui.display_skeleton_bone); ImGui::SameLine();
	ImGui::Checkbox("Joint", &gui.display_skeleton_joint_sphere); ImGui::SameLine();
	ImGui::Checkbox("Frame", &gui.display_skeleton_joint_frame);
	ImGui::Unindent();

	ImGui::Spacing(); ImGui::Spacing();

	// Effects
	ImGui::Spacing(); ImGui::Separator(); 
	ImGui::Text("Effects: "); 
	ImGui::Indent();
	bool is_walk_clicked = ImGui::Checkbox("Walk", &effect_walk.active); ImGui::SameLine();
	ImGui::Checkbox("Head", &gui.rotate_head_effect_active); ImGui::SameLine();
	ImGui::Unindent();


	// Handle start of walk in setting the current animation to Idle
	if(is_walk_clicked && effect_walk.active){
		effect_transition[current_active_character].transition_time = 0.2f;
		characters[current_active_character].set_current_animation("Idle");
		effect_walk.root_position = vec3(0,0,0);
	}

	ImGui::Spacing(); ImGui::Spacing();

	ImGui::Text("Current character: "); ImGui::SameLine();
	ImGui::Text(current_active_character.c_str(), "%s");

	// Display info for all characters
	for(auto& entry : characters) {

		ImGui::Spacing(); ImGui::Spacing(); ImGui::Separator(); 

		std::string name = entry.first;
		auto& character = entry.second;
	
		// Name of the character - special color for the active one
		ImVec4 current_color = name==current_active_character? ImVec4(1.0f, 0.0f, 0.0f, 0.5f):ImVec4(0.5f, 0.5f, 1.0f, 0.3f);
		ImGui::PushStyleColor(ImGuiCol_Button, current_color);
		if( ImGui::Button(name.c_str()) ) {
			current_active_character = name;
			// Update the values
			if(effect_walk.active) {
				effect_transition[current_active_character].transition_time = 0.2f;
				characters[current_active_character].set_current_animation("Idle");
				effect_walk.root_position = vec3(0,0,0);
				effect_walk.root_angle = 0.0f;
			}
		}
		ImGui::PopStyleColor();

		// Timers associated to the character
		ImGui::Indent();
		std::string time_scale_txt = "Time scale##"+name;
		std::string transition_time_txt = "Transition time##"+name;
		ImGui::SliderFloat(time_scale_txt.c_str(), &character.timer.scale, 0.0f, 2.0f);
		ImGui::SliderFloat(transition_time_txt.c_str(), &effect_transition[name].transition_time, 0.1f, 5.0f);

		std::string local_time_txt = "Anim cycle time##"+name;
		ImGui::SliderFloat(local_time_txt.c_str(), &character.timer.t_periodic, 0.0f, character.timer.event_period);
				
		// List all possible animations
		for(auto& entry_anim : character.animated_model.animation) {
			std::string animation_name = entry_anim.first;
			std::string animation_name_button_txt = animation_name+"##"+name;

			bool is_active_animation = animation_name==character.current_animation_name;
			ImVec4 current_color = is_active_animation? ImVec4(1.0f, 0.0f, 0.0f, 0.5f):ImVec4(0.5f, 0.5f, 1.0f, 0.3f);
			ImGui::PushStyleColor(ImGuiCol_Button, current_color);
			bool click_anim = ImGui::Button(animation_name_button_txt.c_str()); ImGui::SameLine();
			ImGui::PopStyleColor();

			// If we click on an animation: start a transition toward the new animation
			if(click_anim) {
				if(effect_transition[name].active) {
					std::cout<<"Character "<<name<<" did not finish his previous transition "<<animation_name<<std::endl;
				}
				effect_transition_start(effect_transition[name], character, animation_name);
			}
		}
		ImGui::NewLine();
		ImGui::Unindent();
		
	}

	ImGui::Spacing(); ImGui::Spacing();

	ImGui::Text("Simulation parameters");
	ImGui::SliderFloat("Time step", &parameters.dt, 0.0001f, 0.02f, "%.4f", 2.0f);
	ImGui::SliderFloat("Stiffness", &parameters.K, 0.2f, 50.0f, "%.3f", 2.0f);
	ImGui::SliderFloat("Wind magnitude", &parameters.wind.magnitude, 0, 60, "%.3f", 2.0f);
	ImGui::SliderFloat("Damping", &parameters.mu, 1.0f, 30.0f);
	ImGui::SliderFloat("Mass", &parameters.mass_total, 0.2f, 5.0f, "%.3f", 2.0f);

	ImGui::Spacing(); ImGui::Spacing();

	ImGui::SliderInt("Cloth samples", &gui.N_sample_edge, 4, 80);

}

void scene_structure::mouse_move_event()
{
	if (!inputs.keyboard.shift)
		camera_control.action_mouse_move(environment.camera_view);
}
void scene_structure::mouse_click_event()
{
	camera_control.action_mouse_click(environment.camera_view);
}
void scene_structure::keyboard_event()
{
	camera_control.action_keyboard(environment.camera_view);

	if(effect_walk.active){
		effect_walking_keyboard_event(effect_transition[current_active_character], characters[current_active_character], inputs, effect_walk);
	}
}
void scene_structure::idle_frame()
{
	camera_control.idle_frame(environment.camera_view);
}

// Compute a new cloth in its initial position (can be called multiple times)
void scene_structure::initialize_cloth(int N_sample)
{
	cloth.initialize(N_sample);
	cloth_drawable.initialize(N_sample);
	cloth_drawable.drawable.texture = cloth_texture;
	cloth_drawable.drawable.material.texture_settings.two_sided = true;

	constraint.fixed_sample.clear();
  //constraint.add_fixed_position(0, 0, cloth);
  //constraint.add_fixed_position(0, N_sample - 1, cloth);
}

void initialize_ground(mesh_drawable& ground) {
	mesh ground_mesh = mesh_primitive_quadrangle();
	ground_mesh.translate({-0.5f,-0.5f,0.0f});
	ground_mesh.rotate({1,0,0},-Pi/2.0f);
	ground_mesh.translate({0.0f,0.0f,-0.05f});
	ground_mesh.scale(15.0f);
	ground_mesh.uv *= 5.0f;
	
	ground.initialize_data_on_gpu(ground_mesh);
	ground.texture.load_and_initialize_texture_2d_on_gpu(project::path+"assets/texture_wood.jpg",GL_REPEAT,GL_REPEAT);
	ground.material.phong = {1,0,0,1};

}
