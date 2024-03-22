#include "effects.hpp"

using namespace cgp;

// Update a transition_structure before starting a new transition
void effect_transition_start(effect_transition_structure& effect_transition, character_structure& character, std::string const& destination_anim)
{
	effect_transition.source_anim = character.current_animation_name;
	effect_transition.timer_source_anim = character.timer;

	effect_transition.destination_anim = destination_anim;
	effect_transition.timer_destination_anim.event_period = character.animated_model.animation.at(destination_anim).time_max;
	effect_transition.timer_destination_anim.t_periodic=0;
	effect_transition.timer_destination_anim.start();

	effect_transition.timer_completion.t=0;
	effect_transition.timer_completion.scale = 1.0f/effect_transition.transition_time; // force the timer to go from 0 to 1 during the transition_time
	effect_transition.timer_completion.start();
	effect_transition.active = true;

	character.current_animation_name = destination_anim;
}

// Check if a transition need to be de-activated if it is fully completed
void effect_transition_stop_if_completed(effect_transition_structure& transition, character_structure& character) {
	if(transition.timer_completion.t >= 1.0f) {
		character.set_current_animation(transition.destination_anim);
		character.timer = transition.timer_destination_anim;
		transition.active = false;
	}
}

quaternion nlerp(quaternion q_0, quaternion q_1, float alpha) 
{
	quaternion result = quaternion(0, 0, 0, 1);
	float dot = q_0.w * q_1.w + q_0.x * q_1.x + q_0.y * q_1.y + q_0.z * q_1.z;
	float alpha_inverse = 1.0f - alpha;

	if (dot < 0) {
		result.w = alpha_inverse * q_0.w + alpha * -q_1.w;
		result.x = alpha_inverse * q_0.x + alpha * -q_1.x;
		result.y = alpha_inverse * q_0.y + alpha * -q_1.y;
		result.z = alpha_inverse * q_0.z + alpha * -q_1.z;
	} else {
		result.w = alpha_inverse * q_0.w + alpha * q_1.w;
		result.x = alpha_inverse * q_0.x + alpha * q_1.x;
		result.y = alpha_inverse * q_0.y + alpha * q_1.y;
		result.z = alpha_inverse * q_0.z + alpha * q_1.z;
	}

	result /= sqrt(pow(result.x, 2) + pow(result.y, 2) + pow(result.z, 2) + pow(result.w, 2));
	return result;
}



void effect_transition_compute(effect_transition_structure& effect_transition, character_structure& character)
{
	effect_transition_structure& transition = effect_transition;
	animated_model_structure& model = character.animated_model;

	// Compute the skeleton from the source animation
	model.set_skeleton_from_animation(transition.source_anim, transition.timer_source_anim.t_periodic);
	numarray<mat4> joint_source_anim_local = model.skeleton.joint_matrix_local;
	numarray<mat4> joint_source_anim_global = model.skeleton.joint_matrix_local;

	// Compute the skeleton from the destination animation
	model.set_skeleton_from_animation(transition.destination_anim, transition.timer_destination_anim.t_periodic);
	numarray<mat4> joint_destination_anim_local = model.skeleton.joint_matrix_local;
	numarray<mat4> joint_destination_anim_global = model.skeleton.joint_matrix_global;

	float alpha_completion = transition.timer_completion.t;

	numarray<mat4> joint_source_anim = joint_source_anim_local;
	numarray<mat4> joint_destination_anim = joint_destination_anim_local;

	numarray<mat4> joint_interpolated = joint_source_anim;
	for(int k=0; k<joint_interpolated.size(); ++k){
		joint_interpolated[k] = mat4_interpolate_quaternion(joint_source_anim[k], joint_destination_anim[k], alpha_completion);
		// joint_interpolated[k] = (1-alpha_completion)*joint_current_anim[k] + alpha_completion*joint_next_anim[k];
	}
	model.skeleton.joint_matrix_local = joint_interpolated;
	model.skeleton.update_joint_matrix_local_to_global();

	// Update the timers
	transition.timer_source_anim.update();
	transition.timer_destination_anim.update();
	transition.timer_completion.update();
	character.timer = transition.timer_destination_anim;
}




// This function implements the change of animation when we press or release the UP key during a walk effect
void effect_walking_keyboard_event(effect_transition_structure& effect_transition, character_structure& character, cgp::input_devices const& inputs, effect_walking_structure const& effect_walking)
{
	// If we just press the key up (or W), start the Walk cycle animation
	if(inputs.keyboard.last_action.is_pressed(GLFW_KEY_UP) || inputs.keyboard.last_action.is_pressed(GLFW_KEY_W)) {
		effect_transition_start(effect_transition, character, effect_walking.walk_anim_name);
	}
	// If we just release the key up (or W), start the Idle cycle animation
	if(inputs.keyboard.last_action.is_released(GLFW_KEY_UP) || inputs.keyboard.last_action.is_released(GLFW_KEY_W)) {
		effect_transition_start(effect_transition, character, effect_walking.idle_anim_name);
	}

}


// This function implements the change of position of the character when a directional key is pressed
// This function is called at every frame when the walk effect is active
void effect_walking(effect_walking_structure& effect_walking,  character_structure& character, cgp::input_devices const& inputs, effect_transition_structure const& effect_transition)
{
  float dt = effect_walking.timer.update();
	float current_time = character.timer.t_periodic;
	float previous_time = character.timer.t_periodic - dt;

	// The initial bind matrix in case the character is initially rotated
	mat4 B0 = character.animated_model.animation[effect_walking.walk_anim_name].evaluate(0,0);

	// Take into account the real translation speed of the walk cycle (not obligatory)
	float real_speed = norm(character.animated_model.animation[effect_walking.walk_anim_name].evaluate(0,previous_time).get_block_translation() - character.animated_model.animation[effect_walking.walk_anim_name].evaluate(0,current_time).get_block_translation());
	
	if(inputs.keyboard.is_pressed(GLFW_KEY_UP) || inputs.keyboard.is_pressed(GLFW_KEY_W)) {
		effect_walking.root_position += rotation_axis_angle({0,1,0},effect_walking.root_angle)  * vec3(0.0f,0.0f,1.0f) * real_speed;
	}
	if(inputs.keyboard.is_pressed(GLFW_KEY_LEFT) || inputs.keyboard.is_pressed(GLFW_KEY_A)) {
		effect_walking.root_angle += 3.0f*dt;
	}
	if(inputs.keyboard.is_pressed(GLFW_KEY_RIGHT) || inputs.keyboard.is_pressed(GLFW_KEY_D)) {
		effect_walking.root_angle -= 3.0f*dt;
	}


	mat4& root = character.animated_model.skeleton.joint_matrix_local[0];
	mat3 R = rotation_axis_angle({0,1,0},effect_walking.root_angle).matrix();

	root.set_block_translation( vec3(root.get_block_translation().xy(),0.0f)+effect_walking.root_position );
	root.set_block_linear(B0.get_block_linear() * R * root.get_block_linear());


	character.animated_model.skeleton.update_joint_matrix_local_to_global();
}

mat4 mat4_interpolate_quaternion(mat4 const& M1, mat4 const& M2, float t) {
	affine_rt a1 = affine_rt::from_matrix(M1);
	affine_rt a2 = affine_rt::from_matrix(M2);
	affine_rt a; 
	a.rotation = rotation_transform::lerp(a1.rotation,a2.rotation,t);
	a.translation = (1-t)*a1.translation+t*a2.translation;

	return a.matrix();
}
