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
	numarray<mat4> joint_source_anim_global = model.skeleton.joint_matrix_global;

	// Compute the skeleton from the destination animation
	model.set_skeleton_from_animation(transition.destination_anim, transition.timer_destination_anim.t_periodic);
	numarray<mat4> joint_destination_anim_local = model.skeleton.joint_matrix_local;
	numarray<mat4> joint_destination_anim_global = model.skeleton.joint_matrix_global;

	float alpha_completion = transition.timer_completion.t;

	/** TO DO : Compute a smooth transition between two animation 
	 *  The objective is to fill "model.skeleton.joint_matrix_local and global"
	 *     with the correct interpolated matrices corresponding to the blending between the source and destination animation 
	 * 
	 * 	
	 * 
	 *  Notes:
	 *    - model.skeleton.joint_matrix_local is a numarray<mat4>. Its size corresponds to the number of joints.
	 *    - joint_source_anim contains the (local) matrices of the source animation
	 *    - joint_destination_anim contains the (local) matrices of the destination animation
	 * 	  - alpha_completion contains the ratio of completion.
	 *       e.g.: alpha_completion=0 => we have the source anim
	 *             alpha_completion=1 => we have the destination anim
	 *             alpha_completion=0.5 => we have "0.5 source anim + 0.5 destination anim"
	 *    - if model.skeleton.joint_matrix_local is filled, you can update the global matrices using "model.skeleton.update_joint_matrix_local_to_global();"
	 *    - if model.skeleton.joint_matrix_global is filled, you can update the global matrices using "model.skeleton.update_joint_matrix_global_to_local();"
	 * 
	 *  Help:
	 *    - You can convert a matrix M to its translation t and rotation r (or quaternion q) representation using the following syntax:
	 *      affine_rt a = affine_rt::from_matrix(M);
	 *      vec3 t = a.translation;
	 *      rotation_transform r = a.rotation;
	 *      quaternion q = r.get_quaternion();
	 *    - An affine_rt structure is a container for a rotation_transform and a translation. Its corresponding 4x4 matrix can be obtained
	 *        with {affine_rt}.matrix();
	 * 
	 * 
	 */

	for (int k = 0; k < joint_source_anim_local.size(); k++) {
		mat4 M_src = joint_source_anim_local[k];
		mat4 M_dst = joint_destination_anim_local[k];

		affine_rt a_src = affine_rt::from_matrix(M_src);
		affine_rt a_dst = affine_rt::from_matrix(M_dst);
		
		rotation_transform r_src = a_src.rotation;
		quaternion q_src = r_src.quat();

		rotation_transform r_dst = a_dst.rotation;
		quaternion q_dst = r_dst.quat();

		// Implemented nlerp for quaternion intrepolation
		quaternion q_interpolate = nlerp(q_src, q_dst, alpha_completion);
		mat3 R = rotation_transform::convert_quaternion_to_matrix(q_interpolate);

		mat4 M_new = mat4(
        vec4(R[0], 0.0f),
        vec4(R[1], 0.0f),
        vec4(R[2], 0.0f),
        vec4(0.0f, 0.0f, 0.0f, 1.0f)
    );

		// Adds the tranlstion interpolation part with the quaternion interpolation
		mat4 T = M_src * (1 - alpha_completion) + M_dst * alpha_completion;
		M_new.set_block_translation(T.get_block_translation());
		
		model.skeleton.joint_matrix_local[k] = M_new;
	}

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

	/** TO DO : An displacement of the character along the direction of the walk.
	 *   The character should be able to move straight, and turn, based on the user command.
	 *  
	 *   Inputs: 
	 *      - The effect_walking structure storing the current position and angle of the character (root joint position and angle)
	 *      - The skeleton to be modified by the walk effect
	 *      - The current state of the keyboard input (e.g. inputs.keyboard.is_pressed(...))
	 *   Output: 
	 * 		- A modified effect_walking structure and skeleton corresponding to the expected displacement
	 * 		  
	 * 
	 *   Help:
	 *      - The following syntax allows to check if a specific key is currently pressed:
	 *        if( inputs.keyboard.is_pressed({KeyName}) ) {...} , with
	 * 		   {KeyName} being: GLFW_KEY_UP, GLFW_KEY_RIGHT, GLFW_KEY_LEFT, GLFW_KEY_DOWN for the arrows
	 *                      or: GLFW_KEY_W, GLFW_KEY_A, GLFW_KEY_D, GLFW_KEY_S using for instance the letters
	 *      - Given a mat4 structure representing an affine transformation, you can apply the following block operations
	 *        - {mat4}.get/set_block_translation({vec3}) : get/set the vec3 block corresponding to the translation part
	 *        - {mat4}.get/set_block_linear({mat3}) : get/set the mat3 block corresponding to the linear part
	 *      - A rotation with a given axis and angle can be created with the syntax
	 *         rotation_transform r = rotation_axis_angle({vec3 axis}, {float angle});
	 *         The associated 3x3 matrix can be obtained using r.matrix();  
	 *      - An internal timer is available in effect_walking.timer
	 *        The timer can be updated via effect_walking.timer.update() and the elapsed time since the last update is returned.
	 *        This can be used to enforce a constant speed independently of the frame rate
	 * 
	 * 
	 * 	 Algorithm:     
	 * 		If(press UP)
	 *        Advance straight and update effect_walking.root_position
	 *      If(press Left/Right)
	 *        Rotate and update effect_walking.root_angle
	 *      Update the root joint of the skeleton
	 * 
	 * 
	 */
	animated_model_structure& model = character.animated_model;
	numarray<mat4> joint_source_anim_local = model.skeleton.joint_matrix_local;

	effect_walking.root_position.y = joint_source_anim_local[0].get_block_translation().y;

	float rotation_speed = 5 * (Pi / 180);
	float velocity = 0.05;

	if (inputs.keyboard.is_pressed(GLFW_KEY_W)) {
		float angle = effect_walking.root_angle;
		vec3 v({sin(angle), 0.0f, cos(angle)});

		effect_walking.root_position += v * velocity;
	}

	if (inputs.keyboard.is_pressed(GLFW_KEY_A)) {
		effect_walking.root_angle += rotation_speed;
	}

	if (inputs.keyboard.is_pressed(GLFW_KEY_D)) {
		effect_walking.root_angle -= rotation_speed;
	}

	character.animated_model.skeleton.joint_matrix_local[0].set_block_linear_as_rotation(vec3(0., 1., 0.), effect_walking.root_angle);
	character.animated_model.skeleton.joint_matrix_local[0].set_block_translation(effect_walking.root_position);	

	character.animated_model.skeleton.update_joint_matrix_local_to_global();
}


