from moveit_msgs.msg import PickupGoal, PlaceGoal

def create_pickup_goal(possible_grasps=[], 
		       target='part', 
			   group='arm_torso', 
			   links_to_allow_contact=['gripper_left_finger_link', 'gripper_right_finger_link']):
	'''Create a PickupGoal with the provided data'''
	goal = PickupGoal()
	goal.target_name = target
	goal.group_name = group
	goal.possible_grasps.extend(possible_grasps)
	goal.allowed_planning_time = 35.0
	goal.planning_options.planning_scene_diff.is_diff = True
	goal.planning_options.planning_scene_diff.robot_state.is_diff = True
	goal.planning_options.plan_only = False
	goal.planning_options.replan = True
	goal.planning_options.replan_attempts = 1 
	goal.allowed_touch_objects = []
	goal.attached_object_touch_links = ['<octomap>']
	goal.attached_object_touch_links.extend(links_to_allow_contact)

	return goal


def create_place_goal(place_locations, 
		      target='part', 
			  group='arm_torso', 
			  links_to_allow_contact=['gripper_left_finger_link', 'gripper_right_finger_link']):
	'''Create PlaceGoal with the provided data'''
	goal = PlaceGoal()
	goal.group_name = group
	goal.attached_object_name = target
	goal.place_locations = place_locations
	goal.allowed_planning_time = 15.0
	goal.planning_options.planning_scene_diff.is_diff = True
	goal.planning_options.planning_scene_diff.robot_state.is_diff = True
	goal.planning_options.plan_only = False
	goal.planning_options.replan = True
	goal.planning_options.replan_attempts = 1
	goal.allowed_touch_objects = ['<octomap>']
	goal.allowed_touch_objects.extend(links_to_allow_contact)

	return goal