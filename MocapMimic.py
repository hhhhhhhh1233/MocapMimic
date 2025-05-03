import qtm
from qtm.data.series import _3d
from qtm.data.object import trajectory
import math
import json

# ----------------------------------------
# [BEGIN] UTILS
# ----------------------------------------

# ----------------------------------------
# [BEGIN] VECTORS
# ----------------------------------------

def getLength(vec):
	sum = 0
	for i in range(len(vec)):
		sum += vec[i] * vec[i]
	return math.sqrt(sum)

def dotProduct(Vec1, Vec2):
	sum = 0
	for i in range(len(Vec1)):
		sum += Vec1[i] * Vec2[i]
	return sum

def getNormalized(vec):
	length = getLength(vec)
	if length == 0:
		return vec
	resultVec = vec[:]
	for i in range(len(vec)):
		resultVec[i] /= length
	return resultVec
	
def getDistance(vec1, vec2):
	diffVec = vec2[:]
	for i in range(len(diffVec)):
		diffVec[i] -= vec1[i]
	return getLength(diffVec)

def getDifference(lvec, rvec):
	resultVec = lvec[:]
	for i in range(len(lvec)):
		resultVec[i] -= rvec[i]
	return resultVec

# ----------------------------------------
# [END] VECTORS
# ----------------------------------------

# ----------------------------------------
# [BEGIN] TRAJECTORIES
# ----------------------------------------

def printSelected():
	trajectory_selections = qtm.gui.selection.get_selections("trajectory")
	bone_selections = qtm.gui.selection.get_selections("bone")

	print(f"Selected trajectories: {trajectory_selections}")
	print(f"Selected bones: {bone_selections}")

	skeleton_ids = qtm.data.object.skeleton.get_skeleton_ids()

	print(f"skeleton_ids: {skeleton_ids}")

	for trajectory in trajectory_selections:
		skeleton_segment_id = qtm.data.object.trajectory.get_skeleton_segment_id(trajectory["id"])
		print(f"skeleton_segment_id: {skeleton_segment_id}")

def getSelectedRigidBodyTrajectoryIDs():
	trajectory_ids = qtm.data.object.trajectory.get_trajectory_ids()
	rigid_body_trajectory_ids = []
	selections = qtm.gui.selection.get_selections("trajectory")
    
	if len(selections) == 0:
		return []
    
	selected_rigid_body_id = qtm.data.object.trajectory.get_rigid_body_id(selections[0]["id"])

	# Make sure that the user isn't selecting multiple rigid bodies at once
	for selection in selections:
		rigid_body_id = qtm.data.object.trajectory.get_rigid_body_id(selection["id"])
		if rigid_body_id != selected_rigid_body_id:
			qtm.gui.message.add_message("Mocap Mimic: Multiple rigid bodies selected", "Only one rigid body should be selected at a time", "error")
			return []
	
	# Filter all trajectories down to just those that are
	# associated with the rigid body the user selected
	for trajectory_id in trajectory_ids:
		trajectory_id_rigid_body_id = qtm.data.object.trajectory.get_rigid_body_id(trajectory_id)
		if trajectory_id_rigid_body_id == selected_rigid_body_id:
			rigid_body_trajectory_ids.append(trajectory_id)
			
	return rigid_body_trajectory_ids

def getSelectedSkeletonTrajectoryIDs():
	trajectory_ids = qtm.data.object.trajectory.get_trajectory_ids()
	skeleton_trajectory_ids = []
	selections = qtm.gui.selection.get_selections("trajectory")

	if len(selections) == 0:
		return []

	selected_segment_id = qtm.data.object.trajectory.get_skeleton_segment_id(selections[0]["id"])

	if selected_segment_id == None:
		qtm.gui.message.add_message("Mocap Mimic: Non-skeleton trajectory selected", "Not all selected trajectories are associated with a skeleton", "error")
		return []

	selected_skeleton_id = qtm.data.object.skeleton.get_segment_skeleton_id(selected_segment_id)

	# NOTE Make sure that all trajectories selected are associated with the same skeleton
	for selection in selections:
		segment_id = qtm.data.object.trajectory.get_skeleton_segment_id(selection["id"])
		skeleton_id = qtm.data.object.skeleton.get_segment_skeleton_id(segment_id)
		if skeleton_id == None:
			qtm.gui.message.add_message("Mocap Mimic: Non-skeleton trajectory selected", "Not all selected trajectories are associated with a skeleton", "error")
			return []

		if skeleton_id != selected_skeleton_id:
			qtm.gui.message.add_message("Mocap Mimic: Multiple skeletons selected", "Only one skeleton should be selected at a time", "error")
			return []

	# Filter all trajectories down to just those that are
	# associated with the skeleton the user selected
	for trajectory_id in trajectory_ids:
		trajectory_id_segment_id = qtm.data.object.trajectory.get_skeleton_segment_id(trajectory_id)
		if trajectory_id_segment_id != None:
			trajectory_id_skeleton_id = qtm.data.object.skeleton.get_segment_skeleton_id(trajectory_id_segment_id)
			if trajectory_id_skeleton_id == selected_skeleton_id:
				skeleton_trajectory_ids.append(trajectory_id)

	return skeleton_trajectory_ids
	
def getTrajectoriesFormatted(trajectory_ids):
	selected_range = qtm.gui.timeline.get_selected_range()
	rigid_body_trajectories = {}
	
	for trajectory_id in trajectory_ids:
		trajectory_label = qtm.data.object.trajectory.get_label(trajectory_id)
		trajectory_points = _3d.get_samples(trajectory_id, selected_range)
		rigid_body_trajectories.update({trajectory_label: trajectory_points})
	
	return rigid_body_trajectories

# ----------------------------------------
# [END] TRAJECTORIES
# ----------------------------------------

# ----------------------------------------
# [END] UTILS
# ----------------------------------------

# ----------------------------------------
# [BEGIN] SAVING AND LOADING
# ----------------------------------------
	
rigid_body_reference_file_name = f"{qtm.settings.directory.get_project_directory()}MocapMimicRigidBodyReference.json"
skeleton_reference_file_name = f"{qtm.settings.directory.get_project_directory()}MocapMimicSkeletonReference.json"

def saveSelectedRigidBodyAsReference():
	rigid_body_trajectory_ids = getSelectedRigidBodyTrajectoryIDs()
	selected_range = qtm.gui.timeline.get_selected_range()
	rigid_body_trajectories = {}
	
	for trajectory_id in rigid_body_trajectory_ids:
		trajectory_label = qtm.data.object.trajectory.get_label(trajectory_id)
		trajectory_points = _3d.get_samples(trajectory_id, selected_range)
		rigid_body_trajectories.update({trajectory_label: trajectory_points})
	
	with open(rigid_body_reference_file_name, "w") as file:
		json.dump(rigid_body_trajectories, file)
		
def saveSelectedSkeletonAsReference():
	skeleton_trajectory_ids = getSelectedSkeletonTrajectoryIDs()
	selected_range = qtm.gui.timeline.get_selected_range()
	skeleton_trajectories = {}
	
	for trajectory_id in skeleton_trajectory_ids:
		trajectory_label = qtm.data.object.trajectory.get_label(trajectory_id)
		trajectory_points = _3d.get_samples(trajectory_id, selected_range)
		skeleton_trajectories.update({trajectory_label: trajectory_points})
	
	with open(skeleton_reference_file_name, "w") as file:
		json.dump(skeleton_trajectories, file)
		
def getRigidBodyReferenceFromFile():
	with open(rigid_body_reference_file_name, "r") as file:
		rigid_body_trajectories = json.load(file)    
		return rigid_body_trajectories

def getSkeletonReferenceFromFile():
	with open(skeleton_reference_file_name, "r") as file:
		skeleton_trajectories = json.load(file)    
		return skeleton_trajectories

# ----------------------------------------
# [END] SAVING AND LOADING
# ----------------------------------------

# ----------------------------------------
# [BEGIN] COMPARING TRAJECTORIES
# ----------------------------------------

def compareSelectedRigidBodyAgainstReference():
	selected_trajectories = getTrajectoriesFormatted(getSelectedRigidBodyTrajectoryIDs())
	reference_trajectories = getRigidBodyReferenceFromFile()
	sumCorr = 0
	
	if len(selected_trajectories) == 0 or len(selected_trajectories) == 0:
		qtm.gui.message.add_message("Mocap Mimic: No rigid bodies selected", "Must select a rigid body to deal with", "error")
		return

	if len(selected_trajectories) != len(reference_trajectories):
		qtm.gui.message.add_message("Mocap Mimic: Reference capture and current capture are different sizes", "The reference capture saved to file has a different number of labels than the currently selected capture", "error")
		return
	
	# Iterate over all labels in the rigid body
	for label, points in reference_trajectories.items():
		# Iterate over each sample collected per label
		for i in range(len(points) - 1):
			if points[i + 1] == None or points[i] == None:
				continue
			if selected_trajectories[label][i + 1] == None or selected_trajectories[label][i] == None:
				continue

			RefDelta = getDifference(points[i + 1]["position"], points[i]["position"])
			SelDelta = getDifference(selected_trajectories[label][i + 1]["position"], selected_trajectories[label][i]["position"])

			corr = dotProduct(getNormalized(RefDelta), getNormalized(SelDelta))
			sumCorr += max(0, corr)

			if corr <= 0:
				print(f"Reference: {reference_trajectories[i][1][i + 1]} and {reference_trajectories[i][1][i]}")
				print(f"Selected: {selected_trajectories[i][1][i + 1]} and {selected_trajectories[i][1][i]}")

	numberOfLables = len(reference_trajectories)
	numberOfSamples = len(reference_trajectories[list(reference_trajectories.keys())[0]])
	accuracy = sumCorr / (numberOfLables * numberOfSamples)

	qtm.gui.message.add_message(f"Mocap Mimic: Overall accuracy: {accuracy * 100:.2f}%", "", "info")
	print(f"Overall accuracy: {accuracy * 100:.2f}%")

def compareSelectedSkeletonAgainstReference():
	skeleton_trajectory_ids = getSelectedSkeletonTrajectoryIDs()
	print(f"All selected trajectories: {skeleton_trajectory_ids}")
	return

# ----------------------------------------
# [END] COMPARING TRAJECTORIES
# ----------------------------------------

# ----------------------------------------
# [BEGIN] ADDING MENU ITEMS
# ----------------------------------------

# Root menu option
mocap_mimic_menu_name = "Mocap Mimic"
mocap_mimic_menu_handle = qtm.gui.insert_menu_submenu(None, mocap_mimic_menu_name)

rigid_body_submenu_name = "Rigid Body"
rigid_body_submenu_handle = qtm.gui.insert_menu_submenu(mocap_mimic_menu_handle, rigid_body_submenu_name)

skeleton_submenu_name = "Skeleton"
skeleton_submenu_handle = qtm.gui.insert_menu_submenu(mocap_mimic_menu_handle, skeleton_submenu_name)

# Setting up save function
rigid_body_save_reference_function_name = "mocap_mimic_rigid_body_save_reference"
qtm.gui.add_command(rigid_body_save_reference_function_name)
qtm.gui.set_command_execute_function(rigid_body_save_reference_function_name, saveSelectedRigidBodyAsReference)
qtm.gui.insert_menu_button(rigid_body_submenu_handle, "Save Reference", rigid_body_save_reference_function_name)

# Setting up the compare function
rigid_body_compare_selected_to_reference = "mocap_mimic_rigid_body_compare_selected_to_reference"
qtm.gui.add_command(rigid_body_compare_selected_to_reference)
qtm.gui.set_command_execute_function(rigid_body_compare_selected_to_reference, compareSelectedRigidBodyAgainstReference)
qtm.gui.insert_menu_button(rigid_body_submenu_handle, "Compare to Reference", rigid_body_compare_selected_to_reference)

# Setting up save function
skeleton_save_reference_function_name = "mocap_mimic_skeleton_save_reference"
qtm.gui.add_command(skeleton_save_reference_function_name)
qtm.gui.set_command_execute_function(skeleton_save_reference_function_name, saveSelectedSkeletonAsReference)
qtm.gui.insert_menu_button(skeleton_submenu_handle, "Save Reference", skeleton_save_reference_function_name)

# Setting up the compare function
skeleton_compare_selected_to_reference = "mocap_mimic_skeleton_compare_selected_to_reference"
qtm.gui.add_command(skeleton_compare_selected_to_reference)
qtm.gui.set_command_execute_function(skeleton_compare_selected_to_reference, compareSelectedSkeletonAgainstReference)
qtm.gui.insert_menu_button(skeleton_submenu_handle, "Compare to Reference", skeleton_compare_selected_to_reference)

# Setting up the compare function
print_selected_name = "mocap_mimic_print_selected"
qtm.gui.add_command(print_selected_name)
qtm.gui.set_command_execute_function(print_selected_name, printSelected)
qtm.gui.insert_menu_button(mocap_mimic_menu_handle, "Print Selections", print_selected_name)

# ----------------------------------------
# [END] ADDING MENU ITEMS
# ----------------------------------------