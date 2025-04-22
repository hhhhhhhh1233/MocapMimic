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

def getSelectedBodyTrajectoryIds():
	trajectory_ids = qtm.data.object.trajectory.get_trajectory_ids()
	rigid_body_trajectory_ids = []
	selections = qtm.gui.selection.get_selections("trajectory")
    
	if len(selections) == 0:
		return []
    
	selected_rigid_body_id = qtm.data.object.trajectory.get_rigid_body_id(selections[0]["id"])

	for selection in selections:
		rigid_body_id = qtm.data.object.trajectory.get_rigid_body_id(selection["id"])
		if rigid_body_id != selected_rigid_body_id:
			qtm.gui.message.add_message("Mocap Mimic: Multiple rigid bodies selected", "Only one rigid body should be selected at a time", "error")
			return []
	
	for trajectory_id in trajectory_ids:
		trajectory_id_rigid_body_id = qtm.data.object.trajectory.get_rigid_body_id(trajectory_id)
		if trajectory_id_rigid_body_id == selected_rigid_body_id:
			rigid_body_trajectory_ids.append(trajectory_id)
			
	return rigid_body_trajectory_ids
	
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
	
reference_file_name = f"{qtm.settings.directory.get_project_directory()}MocapMimicReference.json"

def saveSelectedAsReference():
	rigid_body_trajectory_ids = getSelectedBodyTrajectoryIds()
	selected_range = qtm.gui.timeline.get_selected_range()
	rigid_body_trajectories = {}
	
	for trajectory_id in rigid_body_trajectory_ids:
		trajectory_label = qtm.data.object.trajectory.get_label(trajectory_id)
		trajectory_points = _3d.get_samples(trajectory_id, selected_range)
		rigid_body_trajectories.update({trajectory_label: trajectory_points})
	
	with open(reference_file_name, "w") as file:
		json.dump(rigid_body_trajectories, file)
		
def getReferenceFromFile():
	with open(reference_file_name, "r") as file:
		rigid_body_trajectories = json.load(file)    
		return rigid_body_trajectories

# ----------------------------------------
# [END] SAVING AND LOADING
# ----------------------------------------

# ----------------------------------------
# [BEGIN] COMPARING TRAJECTORIES
# ----------------------------------------

def compareSelectedAgainstReference():
	selected_trajectories = getTrajectoriesFormatted(getSelectedBodyTrajectoryIds())
	reference_trajectories = getReferenceFromFile()
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

# ----------------------------------------
# [END] COMPARING TRAJECTORIES
# ----------------------------------------

# ----------------------------------------
# [BEGIN] ADDING MENU ITEMS
# ----------------------------------------

# Root menu option
menu_name = "Mocap Mimic"
my_menu_handle = qtm.gui.insert_menu_submenu(None, menu_name)

# Setting up save function
save_reference_function_name = "mocap_mimic_save_reference"
qtm.gui.add_command(save_reference_function_name)
qtm.gui.set_command_execute_function(save_reference_function_name, saveSelectedAsReference)

# Adding it to the menu
qtm.gui.insert_menu_button(my_menu_handle, "Save Reference", save_reference_function_name)

# Setting up the compare function
selected_to_reference_name = "mocap_mimic_compare_selected_to_reference"
qtm.gui.add_command(selected_to_reference_name)
qtm.gui.set_command_execute_function(selected_to_reference_name, compareSelectedAgainstReference)

# Adding it to the menu
qtm.gui.insert_menu_button(my_menu_handle, "Compare to Reference", selected_to_reference_name)

# ----------------------------------------
# [END] ADDING MENU ITEMS
# ----------------------------------------