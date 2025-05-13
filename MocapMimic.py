import qtm
from qtm.data.series import _3d
from qtm.data.object import trajectory
import math
import json
import copy

# ----------------------------------------
# [BEGIN] UTILS
# ----------------------------------------

# strings = ["QA_hips", "QA_wrist", "QA_elbow"]
def getPrefix(strings):
	common = 99999999
	for i in range(len(strings) - 1):
		for j in range(min(len(strings[i]), len(strings[i + 1]))):
			if strings[i][j] != strings[i + 1][j]:
				if j < common:
					common = j
					break

	return strings[0][:common]

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
# [BEGIN] MATRICES
# ----------------------------------------

def print4x4Matrix(mat):
	print(f"\n{mat[0][0]:.2f}, {mat[0][1]:.2f}, {mat[0][2]:.2f}, {mat[0][3]:.2f}\n{mat[1][0]:.2f}, {mat[1][1]:.2f}, {mat[1][2]:.2f}, {mat[1][3]:.2f}\n{mat[2][0]:.2f}, {mat[2][1]:.2f}, {mat[2][2]:.2f}, {mat[2][3]:.2f}\n{mat[3][0]:.2f}, {mat[3][1]:.2f}, {mat[3][2]:.2f}, {mat[3][3]:.2f}")

# NOTE Has not been properly tested for accuracy
# Also it just works for matrices of that are 4x4
def multiplyMatrices(lmat, rmat):
	mat = copy.deepcopy(lmat)

	tempSum =  0
	for k in range(4):
		for i in range(4):
			for j in range(4):
				tempSum += lmat[j][k] * rmat[i][j]
			mat[i][k] = tempSum
			tempSum = 0

	return mat

def multiplyVectorMatrix(vec, mat):
	resultVec = []
	for i in range(len(vec)):
		sum = 0
		for j in range(len(vec)):
			sum += vec[j] * mat[j][i]
		resultVec.append(sum)
	
	return resultVec

	# return vec4(rhs.x * m[0][0] + rhs.y * m[1][0] + rhs.z * m[2][0] + rhs.w * m[3][0] 
	# 			rhs.x * m[0][1] + rhs.y * m[1][1] + rhs.z * m[2][1] + rhs.w * m[3][1]
	# 			rhs.x * m[0][2] + rhs.y * m[1][2] + rhs.z * m[2][2] + rhs.w * m[3][2] 
	# 			rhs.x * m[0][3] + rhs.y * m[1][3] + rhs.z * m[2][3] + rhs.w * m[3][3]


# ----------------------------------------
# [END] MATRICES
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

def getSkeletonSeriesIDs():
	return
	seriesIDs = qtm.data.series.skeleton.get_series_ids()

# NOTE This is heavily based on the above functions for getting the trajectories, it just stops sooner
# It could probably be improved but I will not make it a priority
def getSelectedSkeletonID():
	selections = qtm.gui.selection.get_selections("trajectory")

	if len(selections) == 0:
		return -1

	selected_segment_id = qtm.data.object.trajectory.get_skeleton_segment_id(selections[0]["id"])

	if selected_segment_id == None:
		qtm.gui.message.add_message("Mocap Mimic: Non-skeleton trajectory selected", "Not all selected trajectories are associated with a skeleton", "error")
		return -1

	selected_skeleton_id = qtm.data.object.skeleton.get_segment_skeleton_id(selected_segment_id)

	# NOTE Make sure that all trajectories selected are associated with the same skeleton
	for selection in selections:
		segment_id = qtm.data.object.trajectory.get_skeleton_segment_id(selection["id"])
		skeleton_id = qtm.data.object.skeleton.get_segment_skeleton_id(segment_id)
		if skeleton_id == None:
			qtm.gui.message.add_message("Mocap Mimic: Non-skeleton trajectory selected", "Not all selected trajectories are associated with a skeleton", "error")
			return -1

		if skeleton_id != selected_skeleton_id:
			qtm.gui.message.add_message("Mocap Mimic: Multiple skeletons selected", "Only one skeleton should be selected at a time", "error")
			return -1

	return skeleton_id
	
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
skeleton_reference_bones_file_name = f"{qtm.settings.directory.get_project_directory()}MocapMimicSkeletonBoneReference.json"

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
		
# TODO Have this also save the skeleton bone data to a seperate file
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

def compareTrajectories(base_trajectories, mimic_trajectories):
	sumCorr = 0
	
	if len(base_trajectories) != len(mimic_trajectories):
		qtm.gui.message.add_message("Mocap Mimic: Reference capture and current capture are different sizes", "The reference capture saved to file has a different number of labels than the currently selected capture, they are probably different types of objects", "error")
		return

	base_prefix = getPrefix(list(base_trajectories.keys()))
	mimic_prefix = getPrefix(list(mimic_trajectories.keys()))
	print(f"base_prefix: {base_prefix}\nmimic_prefix: {mimic_prefix}")
	
	# Iterate over all labels in the rigid body
	for label, points in base_trajectories.items():

		# NOTE This creates the matching label that the mimic trajectory ought to have
		# It assumes that the trajectories are identically named aside from their prefix
		mimic_label = mimic_prefix + label[len(base_prefix):]

		# Iterate over each sample collected per label
		for i in range(len(points) - 1):
			# Skip over this iteration if one of the data points is missing
			# TODO This can be fixed by filling in the gaps with an average
			if points[i + 1] == None or points[i] == None:
				continue
			if mimic_trajectories[mimic_label][i + 1] == None or mimic_trajectories[mimic_label][i] == None:
				continue

			RefDelta = getDifference(points[i + 1]["position"], points[i]["position"])
			SelDelta = getDifference(mimic_trajectories[mimic_label][i + 1]["position"], mimic_trajectories[mimic_label][i]["position"])

			corr = 0
			if getLength(RefDelta) == 0 and getLength(SelDelta) == 0:
				corr = 1
			else:
				corr = dotProduct(getNormalized(RefDelta), getNormalized(SelDelta))
			sumCorr += max(0, corr)

			# if corr <= 0:
			# 	print(f"Reference: {base_trajectories[i][1][i + 1]} and {base_trajectories[i][1][i]}")
			# 	print(f"Selected: {mimic_trajectories[i][1][i + 1]} and {mimic_trajectories[i][1][i]}")

	numberOfLables = len(base_trajectories)
	numberOfSamples = len(base_trajectories[list(base_trajectories.keys())[0]])
	accuracy = sumCorr / (numberOfLables * numberOfSamples)

	return accuracy

def compareSelectedRigidBodyAgainstReference():
	selected_trajectories = getTrajectoriesFormatted(getSelectedRigidBodyTrajectoryIDs())
	reference_trajectories = getRigidBodyReferenceFromFile()

	if len(selected_trajectories) == 0 or len(selected_trajectories) == 0:
		qtm.gui.message.add_message("Mocap Mimic: No rigid bodies selected", "Must select a rigid body to deal with", "error")
		return

	accuracy = compareTrajectories(reference_trajectories, selected_trajectories)

	qtm.gui.message.add_message(f"Mocap Mimic: Overall accuracy: {accuracy * 100:.2f}%", "", "info")
	print(f"Overall accuracy: {accuracy * 100:.2f}%")

def compareSelectedSkeletonAgainstReference():
	selected_trajectories = getTrajectoriesFormatted(getSelectedSkeletonTrajectoryIDs())
	reference_trajectories = getSkeletonReferenceFromFile()

	if len(selected_trajectories) == 0 or len(selected_trajectories) == 0:
		qtm.gui.message.add_message("Mocap Mimic: No skeleton selected", "Must select a skeleton to deal with", "error")
		return

	accuracy = compareTrajectories(reference_trajectories, selected_trajectories)
	
	qtm.gui.message.add_message(f"Mocap Mimic: Overall accuracy: {accuracy * 100:.2f}%", "", "info")
	print(f"Overall accuracy: {accuracy * 100:.2f}%")

def compareSelectedSkeletonBonesAgainstReference():
	return

# ----------------------------------------
# [END] COMPARING TRAJECTORIES
# ----------------------------------------

# ----------------------------------------
# [BEGIN] SKELETON FUNCTIONS
# ----------------------------------------

BoneIDs = []

# EXAMPLES STRUCTURE
# Skeleton = {
# 	"Name": "Hips", 
# 	"Transforms": [ [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]], 
# 					[[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]],
# 					[[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]],
# 					[[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]], 
# 	"Children": [
# 		{
# 			"Name": "Spine", 
# 			"Transforms": [ [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]], 
# 							[[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]],
# 							[[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]],
# 							[[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]], 
# 			"Children": [

# 			]
# 		},
# 		{
# 			"Name": "RightUpLeg", 
# 			"Transforms": [ [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]], 
# 							[[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]],
# 							[[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]],
# 							[[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]], 
# 			"Children": [

# 			]
# 		},
# 		{
# 			"Name": "LeftUpLeg", 
# 			"Transforms": [ [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]], 
# 							[[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]],
# 							[[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]],
# 							[[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]], 
# 			"Children": [

# 			]
# 		},
# 	]
# }

def printBoneData():
	selectedSkeleton = getSelectedSkeletonID()
	saveBoneDataToFile(qtm.data.object.skeleton.get_skeleton_root_id(selectedSkeleton))

def saveBoneDataToFile(RootBoneID):
	Skeleton = {}
	RootBoneName = qtm.data.object.skeleton.get_segment_name(RootBoneID)
	RootBoneTransforms = qtm.data.series.skeleton.get_samples(RootBoneID)

	Skeleton.update({"Name": RootBoneName})
	Skeleton.update({"ID": RootBoneID})
	Skeleton.update({"Transforms": RootBoneTransforms})
	Skeleton.update({"Children": []})

	ToConsider = []
	ToConsider.append({"Parent": Skeleton, "Children": qtm.data.object.skeleton.get_segment_child_ids(RootBoneID)})

	while len(ToConsider) > 0:
		CurrentBone = ToConsider.pop(0)
		CurrentBone["Parent"]
		for Child in CurrentBone["Children"]:
			Bone = {}
			Bone.update({"Name": qtm.data.object.skeleton.get_segment_name(Child)})
			Bone.update({"ID": Child})
			Bone.update({"Transforms": qtm.data.series.skeleton.get_samples(Child)})
			Bone.update({"Children": []})

			CurrentBone["Parent"]["Children"].append(Bone)

			ToConsider.append({"Parent": Bone, "Children": qtm.data.object.skeleton.get_segment_child_ids(Child)})

	# print(f"Skeleton: {Skeleton}")
	with open(skeleton_reference_bones_file_name, "w") as file:
		json.dump(Skeleton, file)

def getAllParentTransformsAtIndex(BoneID, Index):
	# Get all of the parent transforms and apply them to the child
	transforms = []
	parent_id = qtm.data.object.skeleton.get_segment_parent_id(BoneID)
	if parent_id != None:
		parent_transform = qtm.data.series.skeleton.get_sample(parent_id, Index)
		transforms.append(parent_transform)

	while parent_id != None:
		parent_id = qtm.data.object.skeleton.get_segment_parent_id(parent_id)
		if parent_id != None:
			parent_transform = qtm.data.series.skeleton.get_sample(parent_id, Index)
			transforms.append(parent_transform)
			
	return transforms

def getAllParentTransformsAtTime(BoneID, MeasurementTime):
	# Get all of the parent transforms and apply them to the child
	transforms = []
	parent_id = qtm.data.object.skeleton.get_segment_parent_id(BoneID)
	if parent_id != None:
		curr_index = qtm.data.series.skeleton.get_sample_index_at_time(parent_id, MeasurementTime)
		parent_transform = qtm.data.series.skeleton.get_sample(parent_id, curr_index)
		transforms.append(parent_transform)

	while parent_id != None:
		parent_id = qtm.data.object.skeleton.get_segment_parent_id(parent_id)
		if parent_id != None:
			curr_index = qtm.data.series.skeleton.get_sample_index_at_time(parent_id, MeasurementTime)
			parent_transform = qtm.data.series.skeleton.get_sample(parent_id, curr_index)
			transforms.append(parent_transform)
			
	return transforms

def getBoneTransformAtTime(BoneID, MeasurementTime):
	curr_index = qtm.data.series.skeleton.get_sample_index_at_time(BoneID, MeasurementTime)
	curr_bone_transform = qtm.data.series.skeleton.get_sample(BoneID, curr_index)
	transforms = getAllParentTransformsAtTime(BoneID, MeasurementTime)
	
	result_transform = copy.deepcopy(curr_bone_transform)
	for transform in transforms:
		result_transform = multiplyMatrices(result_transform, transform)
	
	return result_transform

def getBoneTransformAtIndex(BoneID, Index):
	curr_bone_transform = qtm.data.series.skeleton.get_sample(BoneID, Index)
	transforms = getAllParentTransformsAtIndex(BoneID, Index)
	
	result_transform = copy.deepcopy(curr_bone_transform)
	for transform in transforms:
		result_transform = multiplyMatrices(result_transform, transform)
	
	return result_transform

def compareSkeletonPose(lskel, rskel):
	return

def drawSphere(measurement_time):
	global BoneIDs

	# TODO Save the data in this sort of dictionary instead of fetching it every frame
	# bone_data = {}
	# bone_data.update({"Transform": 0})
	# bone_data.update({"ID": 0})

	for BoneID in BoneIDs:
		# print(qtm.data.object.skeleton.get_segment_name(BoneID))
		# if qtm.data.object.skeleton.get_segment_name(BoneID) != "Hips" and qtm.data.object.skeleton.get_segment_name(BoneID) != "Spine":
			# continue

		curr_index = qtm.data.series.skeleton.get_sample_index_at_time(BoneID, measurement_time)
		curr_bone_transform = qtm.data.series.skeleton.get_sample(BoneID, curr_index)

		# Get all of the parent transforms and apply them to the child
		transforms = []
		parent_id = qtm.data.object.skeleton.get_segment_parent_id(BoneID)
		if parent_id != None:
			curr_index = qtm.data.series.skeleton.get_sample_index_at_time(parent_id, measurement_time)
			parent_transform = qtm.data.series.skeleton.get_sample(parent_id, curr_index)
			transforms.append(parent_transform)

		while parent_id != None:
			parent_id = qtm.data.object.skeleton.get_segment_parent_id(parent_id)
			if parent_id != None:
				curr_index = qtm.data.series.skeleton.get_sample_index_at_time(parent_id, measurement_time)
				parent_transform = qtm.data.series.skeleton.get_sample(parent_id, curr_index)
				transforms.append(parent_transform)
        
		result_transform = copy.deepcopy(curr_bone_transform)#[[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
		for transform in transforms:
			result_transform = multiplyMatrices(result_transform, transform)
        
		x = result_transform[0][3]
		y = result_transform[1][3]
		z = result_transform[2][3]
		qtm.gui._3d.draw_sphere([x, y, z], 100, qtm.utilities.color.rgb(0.2, 0.661, 0.11))

bDrawingEnabled = False
def drawSphereAtSkeletonRoot():
	global BoneIDs
	global bDrawingEnabled

	seriesIDs = qtm.data.series.skeleton.get_series_ids()
    # A list of the bone ids that actually have to do with the selected skeleton
	selectedSkeletonID = getSelectedSkeletonID()
	for seriesID in seriesIDs:
		if qtm.data.object.skeleton.get_segment_skeleton_id(seriesID) == selectedSkeletonID:
			BoneIDs.append(seriesID)
	
	selectedSkeletonID = getSelectedSkeletonID()
	print(f"Selected Skeleton: {selectedSkeletonID}")

	skeletonSegment = qtm.data.object.skeleton.find_segment(selectedSkeletonID, "Hips")
	skeletonSegmentChildren = qtm.data.object.skeleton.get_segment_child_ids(skeletonSegment)
	for child in skeletonSegmentChildren:
		print(f"Child: {qtm.data.object.skeleton.get_segment_name(child)}")

	seriesIDs = qtm.data.series.skeleton.get_series_ids()
	print(f"Series IDs: {seriesIDs}")

	skeletonTrajectories = getSelectedSkeletonTrajectoryIDs()
	print(f"Skeleton Trajectory IDs: {skeletonTrajectories}")

	if not bDrawingEnabled:
		qtm.gui._3d.set_draw_function(drawSphere)
	else:
		qtm.gui._3d.set_draw_function()
	bDrawingEnabled = not bDrawingEnabled

# ----------------------------------------
# [END] SKELETON FUNCTIONS
# ----------------------------------------

# ----------------------------------------
# [BEGIN] HELP
# ----------------------------------------

def printHelp():
	print("Menu for comparing recordings for how similar they are")
	print("")

	print("Before data can be properly compared the reference recording (the one to be mimicked)")
	print("must be saved by highlighting one of the trajectories then clicking save in the menu")
	print("")

	print("If you want to compare rigid bodies use the 'Rigid Body' submenu and if you want to")
	print("compare people use the 'Skeleton' submenu")
	print("")

	print("Make sure to select trajectories associated with what you want to save or compare")
	print("And do not select trajectories associated with multiple different things at once")
	print("")

# ----------------------------------------
# [END] HELP
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

print_help_name = "mocap_mimic_print_help"
qtm.gui.add_command(print_help_name)
qtm.gui.set_command_execute_function(print_help_name, printHelp)
qtm.gui.insert_menu_button(mocap_mimic_menu_handle, "Help", print_help_name)

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
qtm.gui.insert_menu_button(skeleton_submenu_handle, "Compare to Reference (Trajectories)", skeleton_compare_selected_to_reference)

# Setting up the compare function
skeleton_compare_selected_to_reference_using_bones = "mocap_mimic_skeleton_compare_selected_bones_to_reference"
qtm.gui.add_command(skeleton_compare_selected_to_reference_using_bones)
qtm.gui.set_command_execute_function(skeleton_compare_selected_to_reference_using_bones, compareSelectedSkeletonBonesAgainstReference)
qtm.gui.insert_menu_button(skeleton_submenu_handle, "Compare to Reference (Bones)", skeleton_compare_selected_to_reference_using_bones)

# Setting up the skeleton print function
skeleton_print_bone_structure = "mocap_mimic_skeleton_print_bone_structure"
qtm.gui.add_command(skeleton_print_bone_structure)
qtm.gui.set_command_execute_function(skeleton_print_bone_structure, printBoneData)
qtm.gui.insert_menu_button(skeleton_submenu_handle, "Print skeleton structure", skeleton_print_bone_structure)

# Setting up the draw at skeleton function
draw_sphere_at_skeleton = "mocap_mimic_draw_sphere_at_skeleton"
qtm.gui.add_command(draw_sphere_at_skeleton)
qtm.gui.set_command_execute_function(draw_sphere_at_skeleton, drawSphereAtSkeletonRoot)
qtm.gui.insert_menu_button(skeleton_submenu_handle, "Draw Sphere at Skeleton", draw_sphere_at_skeleton)

# Setting up the compare function
print_selected_name = "mocap_mimic_print_selected"
qtm.gui.add_command(print_selected_name)
qtm.gui.set_command_execute_function(print_selected_name, printSelected)
qtm.gui.insert_menu_button(mocap_mimic_menu_handle, "Print Selections", print_selected_name)

# ----------------------------------------
# [END] ADDING MENU ITEMS
# ----------------------------------------