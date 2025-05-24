import qtm
from qtm.data.series import _3d
from qtm.data.object import trajectory
import math
import json
import copy

# ----------------------------------------
# [BEGIN] UTILS
# ----------------------------------------

def getTranslation(transform_matrix: list[list[float]]):
	return [transform_matrix[0][3], transform_matrix[1][3], transform_matrix[2][3]]

# strings = ["QA_hips", "QA_wrist", "QA_elbow"]
def getPrefix(strings: list[str]) -> str:
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

def getLength(vec: list[float]) -> float:
	sum = 0
	for i in range(len(vec)):
		sum += vec[i] * vec[i]
	return math.sqrt(sum)

def dotProduct(Vec1: list[float], Vec2: list[float]) -> float:
	sum = 0
	for i in range(len(Vec1)):
		sum += Vec1[i] * Vec2[i]
	return sum

def getNormalized(vec: list[float]) -> list[float]:
	length = getLength(vec)
	if length == 0:
		return vec
	resultVec = vec[:]
	for i in range(len(vec)):
		resultVec[i] /= length
	return resultVec
	
def getDistance(vec1: list[float], vec2: list[float]) -> float:
	diffVec = vec2[:]
	for i in range(len(diffVec)):
		diffVec[i] -= vec1[i]
	return getLength(diffVec)

def getDifference(lvec: list[float], rvec: list[float]) -> list[float]:
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

def print4x4Matrix(mat: list[list[float]]) -> None:
	print(f"\n{mat[0][0]:.2f}, {mat[0][1]:.2f}, {mat[0][2]:.2f}, {mat[0][3]:.2f}\n{mat[1][0]:.2f}, {mat[1][1]:.2f}, {mat[1][2]:.2f}, {mat[1][3]:.2f}\n{mat[2][0]:.2f}, {mat[2][1]:.2f}, {mat[2][2]:.2f}, {mat[2][3]:.2f}\n{mat[3][0]:.2f}, {mat[3][1]:.2f}, {mat[3][2]:.2f}, {mat[3][3]:.2f}")

# NOTE Has not been properly tested for accuracy
# Also it just works for matrices of that are 4x4
def multiplyMatrices(lmat: list[list[float]], rmat: list[list[float]]) -> list[list[float]]:
	mat = copy.deepcopy(lmat)

	tempSum =  0
	for k in range(4):
		for i in range(4):
			for j in range(4):
				tempSum += lmat[j][k] * rmat[i][j]
			mat[i][k] = tempSum
			tempSum = 0

	return mat

def multiplyVectorMatrix(vec: list[float], mat: list[list[float]]) -> list[float]:
	resultVec = []
	for i in range(len(vec)):
		sum = 0
		for j in range(len(vec)):
			sum += vec[j] * mat[j][i]
		resultVec.append(sum)
	
	return resultVec


# ----------------------------------------
# [END] MATRICES
# ----------------------------------------

# ----------------------------------------
# [BEGIN] TRAJECTORIES
# ----------------------------------------

def printSelected() -> None:
	trajectory_selections = qtm.gui.selection.get_selections("trajectory")
	bone_selections = qtm.gui.selection.get_selections("bone")

	print(f"Selected trajectories: {trajectory_selections}")
	print(f"Selected bones: {bone_selections}")

	skeleton_ids = qtm.data.object.skeleton.get_skeleton_ids()

	print(f"skeleton_ids: {skeleton_ids}")

	for trajectory in trajectory_selections:
		skeleton_segment_id = qtm.data.object.trajectory.get_skeleton_segment_id(trajectory["id"])
		print(f"skeleton_segment_id: {skeleton_segment_id}")

def getSelectedRigidBodyTrajectoryIDs() -> list[int]:
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

def getSelectedSkeletonTrajectoryIDs() -> list[int]:
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

# NOTE This is heavily based on the above functions for getting the trajectories, it just stops sooner
# It could probably be improved but I will not make it a priority
def getSelectedSkeletonID() -> int:
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
	
def getTrajectoriesFormatted(trajectory_ids: list[int]) -> dict[int: list[list[list[float]]]]:
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

gSegments = []
def addSegmentMarker() -> None:
	global gSegments
	current_frame = qtm.gui.timeline.get_current_frame()
	gSegments.append(current_frame)
	# NOTE I'm not really sure this math below is right, it's kind of a fencepost problem
	# I'm too lazy to properly solve it though so this will do
	print(f"Segment added, current segments: {len(gSegments) + 1}")

def clearSegmentMarkers() -> None:
	global gSegments
	gSegments.clear()
	print("Segments cleared!")

markerFrequency: float = 0.5
def setMarkerFrequencyInSeconds(NewValue: float):
	global markerFrequency
	markerFrequency = NewValue
	print(f"Marker Frequency set to once every {markerFrequency} seconds")

# Add in markers of equal distance
def addEquidistantMarkers():
	global gSegments
	global markerFrequency

	# 100.0
	qtmFrequency = qtm.gui.timeline.get_frequency()

	selected_range = qtm.gui.timeline.get_selected_range()

	# If markerFrequency is 0.5 for half a second, then multiplying that value by qtmFrequency
	# will get 50 frames, which represents the size of each segment
	spacing = int(markerFrequency * qtmFrequency)

	for i in range(selected_range["start"], selected_range["end"], spacing):
		gSegments.append(i)
	print(f"Added {len(gSegments)} markers!")

def getSegmentsAsRanges(segments: list[int]) -> dict[str]:
	segment_ranges = []

	for i in range(len(segments) - 1):
		segment_ranges.append({"start": segments[i], "end": segments[i + 1]})
	
	return segment_ranges

def getSegmentsInLocalRange(range):
	global gSegments

	local_segments = [segment - range["start"] for segment in gSegments]

	# NOTE Adds in the start and the end so that the segments can cover that if the user hasn't already
	# But it doesn't add anything if the user didn't add anything either
	if len(gSegments) > 0:
		if local_segments[0] != 0:
			local_segments.insert(0, 0)

		if local_segments[-1] != range["end"] - range["start"]:
			local_segments.append(range["end"] - range["start"])
	
	return local_segments

def saveSelectedRigidBodyAsReference() -> None:
	global gSegments

	rigid_body_trajectory_ids = getSelectedRigidBodyTrajectoryIDs()
	selected_range = qtm.gui.timeline.get_selected_range()
	rigid_body_data = {}
	rigid_body_trajectories = {}
	
	for trajectory_id in rigid_body_trajectory_ids:
		trajectory_label = qtm.data.object.trajectory.get_label(trajectory_id)
		trajectory_points = _3d.get_samples(trajectory_id, selected_range)
		rigid_body_trajectories.update({trajectory_label: trajectory_points})
	
	segments = getSegmentsAsRanges(getSegmentsInLocalRange(selected_range))

	rigid_body_data.update({"trajectories": rigid_body_trajectories})
	rigid_body_data.update({"segments": segments})
	
	with open(rigid_body_reference_file_name, "w") as file:
		json.dump(rigid_body_trajectories, file)

	gSegments.clear()
		
def saveSelectedSkeletonAsReference() -> None:
	global gSegments

	skeleton_trajectory_ids = getSelectedSkeletonTrajectoryIDs()
	selected_range = qtm.gui.timeline.get_selected_range()
	skeleton_trajectories_data = {}
	skeleton_trajectories = {}
	
	for trajectory_id in skeleton_trajectory_ids:
		trajectory_label = qtm.data.object.trajectory.get_label(trajectory_id)
		trajectory_points = _3d.get_samples(trajectory_id, selected_range)
		skeleton_trajectories.update({trajectory_label: trajectory_points})

	segments = getSegmentsAsRanges(getSegmentsInLocalRange(selected_range))

	skeleton_trajectories_data.update({"trajectories": skeleton_trajectories})
	skeleton_trajectories_data.update({"segments": segments})
	
	with open(skeleton_reference_file_name, "w") as file:
		json.dump(skeleton_trajectories_data, file)

	selectedSkeleton = getSelectedSkeletonID()

	SkeletonData = {}
	Skeleton = getSkeletonAsDict(selectedSkeleton, selected_range)
	SkeletonData.update({"skeleton": Skeleton})
	SkeletonData.update({"segments": segments})

	with open(skeleton_reference_bones_file_name, "w") as file:
		json.dump(SkeletonData, file)

	gSegments.clear()

def getSkeletonBonesReferenceFromFile() -> dict[str]:
	with open(skeleton_reference_bones_file_name, "r") as file:
		skeleton = json.load(file)    
		return skeleton

def getRigidBodyReferenceFromFile() -> dict[str]:
	with open(rigid_body_reference_file_name, "r") as file:
		rigid_body_trajectories = json.load(file)    
		return rigid_body_trajectories

def getSkeletonReferenceFromFile() -> dict[str]:
	with open(skeleton_reference_file_name, "r") as file:
		skeleton_trajectories = json.load(file)    
		return skeleton_trajectories

# ----------------------------------------
# [END] SAVING AND LOADING
# ----------------------------------------

# ----------------------------------------
# [BEGIN] COMPARING TRAJECTORIES
# ----------------------------------------

def compareTrajectories(base_trajectories, mimic_trajectories) -> float:
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

	numberOfLables = len(base_trajectories)
	numberOfSamples = len(base_trajectories[list(base_trajectories.keys())[0]])
	accuracy = sumCorr / (numberOfLables * numberOfSamples)

	return accuracy

def compareSelectedRigidBodyAgainstReference() -> None:
	selected_trajectories = getTrajectoriesFormatted(getSelectedRigidBodyTrajectoryIDs())
	reference_trajectories = getRigidBodyReferenceFromFile()["trajectories"]

	if len(selected_trajectories) == 0 or len(selected_trajectories) == 0:
		qtm.gui.message.add_message("Mocap Mimic: No rigid bodies selected", "Must select a rigid body to deal with", "error")
		return

	accuracy = compareTrajectories(reference_trajectories, selected_trajectories)

	qtm.gui.message.add_message(f"Mocap Mimic: Overall accuracy: {accuracy:.2f}", "", "info")
	print(f"Overall accuracy: {accuracy:.2f}")

def compareSelectedSkeletonAgainstReference() -> None:
	selected_trajectories = getTrajectoriesFormatted(getSelectedSkeletonTrajectoryIDs())
	reference_trajectories = getSkeletonReferenceFromFile()["trajectories"]

	if len(selected_trajectories) == 0 or len(selected_trajectories) == 0:
		qtm.gui.message.add_message("Mocap Mimic: No skeleton selected", "Must select a skeleton to deal with", "error")
		return

	accuracy = compareTrajectories(reference_trajectories, selected_trajectories)
	
	qtm.gui.message.add_message(f"Mocap Mimic: Overall accuracy: {accuracy:.2f}", "", "info")
	print(f"Overall accuracy: {accuracy:.2f}")

bDoCoarsePass = False
WindowPassResolution: int = 2

# NOTE To be called in the QTM console
def setCoarsePassEnabled(NewValue: bool):
	global bDoCoarsePass
	bDoCoarsePass = NewValue
	print(f"bDoCoarsePass: {NewValue}")

def setWindowPassResolution(NewIndex: int):
	global WindowPassResolution
	WindowPassResolution = NewIndex
	print(f"WindowPassResolution: {NewIndex}")

def printSegmentedResults(Segments, SegmentedBoneData):
	longestBoneName = 20
	title = "Joint Name"
	titleString = f"{title:{longestBoneName}}|"
	sectionLengths = []
	freq = qtm.gui.timeline.get_frequency()
	for i in range(len(Segments)):
		tempString = f" Segment {i} ({Segments[i]['start'] * (1/freq):.2f}s - {Segments[i]['end'] * (1/freq):.2f}s) |"
		sectionLengths.append(len(tempString) - 1)
		titleString += tempString
	print(titleString)
	
	bufferString = ""
	separatorString = ""
	for i in range(len(titleString)):
		if titleString[i] == "|":
			bufferString += "+"
			separatorString += "|"
		else:
			bufferString += "-"
			separatorString += " "

	for key, val in SegmentedBoneData.items():
		tempString = f"{key:{longestBoneName}}|"
		for i, el in enumerate(val):
			lpadding = (sectionLengths[i] - 5) // 2
			rpadding = math.ceil((sectionLengths[i] - 5) / 2)
			string = f"{'':{lpadding}}{el:0.3f}{'':{rpadding}}|"
			tempString += string
		print(bufferString)
		print(tempString)
		print(separatorString)

def compareSelectedSkeletonBonesAgainstReference() -> None:
	global bDoCoarsePass
	global WindowPassResolution

	selected_range = qtm.gui.timeline.get_selected_range()
	print(f"Selected Range: {selected_range}")
	selectedSkeletonID = getSelectedSkeletonID()

	if selectedSkeletonID == -1:
		print("No Skeleton Selected!")
		return

	mimicSkeleton = getSkeletonAsDict(selectedSkeletonID, selected_range)
	referenceSkeleton = getSkeletonBonesReferenceFromFile()["skeleton"]
	segments = getSkeletonBonesReferenceFromFile()["segments"]
	print(f"segments: {segments}")
	
	Overshoot: int = len(mimicSkeleton["Transforms"]) - len(referenceSkeleton["Transforms"])

	if Overshoot < 0:
		print(f"Mimic must be at least as long as the reference! Need {-Overshoot} more samples")
		return

	BoneData = {}
	numbersOfMeasurement = len(referenceSkeleton["Transforms"])
	BestAverageScore = 0
	MimicComparisonOffset = 0

	print(f"Coarse Pass: {bDoCoarsePass}")

	# Start of Coarse Pass
	if bDoCoarsePass:
		print(f"Resolution: {WindowPassResolution}")
		for j in range(Overshoot):
			for i in range(0, numbersOfMeasurement, WindowPassResolution):
				TempBoneData = compareSkeletonPose(referenceSkeleton, mimicSkeleton, i, j)

				if i == 0:
					BoneData = TempBoneData
					continue

				for key in BoneData:
					BoneData[key] += TempBoneData[key]
			
			AverageScore = sum(BoneData.values()) / len(BoneData)

			if AverageScore > BestAverageScore:
				BestAverageScore = AverageScore
				MimicComparisonOffset = j

		# Set the measured range in QTM to the best chunk we found
		NewRangeStart = selected_range["start"] + MimicComparisonOffset
		NewRangeEnd = NewRangeStart + numbersOfMeasurement
		NewRange = {"start": NewRangeStart, "end": NewRangeEnd}

		print(f"Setting range to: {NewRange}")
		qtm.gui.timeline.set_selected_range(NewRange)
	# End of Coarse Pass

	# NOTE If segments exist, split up the evaluation
	# TODO Add proper evalution
	if len(segments) > 0:
		# Structured like {"Hips": [0.95, 0.584, 0.458], "Spine": [0.95, 0.584, 0.458]}
		SegmentedBoneData = {}
		boneNames = getAllSkeletonBoneNames(referenceSkeleton)
		for boneName in boneNames:
			SegmentedBoneData.update({boneName: []})

		for segment in segments:
			print(segment)
			SegmentComparisonData = {}
			for i in range(segment["start"], segment["end"]):
				TempBoneData = compareSkeletonPose(referenceSkeleton, mimicSkeleton, i, i + MimicComparisonOffset)

				if i == segment["start"]:
					SegmentComparisonData = TempBoneData
					continue

				for key in SegmentComparisonData:
					SegmentComparisonData[key] += TempBoneData[key]
			
			# Calculate the average score over the period
			periodSpan = segment["end"] - segment["start"]
			for key in SegmentComparisonData:
				SegmentComparisonData[key] /= periodSpan

			for key in SegmentedBoneData:
				if not (key in SegmentedBoneData):
					SegmentedBoneData[key].update({key: [SegmentComparisonData[key]]})
				else:
					SegmentedBoneData[key].append(SegmentComparisonData[key])

		
		# Some weird logic to make the output look a bit prettier
		# TODO setting this to a constant value is bad, but I'm too lazy to actually iterate over the array
		printSegmentedResults(segments, SegmentedBoneData)

	# NOTE If no segments exist, judge it in its entirety
	else:
		for i in range(numbersOfMeasurement):
			TempBoneData = compareSkeletonPose(referenceSkeleton, mimicSkeleton, i, i + MimicComparisonOffset)

			if i == 0:
				BoneData = TempBoneData
				continue

			for key in BoneData:
				BoneData[key] += TempBoneData[key]

		padding = 0
		for key in BoneData:
			padding = max(len(key), padding)

		# Sorts the dict by the accuracy of the joint, least accurate first
		BoneData = {k: v for k, v in sorted(BoneData.items(), key=lambda item: item[1])}

		print("Bone accuracy (Sorted):")
		for key, val in BoneData.items():
			val /= numbersOfMeasurement
			print(f"{key:{padding + 1}}: {val:.2f}")

	# qtm.gui.message.add_message(f"Mocap Mimic: Overall accuracy: {accuracy * 100:.2f}%", "", "info")
	# print(f"Overall accuracy: {accuracy * 100:.2f}%")

def compareSelectedSkeletonBonesAgainstReferenceWorldAgnostic() -> None:
	global bDoCoarsePass
	global WindowPassResolution

	selected_range = qtm.gui.timeline.get_selected_range()
	print(f"Selected Range: {selected_range}")
	selectedSkeletonID = getSelectedSkeletonID()

	if selectedSkeletonID == -1:
		print("No Skeleton Selected!")
		return

	mimicSkeleton = getSkeletonAsDict(selectedSkeletonID, selected_range)
	referenceSkeleton = getSkeletonBonesReferenceFromFile()["skeleton"]
	segments = getSkeletonBonesReferenceFromFile()["segments"]
	print(f"segments: {segments}")
	
	Overshoot: int = len(mimicSkeleton["Transforms"]) - len(referenceSkeleton["Transforms"])

	if Overshoot < 0:
		print(f"Mimic must be at least as long as the reference! Need {-Overshoot} more samples")
		return

	BoneData = {}
	numbersOfMeasurement = len(referenceSkeleton["Transforms"])
	BestAverageScore = 0
	MimicComparisonOffset = 0

	print(f"Coarse Pass: {bDoCoarsePass}")

	# Start of Coarse Pass
	if bDoCoarsePass:
		print(f"Resolution: {WindowPassResolution}")
		for j in range(Overshoot):
			for i in range(0, numbersOfMeasurement, WindowPassResolution):
				TempBoneData = compareSkeletonPoseWorldAgnostic(referenceSkeleton, mimicSkeleton, i, j)

				if i == 0:
					BoneData = TempBoneData
					continue

				for key in BoneData:
					BoneData[key] += TempBoneData[key]
			
			AverageScore = sum(BoneData.values()) / len(BoneData)

			if AverageScore > BestAverageScore:
				BestAverageScore = AverageScore
				MimicComparisonOffset = j

		# Set the measured range in QTM to the best chunk we found
		NewRangeStart = selected_range["start"] + MimicComparisonOffset
		NewRangeEnd = NewRangeStart + numbersOfMeasurement
		NewRange = {"start": NewRangeStart, "end": NewRangeEnd}

		print(f"Setting range to: {NewRange}")
		qtm.gui.timeline.set_selected_range(NewRange)
	# End of Coarse Pass

	# NOTE If segments exist, split up the evaluation
	# TODO Add proper evalution
	if len(segments) > 0:
		# Structured like {"Hips": [0.95, 0.584, 0.458], "Spine": [0.95, 0.584, 0.458]}
		SegmentedBoneData = {}
		boneNames = getAllSkeletonBoneNames(referenceSkeleton)
		for boneName in boneNames:
			SegmentedBoneData.update({boneName: []})

		for segment in segments:
			print(segment)
			SegmentComparisonData = {}
			for i in range(segment["start"], segment["end"]):
				TempBoneData = compareSkeletonPoseWorldAgnostic(referenceSkeleton, mimicSkeleton, i, i + MimicComparisonOffset)

				if i == segment["start"]:
					SegmentComparisonData = TempBoneData
					continue

				for key in SegmentComparisonData:
					SegmentComparisonData[key] += TempBoneData[key]
			
			# Calculate the average score over the period
			periodSpan = segment["end"] - segment["start"]
			for key in SegmentComparisonData:
				SegmentComparisonData[key] /= periodSpan

			for key in SegmentedBoneData:
				if not (key in SegmentedBoneData):
					SegmentedBoneData[key].update({key: [SegmentComparisonData[key]]})
				else:
					SegmentedBoneData[key].append(SegmentComparisonData[key])

		
		printSegmentedResults(segments, SegmentedBoneData)

	# NOTE If no segments exist, judge it in its entirety
	else:
		for i in range(numbersOfMeasurement):
			TempBoneData = compareSkeletonPoseWorldAgnostic(referenceSkeleton, mimicSkeleton, i, i + MimicComparisonOffset)

			if i == 0:
				BoneData = TempBoneData
				continue

			for key in BoneData:
				BoneData[key] += TempBoneData[key]

		padding = 0
		for key in BoneData:
			padding = max(len(key), padding)

		# Sorts the dict by the accuracy of the joint, least accurate first
		BoneData = {k: v for k, v in sorted(BoneData.items(), key=lambda item: item[1])}

		print("Bone accuracy (Sorted):")
		for key, val in BoneData.items():
			val /= numbersOfMeasurement
			print(f"{key:{padding + 1}}: {val:.2f}")

	# qtm.gui.message.add_message(f"Mocap Mimic: Overall accuracy: {accuracy * 100:.2f}%", "", "info")
	# print(f"Overall accuracy: {accuracy * 100:.2f}%")

# ----------------------------------------
# [END] COMPARING TRAJECTORIES
# ----------------------------------------

# ----------------------------------------
# [BEGIN] SKELETON FUNCTIONS
# ----------------------------------------

BoneIDs = []

def drawSkeletonSpheresRecursive(BoneDict: dict[str], Index: int = 0, ParentTransform: list[list[float]] = [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]) -> None:
	Transform = multiplyMatrices(BoneDict["Transforms"][Index], ParentTransform)

	for Child in BoneDict["Children"]:
		drawSkeletonSpheresRecursive(Child, Index, Transform)

	position = getTranslation(Transform)
	color = qtm.utilities.color.rgb(0.2, 0.661, 0.11)
	qtm.gui._3d.draw_sphere(position, 100, color)

# NOTE The skeletons have to have the same structure, otherwise this will fail
def compareSkeletonPose(BoneDict, MimicBoneDict, Index = 0, MimicIndex = 0, ParentTransform = [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]], MimicParentTransform = [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]):
	Transform = multiplyMatrices(BoneDict["Transforms"][Index], ParentTransform)
	MimicTransform = multiplyMatrices(MimicBoneDict["Transforms"][MimicIndex], MimicParentTransform)

	BoneData = {}

	for i in range(len(BoneDict["Children"])):
		BoneData.update(compareSkeletonPose(BoneDict["Children"][i], MimicBoneDict["Children"][i], Index, MimicIndex, Transform, MimicTransform))

	currentPosition = getTranslation(Transform)
	parentPosition = getTranslation(ParentTransform)
	jointDirection = getNormalized(getDifference(currentPosition, parentPosition))

	mimicCurrentPosition = getTranslation(MimicTransform)
	mimicParentPosition = getTranslation(MimicParentTransform)
	mimicJointDirection = getNormalized(getDifference(mimicCurrentPosition, mimicParentPosition))
	# print(BoneData)
	BoneData.update({BoneDict["Name"]: dotProduct(jointDirection, mimicJointDirection)})

	return BoneData

def getAllSkeletonBoneNames(BoneDict) -> list[str]:
	bones = [BoneDict["Name"]]
	for child in BoneDict["Children"]:
		bones += getAllSkeletonBoneNames(child)
	return bones

# Principly does it make sense? Yes since we only care about the local relationship, it doesn't really matter what happens further up or down the chain.
# I only care about the direct parent and child relationship between every point, not the chains influence.
# NOTE The skeletons have to have the same structure, otherwise this will fail
def compareSkeletonPoseWorldAgnostic(BoneDict, MimicBoneDict, Index = 0, MimicIndex = 0):
	BoneData = {}
	ToConsider = [BoneDict]
	MimicToConsider = [MimicBoneDict]

	while len(ToConsider) > 0:
		CurrentBone = ToConsider.pop(0)
		MimicCurrentBone = MimicToConsider.pop(0)
		for i in range(len(CurrentBone["Children"])):
			ToConsider.append(CurrentBone["Children"][i])
			MimicToConsider.append(MimicCurrentBone["Children"][i])

			# Gather transforms
			CurrentTransform = CurrentBone["Transforms"][Index]
			ChildTransform = CurrentBone["Children"][i]["Transforms"][Index]
			ChildTransform = multiplyMatrices(ChildTransform, CurrentTransform)

			MimicCurrentTransform = MimicCurrentBone["Transforms"][MimicIndex]
			MimicChildTransform = MimicCurrentBone["Children"][i]["Transforms"][MimicIndex]
			MimicChildTransform = multiplyMatrices(MimicChildTransform, MimicCurrentTransform)

			# Calculate the joint direction vector
			CurrentPosition = getTranslation(CurrentTransform)
			ChildPosition = getTranslation(ChildTransform)
			jointDirection = getNormalized(getDifference(ChildPosition, CurrentPosition))

			# Calculate the mimic's joint direction vector
			mimicCurrentPosition = getTranslation(MimicCurrentTransform)
			mimicChildPosition = getTranslation(MimicChildTransform)
			mimicJointDirection = getNormalized(getDifference(mimicChildPosition, mimicCurrentPosition))

			# Calculate the dot product between the reference and the mimic
			dot = dotProduct(jointDirection, mimicJointDirection)
			BoneData.update({CurrentBone["Children"][i]["Name"]: dot})

	return BoneData

def getSkeletonAsDict(SkeletonID: int, Range: dict[str: int] = None):
	RootBoneID = qtm.data.object.skeleton.get_skeleton_root_id(SkeletonID)

	Skeleton = {}
	RootBoneName = qtm.data.object.skeleton.get_segment_name(RootBoneID)
	RootBoneTransforms = qtm.data.series.skeleton.get_samples(RootBoneID, Range)

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
			Bone.update({"Transforms": qtm.data.series.skeleton.get_samples(Child, Range)})
			Bone.update({"Children": []})

			CurrentBone["Parent"]["Children"].append(Bone)

			ToConsider.append({"Parent": Bone, "Children": qtm.data.object.skeleton.get_segment_child_ids(Child)})
	
	return Skeleton

CurrentSkeleton = {}

def drawSphere(measurement_time):
	global BoneIDs
	global CurrentSkeleton

	# TODO Save the data in this sort of dictionary instead of fetching it every frame
	# bone_data = {}
	# bone_data.update({"Transform": 0})
	# bone_data.update({"ID": 0})

	# NOTE This is just a hacky test, this paragraph can be removed without harm
	# It is notable that this hacky method is significantly faster, since it uses fewer matrix multiplications
	curr_index = qtm.data.series.skeleton.get_sample_index_at_time(BoneIDs[0], measurement_time)
	drawSkeletonSpheresRecursive(CurrentSkeleton, curr_index)
	return

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
        
		position = getTranslation(result_transform)
		qtm.gui._3d.draw_sphere(position, 100, qtm.utilities.color.rgb(0.2, 0.661, 0.11))

bDrawingEnabled = False
def drawSphereAtSkeletonRoot():
	global BoneIDs
	global bDrawingEnabled
	global CurrentSkeleton
	CurrentSkeleton = getSkeletonAsDict(getSelectedSkeletonID())

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

# Setting up the compare function
skeleton_compare_selected_to_reference_using_bones_world_agnostic = "mocap_mimic_skeleton_compare_selected_bones_to_reference_world_agnostic"
qtm.gui.add_command(skeleton_compare_selected_to_reference_using_bones_world_agnostic)
qtm.gui.set_command_execute_function(skeleton_compare_selected_to_reference_using_bones_world_agnostic, compareSelectedSkeletonBonesAgainstReferenceWorldAgnostic)
qtm.gui.insert_menu_button(skeleton_submenu_handle, "Compare to Reference (Bones) (World Agnostic)", skeleton_compare_selected_to_reference_using_bones_world_agnostic)

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

# Setting up the add segment marker function
add_segment_marker_name = "mocap_mimic_add_segment_marker"
qtm.gui.add_command(add_segment_marker_name)
qtm.gui.set_command_execute_function(add_segment_marker_name, addSegmentMarker)
qtm.gui.insert_menu_button(mocap_mimic_menu_handle, "Add Segment Marker", add_segment_marker_name)

# Setting up the clear segment marker function
clear_segment_markers_name = "mocap_mimic_clear_segment_markers"
qtm.gui.add_command(clear_segment_markers_name)
qtm.gui.set_command_execute_function(clear_segment_markers_name, clearSegmentMarkers)
qtm.gui.insert_menu_button(mocap_mimic_menu_handle, "Clear Segment Markers", clear_segment_markers_name)

# Setting up the clear segment marker function
add_equidistant_markers_name = "mocap_mimic_add_equidistant_markers"
qtm.gui.add_command(add_equidistant_markers_name)
qtm.gui.set_command_execute_function(add_equidistant_markers_name, addEquidistantMarkers)
qtm.gui.insert_menu_button(mocap_mimic_menu_handle, "Add Equidistant Segment Markers", add_equidistant_markers_name)

# ----------------------------------------
# [END] ADDING MENU ITEMS
# ----------------------------------------

def PrintAsBox(strings: list[str]) -> None:
	maxLength = 0
	for string in strings:
		maxLength = max(maxLength, len(string))
	
	topString = "+-" + maxLength * "-" + "-+"
	print(topString)
	for string in strings:
		print(f"| {string:{maxLength}} |")
	print(topString)

# Print all of the current settings to the user
info = [
	"Mocap Mimic: Current values of user-set variables:", 
	f"markerFrequency: float = {markerFrequency}, call setMarkerFrequencyInSeconds(NewValue: float) to change this value", 
	f"bDoCoarsePass: bool = {bDoCoarsePass}, call setCoarsePassEnabled(NewValue: bool) to change this value", 
	f"WindowPassResolution: int = {WindowPassResolution}, call setWindowPassResolution(NewIndex: int) to change this value"
]

PrintAsBox(info)