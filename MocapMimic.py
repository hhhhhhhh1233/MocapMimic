import qtm
from qtm.data.series import _3d
from qtm.data.object import trajectory
import math
import json

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

def getSelectedBodyTrajectoryIds():
    trajectory_ids = qtm.data.object.trajectory.get_trajectory_ids()
    
    rigid_body_trajectory_ids = []
    
    selections = qtm.gui.selection.get_selections("trajectory")
    selected_rigid_body_id = qtm.data.object.trajectory.get_rigid_body_id(selections[0]["id"])
    for selection in selections:
        print(selection["id"])
        rigid_body_id = qtm.data.object.trajectory.get_rigid_body_id(selection["id"])
        if rigid_body_id != selected_rigid_body_id:
            qtm.gui.message.add_message("Mocap Mimic: Multiple rigid bodies selected", "Only one rigid body should be selected at a time", "error")
            return []
    
    for trajectory_id in trajectory_ids:
        trajectory_id_rigid_body_id = qtm.data.object.trajectory.get_rigid_body_id(trajectory_id)
        if trajectory_id_rigid_body_id == selected_rigid_body_id:
            rigid_body_trajectory_ids.append(trajectory_id)
            
    return rigid_body_trajectory_ids
    
    # for trajectory_id in rigid_body_trajectory_ids:
        # trajectory_label = qtm.data.object.trajectory.get_label(trajectory_id)
        # a, b = getTrajectorySimilarity(trajectory_label, trajectory_label)
        # print(f"Right: {a}")
        # print(f"Wrong: {b}")
    
def saveTrajectoriesAsReference(arr):
    with open(f"{qtm.settings.directory.get_project_directory()}MocapMimicReference.json", "w") as filehandler:
        json.dump(arr.toList(), filehandler)
     
def getTrajectoriesFormatted(trajectory_ids):
    measured_range = qtm.gui.timeline.get_measured_range()
    selected_range = qtm.gui.timeline.get_selected_range()
    
    label_trajectories = []
    
    for trajectory_id in trajectory_ids:
        trajectory_label = qtm.data.object.trajectory.get_label(trajectory_id)
        label_trajectories.append([trajectory_label, _3d.get_samples(trajectory_id, selected_range)])
    
    return label_trajectories
     
def saveSelectedAsReference():
    reference_file_name = f"{qtm.settings.directory.get_project_directory()}MocapMimicReference.json"
    
    print(reference_file_name)
    
    rigid_body_trajectory_ids = getSelectedBodyTrajectoryIds()
    
    measured_range = qtm.gui.timeline.get_measured_range()
    selected_range = qtm.gui.timeline.get_selected_range()
    print(measured_range)
    print(selected_range)
    
    label_trajectories = []
    
    for trajectory_id in rigid_body_trajectory_ids:
        trajectory_label = qtm.data.object.trajectory.get_label(trajectory_id)
        label_trajectories.append([trajectory_label, _3d.get_samples(trajectory_id, selected_range)])
    
    with open(reference_file_name, "w") as file:
        json.dump(label_trajectories, file)
        
def loadReference():
    reference_file_name = f"{qtm.settings.directory.get_project_directory()}MocapMimicReference.json"
    
    with open(reference_file_name, "r") as file:
        label_trajectories = json.load(file)
        
    for label_trajectory in label_trajectories:
        print(label_trajectory[0])
        
def getReferenceFromFile():
    reference_file_name = f"{qtm.settings.directory.get_project_directory()}MocapMimicReference.json"
    
    with open(reference_file_name, "r") as file:
        label_trajectories = json.load(file)    
        return label_trajectories

def getTrajectorySimilarity(trajectoryLabel1, trajectoryLabel2):
    id1 = trajectory.find_trajectory(trajectoryLabel1)
    id2 = trajectory.find_trajectory(trajectoryLabel2)
    part1 = qtm.data.object.trajectory.get_part(id1, 0)
    part2 = qtm.data.object.trajectory.get_part(id2, 0)

    minimumChange = 5

    RightPeriods = []
    WrongPeriods = []

    LastState = ""
    Start = 0

    for i in range(part1["range"]["end"] - 1):
        xDelta = _3d.get_sample(id1, i + 1)["position"][0] - _3d.get_sample(id1, i)["position"][0]
        yDelta = _3d.get_sample(id1, i + 1)["position"][1] - _3d.get_sample(id1, i)["position"][1]
        zDelta = _3d.get_sample(id1, i + 1)["position"][2] - _3d.get_sample(id1, i)["position"][2]
        
        if i < part2["range"]["end"]:
            xDelta2 = _3d.get_sample(id2, i + 1)["position"][0] - _3d.get_sample(id2, i)["position"][0]
            yDelta2 = _3d.get_sample(id2, i + 1)["position"][1] - _3d.get_sample(id2, i)["position"][1]
            zDelta2 = _3d.get_sample(id2, i + 1)["position"][2] - _3d.get_sample(id2, i)["position"][2]

            corr = (dotProduct(getNormalized([xDelta, yDelta, zDelta]), getNormalized([xDelta2, yDelta2, zDelta2])))
        
            if corr > 0:
                if LastState == "Wrong":
                    WrongPeriods.append([Start, i])
                    Start = i
                LastState = "Right"
                #print(f"Right direction, i: {i}, Start: {Start}")
            if corr <= 0:
                if LastState == "Right":
                    RightPeriods.append([Start, i])
                    Start = i
                LastState = "Wrong"
                #print(f"Wrong direction, i: {i}, Start: {Start}")
                
    if LastState == "Wrong":
        WrongPeriods.append([Start, part1["range"]["end"] - 2])
    else:
        RightPeriods.append([Start, part1["range"]["end"] - 2])
    return RightPeriods, WrongPeriods

def calcAccuracy():    
    rigid_body_trajectory_ids = getSelectedBodyTrajectoryIds()
    
    for trajectory_id in rigid_body_trajectory_ids:
        trajectory_label = qtm.data.object.trajectory.get_label(trajectory_id)
        a, b = getTrajectorySimilarity(trajectory_label, trajectory_label)
        print(f"Right: {a}")
        print(f"Wrong: {b}")

def compareSelectedAgainstReference():
    selected_trajectories = getTrajectoriesFormatted(getSelectedBodyTrajectoryIds())
    
    reference_trajectories = getReferenceFromFile()
    
    print(f"Selected trajectories: {len(selected_trajectories)}")
    print(f"Reference trajectories: {len(reference_trajectories)}")
    
    if len(selected_trajectories) != len(reference_trajectories):
        qtm.gui.message.add_message("Mocap Mimic: Reference capture and current capture are different sizes", "The reference capture saved to file has a different number of labels than the currently selected capture", "error")
        return
    
    for i in range(len(reference_trajectories)):
        for j in range(len(reference_trajectories[i][1]) - 1):
            xDeltaRef = reference_trajectories[i][1][j + 1]["position"][0] - reference_trajectories[i][1][j]["position"][0]
            yDeltaRef = reference_trajectories[i][1][j + 1]["position"][1] - reference_trajectories[i][1][j]["position"][1]
            zDeltaRef = reference_trajectories[i][1][j + 1]["position"][2] - reference_trajectories[i][1][j]["position"][2]
            
            xDeltaSel = selected_trajectories[i][1][j + 1]["position"][0] - selected_trajectories[i][1][j]["position"][0]
            yDeltaSel = selected_trajectories[i][1][j + 1]["position"][1] - selected_trajectories[i][1][j]["position"][1]
            zDeltaSel = selected_trajectories[i][1][j + 1]["position"][2] - selected_trajectories[i][1][j]["position"][2]
        
            corr = (dotProduct(getNormalized([xDeltaRef, yDeltaRef, zDeltaRef]), getNormalized([xDeltaSel, yDeltaSel, zDeltaSel])))
            
            if corr > 0:
                print("Aligned")
            else:
                print("Misaligned")

def TestFunction():
    print("Mocap Mimic: Test")
    qtm.gui.message.add_message("Mocap Mimic: Test", "Test was called in the Mocap Mimic sub menu", "info")

# Root menu option
menu_name = "Mocap Mimic"
my_menu_handle = qtm.gui.insert_menu_submenu(None, menu_name)

# ---------------------
# [BEGIN] TEST BUTTON
# ---------------------

# Setting up the test function
test_function_name = "mocap_mimic_test_function"
qtm.gui.add_command(test_function_name)
qtm.gui.set_command_execute_function(test_function_name, TestFunction)

# Adding it to the menu
qtm.gui.insert_menu_button(my_menu_handle, "Call Test", test_function_name)

# ---------------------
# [END] TEST BUTTON
# ---------------------

# Setting up calc accuracy function
calc_accuracy_name = "mocap_mimic_calculate_accuracy"
qtm.gui.add_command(calc_accuracy_name)
qtm.gui.set_command_execute_function(calc_accuracy_name, calcAccuracy)

# Adding it to the menu
qtm.gui.insert_menu_button(my_menu_handle, "Calculate Accuracy", calc_accuracy_name)

# Setting up save function
save_reference_function_name = "mocap_mimic_save_reference"
qtm.gui.add_command(save_reference_function_name)
qtm.gui.set_command_execute_function(save_reference_function_name, saveSelectedAsReference)

# Adding it to the menu
qtm.gui.insert_menu_button(my_menu_handle, "Save Reference", save_reference_function_name)

# Setting up the test function
selected_to_reference_name = "mocap_mimic_compare_selected_to_reference"
qtm.gui.add_command(selected_to_reference_name)
qtm.gui.set_command_execute_function(selected_to_reference_name, compareSelectedAgainstReference)

# Adding it to the menu
qtm.gui.insert_menu_button(my_menu_handle, "Compare to Reference", selected_to_reference_name)

# measured_range = qtm.gui.timeline.get_measured_range()
# selected_range = qtm.gui.timeline.get_selected_range()
# print(measured_range)
# print(selected_range)