import json

def write_pos_to_json(agent_id, new_pos, navigation_path=None):
    # Load the existing JSON data
    json_file = "/home/nachiketa/dup_auto_ass1/src/data/positions.json"
    try:
        with open(json_file, "r") as f:
            data = json.load(f)
    except FileNotFoundError:
        data = {"agents": {}}

    # Update the agent's position and navigation path
    if agent_id not in data["agents"]:
        data["agents"][agent_id] = {"position": new_pos, "navigation_path": []}
    
    data["agents"][agent_id]["position"] = new_pos
    
    if navigation_path:
        data["agents"][agent_id]["navigation_path"] = navigation_path

    # Write the updated data back to the JSON file
    with open(json_file, "w") as f:
        json.dump(data, f, indent=4)
    
    print(f"Updated JSON file for {agent_id}: Position={new_pos}, Navigation Path={navigation_path}")
