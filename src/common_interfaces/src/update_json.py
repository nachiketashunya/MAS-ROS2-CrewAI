import json
import os
from filelock import FileLock

def write_pos_to_json(agent_id, new_pos, navigation_path=None):
    json_file = "/home/nachiketa/dup_auto_ass1/src/data/positions.json"
    lock_file = f"{json_file}.lock"
    
    with FileLock(lock_file):
        try:
            # Ensure the directory exists
            os.makedirs(os.path.dirname(json_file), exist_ok=True)
            
            # Load the existing JSON data
            try:
                with open(json_file, "r") as f:
                    data = json.load(f)
            except (FileNotFoundError, json.JSONDecodeError):
                data = {"agents": {}}

            # Update the agent's position and navigation path
            if agent_id not in data["agents"]:
                data["agents"][agent_id] = {"position": new_pos, "navigation_path": []}
            
            data["agents"][agent_id]["position"] = new_pos
            
            if navigation_path is not None:
                data["agents"][agent_id]["navigation_path"] = navigation_path

            # Write the updated data back to the JSON file
            with open(json_file, "w") as f:
                json.dump(data, f, indent=4)
            
            print(f"Updated JSON file for {agent_id}: Position={new_pos}, Navigation Path={navigation_path}")
        
        except Exception as e:
            print(f"Error updating JSON file for {agent_id}: {str(e)}")
