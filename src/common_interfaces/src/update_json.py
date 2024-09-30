import json
import os
from filelock import FileLock

def write_pos_to_json(agent_id, next_pos, navigation_path=None):
    json_file = "/home/nachiketa/dup_auto_ass1/src/data/positions.json"
    lock_file = f"{json_file}.lock"
    
    with FileLock(lock_file):
        try:
            # Ensure the directory exists
            # os.makedirs(os.path.dirname(json_file), exist_ok=True)
            
            # Load the existing JSON data
            try:
                with open(json_file, "r") as f:
                    data = json.load(f)
            except (FileNotFoundError, json.JSONDecodeError):
                # Initialize with default positions if file doesn't exist or is empty
                data = {
                    "agents": {
                        "ci_agent_1": {"position": "CI Lobby", "next_position": "CI Lobby"},
                        "ci_agent_2": {"position": "CI Lobby", "next_position": "CI Lobby"},
                        "ci_agent_3": {"position": "CI Lobby", "next_position": "CI Lobby"},
                        "vi_agent_1": {"position": "VI Lobby", "next_position": "VI Lobby"},
                        "vi_agent_2": {"position": "VI Lobby", "next_position": "VI Lobby"},
                        "vi_agent_3": {"position": "VI Lobby", "next_position": "VI Lobby"},
                        "vi_agent_4": {"position": "VI Lobby", "next_position": "VI Lobby"},
                        "vi_agent_5": {"position": "VI Lobby", "next_position": "VI Lobby"},
                        "vi_agent_6": {"position": "VI Lobby", "next_position": "VI Lobby"},
                        "vi_agent_7": {"position": "VI Lobby", "next_position": "VI Lobby"},
                        "bi_agent_A": {"position": "Building A Entrance", "next_position": "Building A Entrance"},
                        "bi_agent_B": {"position": "Building B Entrance", "next_position": "Building B Entrance"},
                        "bi_agent_C": {"position": "Building C Entrance", "next_position": "Building C Entrance"},
                        "bi_agent_D": {"position": "Building D Entrance", "next_position": "Building D Entrance"}
                    }
                }

            # Update the agent's position, next position, and navigation path

            current_pos = data["agents"][agent_id]["next_position"]
            if agent_id not in data["agents"]:
                data["agents"][agent_id] = {"position": current_pos, "next_position": next_pos, "navigation_path": []}
            else:
                data["agents"][agent_id]["position"] = current_pos
                data["agents"][agent_id]["next_position"] = next_pos
            
            if navigation_path is not None:
                data["agents"][agent_id]["navigation_path"] = navigation_path

            # Write the updated data back to the JSON file
            with open(json_file, "w") as f:
                json.dump(data, f, indent=4)
            
            print(f"Updated JSON file for {agent_id}: Position={current_pos}, Next Position={next_pos}, Navigation Path={navigation_path}")
        
        except Exception as e:
            print(f"Error updating JSON file for {agent_id}: {str(e)}")