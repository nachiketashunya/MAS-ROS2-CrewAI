import threading
from graph_manager import GraphManager

def update_agent(manager, agent_id, new_position):
    manager.update_agent_position(agent_id, new_position)

manager = GraphManager()

# Start the Dash server in a separate thread
dash_thread = threading.Thread(target=manager.run)
dash_thread.start()

# Update agent positions from different threads
thread1 = threading.Thread(target=update_agent, args=(manager, 'ci_agent_1', 'Building A Entrance'))
thread2 = threading.Thread(target=update_agent, args=(manager, 'vi_agent_1', 'Building B Entrance'))

thread1.start()
thread2.start()

thread1.join()
thread2.join()

# Keep the main thread running
dash_thread.join()