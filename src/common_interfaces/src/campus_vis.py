import networkx as nx
import matplotlib.pyplot as plt

def create_campus_graph():
    # Initialize a directed graph
    campus_graph = nx.DiGraph()

    # Add the entrance node
    campus_graph.add_node('Entrance', type='entrance')

    # Add buildings and rooms
    buildings = {
        'Building A': ['Room A1', 'Room A2', 'Room A3'],
        'Building B': ['Room B1', 'Room B2'],
        'Building C': ['Room C1', 'Room C2', 'Room C3', 'Room C4']
    }

    # Add buildings and connect them to the entrance
    for building, rooms in buildings.items():
        campus_graph.add_node(building, type='building')
        campus_graph.add_edge('Entrance', building)  # Connect entrance to building

        # Add rooms and connect them to their building
        for room in rooms:
            campus_graph.add_node(room, type='room')
            campus_graph.add_edge(building, room)  # Connect building to room

    return campus_graph

def visualize_campus_graph(graph):
    # Set up node colors and sizes based on their type
    node_colors = []
    node_sizes = []
    for node, data in graph.nodes(data=True):
        if data['type'] == 'entrance':
            node_colors.append('red')
            node_sizes.append(1200)  # Size for entrance
        elif data['type'] == 'building':
            node_colors.append('blue')
            node_sizes.append(3000)  # Larger size for buildings
        elif data['type'] == 'room':
            node_colors.append('green')
            node_sizes.append(500)  # Smaller size for rooms

    # Create a layout that separates buildings and arranges rooms around them
    pos = nx.spring_layout(graph, seed=42)  # Positions for all nodes

    # Manually adjust positions to cluster rooms around their buildings
    offset = {
        'Building A': (0, 0),
        'Building B': (2, 0),
        'Building C': (-2, 0),
    }
    room_offsets = {
        'Room A1': (-0.5, 0.5), 'Room A2': (0.5, 0.5), 'Room A3': (0, -0.5),
        'Room B1': (-0.5, 0.5), 'Room B2': (0.5, 0.5),
        'Room C1': (-0.5, 0.5), 'Room C2': (0.5, 0.5), 'Room C3': (-0.5, -0.5), 'Room C4': (0.5, -0.5)
    }

    # Adjust positions for buildings
    for building, (x_offset, y_offset) in offset.items():
        pos[building] = (x_offset, y_offset)
    
    # Adjust positions for rooms relative to their buildings
    for room, (x_room_offset, y_room_offset) in room_offsets.items():
        building = room.split()[0] + ' ' + room.split()[1][0]  # e.g., 'Room A1' -> 'Building A'
        building_pos = pos[building]
        pos[room] = (building_pos[0] + x_room_offset, building_pos[1] + y_room_offset)

    # Draw the graph
    nx.draw(graph, pos, with_labels=True, node_color=node_colors, node_size=node_sizes, font_size=8, font_color='white', font_weight='bold', arrows=True)

    # Display the graph
    plt.title('Campus Navigation Graph')
    plt.show()

# Main code to create and visualize the campus graph
campus_graph = create_campus_graph()
visualize_campus_graph(campus_graph)