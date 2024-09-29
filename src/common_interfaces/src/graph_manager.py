import networkx as nx
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import threading
import time
import json
import os
import random
from watchdog.observers import Observer
import numpy as np
from watchdog.events import FileSystemEventHandler
from math import cos, sin

class JSONFileHandler(FileSystemEventHandler):
    def __init__(self, callback):
        self.callback = callback

    def on_modified(self, event):
        if not event.is_directory and event.src_path.endswith("positions.json"):
            self.callback()

class GraphManager:
    def __init__(self):
        self.campus_graph = self.create_campus_graph()
        self.pos = self.create_layout()
        self.app = dash.Dash(__name__)
        self.lock = threading.Lock()
        self.setup_dash_layout()
        self.json_file = "/home/nachiketa/dup_auto_ass1/src/data/positions.json"  # Update this path as needed
        self.last_modified_time = 0

        # Define agents with positions and symbols
        self.agents = {
            'ci_agent_1': {'pos': 'CI Lobby', 'color': 'red', 'symbol': 'star'},
            'ci_agent_2': {'pos': 'CI Lobby', 'color': 'red', 'symbol': 'star'},
            'ci_agent_3': {'pos': 'CI Lobby', 'color': 'red', 'symbol': 'star'},
            'vi_agent_1': {'pos': 'VI Lobby', 'color': 'orange', 'symbol': 'diamond'},
            'vi_agent_2': {'pos': 'VI Lobby', 'color': 'orange', 'symbol': 'diamond'},
            'vi_agent_3': {'pos': 'VI Lobby', 'color': 'orange', 'symbol': 'diamond'},
            'vi_agent_4': {'pos': 'VI Lobby', 'color': 'orange', 'symbol': 'diamond'},
            'vi_agent_5': {'pos': 'VI Lobby', 'color': 'orange', 'symbol': 'diamond'},
            'vi_agent_6': {'pos': 'VI Lobby', 'color': 'orange', 'symbol': 'diamond'},
            'vi_agent_7': {'pos': 'VI Lobby', 'color': 'orange', 'symbol': 'diamond'},
            'bi_agent_A': {'pos': 'Building A Entrance', 'color': 'blue', 'symbol': 'circle'},
            'bi_agent_B': {'pos': 'Building B Entrance', 'color': 'green', 'symbol': 'circle'},
            'bi_agent_C': {'pos': 'Building C Entrance', 'color': 'purple', 'symbol': 'circle'},
            'bi_agent_D': {'pos': 'Building D Entrance', 'color': 'cyan', 'symbol': 'circle'}
        }

        self.setup_file_watcher()

    def create_campus_graph(self):
        campus_graph = nx.Graph()
        campus_graph.add_node('Campus Entrance', type='entrance')
        campus_graph.add_node('CI Lobby', type='lobby')
        campus_graph.add_node('VI Lobby', type='lobby')
        buildings = {
            'Building A': ['Room A1', 'Room A2', 'Room A3', 'Room A4'],
            'Building B': ['Room B1', 'Room B2', 'Room B3'],
            'Building C': ['Room C1', 'Room C2', 'Room C3', 'Room C4'],
            'Building D': ['Room D1', 'Room D2']
        }
        
        # Add building entrances and connect to campus entrance
        for building in buildings:
            entrance = f"{building} Entrance"
            campus_graph.add_node(entrance, type='building_entrance')
            campus_graph.add_edge('Campus Entrance', entrance)
        
        # Add edge between 'Building A Entrance' and 'Building B Entrance'
        campus_graph.add_edge('Building A Entrance', 'Building B Entrance')

        # Connect lobbies to campus entrance
        campus_graph.add_edge('Campus Entrance', 'CI Lobby')
        campus_graph.add_edge('Campus Entrance', 'VI Lobby')
        
        # Add rooms and connect to building entrances
        for building, rooms in buildings.items():
            entrance = f"{building} Entrance"
            for room in rooms:
                campus_graph.add_node(room, type='room')
                campus_graph.add_edge(entrance, room)
        
        return campus_graph

    def create_layout(self):
        pos = {}
        pos['Campus Entrance'] = (-9, 0)  # Move campus entrance to the left boundary
        pos['CI Lobby'] = (-7, -2)
        pos['VI Lobby'] = (-7, 2)
        
        building_positions = {
            'Building A': (-5, 5),
            'Building B': (5, 5),
            'Building C': (-5, -5),
            'Building D': (5, -5)
        }

        buildings = {
            'Building A': ['Room A1', 'Room A2', 'Room A3', 'Room A4'],
            'Building B': ['Room B1', 'Room B2', 'Room B3'],
            'Building C': ['Room C1', 'Room C2', 'Room C3', 'Room C4'],
            'Building D': ['Room D1', 'Room D2']
        }
        
        pos['Building A Entrance'] = (-5, 5)
        pos['Room A1'] = (-6, 6)
        pos['Room A2'] = (-6, 4)
        pos['Room A3'] = (-4, 6)
        pos['Room A4'] = (-4, 4)


        pos['Building B Entrance'] = (5, 5)
        pos['Room B1'] = (6, 6)
        pos['Room B2'] = (6, 4)
        pos['Room B3'] = (4, 6)

        pos['Building C Entrance'] = (-5, -5)
        pos['Room C1'] = (-6, -6)
        pos['Room C2'] = (-6, -4)
        pos['Room C3'] = (-4, -6)
        pos['Room C4'] = (-4, -4)

        pos['Building D Entrance'] = (5, -5)
        pos['Room D1'] = (6, -6)
        pos['Room D2'] = (4, -4)
       
        # # Define the size of the rectangle around the building (1.5 units left, right, up, down)
        # building_box_size = 5.0  # The width/height of the rectangle around the building (1.5 units on each side)
        
        # for building, (x, y) in building_positions.items():
        #     entrance = f"{building} Entrance"
            
        #     # Place the entrance at the top of the rectangle
        #     pos[entrance] = (x, y + building_box_size / 2)
            
        #     # Get the rooms corresponding to the building
        #     rooms = buildings[building]
        #     room_count = len(rooms)
            
        #     # Calculate grid dimensions based on the number of rooms
        #     cols = int(room_count ** 0.5) + 1  # Adjust the number of columns for room placement
        #     rows = (room_count + cols - 1) // cols  # Calculate the number of rows
            
        #     # Calculate the spacing between rooms based on the size of the building box
        #     spacing_x = building_box_size / cols
        #     spacing_y = building_box_size / rows
            
        #     for i, room in enumerate(rooms):
        #         row = i // cols
        #         col = i % cols
                
        #         # Calculate the x and y positions within the rectangle for the rooms
        #         room_x = x - building_box_size / 2 + col * spacing_x + spacing_x / 2
        #         room_y = y - building_box_size / 2 + row * spacing_y + spacing_y / 2
                
        #         pos[room] = (room_x, room_y)
        
        return pos

    def get_edge_trace(self):
        edge_traces = []
        
        for edge in self.campus_graph.edges():
            x0, y0 = self.pos[edge[0]]
            x1, y1 = self.pos[edge[1]]
            
            # Create path with 90-degree turns
            path_x = [x0]
            path_y = [y0]
            
            if abs(x1 - x0) > 0.1 and abs(y1 - y0) > 0.1:
                path_x.append(x1)
                path_y.append(y0)
            
            path_x.append(x1)
            path_y.append(y1)
            
            edge_trace = go.Scatter(
                x=path_x, y=path_y,
                line=dict(width=2, color='#888'),
                hoverinfo='none',
                mode='lines')
            
            edge_traces.append(edge_trace)

        return edge_traces

    def create_figure(self):
        fig = go.Figure()

        # Add edge traces first
        for edge_trace in self.get_edge_trace():
            fig.add_trace(edge_trace)
        
        # Add node trace (with corrected labels and positions)
        fig.add_trace(self.get_node_trace())

        # Add agent traces
        for agent_trace in self.get_agent_traces():
            fig.add_trace(agent_trace)

        # Update layout and annotations for buildings, park, cafe, etc.
        fig.update_layout(
            title='Campus Navigation Graph',
            titlefont_size=24,  # Increase title font size
            showlegend=True,
            hovermode='closest',
            margin=dict(b=20, l=5, r=5, t=40),
            width=1400,  # Increase figure width
            height=600,  # Increase figure height
            xaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
            yaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
            shapes=self.get_building_boxes(),
            annotations=[
                dict(
                    x=x, y=y,
                    xref="x", yref="y",
                    text=building,
                    showarrow=False,
                    font=dict(size=16, color="black"),  # Increase annotation font size
                    bgcolor="white",
                    opacity=0.8
                )
                for building, (x, y) in {
                    'Building A': (-5, 6.5), 
                    'Building B': (5, 6.5), 
                    'Building C': (-5, -6.5), 
                    'Building D': (5, -6.5)}.items()
            ] + [
                dict(
                    x=-7, y=-3,
                    xref="x", yref="y",
                    text="CI Lobby",
                    showarrow=False,
                    font=dict(size=16, color="black"),
                    bgcolor="white",
                    opacity=0.8
                ),
                dict(
                    x=-7, y=3,
                    xref="x", yref="y",
                    text="VI Lobby",
                    showarrow=False,
                    font=dict(size=16, color="black"),
                    bgcolor="white",
                    opacity=0.8
                ),
                dict(
                    x=1, y=4,
                    xref="x", yref="y",
                    text="Shamiyana",
                    showarrow=False,
                    font=dict(size=16, color="black"),
                    bgcolor="white",
                    opacity=0.8
                ),
                dict(
                    x=-1, y=-6,
                    xref="x", yref="y",
                    text="Sports Complex",
                    showarrow=False,
                    font=dict(size=16, color="black"),
                    bgcolor="white",
                    opacity=0.8
                ),
                # New Lecture Hall Complex annotation
                dict(
                    x=1, y=-4,
                    xref="x", yref="y",
                    text="LHC",
                    showarrow=False,
                    font=dict(size=16, color="black"),
                    bgcolor="white",
                    opacity=0.8
                )
            ]
        )

        return fig

    def get_building_boxes(self):
        shapes = []
        for building, (x, y) in {'Building A': (-5, 5), 'Building B': (5, 5), 'Building C': (-5, -5), 'Building D': (5, -5)}.items():
            shapes.append(dict(
                type="rect",
                x0=x-1.5, y0=y-1.5, x1=x+1.5, y1=y+1.5,
                line=dict(color="RoyalBlue", width=2),
                fillcolor="LightSkyBlue",
                opacity=0.3
            ))
        
        # Add large squares for the lobbies
        shapes.append(dict(
            type="rect",
            x0=-8, y0=-3, x1=-6, y1=-1,
            line=dict(color="Orange", width=2),
            fillcolor="LightSalmon",
            opacity=0.3
        ))
        shapes.append(dict(
            type="rect",
            x0=-8, y0=1, x1=-6, y1=3,
            line=dict(color="Orange", width=2),
            fillcolor="LightSalmon",
            opacity=0.3
        ))
        
        # Add small boxes for rooms
        for room, (x, y) in self.pos.items():
            if room.startswith('Room'):
                shapes.append(dict(
                    type="rect",
                    x0=x-0.3, y0=y-0.3, x1=x+0.3, y1=y+0.3,
                    line=dict(color="Green", width=1),
                    fillcolor="LightGreen",
                    opacity=0.3
                ))
        
        # Add a cafe
        shapes.append(dict(
            type="rect",
            x0=0, y0=2, x1=2, y1=4,
            line=dict(color="Brown", width=2),
            fillcolor="Tan",
            opacity=0.5
        ))
        
        # Add a park
        shapes.append(dict(
            type="circle",
            x0=-2, y0=-4, x1=0, y1=-6,
            line=dict(color="ForestGreen", width=2),
            fillcolor="LightGreen",
            opacity=0.5
        ))

        # Add Lecture Hall Complex (new box)
        shapes.append(dict(
            type="rect",
            x0=0, y0=-2, x1=2, y1=-4,
            line=dict(color="Navy", width=2),
            fillcolor="LightSteelBlue",
            opacity=0.5
        ))
        
        return shapes

    def get_agent_traces(self):
        agent_traces = []
        for agent, info in self.agents.items():
            if info['pos'] in ['CI Lobby', 'VI Lobby']:
                # Randomly position agents within the lobby
                base_x, base_y = self.pos[info['pos']]
                x = base_x + (random.random() - 0.5)
                y = base_y + (random.random() - 0.5)
            else:
                x, y = self.pos[info['pos']]
            
            agent_trace = go.Scatter(
                x=[x], y=[y],
                mode='markers',
                marker=dict(
                    symbol=info['symbol'],
                    size=15,
                    color=info['color'],
                    line=dict(width=2, color='DarkSlateGrey')
                ),
                name=agent,
                text=agent,
                hoverinfo='text'
            )
            agent_traces.append(agent_trace)
        
        return agent_traces
    
    def get_node_trace(self):
        # Use the exact order of nodes in self.campus_graph to ensure alignment
        node_x = []
        node_y = []
        node_text = []
        
        for node in self.campus_graph.nodes():
            x, y = self.pos[node]  # Ensure the position corresponds to the node label
            node_x.append(x)
            node_y.append(y)
            node_text.append(node)  # Store the correct node label
        
        node_trace = go.Scatter(
            x=node_x, y=node_y,
            mode='markers+text',
            hoverinfo='text',
            marker=dict(
                showscale=False,
                colorscale='YlGnBu',
                size=10,
                colorbar=dict(thickness=15, title='Node Connections', xanchor='left', titleside='right'),
                line_width=2
            ),
            text=node_text,  # Ensure the correct label is displayed for each node
            textposition="top center"
        )

        # Coloring and sizing logic
        node_colors = []
        node_sizes = []
        
        for node in self.campus_graph.nodes():
            node_type = self.campus_graph.nodes[node]['type']
            if node_type == 'entrance':
                node_colors.append('yellow')
                node_sizes.append(20)
            elif node_type == 'building_entrance':
                node_colors.append('lightblue')
                node_sizes.append(15)
            elif node_type == 'lobby':
                node_colors.append('orange')
                node_sizes.append(20)
            else:  # rooms
                node_colors.append('lightgreen')
                node_sizes.append(10)
        
        node_trace.marker.color = node_colors
        node_trace.marker.size = node_sizes
        
        return node_trace

    def setup_dash_layout(self):
        self.app.layout = html.Div([
            dcc.Graph(id='live-graph', animate=False),
            dcc.Interval(
                id='interval-component',
                interval=1000,  # in milliseconds
                n_intervals=0
            )
        ])

        @self.app.callback(Output('live-graph', 'figure'), Input('interval-component', 'n_intervals'))
        def update_graph_live(n):
            return self.create_figure()

    def read_pos_from_json(self):
        try:
            current_modified_time = os.path.getmtime(self.json_file)
            if current_modified_time > self.last_modified_time:
                with open(self.json_file, "r") as f:
                    data = json.load(f)

                with self.lock:
                    agents_data = data.get('agents', {})
                    for agent_id, agent_data in agents_data.items():
                        if agent_id in self.agents:
                            self.agents[agent_id]['pos'] = agent_data.get('position', self.agents[agent_id]['pos'])

                self.last_modified_time = current_modified_time
                print(f"Updated positions: {self.agents}")

        except FileNotFoundError:
            print(f"Error: {self.json_file} not found.")
        except Exception as e:
            print(f"Error reading from JSON file: {e}")

    def setup_file_watcher(self):
        event_handler = JSONFileHandler(self.read_pos_from_json)
        observer = Observer()
        observer.schedule(event_handler, path=os.path.dirname(self.json_file), recursive=False)
        observer.start()

    def run(self):
        print("Starting Dash server...")
        self.app.run_server(debug=False, use_reloader=False)

if __name__ == "__main__":
    manager = GraphManager()
    manager.run()