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
import math
from watchdog.observers import Observer
import numpy as np
from watchdog.events import FileSystemEventHandler
from math import cos, sin
from collections import defaultdict
from collections import deque

class GraphManager:
    def __init__(self):
        self.campus_graph = self.create_campus_graph()
        self.pos = self.create_layout()
        self.app = dash.Dash(__name__)
        self.lock = threading.Lock()
        self.setup_dash_layout()

        self.agent_movements = {}
        self.agent_sizes = {
            'ci': 30,
            'vi': 26,
            'bi': 18
        }

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

        self.position_buffers = {agent: deque(maxlen=10) for agent in self.agents}
        self.update_interval = 0.1  # seconds
        self.lobby_positions = {
            'CI Lobby': self.generate_lobby_positions(9),
            'VI Lobby': self.generate_lobby_positions(12)
        }

        self.lobby_radius = 0.5  # Radius of the lobby area
        self.agent_movement_interval = 2.0  # Time between movements in seconds
        self.agent_last_moved = {agent: 0 for agent in self.agents}

        # Initialize agent_movements with initial positions
        for agent_id, agent_data in self.agents.items():
            self.agent_movements[agent_id] = {
                'current': agent_data['pos'],
                'next': agent_data['pos'],
                'start_time': time.time(),
                'completed': True
            }

    def update_agent_position(self, agent_id, new_position):
        with self.lock:
            if agent_id in self.agents:
                current_pos = self.agents[agent_id]['pos']
                self.agents[agent_id]['pos'] = new_position
                self.agent_movements[agent_id] = {
                    'current': current_pos,
                    'next': new_position,
                    'start_time': time.time(),
                    'completed': False
                }
                print(f"Updated position for {agent_id}: {new_position}")
            else:
                print(f"Agent {agent_id} not found.")


    def generate_lobby_positions(self, num_positions):
        base_x, base_y = self.pos['CI Lobby'] if 'CI Lobby' in self.pos else self.pos['VI Lobby']
        positions = []
        for i in range(num_positions):
            angle = 2 * math.pi * i / num_positions
            x = base_x + 0.5 * math.cos(angle)
            y = base_y + 0.5 * math.sin(angle)
            positions.append((x, y))
        return positions
    
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
        

        # List of all dictionaries containing 'Jn' annotations
        annotations = [
            dict(x=-7, y=0),
            dict(x=-5, y=0),
            dict(x=5, y=0),
            dict(x=-4, y=-5),
            dict(x=-6, y=-5),
            dict(x=-4, y=5),
            dict(x=-6, y=5),
            dict(x=4, y=5),
            dict(x=6, y=5),
            dict(x=6, y=-5),
            dict(x=4, y=-5),
        ]

        # Creating a dictionary with 'jn' keys and their corresponding coordinates
        for i, item in enumerate(annotations):
            pos[f"jn{i+1}"] = (item["x"], item["y"])
        
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
                    x=-7, y=0,
                    xref="x", yref="y",
                    text="Jn1",
                    showarrow=False,
                    font=dict(size=10, color="black"),
                    bgcolor="white",
                    opacity=0.8
                ),
                dict(
                    x=-5, y=0,
                    xref="x", yref="y",
                    text="Jn2",
                    showarrow=False,
                    font=dict(size=10, color="black"),
                    bgcolor="white",
                    opacity=0.8
                ),
                dict(
                    x=5, y=0,
                    xref="x", yref="y",
                    text="Jn3",
                    showarrow=False,
                    font=dict(size=10, color="black"),
                    bgcolor="white",
                    opacity=0.8
                ),
                dict(
                    x=-4, y=-5,
                    xref="x", yref="y",
                    text="Jn4",
                    showarrow=False,
                    font=dict(size=10, color="black"),
                    bgcolor="white",
                    opacity=0.8
                ),
                dict(
                    x=-6, y=-5,
                    xref="x", yref="y",
                    text="Jn5",
                    showarrow=False,
                    font=dict(size=10, color="black"),
                    bgcolor="white",
                    opacity=0.8
                ),
                dict(
                    x=-4, y=5,
                    xref="x", yref="y",
                    text="Jn6",
                    showarrow=False,
                    font=dict(size=10, color="black"),
                    bgcolor="white",
                    opacity=0.8
                ),
                dict(
                    x=-6, y=5,
                    xref="x", yref="y",
                    text="Jn7",
                    showarrow=False,
                    font=dict(size=10, color="black"),
                    bgcolor="white",
                    opacity=0.8
                ),
                dict(
                    x=4, y=5,
                    xref="x", yref="y",
                    text="Jn8",
                    showarrow=False,
                    font=dict(size=10, color="black"),
                    bgcolor="white",
                    opacity=0.8
                ),
                dict(
                    x=6, y=5,
                    xref="x", yref="y",
                    text="Jn9",
                    showarrow=False,
                    font=dict(size=10, color="black"),
                    bgcolor="white",
                    opacity=0.8
                ),
                dict(
                    x=6, y=-5,
                    xref="x", yref="y",
                    text="Jn10",
                    showarrow=False,
                    font=dict(size=10, color="black"),
                    bgcolor="white",
                    opacity=0.8
                ),
                dict(
                    x=4, y=-5,
                    xref="x", yref="y",
                    text="Jn11",
                    showarrow=False,
                    font=dict(size=10, color="black"),
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


    def interpolate_position(self, start_pos, end_pos, progress):
        start_x, start_y = self.pos[start_pos]
        end_x, end_y = self.pos[end_pos]
        
        if abs(end_x - start_x) > abs(end_y - start_y):
            # Move horizontally first, then vertically
            if progress < 0.5:
                x = start_x + (end_x - start_x) * (progress * 2)
                y = start_y
            else:
                x = end_x
                y = start_y + (end_y - start_y) * ((progress - 0.5) * 2)
        else:
            # Move vertically first, then horizontally
            if progress < 0.5:
                x = start_x
                y = start_y + (end_y - start_y) * (progress * 2)
            else:
                x = start_x + (end_x - start_x) * ((progress - 0.5) * 2)
                y = end_y
        
        return x, y

    def get_lobby_position(self, agent_id, lobby):
        agent_type = agent_id.split('_')[0]
        base_x, base_y = self.pos[lobby]
        
        current_time = time.time()
        if current_time - self.agent_last_moved[agent_id] > self.agent_movement_interval:
            # Generate a new random position within the lobby
            angle = random.uniform(0, 2 * math.pi)
            radius = random.uniform(0, self.lobby_radius)
            x = base_x + radius * math.cos(angle)
            y = base_y + radius * math.sin(angle)
            self.agent_last_moved[agent_id] = current_time
        else:
            # Use the existing position from the buffer
            x, y = self.position_buffers[agent_id][-1] if self.position_buffers[agent_id] else (base_x, base_y)
        
        self.position_buffers[agent_id].append((x, y))
        return x, y

    def get_agent_traces(self):
        agent_traces = []
        current_time = time.time()
        
        with self.lock:
            for agent, agent_data in self.agents.items():
                movement = self.agent_movements[agent]
                if movement['completed']:
                    if movement['next'] in ['CI Lobby', 'VI Lobby']:
                        x, y = self.get_lobby_position(agent, movement['next'])
                    else:
                        x, y = self.pos[movement['next']]
                else:
                    start_pos = movement['current']
                    end_pos = movement['next']
                    start_time = movement['start_time']
                    duration = 5.0  # Assume it takes 5 seconds to move between nodes
                    progress = min(1.0, (current_time - start_time) / duration)
                    
                    if start_pos in ['CI Lobby', 'VI Lobby']:
                        x, y = self.get_lobby_position(agent, start_pos)
                    elif end_pos in ['CI Lobby', 'VI Lobby']:
                        if progress == 1.0:
                            x, y = self.get_lobby_position(agent, end_pos)
                        else:
                            x, y = self.interpolate_position(start_pos, end_pos, progress)
                    else:
                        x, y = self.interpolate_position(start_pos, end_pos, progress)
                    
                    if progress == 1.0:
                        movement['completed'] = True
                
                agent_type = agent.split('_')[0]
                size = self.agent_sizes[agent_type]
                
                agent_trace = go.Scatter(
                    x=[x], y=[y],
                    mode='markers',
                    marker=dict(
                        symbol=agent_data['symbol'],
                        size=size,
                        color=agent_data['color'],
                        line=dict(width=2, color='DarkSlateGrey')
                    ),
                    name=agent,
                    text=agent,
                    hoverinfo='text'
                )
                agent_traces.append(agent_trace)
        
        return agent_traces

    def setup_dash_layout(self):
        self.app.layout = html.Div([
            dcc.Graph(id='live-graph', animate=True),
            dcc.Interval(
                id='interval-component',
                interval=int(2000),  # in milliseconds
                n_intervals=0
            )
        ])

        @self.app.callback(Output('live-graph', 'figure'), Input('interval-component', 'n_intervals'))
        def update_graph_live(n):
            return self.create_figure()

    def run(self):
        print("Starting Dash server...")
        self.app.run_server(debug=False, use_reloader=False)

if __name__ == "__main__":
    manager = GraphManager()
    manager.run()