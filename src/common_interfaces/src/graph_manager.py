import networkx as nx
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import dash
from dash import dcc, html
from dash.dependencies import Input, Output, State
import threading
import time
import json
import os
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

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
        self.json_file = "/home/nachiketa/dup_auto_ass1/src/data/positions.json"
        self.last_modified_time = 0
        self.agents = {
            'ci_agent_1': {'pos': 'Entrance', 'color': 'red', 'symbol': 'star'},
            'bi_agent_A': {'pos': 'Building A', 'color': 'blue', 'symbol': 'circle'},
            'bi_agent_B': {'pos': 'Building B', 'color': 'green', 'symbol': 'circle'},
            'bi_agent_C': {'pos': 'Building C', 'color': 'purple', 'symbol': 'circle'}
        }

        self.setup_file_watcher()

    def create_campus_graph(self):
        campus_graph = nx.DiGraph()
        campus_graph.add_node('Entrance', type='entrance')
        buildings = {
            'Building A': ['Room A1', 'Room A2', 'Room A3'],
            'Building B': ['Room B1', 'Room B2'],
            'Building C': ['Room C1', 'Room C2', 'Room C3', 'Room C4']
        }
        for building, rooms in buildings.items():
            campus_graph.add_node(building, type='building')
            campus_graph.add_edge('Entrance', building)
            for room in rooms:
                campus_graph.add_node(room, type='room')
                campus_graph.add_edge(building, room)
        return campus_graph

    def create_layout(self):
        pos = nx.spring_layout(self.campus_graph, k=1.2, iterations=50)
        # Define building positions
        pos['Entrance'] = (0, 1)
        pos['Building A'] = (-1, 0)
        pos['Building B'] = (0, -1)
        pos['Building C'] = (1, 0)
       
        # Adjust room positions near their respective buildings
        building_offsets = {
            'Building A': (-1, 0),
            'Building B': (0, -1),
            'Building C': (1, 0),
        }
        room_offsets = {
            'Building A': [(-1.2, 0.6), (-1.2, -0.4), (-0.8, 0.8)],
            'Building B': [(0.2, -1.2), (-0.2, -1.2)],
            'Building C': [(1.2, 0.4), (1.2, -0.5), (0.8, 0.8), (0.8, -0.7)]
        }
       
        for building, rooms in room_offsets.items():
            for i, room in enumerate(self.campus_graph.neighbors(building)):
                pos[room] = (building_offsets[building][0] + rooms[i][0], building_offsets[building][1] + rooms[i][1])

        return pos

    def get_node_trace(self):
        node_x, node_y = [], []
        for node in self.campus_graph.nodes():
            x, y = self.pos[node]
            node_x.append(x)
            node_y.append(y)
       
        node_trace = go.Scatter(
            x=node_x, y=node_y,
            mode='markers+text',
            hoverinfo='text',
            marker=dict(
                showscale=True,
                colorscale='YlGnBu',
                size=20,
                colorbar=dict(
                    thickness=15,
                    title='Node Connections',
                    xanchor='left',
                    titleside='right'
                ),
                line_width=2
            ),
            text=[node for node in self.campus_graph.nodes()],
            textposition="top center"
        )

        node_colors = []
        node_sizes = []
        for node in self.campus_graph.nodes():
            if self.campus_graph.nodes[node]['type'] == 'entrance':
                node_colors.append('yellow')
            elif self.campus_graph.nodes[node]['type'] == 'building':
                node_colors.append('lightblue')
            else:
                node_colors.append('lightgreen')
            node_sizes.append(20)

        node_trace.marker.color = node_colors
        node_trace.marker.size = node_sizes

        return node_trace

    def get_edge_trace(self):
        edge_x, edge_y = [], []
        for edge in self.campus_graph.edges():
            x0, y0 = self.pos[edge[0]]
            x1, y1 = self.pos[edge[1]]
            edge_x.extend([x0, x1, None])
            edge_y.extend([y0, y1, None])

        edge_trace = go.Scatter(
            x=edge_x, y=edge_y,
            line=dict(width=0.5, color='#888'),
            hoverinfo='none',
            mode='lines')

        return edge_trace

    def get_agent_traces(self):
        agent_traces = []
        with self.lock:
            for agent_id, agent_data in self.agents.items():
                agent_x, agent_y = self.pos[agent_data['pos']]
                agent_x, agent_y = agent_x + 0.05, agent_y + 0.05
                agent_trace = go.Scatter(
                    x=[agent_x], y=[agent_y],
                    mode='markers',
                    marker=dict(
                        symbol=agent_data['symbol'],
                        size=30,
                        color=agent_data['color'],
                        line=dict(width=2, color='black')
                    ),
                    name=agent_id
                )
                agent_traces.append(agent_trace)
        return agent_traces

    def create_figure(self):
        fig = make_subplots(rows=1, cols=1)
        fig.add_trace(self.get_edge_trace())
        fig.add_trace(self.get_node_trace())
        for agent_trace in self.get_agent_traces():
            fig.add_trace(agent_trace)
        fig.update_layout(
            title='Campus Navigation Graph',
            titlefont_size=16,
            showlegend=True,
            hovermode='closest',
            margin=dict(b=20, l=5, r=5, t=40),
            xaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
            yaxis=dict(showgrid=False, zeroline=False, showticklabels=False))
        return fig

    def setup_dash_layout(self):
        self.app.layout = html.Div([
            dcc.Graph(id='live-graph', animate=False),
            dcc.Interval(
                id='interval-component',
                interval=1 * 10000,  # in milliseconds
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
                            self.agents[agent_id]['navigation_path'] = agent_data.get('navigation_path', [])

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

# Usage
if __name__ == "__main__":
    manager = GraphManager()
    manager.run()