import networkx as nx
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import dash
from dash import dcc, html
from dash.dependencies import Input, Output, State
import threading
import time
import json

class GraphManager:
    def __init__(self):
        self.campus_graph = self.create_campus_graph()
        self.agent_pos = 'Entrance'  # Initial agent position
        self.pos = self.create_layout()
        self.app = dash.Dash(__name__)
        self.lock = threading.Lock()  # Lock to synchronize access to agent_pos
        self.setup_dash_layout()
        self.json_file = "/home/nachiketa/dup_auto_ass1/src/data/positions.json"

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
        with self.lock:  # Use lock to safely read the agent's position
            for node in self.campus_graph.nodes():
                if node == self.agent_pos:
                    node_colors.append('red')
                elif self.campus_graph.nodes[node]['type'] == 'entrance':
                    node_colors.append('yellow')
                elif self.campus_graph.nodes[node]['type'] == 'building':
                    node_colors.append('blue')
                else:
                    node_colors.append('green')

        node_trace.marker.color = node_colors
        node_trace.marker.size = [30 if node == self.agent_pos else 20 for node in self.campus_graph.nodes()]

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

    def get_agent_trace(self):
        with self.lock:  # Use lock to safely read the agent's position
            agent_x, agent_y = self.pos[self.agent_pos]
       
        # Add a small offset to the agent position for visibility
        agent_x, agent_y = agent_x + 0.05, agent_y + 0.05

        agent_trace = go.Scatter(
            x=[agent_x], y=[agent_y],
            mode='markers',
            marker=dict(
                symbol='star',  # Shape of the agent marker
                size=30,
                color='red',  # Color of the agent marker
                line=dict(width=2, color='black')
            ),
            name='Agent'
        )
        return agent_trace

    def create_figure(self):
        fig = make_subplots(rows=1, cols=1)
        fig.add_trace(self.get_edge_trace())
        fig.add_trace(self.get_node_trace())
        fig.add_trace(self.get_agent_trace())  # Add agent trace
        fig.update_layout(
            title='Campus Navigation Graph',
            titlefont_size=16,
            showlegend=False,
            hovermode='closest',
            margin=dict(b=20, l=5, r=5, t=40),
            xaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
            yaxis=dict(showgrid=False, zeroline=False, showticklabels=False))
        return fig

    def setup_dash_layout(self):
        self.app.layout = html.Div([
            dcc.Graph(id='live-graph', animate=True),
            dcc.Interval(
                id='interval-component',
                interval=1 * 1000,  # in milliseconds
                n_intervals=0
            )
        ])

        @self.app.callback(Output('live-graph', 'figure'), Input('interval-component', 'n_intervals'))
        def update_graph_live(n):
            self.read_pos_from_json()
            return self.create_figure()
    
    def read_pos_from_json(self):
        try:
            with open(self.json_file, "r") as f:
                data = json.load(f)
                with self.lock:
                    self.agent_pos = data.get('position', 'Entrance')
                
            print(f"Read new position from json {self.agent_pos}")
        
        except Exception as e:
            print(f"Error reading from JSON file: {e}")       


    def run(self):
        print("Starting Dash server...")  # Debug print
        self.app.run_server(debug=True, use_reloader=False)

# Usage
if __name__ == "__main__":
    manager = GraphManager()
    manager.run()
    # Start the graph manager in a separate thread
    # graph_thread = threading.Thread(target=manager.run)
    # graph_thread.daemon = True
    # graph_thread.start()

    # # Simulate changing the agent's position over time
    # positions = ['Building A', 'Room A1', 'Room A2', 'Building B', 'Room B1', 'Entrance']
    # for pos in positions:
    #     time.sleep(2)  # Delay to simulate real-time updates
    #     manager.update_agent_position(pos)