from crewai import Task

class EscortVisitorTask(Task):
    def __init__(self):
        """
        Initialize the Escort Visitor Task.
        This task is responsible for guiding the visitor from the campus entrance
        to the building entrance and then requesting navigation assistance from the
        BI agent to guide the visitor to the host inside the building.
        """
        # Fix: Corrected the __init__ method with double underscores
        super().__init__(
            description='Escort visitor {visitor_id} to building {building_id} entrance and then to the host.',
            expected_output='Visitor successfully escorted to the host.'
        )
        
        # Ensure that the '_at_building_entrance' field is correctly initialized
        self._at_building_entrance = False
        print("EscortVisitorTask initialized: _at_building_entrance set to", self._at_building_entrance)

    def execute(self, inputs, send_navigation_request, receive_navigation_response):
        """
        Execute the task of escorting the visitor.

        inputs: A dictionary containing the visitor and building information.
        send_navigation_request: Function to request navigation from the BI agent.
        receive_navigation_response: Function to receive and handle the navigation path from the BI agent.
        """
        visitor_id = inputs.get('visitor_id')
        building_id = inputs.get('building_id')

        # Phase 1: Escort the visitor to the building entrance
        self.guide_to_building_entrance(visitor_id, building_id)

        # Phase 2: Request navigation from BI agent once at the entrance
        if self._at_building_entrance:
            send_navigation_request(visitor_id, building_id)

        # Wait for the BI agent to provide navigation
        navigation_path = receive_navigation_response()

        # Phase 3: Escort the visitor to the host using the provided navigation path
        if navigation_path:
            self.guide_to_host(visitor_id, building_id, navigation_path)

        return f"Visitor {visitor_id} successfully escorted to the host in building {building_id}."

    def guide_to_building_entrance(self, visitor_id, building_id):
        """
        Phase 1: Simulate guiding the visitor to the building entrance.
        """
        print(f"CI Agent: Escorting visitor {visitor_id} to building {building_id} entrance...")
        # Simulate the movement to the building entrance
        import time
        time.sleep(2)  # Simulate some delay as the visitor moves
        self._at_building_entrance = True
        print(f"CI Agent: Reached the building {building_id} entrance with visitor {visitor_id}.")
        print(f"_at_building_entrance set to {self._at_building_entrance}")

    def guide_to_host(self, visitor_id, building_id, navigation_path):
        """
        Phase 3: Escort the visitor inside the building using the provided navigation path.
        navigation_path: The internal navigation directions from the BI agent.
        """
        print(f"CI Agent: Navigating inside the building {building_id} using the provided path...")
        # Simulate guiding the visitor through the building to the host
        print(f"CI Agent: Successfully escorted visitor {visitor_id} to the host in building {building_id}.")