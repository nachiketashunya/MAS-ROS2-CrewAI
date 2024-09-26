from crewai import Task

class HandleNavigationTask(Task):
    def __init__(self):
        """
        Initialize the Handle Navigation Task.
        This task is responsible for providing navigation paths within the building
        and responding to CI agent requests.
        """
        super().__init__(
            description='Provide navigation paths within the building for visitors escorted by CI agents.',
            expected_output='Navigation path or response provided to CI agent.'
        )

    def execute(self, request_data, send_navigation_response):
        """
        Execute the task of handling the navigation request.

        request_data: Information regarding the navigation request from CI agent.
        send_navigation_response: Function to send navigation response back to the CI agent.
        """
        print(f"Handling navigation request: {request_data}")

        # Simulate providing a navigation path
        navigation_path = "Path to the requested room inside the building."
        send_navigation_response(navigation_path)

        return f"Handled navigation request and provided path: {navigation_path}"