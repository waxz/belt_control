class ConveyorBeltAgent:
    """
    Conveyor Belt Agent.
    Executes the motion profiles and drives the physical belt.
    """
    def __init__(self, a_max: float, initial_v: float = 0.0):
        self.v_belt = initial_v
        self.x_belt = 0.0
        self.a_max = a_max

    def update(self, dt: float, v_cmd: float):
        """
        Updates the physical state of the conveyor belt.
        For simplicity, in ideal conditions, the belt velocity tracks the command,
        but we could also model motor lag, slip, or limits here.
        """
        # Command input
        self.v_belt = v_cmd
        self.x_belt += self.v_belt * dt
