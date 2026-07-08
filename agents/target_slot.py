class TargetSlotAgent:
    """
    Target Slot Agent (Virtual).
    Represents the virtual gap or slot on the mainline conveyor.
    """
    def __init__(self, initial_x: float, velocity: float):
        self.x_target = initial_x
        self.v_target = velocity

    def update(self, dt: float):
        """
        Updates the target slot position.
        """
        self.x_target += self.v_target * dt
