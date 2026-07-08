import random

class ParcelAgent:
    """
    Parcel Agent.
    Represents the physical material package being transported.
    """
    def __init__(self, initial_x: float = 0.0, initial_v: float = 0.0):
        self.x_parcel = initial_x
        self.v_parcel = initial_v

    def update(self, dt: float, v_belt: float, noise_std: float = 0.005):
        """
        Updates parcel physical state with frictional coupling and process/measurement noise.
        """
        noise = random.gauss(0.0, noise_std) if noise_std > 0.0 else 0.0
        self.v_parcel = v_belt + noise
        self.x_parcel += self.v_parcel * dt
