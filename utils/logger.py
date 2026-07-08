class SimulationLogger:
    """
    Logger class to record simulation telemetry for plotting and analysis.
    """
    def __init__(self):
        self.time = []
        self.x_target = []
        self.v_target = []
        self.x_parcel = []
        self.v_parcel = []
        self.x_ref = []
        self.v_ref = []
        self.a_ref = []
        self.pos_error = []
        self.vel_error = []
        self.dt = []

    def log_sample(self, t: float, target_x: float, target_v: float, 
                   parcel_x: float, parcel_v: float, 
                   ref_x: float, ref_v: float, ref_a: float, dt: float):
        self.time.append(t)
        self.x_target.append(target_x)
        self.v_target.append(target_v)
        self.x_parcel.append(parcel_x)
        self.v_parcel.append(parcel_v)
        self.x_ref.append(ref_x)
        self.v_ref.append(ref_v)
        self.a_ref.append(ref_a)
        self.pos_error.append(target_x - parcel_x)
        self.vel_error.append(target_v - parcel_v)
        self.dt.append(dt)
