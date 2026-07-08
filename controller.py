import math

def required_gap(v0: float, vf: float, a_max: float) -> float:
    """
    Computes the safety distance gap needed for collision-free synchronization.
    
    Args:
        v0: Initial parcel velocity (m/s)
        vf: Target velocity (m/s)
        a_max: Maximum acceleration capability limit (m/s^2)
    
    Returns:
        The required initial synchronization gap (m)
    """
    dv = vf - v0
    if dv <= 0.0:
        return 0.0
    return 0.75 * (dv ** 2) / a_max


def interpolate_crossing(t0: float, t1: float, gap0: float, gap1: float, threshold: float) -> float:
    """
    Performs linear interpolation for precise trigger timing.
    """
    denom = gap0 - gap1
    if abs(denom) < 1e-9:
        alpha = 0.0
    else:
        alpha = (gap0 - threshold) / denom
    alpha = min(max(alpha, 0.0), 1.0)
    return t0 + alpha * (t1 - t0)


def eval_smoothstep_profile(v0: float, vf: float, a_max: float, t: float) -> tuple[float, float, float]:
    """
    Generates the analytical C1-smooth motion profile.
    
    Args:
        v0: Initial velocity at trigger time (m/s)
        vf: Target/final reference velocity (m/s)
        a_max: Maximum acceleration limit (m/s^2)
        t: Elapsed time since trigger (s)
        
    Returns:
        tuple containing:
            - v: Command/reference velocity (m/s)
            - a: Command/reference acceleration (m/s^2)
            - x: Command/reference relative displacement (m)
    """
    dv = vf - v0
    if dv <= 0.0:
        return vf, 0.0, vf * max(t, 0.0)

    T = 1.5 * dv / a_max
    if t <= 0.0:
        return v0, 0.0, 0.0

    if t >= T:
        x_ramp = (v0 + vf) / 2.0 * T
        return vf, 0.0, x_ramp + vf * (t - T)

    s = t / T
    v = v0 + dv * (3.0 * (s ** 2) - 2.0 * (s ** 3))
    a = (dv / T) * (6.0 * s - 6.0 * (s ** 2))
    x = v0 * t + dv * (t ** 3) / (T ** 2) - dv * (t ** 4) / (2.0 * (T ** 3))
    return v, a, x


class ConveyorSyncController:
    """
    Standalone, self-contained closed-loop controller for conveyor belt synchronization.
    This class is designed to be easily portable to PLC or embedded code (e.g. Structured Text or C).
    """
    def __init__(self, target_velocity: float = 0.0, max_acceleration: float = 0.0):
        self.target_velocity = target_velocity
        self.max_acceleration = max_acceleration
        
        self.active = False
        
        self.trigger_time = float('nan')
        self.trigger_position = float('nan')
        self.trigger_velocity = 0.0
        
        self.previous_time = 0.0
        self.previous_gap = float('nan')

    def reset(self):
        self.active = False
        self.trigger_time = float('nan')
        self.trigger_position = float('nan')
        self.trigger_velocity = 0.0
        self.previous_time = 0.0
        self.previous_gap = float('nan')

    def update(self, 
               t_now: float, 
               target_pos: float, 
               target_vel: float, 
               parcel_pos: float, 
               parcel_vel: float, 
               belt_vel: float, 
               max_accel: float) -> tuple[float, float, float, float]:
        """
        Executes the control cycle.
        
        Args:
            t_now: Current simulation/system time (s)
            target_pos: Current target slot position (m)
            target_vel: Current target slot velocity (m/s)
            parcel_pos: Current parcel position (m)
            parcel_vel: Current parcel velocity (m/s)
            belt_vel: Current belt linear velocity (m/s)
            max_accel: Maximum belt acceleration capability (m/s^2)
            
        Returns:
            tuple containing:
                - v_cmd: Commanded belt velocity (m/s)
                - x_ref: Reference position from planner (m, NaN if inactive)
                - v_ref: Reference velocity from planner (m/s, NaN if inactive)
                - a_ref: Reference acceleration from planner (m/s^2, 0.0 if inactive)
        """
        self.target_velocity = target_vel
        self.max_acceleration = max_accel
        
        gap_now = parcel_pos - target_pos
        
        if math.isnan(self.previous_gap):
            self.previous_gap = gap_now
            self.previous_time = t_now
            
        # Trigger Logic
        if not self.active:
            req_gap = required_gap(parcel_vel, self.target_velocity, self.max_acceleration)
            crossed = (self.previous_gap > req_gap) and (gap_now <= req_gap)
            
            if crossed:
                t_cross = interpolate_crossing(
                    self.previous_time,
                    t_now,
                    self.previous_gap,
                    gap_now,
                    req_gap
                )
                self.active = True
                self.trigger_time = t_cross
                self.trigger_position = parcel_pos
                self.trigger_velocity = parcel_vel
                
        # Evaluate Planner & Control
        if self.active:
            elapsed = max(t_now - self.trigger_time, 0.0)
            v_ref, a_ref, x_rel = eval_smoothstep_profile(
                self.trigger_velocity,
                self.target_velocity,
                self.max_acceleration,
                elapsed
            )
            x_ref = self.trigger_position + x_rel
            
            # Closed loop feedback: Proportional control on tracking error
            Kp = 8.0
            v_cmd = v_ref + Kp * (x_ref - parcel_pos)
        else:
            v_cmd = parcel_vel
            x_ref = float('nan')
            v_ref = float('nan')
            a_ref = 0.0
            
        self.previous_gap = gap_now
        self.previous_time = t_now
        
        return v_cmd, x_ref, v_ref, a_ref
