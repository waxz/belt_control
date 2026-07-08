import time
import argparse
import matplotlib.pyplot as plt
import numpy as np

from controller import ConveyorSyncController
from agents.conveyor_belt import ConveyorBeltAgent
from agents.parcel import ParcelAgent
from agents.target_slot import TargetSlotAgent
from utils.logger import SimulationLogger

def run_simulation(real_time: bool = False):
    # Configuration
    target_x0 = -5.0
    target_v = 2.5
    max_accel = 3.0
    nominal_dt = 0.01
    t_final = 10.0
    parcel_v0 = 1.0

    # Instantiate agents
    target = TargetSlotAgent(initial_x=target_x0, velocity=target_v)
    parcel = ParcelAgent(initial_x=0.0, initial_v=parcel_v0)
    belt = ConveyorBeltAgent(a_max=max_accel, initial_v=parcel_v0)
    
    # Instantiate controller
    controller = ConveyorSyncController(target_velocity=target_v, max_acceleration=max_accel)
    
    # Logger
    logger = SimulationLogger()
    
    # Simulation Timing / Loop
    t_now = 0.0
    t_prev = 0.0
    t_start = time.perf_counter() if real_time else 0.0
    
    parcel_v_out_last = parcel_v0

    print("Starting Conveyor Synchronization Simulation...")
    print(f"Mode: {'Real-Time' if real_time else 'Fixed-Step'}")
    
    while t_now < t_final:
        if real_time:
            t_now = time.perf_counter() - t_start
            dt = t_now - t_prev
            if dt < nominal_dt:
                time.sleep(nominal_dt - dt)
                t_now = time.perf_counter() - t_start
                dt = t_now - t_prev
            if dt <= 0.0:
                continue
        else:
            dt = nominal_dt
            t_now += dt

        # 1. Update target slot physical state
        target.update(dt)
        
        # 2. Update parcel physical state (with some noise on coupling)
        # Note: We model the physical dynamics update based on last output
        parcel.update(dt, v_belt=parcel_v_out_last, noise_std=0.005)

        # 3. Call the controller API
        # Parameters: current time, target state, parcel state, belt state
        v_cmd, x_ref, v_ref, a_ref = controller.update(
            t_now=t_now,
            target_pos=target.x_target,
            target_vel=target.v_target,
            parcel_pos=parcel.x_parcel,
            parcel_vel=parcel.v_parcel,
            belt_vel=belt.v_belt,
            max_accel=belt.a_max
        )

        # 4. Update conveyor belt physical state using command
        belt.update(dt, v_cmd)
        
        # Keep track of last velocity command output for physical simulation in next step
        parcel_v_out_last = v_cmd

        # 5. Log telemetry
        logger.log_sample(
            t=t_now,
            target_x=target.x_target,
            target_v=target.v_target,
            parcel_x=parcel.x_parcel,
            parcel_v=parcel.v_parcel,
            ref_x=x_ref,
            ref_v=v_ref,
            ref_a=a_ref,
            dt=dt
        )
        
        t_prev = t_now

    print("Simulation finished. Plotting results...")
    plot_results(logger)

def plot_results(logger: SimulationLogger):
    fig, axs = plt.subplots(5, 1, figsize=(12, 10), sharex=True)
    
    # 1. Position Tracking
    axs[0].plot(logger.time, logger.x_target, 'r--', label='Target', linewidth=1.5)
    axs[0].plot(logger.time, logger.x_parcel, 'b-', label='Parcel', linewidth=1.5)
    
    # Filter out NaNs for reference plotting
    time_ref = [t for t, x in zip(logger.time, logger.x_ref) if not np.isnan(x)]
    x_ref_clean = [x for x in logger.x_ref if not np.isnan(x)]
    if x_ref_clean:
        axs[0].plot(time_ref, x_ref_clean, 'g:', label='Reference', linewidth=2.0)
        
    axs[0].grid(True, linestyle=':', alpha=0.6)
    axs[0].set_title('Position Tracking')
    axs[0].set_ylabel('Position (m)')
    axs[0].legend(loc='best')
    
    # 2. Velocity Tracking
    axs[1].plot(logger.time, logger.v_target, 'r--', label='Target', linewidth=1.5)
    axs[1].plot(logger.time, logger.v_parcel, 'b-', label='Parcel', linewidth=1.5)
    
    v_ref_clean = [v for v in logger.v_ref if not np.isnan(v)]
    if v_ref_clean:
        axs[1].plot(time_ref, v_ref_clean, 'g:', label='Reference', linewidth=2.0)
        
    axs[1].grid(True, linestyle=':', alpha=0.6)
    axs[1].set_title('Velocity Tracking')
    axs[1].set_ylabel('Velocity (m/s)')
    axs[1].legend(loc='best')
    
    # 3. Acceleration
    axs[2].plot(logger.time, logger.a_ref, 'm-', label='Acceleration Ref', linewidth=1.5)
    axs[2].grid(True, linestyle=':', alpha=0.6)
    axs[2].set_title('Reference Acceleration')
    axs[2].set_ylabel(r'Acceleration ($m/s^2$)')
    
    # 4. Tracking Errors
    axs[3].plot(logger.time, logger.pos_error, 'c-', label='Position Error', linewidth=1.5)
    axs[3].plot(logger.time, logger.vel_error, 'y-', label='Velocity Error', linewidth=1.5)
    axs[3].grid(True, linestyle=':', alpha=0.6)
    axs[3].set_title('Tracking Errors (Target - Parcel)')
    axs[3].set_ylabel('Error')
    axs[3].legend(loc='best')
    
    # 5. Measured Loop Time
    dt_ms = [d * 1000.0 for d in logger.dt]
    axs[4].plot(logger.time, dt_ms, 'k-', linewidth=1.0)
    axs[4].grid(True, linestyle=':', alpha=0.6)
    axs[4].set_title('Measured Loop Time')
    axs[4].set_ylabel('dt (ms)')
    axs[4].set_xlabel('Time (s)')
    
    plt.tight_layout()
    plt.savefig('plot_python.png', dpi=150)
    print("Results saved to plot_python.png")
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Conveyor synchronization multi-agent simulation")
    parser.add_argument('--real-time', action='store_true', help="Run in real-time mode with sleep")
    args = parser.parse_args()
    
    run_simulation(real_time=args.real_time)
