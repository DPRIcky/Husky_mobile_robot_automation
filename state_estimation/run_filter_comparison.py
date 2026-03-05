#!/usr/bin/env python3
"""
Run all three filters (EKF, UKF, PF) simultaneously for comparison
"""

import subprocess
import time
import signal
import sys
import os

# List to track all processes
processes = []

def signal_handler(sig, frame):
    """Handle Ctrl+C to cleanly terminate all processes"""
    print("\n\nStopping all filters...")
    for proc in processes:
        if proc.poll() is None:  # Process still running
            proc.terminate()
            time.sleep(0.5)
            if proc.poll() is None:  # Still not terminated
                proc.kill()
    print("All filters stopped.")
    sys.exit(0)

def main():
    global processes
    
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    print("="*70)
    print("  STATE ESTIMATION FILTER COMPARISON")
    print("  Running: EKF, UKF, and Particle Filter")
    print("="*70)
    print()
    
    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    print("Starting filters...")
    print()
    
    # Start mock sensor publisher
    print("1. Starting mock sensor publisher...")
    proc_mock = subprocess.Popen(
        ['python3', os.path.join(script_dir, 'test_mock_sensors.py')],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    processes.append(proc_mock)
    time.sleep(2)
    print("   ✓ Mock sensors running")
    
    # Start EKF
    print("2. Starting Extended Kalman Filter (EKF)...")
    proc_ekf = subprocess.Popen(
        ['python3', os.path.join(script_dir, 'sensor_fusion_ekf.py')],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    processes.append(proc_ekf)
    time.sleep(1)
    print("   ✓ EKF running")
    
    # Start UKF
    print("3. Starting Unscented Kalman Filter (UKF)...")
    proc_ukf = subprocess.Popen(
        ['python3', os.path.join(script_dir, 'sensor_fusion_ukf.py')],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    processes.append(proc_ukf)
    time.sleep(1)
    print("   ✓ UKF running")
    
    # Start Particle Filter
    print("4. Starting Particle Filter (PF)...")
    proc_pf = subprocess.Popen(
        ['python3', os.path.join(script_dir, 'sensor_fusion_pf.py')],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    processes.append(proc_pf)
    time.sleep(1)
    print("   ✓ Particle Filter running")
    
    print()
    print("All filters are now running!")
    print()
    print("Starting comparison visualizer...")
    print()
    
    # Start comparison visualizer (in foreground)
    proc_viz = subprocess.Popen(
        ['python3', os.path.join(script_dir, 'compare_filters.py')]
    )
    processes.append(proc_viz)
    
    print("="*70)
    print("FILTER COMPARISON ACTIVE")
    print("="*70)
    print()
    print("What you should see:")
    print("  • Matplotlib window with 6 plots comparing all filters")
    print("  • Main plot: All three trajectories overlaid")
    print("  • Error plots: X and Y errors vs ground truth")
    print("  • Uncertainty comparison")
    print("  • RMSE bar chart")
    print("  • Performance statistics")
    print()
    print("Press Ctrl+C to stop all filters")
    print("="*70)
    
    # Wait for visualizer to finish
    proc_viz.wait()
    
    # Cleanup
    signal_handler(None, None)

if __name__ == '__main__':
    main()
