import matplotlib.pyplot as plt
import re
from datetime import datetime
import numpy as np
import os

def read_data_from_file(filename):
    """Read data from log file"""
    try:
        with open(filename, 'r') as file:
            return file.read()
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found")
        return None
    except Exception as e:
        print(f"Error reading file: {e}")
        return None

def parse_data(data_string):
    """Parse the data string and extract timestamps and error values"""
    lines = data_string.strip().split('\n')
    errors = []
    timestamps = []
    
    for line in lines:
        # Extract timestamp and error value using regex
        match = re.match(r'\[(\d+:\d+:\d+)\] ESP32: ([-+]?\d*\.\d+|\d+)', line)
        if match:
            time_str = match.group(1)
            error_val = float(match.group(2))
            
            # Convert timestamp to seconds since start
            time_obj = datetime.strptime(time_str, '%H:%M:%S')
            total_seconds = time_obj.hour * 3600 + time_obj.minute * 60 + time_obj.second
            
            timestamps.append(total_seconds)
            errors.append(error_val)
    
    return timestamps, errors

def create_time_axis(timestamps, sampling_time_ms=20):
    """Create a proper time axis with 20ms sampling time"""
    # Convert timestamps to relative time in seconds
    start_time = timestamps[0]
    relative_times = [(ts - start_time) for ts in timestamps]
    
    # Since the timestamps in your data have coarse resolution (1 second),
    # we'll create a proper time axis with 20ms intervals
    num_samples = len(timestamps)
    time_axis = np.arange(0, num_samples * sampling_time_ms / 1000.0, sampling_time_ms / 1000.0)
    
    return time_axis

def plot_error_data(time_axis, errors, sampling_time_ms=20):
    """Plot the error data with proper formatting"""
    plt.figure(figsize=(12, 6))
    
    # Plot error values
    plt.plot(time_axis, errors, 'b-', linewidth=2, label='Error')
    plt.plot(time_axis, errors, 'ro', markersize=4, alpha=0.7)
    
    # Add zero reference line
    plt.axhline(y=0, color='k', linestyle='--', alpha=0.3, label='Zero Reference')
    
    # Customize the plot
    plt.xlabel('Time (seconds)')
    plt.ylabel('Error Value')
    plt.title(f'Error Data Plot (Sampling Time: {sampling_time_ms}ms)')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    # Add some statistics
    max_error = max(errors)
    min_error = min(errors)
    mean_error = np.mean(errors)
    
    plt.text(0.02, 0.98, f'Max Error: {max_error:.2f}\nMin Error: {min_error:.2f}\nMean Error: {mean_error:.2f}', 
             transform=plt.gca().transAxes, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    plt.show()
    
    # Print some statistics
    print(f"Data Statistics:")
    print(f"Total samples: {len(errors)}")
    print(f"Time duration: {time_axis[-1]:.2f} seconds")
    print(f"Sampling rate: {1000/sampling_time_ms:.1f} Hz")
    print(f"Max error: {max_error:.2f}")
    print(f"Min error: {min_error:.2f}")
    print(f"Mean error: {mean_error:.2f}")
    print(f"Standard deviation: {np.std(errors):.2f}")

def main():
    # Read data from file
    filename = "./test/log/esp32_log_curve.txt"
    data = read_data_from_file(filename)
    
    if data is None:
        return
    
    # Parse the data
    timestamps, errors = parse_data(data)
    
    # Create proper time axis with 20ms sampling
    time_axis = create_time_axis(timestamps, sampling_time_ms=20)
    
    # Plot the data
    plot_error_data(time_axis, errors, sampling_time_ms=20)
    
    # Additional analysis: Show when non-zero errors occur
    non_zero_indices = [i for i, err in enumerate(errors) if err != 0]
    if non_zero_indices:
        print(f"\nNon-zero errors occur at samples: {non_zero_indices}")
        print(f"Corresponding times: {[time_axis[i] for i in non_zero_indices]} seconds")

if __name__ == "__main__":
    main()