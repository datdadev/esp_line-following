import numpy as np

class KalmanFilter:
    def __init__(self, initial_state, initial_error, process_noise, measurement_noise):
        """
        Initialize the Kalman Filter
        
        Args:
            initial_state: Initial estimate of the state (RPM)
            initial_error: Initial uncertainty in the state
            process_noise: Process noise (how much the system changes)
            measurement_noise: Measurement noise (how noisy the measurements are)
        """
        self.state = initial_state  # Current estimate of RPM
        self.error = initial_error  # Uncertainty in the estimate
        self.process_noise = process_noise  # Model uncertainty
        self.measurement_noise = measurement_noise  # Measurement uncertainty
        
    def predict(self):
        """
        Prediction step: estimate the next state
        For RPM, we assume it doesn't change much between samples
        """
        # For a simple system like RPM, prediction doesn't change the state much
        # This step would be more complex for systems with known dynamics
        return self.state
    
    def update(self, measurement):
        """
        Update step: incorporate the new measurement
        
        Args:
            measurement: The new RPM measurement from encoder
        """
        # Calculate the Kalman gain
        kalman_gain = self.error / (self.error + self.measurement_noise)
        
        # Update the state estimate
        self.state = self.state + kalman_gain * (measurement - self.state)
        
        # Update the error
        self.error = (1 - kalman_gain) * self.error + self.process_noise
        
        return self.state

def add_kalman_rpm_column(input_file, output_file):
    """
    Reads the sine_wave.txt file and adds a new column with Kalman-filtered RPM values
    """
    # Initialize Kalman filter parameters
    # These values can be adjusted based on the characteristics of your system
    initial_rpm = 0  # Initial RPM estimate
    initial_error = 100  # Initial uncertainty
    process_noise = 10  # How much RPM can change between samples
    measurement_noise = 50  # Noise in RPM measurements
    
    kf = KalmanFilter(initial_rpm, initial_error, process_noise, measurement_noise)
    
    # Read the input file
    with open(input_file, 'r') as f:
        lines = f.readlines()
    
    # Prepare the output data
    output_lines = []
    
    for line in lines:
        line = line.strip()
        
        # Skip header lines (lines that don't start with a number)
        if not line or not line[0].isdigit():
            # Add a header for the new column
            if line.startswith('==='):
                output_lines.append(line + ", Filtered_RPM_Kalman")
            else:
                output_lines.append(line)
            continue
        
        # Parse the data line
        parts = line.split(', ')
        if len(parts) < 5:
            output_lines.append(line)
            continue
        
        # Extract RPM value (5th column - index 4)
        try:
            rpm = float(parts[4])
        except ValueError:
            output_lines.append(line)
            continue
        
        # Apply Kalman filter to get the filtered RPM
        filtered_rpm = kf.update(rpm)
        
        # Create the new line with the additional column
        new_line = line + ', ' + f'{filtered_rpm:.2f}'
        output_lines.append(new_line)
    
    # Write the output file
    with open(output_file, 'w') as f:
        for line in output_lines:
            f.write(line + '\n')

if __name__ == '__main__':
    input_file = 'sine_wave.txt'
    output_file = 'sine_wave_with_kalman.txt'
    
    print(f"Processing {input_file} to add Kalman-filtered RPM column...")
    add_kalman_rpm_column(input_file, output_file)
    print(f"Output saved to {output_file}")
    print("Done!")