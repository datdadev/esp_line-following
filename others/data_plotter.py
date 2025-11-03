import matplotlib.pyplot as plt

# Read data from file (skip first line)
time = []
pwm = []
rpm = []

with open(r"sine_wave.txt", "r") as f:
    next(f)  # Skip header line
    for line in f:
        if line.strip():  # Skip empty lines
            parts = line.strip().split(',')
            if len(parts) >= 5:  # Ensure we have enough columns
                time.append(float(parts[1]))  # 2nd column (time)
                pwm.append(int(parts[2]))     # 3rd column (PWM)
                rpm.append(float(parts[4]))   # 5th column (RPM)

# Create plots
plt.figure(figsize=(10, 6))

# Plot PWM over time
plt.subplot(2, 1, 1)
plt.plot(time, pwm, 'b-', label='PWM')
plt.ylabel('PWM Value')
plt.title('Motor Control Data')
plt.grid(True)
plt.legend()

# Plot RPM over time
plt.subplot(2, 1, 2)
plt.plot(time, rpm, 'r-', label='RPM')
plt.xlabel('Time (s)')
plt.ylabel('RPM')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()