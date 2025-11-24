import matplotlib.pyplot as plt
import re
import numpy as np

def read_data_from_file(filename):
    """Read data from log file."""
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
    """
    Parse log lines of the form:
    [14:37:30] ESP32: 0.00, 0.00, 0.000
      -> error, steering, pwm
    """
    lines = data_string.strip().split('\n')

    errors = []
    steerings = []
    pwms = []

    # regex: [HH:MM:SS] ESP32: e, s, p
    pattern = re.compile(
        r'\[\d+:\d+:\d+\]\s+ESP32:\s*'      # timestamp + "ESP32:"
        r'([-+]?\d*\.?\d+)\s*,\s*'          # error
        r'([-+]?\d*\.?\d+)\s*,\s*'          # steering
        r'([-+]?\d*\.?\d+)'                 # pwm
    )

    for line in lines:
        line = line.strip()
        if not line:
            continue

        match = pattern.match(line)
        if not match:
            continue

        try:
            e = float(match.group(1))
            s = float(match.group(2))
            p = float(match.group(3))

            errors.append(e)
            steerings.append(s)
            pwms.append(p)
        except ValueError:
            continue

    return errors, steerings, pwms

def create_time_axis(num_samples, sampling_time_ms=20):
    """Create time axis assuming fixed sampling interval."""
    dt = sampling_time_ms / 1000.0  # seconds
    return np.arange(0, num_samples * dt, dt)

def plot_data(time_axis, errors, steerings, pwms, sampling_time_ms=20):
    """Plot error, steering, and PWM in three stacked subplots."""
    errors = np.array(errors)
    steerings = np.array(steerings)
    pwms = np.array(pwms)

    plt.figure(figsize=(14, 8))

    # ---- Error ----
    ax1 = plt.subplot(3, 1, 1)
    ax1.plot(time_axis, errors, linewidth=1.5, label='Error', color='r')
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax1.set_ylabel('Error')
    ax1.set_title(f'Error, Steering, PWM (Ts = {sampling_time_ms} ms)')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right')

    # ---- Steering ----
    ax2 = plt.subplot(3, 1, 2, sharex=ax1)
    ax2.plot(time_axis, steerings, linewidth=1.5, label='Steering', color='g')
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.set_ylabel('Steering (deg)')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right')

    # ---- PWM ----
    ax3 = plt.subplot(3, 1, 3, sharex=ax1)
    ax3.plot(time_axis, pwms, linewidth=1.5, label='PWM')
    ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('PWM')
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='upper right')

    plt.tight_layout()
    plt.show()

    # Stats
    print("=== Data Statistics ===")
    print(f"Total samples: {len(time_axis)}")
    print(f"Time duration: {time_axis[-1]:.2f} s")
    print(f"Sampling rate: {1000 / sampling_time_ms:.1f} Hz")

    def stats(name, arr):
        print(f"\n{name}:")
        print(f"  Max:   {np.max(arr):.3f}")
        print(f"  Min:   {np.min(arr):.3f}")
        print(f"  Mean:  {np.mean(arr):.3f}")
        print(f"  Std:   {np.std(arr):.3f}")

    stats("Error", errors)
    stats("Steering", steerings)
    stats("PWM", pwms)

def main():
    filename = "./test/log/esp32_log_curve_trial_99.txt"
    data = read_data_from_file(filename)
    if data is None:
        return

    errors, steerings, pwms = parse_data(data)
    if not errors:
        print("No valid data parsed.")
        return

    sampling_time_ms = 20
    time_axis = create_time_axis(len(errors), sampling_time_ms=sampling_time_ms)

    plot_data(time_axis, errors, steerings, pwms, sampling_time_ms=sampling_time_ms)

if __name__ == "__main__":
    main()
