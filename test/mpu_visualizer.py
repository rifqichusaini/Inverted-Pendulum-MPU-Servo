import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np

# Konfigurasi Serial
SERIAL_PORT = 'COM7'  # Ganti dengan port ESP32 kamu (Linux: /dev/ttyUSB0, Mac: /dev/cu.usbserial-xxx)
BAUD_RATE = 115200
MAX_DATA_POINTS = 100

# Data buffers
accel_x = deque(maxlen=MAX_DATA_POINTS)
accel_y = deque(maxlen=MAX_DATA_POINTS)
accel_z = deque(maxlen=MAX_DATA_POINTS)
gyro_x = deque(maxlen=MAX_DATA_POINTS)
gyro_y = deque(maxlen=MAX_DATA_POINTS)
gyro_z = deque(maxlen=MAX_DATA_POINTS)
time_data = deque(maxlen=MAX_DATA_POINTS)

# Initialize serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT}")
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit()

# Setup plot
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
fig.suptitle('MPU6050 Real-time Data Visualization', fontsize=16)

# Accelerometer plot
line_ax, = ax1.plot([], [], 'r-', label='Accel X', linewidth=2)
line_ay, = ax1.plot([], [], 'g-', label='Accel Y', linewidth=2)
line_az, = ax1.plot([], [], 'b-', label='Accel Z', linewidth=2)
ax1.set_xlim(0, MAX_DATA_POINTS)
ax1.set_ylim(-15, 15)
ax1.set_xlabel('Samples')
ax1.set_ylabel('Acceleration (m/s²)')
ax1.set_title('Accelerometer Data')
ax1.legend(loc='upper right')
ax1.grid(True, alpha=0.3)

# Gyroscope plot
line_gx, = ax2.plot([], [], 'r-', label='Gyro X', linewidth=2)
line_gy, = ax2.plot([], [], 'g-', label='Gyro Y', linewidth=2)
line_gz, = ax2.plot([], [], 'b-', label='Gyro Z', linewidth=2)
ax2.set_xlim(0, MAX_DATA_POINTS)
ax2.set_ylim(-6, 6)
ax2.set_xlabel('Samples')
ax2.set_ylabel('Angular Velocity (rad/s)')
ax2.set_title('Gyroscope Data')
ax2.legend(loc='upper right')
ax2.grid(True, alpha=0.3)

plt.tight_layout()

counter = 0

def init():
    line_ax.set_data([], [])
    line_ay.set_data([], [])
    line_az.set_data([], [])
    line_gx.set_data([], [])
    line_gy.set_data([], [])
    line_gz.set_data([], [])
    return line_ax, line_ay, line_az, line_gx, line_gy, line_gz

def animate(i):
    global counter
    
    try:
        # Read line from serial
        line = ser.readline().decode('utf-8').strip()
        
        if line:
            # Parse CSV data
            data = line.split(',')
            if len(data) == 6:
                # Parse values
                ax_val = float(data[0])
                ay_val = float(data[1])
                az_val = float(data[2])
                gx_val = float(data[3])
                gy_val = float(data[4])
                gz_val = float(data[5])
                
                # Append to buffers
                accel_x.append(ax_val)
                accel_y.append(ay_val)
                accel_z.append(az_val)
                gyro_x.append(gx_val)
                gyro_y.append(gy_val)
                gyro_z.append(gz_val)
                time_data.append(counter)
                counter += 1
                
                # Update plot data
                line_ax.set_data(range(len(accel_x)), accel_x)
                line_ay.set_data(range(len(accel_y)), accel_y)
                line_az.set_data(range(len(accel_z)), accel_z)
                line_gx.set_data(range(len(gyro_x)), gyro_x)
                line_gy.set_data(range(len(gyro_y)), gyro_y)
                line_gz.set_data(range(len(gyro_z)), gyro_z)
                
                # Auto-scale Y axis if needed
                if len(accel_x) > 0:
                    all_accel = list(accel_x) + list(accel_y) + list(accel_z)
                    margin = 2
                    ax1.set_ylim(min(all_accel) - margin, max(all_accel) + margin)
                    
                    all_gyro = list(gyro_x) + list(gyro_y) + list(gyro_z)
                    ax2.set_ylim(min(all_gyro) - margin, max(all_gyro) + margin)
                
    except Exception as e:
        print(f"Error: {e}")
    
    return line_ax, line_ay, line_az, line_gx, line_gy, line_gz

# Create animation
ani = animation.FuncAnimation(
    fig, animate, init_func=init,
    interval=50, blit=True, cache_frame_data=False
)

print("Starting visualization... Press Ctrl+C to stop")
plt.show()

# Cleanup
ser.close()
print("Serial port closed")