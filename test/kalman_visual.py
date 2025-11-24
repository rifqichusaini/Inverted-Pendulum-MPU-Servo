import serial
import threading
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# --- Pengaturan ---
PORT = 'COM6'      # sesuai info kamu
BAUD = 115200
MAX_SAMPLES = 500  # berapa banyak titik ditampilkan (scrolling window)

# --- Buffer data ---
rawx_buf = deque([0.0]*MAX_SAMPLES, maxlen=MAX_SAMPLES)
kalx_buf = deque([0.0]*MAX_SAMPLES, maxlen=MAX_SAMPLES)
rawy_buf = deque([0.0]*MAX_SAMPLES, maxlen=MAX_SAMPLES)
kaly_buf = deque([0.0]*MAX_SAMPLES, maxlen=MAX_SAMPLES)
lock = threading.Lock()
running = True

def reader_thread():
    global running
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        # flush any startup text
        time.sleep(0.5)
        ser.reset_input_buffer()
        while running:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            # Skip non-data lines (e.g., "READY")
            if line.upper().startswith("READY"):
                continue
            parts = line.split(',')
            if len(parts) >= 4:
                try:
                    rx = float(parts[0])
                    kx = float(parts[1])
                    ry = float(parts[2])
                    ky = float(parts[3])
                    with lock:
                        rawx_buf.append(rx)
                        kalx_buf.append(kx)
                        rawy_buf.append(ry)
                        kaly_buf.append(ky)
                except ValueError:
                    # parse error - ignore
                    continue
    except Exception as e:
        print("ERROR serial:", e)
    finally:
        running = False

# --- Start reader thread ---
t = threading.Thread(target=reader_thread, daemon=True)
t.start()

# --- Setup plot ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10,6), sharex=True)
plt.tight_layout(pad=3.0)

line_rawx, = ax1.plot([], [], label='Raw X')
line_kalx, = ax1.plot([], [], label='Kalman X')
ax1.set_title('Accel X - Raw vs Kalman')
ax1.set_ylabel('g')
ax1.legend()
ax1.grid(True)

line_rawy, = ax2.plot([], [], label='Raw Y')
line_kaly, = ax2.plot([], [], label='Kalman Y')
ax2.set_title('Accel Y - Raw vs Kalman')
ax2.set_ylabel('g')
ax2.set_xlabel('Samples (most recent → right)')
ax2.legend()
ax2.grid(True)

def init():
    ax1.set_xlim(0, MAX_SAMPLES)
    ax1.set_ylim(-2.0, 2.0)  # asumsi akselerasi ±2g, ubah bila perlu
    ax2.set_xlim(0, MAX_SAMPLES)
    ax2.set_ylim(-2.0, 2.0)
    line_rawx.set_data([], [])
    line_kalx.set_data([], [])
    line_rawy.set_data([], [])
    line_kaly.set_data([], [])
    return line_rawx, line_kalx, line_rawy, line_kaly

def update(frame):
    with lock:
        rx = list(rawx_buf)
        kx = list(kalx_buf)
        ry = list(rawy_buf)
        ky = list(kaly_buf)

    x_axis = list(range(len(rx)))
    if len(rx) < MAX_SAMPLES:
        # pad for stable axis
        x_axis = list(range(MAX_SAMPLES - len(rx), MAX_SAMPLES))

    line_rawx.set_data(x_axis, rx)
    line_kalx.set_data(x_axis, kx)
    line_rawy.set_data(x_axis, ry)
    line_kaly.set_data(x_axis, ky)

    # autoscale Y if needed (optional)
    # ax1.relim(); ax1.autoscale_view()
    # ax2.relim(); ax2.autoscale_view()

    return line_rawx, line_kalx, line_rawy, line_kaly

ani = animation.FuncAnimation(fig, update, init_func=init, interval=50, blit=True)

try:
    plt.show()
except KeyboardInterrupt:
    pass

running = False
t.join(timeout=1.0)
print("Exited")
