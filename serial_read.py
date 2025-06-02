from collections import deque
import threading
import requests
import serial
import time
import csv
import os
import re

# Global data
latest_line = None
condition = threading.Condition(threading.Lock())
is_windows = os.name == 'nt'
port = 'COM6' if is_windows else '/dev/ttyACM0'
ser = None
new_line_received = False

# CSV data
row_limit = 100                                 # Número de filas a almacenar
update_rate = 1                                 # Frecuencia de actualización en segundos
header = ["t", "Vel", "Motor", "Marcha"]        # Encabezado del archivo CSV
port = 'COM6' if is_windows else '/dev/ttyACM0' # Puerto serial
history_file = 'history.csv'
graph_file = 'graph.csv'
data_list = deque(maxlen=row_limit)             # Cola de datos
program_start = None

# Send data
#car_url = 'https://2dd7ea69e62943c5a723596cf8f5bc8c.api.mockbin.io/'
car_url = 'http://192.168.137.215/ctrl'
prev_throttle = 0
prev_steering = 0
prev_brake = 0

# Read serial data
def read_line():
    global ser
    try:
        if ser is None:
            ser = serial.Serial(port, 115200, timeout=1)
        latest_line = None
        while ser.in_waiting:
            latest_line = ser.readline()
        if latest_line:
            latest_line = latest_line.decode('utf-8').strip()
        return latest_line
    except Exception as e:
        if ser is not None:
            ser.close()
            ser = None

# Funciones auxiliares
def extract_numbers(line):
    pattern = r'[-+]?\d*\.\d+|[-+]?\d+'
    matches = re.findall(pattern, line)
    return [float(num) for num in matches]
def no_drift_sleep(interval, offset=0): # Es como un sleep() pero sin drift
    current_time = time.time()
    time_to_sleep = (interval - (current_time % interval) + offset) % interval
    time.sleep(time_to_sleep)

# Send data
def send_values(line):
    global prev_throttle, prev_steering, prev_brake

    # if an X surrounded by whitespace is found, return
    if 'X' in line.split():
        return False
    
    try:
        # Get and format values
        vals = extract_numbers(line)
        throttle = round(vals[2], 2)
        brake = int(vals[3])
        steering = int(vals[4])
    except Exception as e:
        print("Error parsing values:", e)
        return
    
    # Determine which values changed
    throttle_changed = (abs(throttle - prev_throttle) >= 0.05) or (prev_throttle != 0 and throttle == 0) or (prev_throttle != 1 and throttle == 1)
    steering_changed = steering != prev_steering
    brake_changed = brake != prev_brake
    
    # if no values changed, don't send request
    if not throttle_changed and not steering_changed and not brake_changed:
        return False
    
    # Build request parameters
    params = []
    if throttle_changed:
        params.append(f"t={throttle}")
    if steering_changed:
        params.append(f"s={steering}")
    if brake_changed:
        params.append(f"b={brake}")
    
    # Save current values
    prev_throttle = throttle
    prev_steering = steering
    prev_brake = brake
    
    try:
        # Send request
        url = f"{car_url}?{'&'.join(params)}"
        print("Sending:", url)
        response = requests.get(url, timeout=2)
        print("Done")
    except Exception as e:
        print("Error sending values:", e)
    
    return True

# Save data
def save_vals(line):
    global program_start
    if program_start is None:
        program_start = time.time()
    
    vals = extract_numbers(line)
    print("Received:", vals)
        
    # Get row values
    t = time.time()
    vals = (time.time(), vals[-2], vals[-1], vals[2])
    data_list.append(vals)
    
    # Save data to files
    overwrite_graph()
    append_history()
def overwrite_graph(): # Actualiza el archivo con los últimos n datos de la lista
    # Obtenemos el tiempo de inicio
    strt = data_list[0][0]
    
    # Formateamos los datos
    new_data = list(data_list)
    new_data.reverse()
    for i in range(len(new_data)):
        row = list(new_data[i])
        if not is_windows:
            row[0] = round(row[0] - program_start, 0)
        else:
            row[0] = round(row[0] - strt, 0)
        row[1] = round(row[1], 2)
        row[2] = round(row[2], 2)
        row[3] = round(row[3], 2)
        new_data[i] = row
    while len(new_data) < row_limit:
        new_data.append([0, 0, 0])
    
    # Los escribimos en el archivo
    with open(graph_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)  # Write header
        for row in new_data:
            writer.writerow(row)
def append_history(): # Agrega el último valor generado a history.csv
    latest_value = data_list[-1]
    
    file_exists = os.path.isfile(history_file)
    
    with open(history_file, mode='a', newline='') as file:
        writer = csv.writer(file)
        if not file_exists:
            writer.writerow(header)  # Write header if file doesn't exist
        writer.writerow([round(val, 2) for val in latest_value])

# Thread loops
def send_loop():
    global latest_line
    while True:
        with condition:
            condition.wait()  # Wait for a new line to be available
            line = latest_line
        while line:
            if not send_values(line):
                break
            with condition:
                line = latest_line
def save_loop():
    global new_line_received
    while True:
        with condition:
            if new_line_received:
                line = latest_line
                new_line_received = False
            else:
                line = None
        if line:
            save_vals(line)
        no_drift_sleep(update_rate)

def main():
    global ser, latest_line, new_line_received
    
    # Change working directory to the script's directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    
    # Start send thread
    send_thread = threading.Thread(target=send_loop)
    send_thread.daemon = True
    send_thread.start()
    
    # Start save thread
    save_thread = threading.Thread(target=save_loop)
    save_thread.daemon = True
    save_thread.start()
    
    try:
        while True:
            line = read_line()
            
            if line and '\t' in line:  # Check if the line contains a tab character
                with condition:
                    latest_line = line
                    new_line_received = True
                    condition.notify()  # Notify the send_loop thread that a new line is available
    except:
        if ser is not None:
            ser.close()
            ser = None
        print("Program stopped.")

if __name__ == '__main__':
    main()