from multiprocessing import Process, Queue, Event
from hx711_module import HX711
import can
import time
import os

# LoadCell Configuration
load_cells_config = [
    {"dout": 5,  "sck": 6},
    {"dout": 13, "sck": 19},
    {"dout": 17, "sck": 27},
    {"dout": 22, "sck": 23},
    {"dout": 24, "sck": 25},
    {"dout": 12, "sck": 16},
    {"dout": 20, "sck": 21},
    {"dout": 26, "sck": 18}
]

load_cell_labels = [
    "Thrust_0deg",
    "Thrust_90deg",
    "Thrust_180deg",
    "Thrust_270deg",
    "Torque_0deg",
    "Torque_90deg",
    "Torque_180deg",
    "Torque_270deg"
]

hardcoded_offsets = [
    262158.0, 841231, 844120, 842998, 843102, 840001, 841555, 843888
]

calibration_factors = [
    40.04, 142.8, 144.1, 143.7, 145.2, 144.5, 146.0, 143.9
]

# Radius for torque calculation in meters
R = 0.1  # Replace with your actual radius value

# CAN Configuration
CAN_INTERFACE = "can0"
BITRATE = 500000
BAMOCAR_ID = 0x201  # BAMOCAR Receive ID
BAMOCAR_RESPONSE_ID = 0x181  # BAMOCAR Response ID

# ESC Parameter Registers
RPM_REGISTER = 0x30       # RPM register address
VOLTAGE_REGISTER = 0x90   # Voltage register address (replace with correct value)
CURRENT_REGISTER = 0x20   # Current register address (replace with correct value)
TEMP_REGISTER = 0xA0      # Temperature register address (replace with correct value)

# Conversion factors
RPM_CONVERSION = 0.091547146780592
VOLTAGE_CONVERSION = 0.1  # Replace with actual conversion factor
CURRENT_CONVERSION = 0.1  # Replace with actual conversion factor
TEMP_CONVERSION = 0.1     # Replace with actual conversion factor

def is_can_interface_up():
    """Check if the CAN interface is already up."""
    result = os.popen(f"ip link show {CAN_INTERFACE}").read()
    return "<UP," in result

def setup_can_interface():
    """Set up the CAN interface if not already up."""
    if is_can_interface_up():
        print(f"CAN interface {CAN_INTERFACE} is already up.")
    else:
        print(f"Bringing up CAN interface {CAN_INTERFACE} at {BITRATE} bps...")
        os.system(f"sudo ip link set {CAN_INTERFACE} up type can bitrate {BITRATE}")

def set_process_affinity(process_id, core_id):
    """Set which CPU core a process runs on"""
    try:
        os.sched_setaffinity(process_id, {core_id})
        print(f"Process {process_id} assigned to core {core_id}")
    except Exception as e:
        print(f"Error setting affinity: {e}")

def loadcell_worker(dout, sck, q, index, offset, calibration_factor, stop_event):
    """Worker process for reading from a load cell"""
    hx = HX711(dout, sck)
    hx.offset = offset
    hx.calibration_factor = calibration_factor
    
    while not stop_event.is_set():
        weight, raw = hx.get_weight()
        q.put((index, weight, raw))
        time.sleep(0.2)

def can_worker(param_register, param_name, param_conversion, result_queue, stop_event):
    """Worker process for reading a specific parameter from the ESC via CAN"""
    setup_can_interface()
    bus = can.interface.Bus(channel=CAN_INTERFACE, interface="socketcan")
    
    try:
        while not stop_event.is_set():
            # Send request to read the parameter
            msg = can.Message(
                arbitration_id=BAMOCAR_ID, 
                data=[0x3D, param_register, 0x64], 
                is_extended_id=False
            )
            bus.send(msg)
            
            # Wait for response
            start_time = time.time()
            while time.time() - start_time < 1.0 and not stop_event.is_set():
                message = bus.recv(timeout=0.1)
                if message and message.arbitration_id == BAMOCAR_RESPONSE_ID:
                    data = message.data
                    if data[0] == param_register:
                        raw_value = (data[2] << 8) | data[1]
                        value = param_conversion * raw_value
                        result_queue.put((param_name, value))
                        break
            
            time.sleep(0.2)  # Delay between requests
    
    finally:
        bus.shutdown()

def main():
    # Set up stop event for clean shutdown
    stop_event = Event()
    
    # Set up queues for loadcells and CAN parameters
    loadcell_queues = []
    can_queue = Queue()
    
    # Start loadcell processes (8 processes)
    loadcell_processes = []
    for i, config in enumerate(load_cells_config):
        q = Queue()
        loadcell_queues.append(q)
        offset = hardcoded_offsets[i]
        cal_factor = calibration_factors[i]
        
        p = Process(
            target=loadcell_worker, 
            args=(config['dout'], config['sck'], q, i, offset, cal_factor, stop_event)
        )
        p.start()
        
        # Assign loadcell processes to cores 0 and 1 (4 processes per core)
        core_id = 0 if i < 4 else 1
        set_process_affinity(p.pid, core_id)
        
        loadcell_processes.append(p)
    
    # Start CAN parameter processes (4 processes)
    can_processes = []
    can_parameters = [
        (RPM_REGISTER, "RPM", RPM_CONVERSION),
        (VOLTAGE_REGISTER, "Voltage", VOLTAGE_CONVERSION),
        (CURRENT_REGISTER, "Current", CURRENT_CONVERSION),
        (TEMP_REGISTER, "Temperature", TEMP_CONVERSION)
    ]
    
    for i, (register, name, conversion) in enumerate(can_parameters):
        p = Process(
            target=can_worker,
            args=(register, name, conversion, can_queue, stop_event)
        )
        p.start()
        
        # Assign CAN processes to cores 2 and 3 (2 processes per core)
        core_id = 2 if i < 2 else 3
        set_process_affinity(p.pid, core_id)
        
        can_processes.append(p)
    
    # Dictionary to store the latest CAN parameter values
    can_values = {
        "RPM": 0,
        "Voltage": 0,
        "Current": 0,
        "Temperature": 0
    }
    
    try:
        while True:
            # Process loadcell data
            loadcell_readings = {}
            for i, q in enumerate(loadcell_queues):
                while not q.empty():
                    idx, weight, raw = q.get()
                    loadcell_readings[idx] = (weight, raw)
            
            # Process CAN data
            while not can_queue.empty():
                param_name, value = can_queue.get()
                can_values[param_name] = value
            
            # Display all data
            if loadcell_readings:
                print("\n===== LOADCELL READINGS =====")
                
                # Display thrust loadcell data
                print("--- THRUST MEASUREMENTS ---")
                thrust_values = []
                for i in range(4):
                    if i in loadcell_readings:
                        weight, raw = loadcell_readings[i]
                        label = load_cell_labels[i]
                        weight_kg = weight / 1000.0  # Convert from g to kg
                        thrust = weight_kg * 9.8  # Thrust in Newtons
                        thrust_values.append(thrust)
                        print(f"[{label}] Raw: {raw:.2f}, Weight: {weight:.2f} g, Thrust: {thrust:.2f} N")
                
                # Display torque loadcell data
                print("\n--- TORQUE MEASUREMENTS ---")
                torque_values = []
                for i in range(4, 8):
                    if i in loadcell_readings:
                        weight, raw = loadcell_readings[i]
                        label = load_cell_labels[i]
                        weight_kg = weight / 1000.0  # Convert from g to kg
                        torque = weight_kg * 9.8 * R  # Torque in Nm
                        torque_values.append(torque)
                        print(f"[{label}] Raw: {raw:.2f}, Weight: {weight:.2f} g, Torque: {torque:.2f} Nm")
                
                # Calculate and display total thrust and torque
                if thrust_values:
                    total_thrust = sum(thrust_values)
                    print(f"\nTotal Thrust: {total_thrust:.2f} N")
                
                if torque_values:
                    total_torque = sum(torque_values)
                    print(f"Total Torque: {total_torque:.2f} Nm")
            
            # Display ESC data from CAN bus
            print("\n===== ESC CONTROLLER DATA (CAN) =====")
            print(f"RPM: {can_values['RPM']:.2f} rpm")
            print(f"Voltage: {can_values['Voltage']:.2f} V")
            print(f"Current: {can_values['Current']:.2f} A")
            print(f"Temperature: {can_values['Temperature']:.2f} °C")
            
            # Sleep to reduce CPU usage
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\nExiting...")
        # Signal all processes to stop
        stop_event.set()
        
        # Wait for all processes to terminate
        for p in loadcell_processes + can_processes:
            p.join(timeout=1.0)
            if p.is_alive():
                p.terminate()
        
        print("All processes terminated.")

if __name__ == "__main__":
    main()