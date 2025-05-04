# main_multiprocess.py
from multiprocessing import Process, Queue
from hx711_module import HX711
import time

# Define pins for each HX711 (8 load cells)
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

# Manually measured offsets (from taring)
hardcoded_offsets = [
    262158.0,  # Loadcell 0
    841231,  # Loadcell 1
    844120,  # Loadcell 2
    842998,  # Loadcell 3
    843102,  # Loadcell 4
    840001,  # Loadcell 5
    841555,  # Loadcell 6
    843888   # Loadcell 7
]

# 📐 Manually calculated calibration factors (based on known weights)
calibration_factors = [
    40.04,   # Loadcell 0
    142.8,   # Loadcell 1
    144.1,   # Loadcell 2
    143.7,   # Loadcell 3
    145.2,   # Loadcell 4
    144.5,   # Loadcell 5
    146.0,   # Loadcell 6
    143.9    # Loadcell 7
]

def loadcell_worker(dout, sck, q, index, offset, calibration_factor):
    hx = HX711(dout, sck)
    hx.offset = offset
    hx.calibration_factor = calibration_factor
    while True:
        weight, raw = hx.get_weight()
        q.put((index, weight, raw))
        time.sleep(0.2)

def main():
    processes = []
    queues = []

    for i, config in enumerate(load_cells_config):
        q = Queue()
        queues.append(q)
        offset = hardcoded_offsets[i]
        cal_factor = calibration_factors[i]
        p = Process(target=loadcell_worker, args=(config['dout'], config['sck'], q, i, offset, cal_factor))
        p.start()
        processes.append(p)

    try:
        while True:
            for i, q in enumerate(queues):
                if not q.empty():
                    idx, weight, raw = q.get()
                    print(f"[Loadcell {idx}] Raw: {raw:.2f}, Weight: {weight:.2f} g")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Exiting...")
        for p in processes:
            p.terminate()
            p.join()

if __name__ == "__main__":
    main()


