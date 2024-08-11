import os
import time
from utils.get_latency import ping

def record_timestamp(name, previous_time=None):
    current_time = time.time()
    print(f"{name} at {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(current_time))}", end="")
    if previous_time is not None:
        interval = current_time - previous_time
        print(f", lasted {interval:.2f} seconds")
    else:
        print()
    return current_time

def output_to_file(detected_command, output_command, timestamps):
    filename = time.strftime("logs/%Y-%m-%d-%H%M%S-log.txt", time.localtime())
    log_dir = "logs"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    ping_s = ping('https://api.openai.com') * 1000
    with open(filename, "w") as file:
        file.write("Ping to OpenAI API\n")
        file.write(f"{ping_s} ms\n\n")
        file.write("Detected Command\n")
        file.write(f"{detected_command}\n\n")
        file.write("Output Command\n")
        file.write(f"{output_command}\n\n")
        for ts in timestamps:
            file.write(f"{ts[0]}: {ts[1]}\n")