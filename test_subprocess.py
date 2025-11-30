import subprocess
import os
import threading
import time

evm_file = "test2.csv"
ati_file = "ati2.csv"
script_cmd = f"source /opt/ros/foxy/setup.bash && pwd && cd ~/TI_EVM_logger/ && ./run_inductance_test.sh -c 2 -e {evm_file} -a {ati_file}"

print(f"Running command: {script_cmd}")

# Start the bash process
current_bash_process = subprocess.Popen(
    script_cmd,
    shell=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    text=True,
    preexec_fn=os.setsid,
    executable='/bin/bash',
    bufsize=1,  # Line buffered
    universal_newlines=True
)

print(f"Process started with PID: {current_bash_process.pid}")

# Function to read stdout
def read_stdout():
    try:
        for line in iter(current_bash_process.stdout.readline, ''):
            if line:
                print(f"STDOUT: {line.strip()}")
            if current_bash_process.poll() is not None:
                break
    except Exception as e:
        print(f"Error reading stdout: {e}")

# Function to read stderr
def read_stderr():
    try:
        for line in iter(current_bash_process.stderr.readline, ''):
            if line:
                print(f"STDERR: {line.strip()}")
            if current_bash_process.poll() is not None:
                break
    except Exception as e:
        print(f"Error reading stderr: {e}")

# Start threads to read output
stdout_thread = threading.Thread(target=read_stdout)
stderr_thread = threading.Thread(target=read_stderr)

stdout_thread.daemon = True
stderr_thread.daemon = True

stdout_thread.start()
stderr_thread.start()

# Let it run for a bit to see initial output
print("Letting process run for 10 seconds...")
time.sleep(10)

# Check if process is still running
if current_bash_process.poll() is None:
    print("Process is still running")
else:
    print(f"Process has terminated with return code: {current_bash_process.returncode}")

# Terminate the process
try:
    print("Terminating process...")
    os.killpg(os.getpgid(current_bash_process.pid), 15)  # SIGTERM
    current_bash_process.wait(timeout=5)
    print("Process terminated gracefully")
except:
    print("Force killing process...")
    os.killpg(os.getpgid(current_bash_process.pid), 9)  # SIGKILL

print("Test complete")