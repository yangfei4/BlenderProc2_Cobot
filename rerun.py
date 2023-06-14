import subprocess
import sys
import os
import time

from tqdm import tqdm

# this sets the amount of runs, which are performed
amount_of_runs = int(sys.argv[1])

# set the folder in which the cli.py is located
rerun_folder = os.path.abspath(os.path.dirname(__file__))

# the first one is the rerun.py script, the last is the output
used_arguments = sys.argv[2:-1]
output_location = os.path.abspath(sys.argv[-1])
time_start = time.time()

for run_id in tqdm(range(amount_of_runs)):
    # in each run, the arguments are reused
    cmd = ["python", os.path.join(rerun_folder, "cli.py")]
    cmd.extend(used_arguments)
    # the only exception is the output, which gets changed for each run, so that the examples are not overwritten
    cmd.append(output_location)
    # cmd.append(str(run_id%3))
    # cmd.append(os.path.join(output_location, str(run_id)))
    print(" ".join(cmd))
    # execute one BlenderProc run
    subprocess.call(" ".join(cmd), shell=True)

print(f"[Timing] Generating {amount_of_runs} images: {time.time() - time_start}")
print()