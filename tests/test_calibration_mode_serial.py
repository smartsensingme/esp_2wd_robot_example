"""Exercise the actual Octave MODE client through a pseudo-terminal (no board)."""
import os
import pty
import select
import subprocess
import time

for mode in ("normal", "calibration", "startup_error"):
    master, slave = pty.openpty()
    port = os.ttyname(slave)
    code = (
        'addpath("components/esp_2wd_robot/tools/octave"); '
        'try; [r,d]=robot_calibration_connect("' + port + '",2); '
        'assert(strcmp("' + mode + '","calibration")); clear d; '
        'catch err; assert(!strcmp("' + mode + '","calibration")); '
        'if(strcmp("' + mode + '","startup_error")); '
        'assert(!isempty(strfind(err.message,"resource=left"))); '
        'assert(!isempty(strfind(err.message,"ESP_ERR_TIMEOUT"))); '
        'else; assert(!isempty(strfind(err.message,"modo normal"))); endif; end;'
    )
    process = subprocess.Popen(
        ["octave", "--quiet", "--eval", code],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    try:
        command = b""
        deadline = time.monotonic() + 8
        while b"\n" not in command and time.monotonic() < deadline:
            if select.select([master], [], [], 0.1)[0]:
                command += os.read(master, 1024)
        assert command.strip() == b"MODE", command
        response = (
            "ERR command=STARTUP code=ESP_ERR_TIMEOUT resource=left "
            "message=hardware_initialization_failed"
            if mode == "startup_error" else "OK command=MODE mode=" + mode
        )
        os.write(master, ("> " + response + "\n").encode())
        output, _ = process.communicate(timeout=5)
        assert process.returncode == 0, output
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        os.close(master)
        os.close(slave)
print("Octave physical-mode acceptance/rejection integration passed")
