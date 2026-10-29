"""Push the project to the Jetson and run commands there.

    JETSON_PASSWORD=... python tools/jetson.py push
    JETSON_PASSWORD=... python tools/jetson.py run "python3 -m pytest -q tests"

Commands run from ~/weld_cobot. Requires paramiko on this PC.
"""
import os
import sys
from pathlib import Path

import paramiko

HOST, USER, REMOTE = "192.168.94.94", "jetson2", "weld_cobot"
ROOT = Path(__file__).resolve().parents[1]
PUSH_DIRS = ["common", "calibration", "tests", "tools"]
SKIP_SUFFIXES = {".pyc", ".pdf", ".png"}


def connect():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=os.environ["JETSON_PASSWORD"], timeout=15)
    return c


def push(c):
    sftp = c.open_sftp()
    home = sftp.normalize(".")
    n = 0
    for d in PUSH_DIRS:
        for path in (ROOT / d).rglob("*"):
            if "__pycache__" in path.parts or path.suffix in SKIP_SUFFIXES:
                continue
            remote = f"{home}/{REMOTE}/{path.relative_to(ROOT).as_posix()}"
            if path.is_dir():
                c.exec_command(f"mkdir -p '{remote}'")[1].channel.recv_exit_status()
            else:
                c.exec_command(f"mkdir -p '{os.path.dirname(remote)}'")[1].channel.recv_exit_status()
                sftp.put(str(path), remote)
                n += 1
    print(f"pushed {n} files")


def run(c, cmd):
    _, out, err = c.exec_command(f"cd ~/{REMOTE} && {cmd}", get_pty=True)
    for line in iter(out.readline, ""):
        print(line, end="")
    return out.channel.recv_exit_status()


if __name__ == "__main__":
    client = connect()
    try:
        if sys.argv[1] == "push":
            push(client)
        elif sys.argv[1] == "run":
            sys.exit(run(client, sys.argv[2]))
    finally:
        client.close()
