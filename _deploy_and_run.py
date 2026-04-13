"""Deploy firmware to Red Pitaya, compile, and run."""
import paramiko
import os
import sys
import time

RP_HOST = "10.0.0.15"
RP_USER = "root"
RP_PASS = "root"
REMOTE_DIR = "/root/redPitaLock/firmware"
LOCAL_DIR = os.path.join(os.path.dirname(__file__), "firmware")


def ssh_exec(ssh, cmd):
    print(f"  > {cmd}")
    _, stdout, stderr = ssh.exec_command(cmd)
    out = stdout.read().decode()
    err = stderr.read().decode()
    rc = stdout.channel.recv_exit_status()
    if out.strip():
        print(out.strip())
    if err.strip():
        print(err.strip(), file=sys.stderr)
    return rc, out, err


def upload_dir(sftp, local, remote):
    try:
        sftp.stat(remote)
    except FileNotFoundError:
        sftp.mkdir(remote)
    for item in os.listdir(local):
        local_path = os.path.join(local, item)
        remote_path = f"{remote}/{item}"
        if os.path.isdir(local_path):
            upload_dir(sftp, local_path, remote_path)
        else:
            print(f"  uploading {item}")
            sftp.put(local_path, remote_path)


def main():
    print(f"Connecting to {RP_HOST}...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(RP_HOST, username=RP_USER, password=RP_PASS, timeout=10)
    print("Connected.\n")

    print("Uploading firmware...")
    sftp = ssh.open_sftp()
    try:
        sftp.stat("/root/redPitaLock")
    except FileNotFoundError:
        sftp.mkdir("/root/redPitaLock")
    upload_dir(sftp, LOCAL_DIR, REMOTE_DIR)
    sftp.close()
    print("Upload complete.\n")

    print("Stopping old instance...")
    ssh_exec(ssh, "killall stabilizer_rp 2>/dev/null; sleep 1; echo done")

    print("\nCompiling...")
    rc, _, _ = ssh_exec(ssh, f"cd {REMOTE_DIR} && make clean && make")
    if rc != 0:
        print("Compilation failed!", file=sys.stderr)
        ssh.close()
        sys.exit(1)
    print("\nCompilation successful!")

    print("\nStarting stabilizer_rp...")
    chan = ssh.get_transport().open_session()
    chan.exec_command(
        f"cd {REMOTE_DIR} && nohup ./stabilizer_rp > /tmp/stabilizer_rp.log 2>&1 &"
    )
    time.sleep(2)
    chan.close()

    rc, out, _ = ssh_exec(ssh, "pgrep -a stabilizer_rp")
    if "stabilizer_rp" in out:
        print(f"\nstabilizer_rp is running on {RP_HOST}:5000")
    else:
        print("\nFailed to start! Log:")
        ssh_exec(ssh, "cat /tmp/stabilizer_rp.log")

    ssh_exec(ssh, "cat /tmp/stabilizer_rp.log")
    ssh.close()


if __name__ == "__main__":
    main()
