import platform
import subprocess
import sys
import os

def run(cmd):
    print(f"[*] Running: {cmd}")
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print("[!] Command failed")
        sys.exit(result.returncode)

def main():
    system = platform.system()
    print(f"[*] Detected OS: {system}")

    run([sys.executable, "-m", "pip", "install", "--upgrade", "pip", "setuptools", "wheel", "build"])

    run([sys.executable, "-m", "build", "--wheel"])

    dist_dir = os.path.join(os.path.dirname(__file__), "dist")
    wheel_files = [f for f in os.listdir(dist_dir) if f.endswith(".whl")]
    if not wheel_files:
        print("[!] No wheel file found in 'dist/' directory")
        sys.exit(1)
    wheel_path = os.path.join(dist_dir, wheel_files[0])

    run([sys.executable, "-m", "pip", "install", "--force-reinstall", wheel_path])

    print("[*] Installation process completed")

if __name__ == "__main__":
    main()