#!/usr/bin/env python3
"""One-time laptop setup for Task 1. Works on Windows, macOS and Linux.

    python laptop_setup.py

Builds the two virtualenvs with the exact package versions from the laptop
that ran Task 1 3/3, and fetches the recognition model:

    Algo/.venv                  <- Algo/requirements.lock.txt
    RPi-services/image-api/venv <- RPi-services/image-api/requirements.txt
    RPi-services/image-api/best_v11i_m.onnx  (GitHub release, checksum-verified)

Uses uv if it is installed (much faster, and fetches Python 3.12 itself);
otherwise the Python running this script, which must be 3.12 or 3.13.

Then: python laptop_start.py, and python laptop_verify.py to confirm.
"""
import hashlib
import os
import shutil
import subprocess
import sys
import tempfile
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
ALGO = REPO / "Algo"
IMAGE_API = HERE / "image-api"

REPO_SLUG = "jc-ongg/26S1-SC2079-Multidisciplinary-Project-Group-24"
MODEL_TAG = "cv-model-v11i"
MODEL = IMAGE_API / "best_v11i_m.onnx"
MODEL_SHA256 = "8c4f382f800d123e069fe93f0069e279175594c0a6ef631c699fbc3d957ee7d5"
MODEL_PAGE = f"https://github.com/{REPO_SLUG}/releases/tag/{MODEL_TAG}"

ENVS = [
    # (name, venv dir, lock file)
    ("Algo", ALGO / ".venv", ALGO / "requirements.lock.txt"),
    ("image-api", IMAGE_API / "venv", IMAGE_API / "requirements.txt"),
]


def venv_python(venv: Path) -> Path:
    if os.name == "nt":
        return venv / "Scripts" / "python.exe"
    return venv / "bin" / "python"


def run(cmd, **kw):
    print("  $ " + " ".join(str(c) for c in cmd), flush=True)
    subprocess.run([str(c) for c in cmd], check=True, **kw)


def build_env(name, venv, lock):
    print(f"\n== {name}: {venv.relative_to(REPO)} ==")
    py = venv_python(venv)
    uv = shutil.which("uv")
    if uv:
        if not py.exists():
            run([uv, "venv", "--python", "3.12", venv])
        run([uv, "pip", "install", "--python", py, "-r", lock])
    else:
        if not py.exists():
            run([sys.executable, "-m", "venv", venv])
        run([py, "-m", "pip", "install", "--upgrade", "pip"])
        run([py, "-m", "pip", "install", "-r", lock])


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def fetch_model() -> bool:
    print(f"\n== model: {MODEL.relative_to(REPO)} ==")
    if MODEL.exists() and sha256(MODEL) == MODEL_SHA256:
        print("  already present, checksum OK")
        return True

    if MODEL.exists():
        print(f"  Existing model has a different checksum; leaving it untouched: {MODEL}")
        print("  Preserve/rename it yourself before requesting a new download.")
        return False

    # Never install a partial or unchecked model into the active model path.
    with tempfile.TemporaryDirectory(prefix="mdp-model-", dir=IMAGE_API) as folder:
        candidate = Path(folder) / MODEL.name
        if not download_model(candidate):
            return False
        if sha256(candidate) != MODEL_SHA256:
            print("  !! checksum mismatch; downloaded candidate not installed.")
            return False
        # Atomic, no-overwrite installation on this same filesystem. A model
        # someone placed concurrently makes this fail rather than replacing it.
        os.link(candidate, MODEL)
    print("  downloaded, checksum OK")
    return True


def download_model(candidate: Path) -> bool:

    # The repo is private, so an anonymous download of the release asset fails.
    gh = shutil.which("gh")
    if gh:
        try:
            run([gh, "release", "download", MODEL_TAG, "--repo", REPO_SLUG,
                 "--pattern", MODEL.name, "--dir", candidate.parent])
        except subprocess.CalledProcessError:
            print("  gh download failed (logged in? `gh auth login`)")
    elif os.environ.get("GITHUB_TOKEN"):
        api = f"https://api.github.com/repos/{REPO_SLUG}/releases/tags/{MODEL_TAG}"
        hdr = {"Authorization": f"Bearer {os.environ['GITHUB_TOKEN']}"}
        import json
        rel = json.load(urllib.request.urlopen(urllib.request.Request(api, headers=hdr), timeout=30))
        asset = next(a for a in rel["assets"] if a["name"] == MODEL.name)
        req = urllib.request.Request(asset["url"], headers={**hdr, "Accept": "application/octet-stream"})
        print(f"  downloading {asset['size'] / 1e6:.0f} MB ...", flush=True)
        with urllib.request.urlopen(req, timeout=120) as r, open(candidate, "wb") as f:
            shutil.copyfileobj(r, f)

    if not candidate.exists():
        print(f"  !! model missing. Download {MODEL.name} from\n     {MODEL_PAGE}\n"
              f"     into {IMAGE_API} and run this again.")
        return False
    return True


def main():
    if not shutil.which("uv") and sys.version_info[:2] not in ((3, 12), (3, 13)):
        sys.exit(f"Python {sys.version.split()[0]} found. Use Python 3.12 or 3.13 "
                 "(or install uv: https://docs.astral.sh/uv/).")
    for name, venv, lock in ENVS:
        build_env(name, venv, lock)
    ok = fetch_model()
    print("\n" + ("Setup complete. Next: python laptop_start.py" if ok
                  else "Setup incomplete -- see the model message above."))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
