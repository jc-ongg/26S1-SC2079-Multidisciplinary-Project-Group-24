<br />
<p align="center">
  <h1 align="center">
    CZ3004/SC2079 Multidisciplinary Project - Algorithm API
  </h1>
</p>

## Current integration setup

See [the root startup guide](../README.md) and [active calibration](CALIBRATION.md).
Algo runs on the laptop at **port 8000**; the separate CV service uses **8001**.
The original instructions below describe dependencies, not the whole robot setup.

## Code Setup

```bash
pip install -r requirements.txt
```

Start the server by

```bash
python main.py
```

The Algo server listens on `0.0.0.0:8000` by default (`ALGO_PORT` overrides it).
The optional Simulator/frontend is separate; it is not needed for tablet-to-Pi Task 1.
