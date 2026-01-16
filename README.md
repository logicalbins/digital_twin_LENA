# DIGITAL TWIN

Minimal prototype for a 5G NR digital twin workflow combining real measurements and LENA (ns-3) simulations

This repository models a digital twin of the **5G NR network deployed in the Cumucore laboratory**:
an indoor office mixed scenario with a **single gNB** covering an
approximately **10 × 10 m²** area.


## Repository Structure
```
├── data_ingestion/
│ └── client_v3.py
├── cttc-nr-mimo-demo.cc
├── rem-example.cc
└── README.md
```

## Components

### data_ingestion/client_v3.py
Python client for collecting network measurements from a live 5G setup.

- Collects KPIs such as:
  - Uplink / downlink throughput
  - RTT / latency
  - Radio metrics (e.g. SINR, RSRP)
- Outputs timestamped CSV files
- Intended to run on a UE-side measurement device

### cttc-nr-mimo-demo.cc
LENA (ns-3) demonstrating:
- 5G NR MIMO configuration
- Beamforming and scheduler behavior
- Used as a basis for digital twin calibration and evaluation

### rem-example.cc
LENA (ns-3) radio environment mapping (REM) example.

- Generates SINR / interference heatmaps
- Used for propagation and coverage validation
- Supports comparison with measured radio data

## Dependencies

- ns-3.42 with 5G-LENA-v3.3.y module
- Python 3.x
- iperf3 (for measurement setup)

## Usage

1. Run `client_v3.py` to collect measurement data.
2. Process and select stable measurement windows.
3. Configure and run LENA simulations (`*.cc`) to reproduce measured conditions.
4. Modify simulation parameters (e.g. TDD pattern, Tx power) to evaluate alternatives.

## Running the ns-3

The C++ simulation files in this repository are intended to be used within a
standard **ns-3 LENA** setup.

### File Location

Copy the `.cc` files into the LENA examples directory:

```
ns-3/
└── contrib/
 └── nr/
  └── examples/
   ├── cttc-nr-mimo-demo.cc
   └── rem-example.cc
```

## Notes

- TCP is used for throughput measurements.
- Latency and backhaul congestion are not fully modeled in LENA.