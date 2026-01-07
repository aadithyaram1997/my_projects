# 🎓 Application of Motion-Cueing Algorithm on a Cable-Driven Parallel Robot (CDPR)

This repository contains the complete project files (code, report, and presentation) for the Master thesis **“Application of Motion-Cueing Algorithm on a Cable-Driven parallel robot”**.

The work focuses on implementing and evaluating a **Classical Washout Algorithm (CWA)** for motion cueing on a **Cable-Driven Parallel Robot (CDPR)**, with feasibility enforced using **cable force distribution constraints** (target platform: IPAnema 3 at Fraunhofer IPA, 8-cable CDPR).

---

## 🗂️ Repository structure

- `Codes/`  
  💻 Python scripts and datasets used for data logging, motion cue generation (CWA), cable force feasibility validation, and cost-function based optimisation.

- `Report/`  
  📄 Full thesis report (methodology, equations, results, evaluation).

- `Presentation/`  
  🖥️ Final thesis presentation (high-level overview, pipeline, key plots and results).

---

## 🧠 Project overview (what this thesis does)

### ✈️ 1) Dataset generation (FlightGear)
A flight trajectory dataset is created using FlightGear by logging:
- 📈 Translational accelerations (x, y, z)
- 🧭 Rotations / attitude (roll, pitch, yaw)

These signals are exported to CSV and used as the input to the motion cueing algorithm.

### 🌀 2) Classical Washout Algorithm (CWA) implementation
The Classical Washout Algorithm is implemented in Python following the typical 3-channel structure:
- ↔️ Translational channel (high-frequency cues via high-pass filtering + integration)
- 🔄 Rotational channel (high-frequency rotational cues via high-pass filtering + integration)
- 🧷 Tilt-coordination channel (low-frequency translational cues approximated using gravity-induced tilt)

Key parameters (scaling factors, washout damping, filter order, cut-off frequencies, sampling rate) can be tuned to control:
- 🎚️ How aggressively motion is reproduced
- 🧼 How fast the platform “washes out” back to neutral
- 🎛️ How the signal is separated into low/high frequency components

The algorithm output is a 6-DOF pose trajectory (motion cues), saved as CSV and visualized with plots.

### 🧵 3) Force feasibility using cable force distribution
Because CDPRs are constrained by cable tension limits, every generated pose is checked for feasibility using cable force distribution.
If a pose is not feasible (forces outside limits), it is flagged as invalid in the force distribution results.

### 🧮 4) Cost-function based optimisation (optional)
A force-based cost function is defined to compare:
- forces implied by the input trajectory  
vs.  
- forces implied by the motion-cued output trajectory

Optimisation is performed to tune selected washout parameters (primarily translational washout damping), while enforcing feasibility via cable force checks.

---

## ▶️ Running the project (typical workflow)

1. Go to `Codes/Flight gear parameters/` ✈️  
   Generate / log FlightGear motion data to CSV.

2. Go to `Codes/MCA Classical Washout/` 🌀  
   Run the CWA implementation to generate motion cues (output CSV + plots).

3. Go to `Codes/Force distribution/` 🧵  
   Compute cable forces for the generated motion cues and validate feasibility.

4. (Optional) Use `Codes/Cost_factor/` and `Codes/FD_cost_factor/` 🧮  
   Evaluate force-factor metrics and run optimisation variants.

---

## 🧰 Environment notes

- Some scripts require **WiPy3** and are intended to run on **Python 3.9.19**.
- Several scripts contain hardcoded paths and filenames from the original thesis setup.  
  If something fails, first check and update the input/output file paths inside the script.

---

## ✅ Key results (high level)

- 🧩 A complete simulation pipeline was developed to generate **force-feasible** motion cues for a CDPR using a Classical Washout Algorithm.
- 📉 The cost-function optimisation produced modest improvements in the cost metric in some cases, and improved overall force stability in cable force distribution plots.
- 🧪 The approach was additionally tested on different trajectories and payload conditions.

---

## 🚀 Future work (from thesis outlook)

- 🏗️ Hardware implementation on a real motion platform
- 🔄 Optimisation of rotational parameters (not only translational damping)
- 🧭 Axis-wise / decoupled optimisation strategies
- 📍 Adaptive platform orientation / origin shifting for better workspace usage
- ⏱️ Real-time optimisation and generalisation to other CDPR systems
