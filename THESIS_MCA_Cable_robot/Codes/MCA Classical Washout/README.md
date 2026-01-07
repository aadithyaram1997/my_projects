# 🌀 Classical Washout Motion Cueing (CWA) — From FlightGear CSV to Motion Cues

This folder contains the **Classical Washout Algorithm (CWA)** implementation used to generate 6‑DOF motion cues from FlightGear-logged motion data.

The main script reads translational accelerations (**x, y, z**) and orientation signals (**roll, pitch, yaw**) from a CSV file and outputs **motion cue trajectories**, which are saved as a new CSV and visualized in plots.

---

## ✅ Requirements
- Python (recommended to match the thesis environment)
- Common libraries typically used here: `numpy`, `pandas`, `matplotlib`, `scipy` (filters)

> Some other thesis scripts depend on WiPy3 (Python 3.9.19), but this folder focuses on generating motion cues from CSV.  
> If your codebase expects Python 3.9.19, stick with that for consistency.

---

## 📌 Main script
### `MCA_classical_washout.py`
What it does:
- Loads a CSV of logged FlightGear parameters (accelerations + roll/pitch/yaw).
- Runs the classical washout pipeline (filtering + scaling + washout return-to-zero behavior).
- Saves the generated motion cues to a CSV output file.
- Shows plots of signals when executed (inputs vs filtered / output cues).

---

## 📂 Example input data
- `processed_motion_data.csv` — Example dataset already included in this folder.

---

## ▶️ How to run
From inside this folder:

```bash
python MCA_classical_washout.py
```

If the script has a hardcoded filename/path, update it to point to your CSV (e.g., processed_motion_data.csv).

## 🔧 Parameters you can tune

You can modify parameters inside the script depending on your experiment:

Common tuning knobs
- Scaling factors (reduce initial input magnitude so outputs stay within platform limits).
- Washout damping / washout speed (how quickly the cues return to neutral).

Optional/advanced tuning
- Filter order
- High-pass and low-pass cut-off frequencies
- Sampling rate (important for correct filter behavior)

``` text
Tip: When changing cut-off frequencies, make sure the sampling rate in the script matches the CSV logging rate.
```

## Output

When you run the script you should get:
- A generated CSV containing motion cues (output trajectory)
- One or more plots visualizing the input data and the generated cues

## 📝 Notes

This script expects the CSV to contain the relevant columns for:

- Accelerations: x, y, z
- Rotations: roll, pitch, yaw

If your logged CSV uses different column names, rename the columns or adjust the script accordingly.



