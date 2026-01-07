# 💻 Thesis Code — Motion Cueing on Cable-Driven Parallel Robots (CDPR)

This folder contains the scripts and datasets used in the Master thesis:

**“Application of motion cueing on Cable-Driven Parallel Robots”**

The codebase covers the complete pipeline:
1) log motion data from FlightGear,  
2) generate motion cues using a Classical Washout Algorithm (CWA),  
3) validate poses using cable force feasibility and force distribution on an 8‑cable CDPR,  
4) compute force-based cost metrics and run parameter optimisation.

---

## ✅ Requirements / environment
- Some scripts depend on **WiPy3** and were developed to run on **Python 3.9.19**.
- Other scripts may run with standard scientific Python packages (NumPy / Pandas / SciPy / Matplotlib), but keeping Python 3.9.19 ensures consistency across the full pipeline.

---

## 📁 Folder overview

- `Flight gear parameters/`  
  FlightGear export configuration (`flightgear_dataset.xml`) + example logged CSV files.
  Used to record accelerations and orientations from the simulator.

- `MCA Classical Washout/`  
  Classical Washout Algorithm implementation. Reads FlightGear CSV (ax, ay, az, roll, pitch, yaw) and generates motion cues.
  Outputs motion-cue CSV and plots.

- `Force distribution/`  
  Computes force distribution for each pose on the **8 cables** and checks feasibility using `IRobot.getForceDistribution()`.
  If a pose is infeasible, zeros are inserted for that row and the pose index is printed. Also generates force plots.

- `Cost_factor/`  
  Scripts and outputs related to the force-factor / cost calculation used to evaluate how well motion cues match input forces.

- `FD_cost_factor/`  
  Cost-factor workflow combined with force distribution / feasibility checks.

- `Final_codes/`  
  Final / consolidated scripts used for thesis evaluation runs.

---

## 📄 Datasets in this folder (CSV)
Example CSV files are included at the `Codes/` root (names may vary), such as:
- `processed_motion_data*.csv` — processed/logged motion data
- `forces_after_motion_cueing.csv` — forces after applying motion cueing
- `force_distribution*.csv` — cable force distribution results
- `force_factor*.csv` — cost metric outputs

---

## ▶️ Typical workflow (recommended order)

1. **Log data from FlightGear**  
   Go to `Flight gear parameters/` and use the XML config to export motion data to CSV.

2. **Generate motion cues (CWA)**  
   Go to `MCA Classical Washout/` and run `MCA_classical_washout.py` on the logged dataset.  
   This produces the motion-cue output CSV + plots.

3. **Check feasibility + cable forces**  
   Go to `Force distribution/` and run `force_dis_MCA.py` using the generated motion-cue CSV.  
   This outputs cable forces for all 8 cables and highlights infeasible poses.

4. **Evaluate / optimise parameters (optional)**  
   Use `Cost_factor/` and `FD_cost_factor/` to compute cost metrics and test optimisation variants.

---

## ⚠️ Notes
- Many scripts contain hardcoded input/output paths (from the thesis development environment).  
  If something fails, first check the CSV filename/path inside the script.
- If you’re packaging this publicly later, a good next step is adding:
  - `requirements.txt`
  - one “entry script” to run the whole pipeline end-to-end
  - a small `data/sample/` dataset for quick testing
