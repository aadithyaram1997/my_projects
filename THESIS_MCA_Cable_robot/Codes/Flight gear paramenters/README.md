# ✈️ FlightGear data logging (CSV) — Parameters for Motion Cueing

This folder contains the FlightGear logging setup used to record motion data for the thesis (input to the Classical Washout motion cueing pipeline).

---

## 📌 What is logged?
The XML config logs the following signals from the pilot / aircraft perspective:

- Translational accelerations: **ax, ay, az**
- Rotations / attitude: **roll, pitch, yaw**

Output is saved as a **CSV** file after the FlightGear run is finished.

---

## 📁 Files in this folder
- `flightgear_dataset.xml` — FlightGear logging configuration (defines which parameters are exported).
- `*.csv` — Example logged datasets (recorded after flying and closing FlightGear).

---

## ▶️ How to log data in FlightGear (Windows)

### 1) Copy the XML to the FlightGear Export folder
FlightGear only exports logs to this directory:

```text
C:\Users\<YOUR_USERNAME>\AppData\Roaming\flightgear.org\Export
```
### 2)  Run FlightGear with the config file
Open the FlightGear built-in terminal (Settings → Terminal) and run:

```bash
fgfs --config="C:\Users\<YOUR_USERNAME>\AppData\Roaming\flightgear.org\Export\flightgear_dataset.xml"
```

(Replace <YOUR_USERNAME> or adjust the path to match your machine.)

### 3) Fly the trajectory and close FlightGear

After you finish flying and close the FlightGear application, the logged data is written to a CSV file in:

```text
...\AppData\Roaming\flightgear.org\Export

```

## ✅ Notes / common issues

- Logging works only if the XML is placed in:
``` text
AppData\Roaming\flightgear.org\Export
```
- If no CSV appears:
    - Double-check the XML filename/path in the --config="..." command.
    - Ensure FlightGear was closed properly (logging finalizes on exit).

