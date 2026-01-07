
# 🤖 Projects & Experience Portfolio



## **Aadithya Ramamurthy** | M.Sc. Mechatronics  

<img align="right" src="https://avatars.githubusercontent.com/u/249375404" width="260" alt="Profile photo" />

*Robotics, Automation & Control Systems, Mobile Robotics and Autonomous Systems, Software Engineering*

Mechatronics engineer (M.Sc.) with a focus on robotics, automation and control systems, and autonomous mobile systems. Currently building this portfolio to document selected academic and research projects completed during my M.Sc. Mechatronics at **FH Aachen**.

Hands-on research experience includes working as a student research assistant at **Fraunhofer IPA (Stuttgart)** and **RWTH Aachen**, contributing to practical robotics and automation tasks, prototyping, and software-focused development in interdisciplinary environments. This repository brings together code, documentation, presentations, and reports across topics such as ROS/ROS2-based mobile robotics, PLC automation, computer vision pipelines, embedded prototyping, sensor integration, and simulation workflows.

The goal is to provide clear, reproducible project snapshots (what was built, how it works, and key results), while continuously improving structure and documentation as the repository is migrated and organized.

This repository contains academic and professional projects from my Master's studies and research work, showcasing expertise in:
- **Robotics & Automation** (ROS/ROS2, PLC, Python, C++)
- **Computer Vision** (OpenCV, object detection)
- **Embedded Systems** (Arduino, ESP32, C)
- **Sensor Integration** (EtherCAT, MEMS)
- **Simulation** (Gazebo, LabVIEW, CAD)

---

## 📂 Projects Overview


### 🎓 Master Thesis - Motion Cueing Algorithm (MCA) for Cable Robot
**Folder:** `THESIS_MCA_Cable_robot`  
- Implemented **Classical Washout MCA** in Python for cable-driven parallel robot for motion simulation
- Achieved **up to 80% of real platform forces** for standard flight trajectories compared to the simulated forces on FlighGear Software
- Performed parameter optimization using cost function for realistic motion perception
- Integrated libraries for **cable force distribution** and **geometric constraints**
- **Grade:** 1.3  
- **Tech:** Python, Motion Cueing, Cable Robots, Optimization

---


###  Autonomous Guided Vehicle (AGV) for Shopfloor Delivery
**Folder:** `AGV_Mower`  
- Developed AGV for Industry 4.0 lab with **ROS2** on **Odroid N2**
- Implemented object detection using **IFM 3D camera**
- Applied **SLAM with LiDAR** for lab environment mapping
- Tested **AR markers** and **RFID tags** for navigation
- **Tech:** ROS2, Python, OpenCV, LiDAR, 3D Camera, Odroid

---

###  Robot Art Gallery
**Folder:** `Robot_art_gallery`  
- Programmed **TurtleBot** with **ROS2** to autonomously navigate art gallery environment
- Implemented **SLAM mapping** to create environment map and navigate to specified coordinates
- Developed **image detection system** using **YOLO** and **OpenCV** to identify artworks on walls
- Detected **AprilTag markers** associated with each artwork for identification
- **Tech:** ROS2, Python, OpenCV, YOLO, TurtleBot, AprilTag Detection, SLAM

---

###  Autonomous Unmanned Aerial Vehicle (UAV)
**Folder:** `UAV_Project`  
- Programmed UAV in **Gazebo** for autonomous takeoff, navigation through color-coded gates, and landing
- Developed gate detection system using **OpenCV** (HSV filtering, contour tracking, center alignment)
- Implemented sequential **yaw correction** and loop maneuvers
- **Tech:** Gazebo, ROS2, OpenCV, Python, Computer Vision

---

###  Cost-Effective Autonomous Lawn Mower Prototype
**Folder:** `Lawn_mower_prototype`  
- Built autonomous lawn mower with **Arduino** and **NodeMCU ESP32**
- Programmed in **Embedded C** with obstacle avoidance, boundary detection, line following
- Created requirements specification with **FMEA**
- Developed **SysML/UML diagrams** (state, use case, flow, parametric)
- **Tech:** Arduino, ESP32, C, SysML/UML, Embedded Programming

---

###  PLC Project - Beckhoff TwinCAT System
**Folder:** `PLC_Project_Block_Diagram`  
- Automated robotic test setup using **Beckhoff TwinCAT PLC**
- Developed **Structured Text (IEC 61131)** and **HMI UI** for control visualization
- Integrated sensors via **EtherCAT** and **Digital I/O**
- Documented applicability of **ISO 12100** for humanoid robots
- **Tech:** Beckhoff TwinCAT, Structured Text, EtherCAT, Industrial Automation

---


###  Angular Rate Sensor Characterization
**Folder:** `Characterisation_of_angular_rate_sensor`  
- Characterized **ADXRS300** and **Bosch SMG040** sensors with **LabVIEW**
- Built measurement setup with turntable control
- Achieved rotation rates up to **350°/s** with sensitivity **5.17–6.78 mV/°/s**
- Analyzed response curves and saturation behavior
- **Tech:** LabVIEW State Machine, Sensor Testing, Data Acquisition

---

###  Angular Rate Sensor Concept Design
**Folder:** `Design_of_angular_rate_sensor`  
- Designed **Coriolis-based angular rate sensor** in **MEMSPro**
- Sized spring-mass system with differential detection and self-test
- Calculated **PolyMUMPs** process structural parameters in **MATLAB**
- **Tech:** MEMS Design, PolyMUMPs Process

---

###  Additive Manufacturing Project
**Folder:** `Additive_Manufacturing_Project`  
- Evaluated additive manufacturing process for motor block production
- Compared with conventional manufacturing methods
- Conducted **break-even analysis** within given specifications
- **Tech:** CAD, Manufacturing Process Analysis

---



## 🛠️ Tech Stack

**Programming Languages:**  
![Python](https://img.shields.io/badge/-Python-3776AB?logo=python&logoColor=white) 
![C](https://img.shields.io/badge/-C-A8B9CC?logo=c&logoColor=black) 
![C++](https://img.shields.io/badge/-C++-00599C?logo=cplusplus&logoColor=white) 
![IEC 61131](https://img.shields.io/badge/-IEC_61131_ST-orange)

**Robotics & Automation:**  
ROS/ROS2 · Beckhoff TwinCAT · OpenCV · Gazebo · Visual Components · Arduino IDE

**CAD/Simulation:**  
CATIA V5 · SolidWorks · Autodesk · MEMSPro · MATLAB/Simulink · Ansys

**Tools:**  
Git · Docker · CI/CD · LabVIEW · Jira · Confluence

---

## 💼 Work Experience

### Research Assistant — Fraunhofer IPA, Stuttgart (11/2023 – 07/2025)
- Automated and commissioned a closed-loop robotics test setup using Beckhoff TwinCAT PLC, including a UI/HMI for control and visualization (Structured Text).
- Integrated sensors to the PLC via EtherCAT and Digital I/O.
- Documented the applicability of ISO 12100 for humanoid-related systems/workflows.
- Developed approaches for collision path programming for a UR-10 cobot in Visual Components (VC), including adapting the Python add-in and UI.
- Simulated collision scenarios for the UR10 by varying tool orientations, positions, and collision surfaces in VC.
- Completed training in KUKA System Software 8.5, including robot programming and teaching.

### Research Assistant — RWTH Aachen, Aachen (02/2022 – 12/2022)
- Contributed to an autonomous and connected driving project by debugging and validating a ROS1 pipeline.
- Helped develop object segmentation and lane detection nodes in ROS2/RViz and ran scenario tests in Virtual Test Drive.

### Intern — Villgro Innovations Foundation, Bengaluru (11/2019 – 01/2020)
- Supported planning, marketing, and organization of “Unconvention” (annual summit for investors).
- Conducted time-to-market analysis for CleanTech start-ups and built a database of investment opportunities.

### Intern — Technologies Global Private Limited, Bengaluru (01/2019 – 03/2019)
- Worked on mechanical design tasks using CATIA V5 and PLM processes.

### Intern — Bharat Electronics Limited, Bengaluru (01/2018 – 02/2018)
- Training in the manufacturing department, including exposure to CNC machines and additive manufacturing technologies.

---
## 📚 Repository Structure

```text
my-projects/
├── AGV_Mower/ # AGV project code & docs
├── Additive_Manufacturing_Project/ # Manufacturing analysis
├── Characterisation_of_angular_rate_sensor/ # Sensor characterization
├── Design_of_angular_rate_sensor/ # MEMS sensor design
├── Lawn_mower_prototype/ # Arduino lawn mower
├── PLC_Project_Block_Diagram/ # TwinCAT PLC system
├── Robot_art_gallery/ # Robot Art Gallery
├── THESIS_MCA_Cable_robot/ # Master thesis files
├── UAV_Project/ # Autonomous UAV code
└── README.md # This file
```

---

## 📫 Contact

**Aadithya Ramamurthy**  
📧 aadithyaram1997@gmail.com  
🔗 [GitHub](https://github.com/aadithyaram1997)  
💼 [LinkedIn](#) *(www.linkedin.com/in/aadithya-ramamurthy)*

---

## 📜 License

This repository is for **portfolio and educational purposes**. Individual project licenses may apply.

---

<div align="center">

**⭐ Star this repo if you find it useful!**  
**👁️ Watch for project updates as files are being added**

**Repository migration in progress**  
Importing project files and documentation from various sources

📁 Adding project files  
🔧 Organizing structure  
⏳ Full setup coming soon  


</div>

<br>

</div>

