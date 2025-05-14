# Helium-Bipedal-Robot
Master's thesis project – helium-assisted bipedal robot with 1DOF and 2DOF designs

This repository contains all code, CAD models, and result files used in my master's thesis in Robotics and Intelligent Systems at the University of Oslo. The project investigates the impact of mechanical complexity on walking performance in lightweight, buoyancy-assisted bipedal robots.

Two design configurations are included:

---

## 🔧 1DOF Design – Knee Only

**Description:**  
This design uses one actuated joint per leg (the knee). The hip is fixed, and movement is generated solely through the knees.

**Used components:**
- 2 × EMAX ES08A II servos (one per knee)
- 9 helium balloons (8 pelvis-mounted, 1 central fine-tuning balloon)
- Raspberry Pi Zero 2 W
- Custom 3D printed knees and feet
- Motion capture tracking (Qualisys)

**Relevant files:**
- `/1DOF/models/`: CAD files for 1DOF joints
- `/1DOF/code/`: control scripts and server logic


---

## 🔧 2DOF Design – Hip and Knee

**Description:**  
This design adds hip actuation to each leg, increasing mechanical complexity and enabling more human-like walking.

**Used components:**
- 4 × EMAX ES08A II servos (2 per leg: hip + knee)
- 8 helium balloons mounted on the pelvis + one in the middle and two more on the top
- Raspberry Pi Zero 2 W 
- Redesigned hip mechanism (3D printed)
- Motion capture tracking (Qualisys)

**Relevant files:**
- `/2DOF/models/`: CAD files for 2DOF hips and knees
- `/2DOF/code/`: control and optimization scripts


---



## 🧠 Thesis Summary

The findings show that although the 2DOF design increases mechanical complexity, the 1DOF configuration achieved more repeatable gait performance with minimal weight. The work highlights a trade-off between mechanical complexity and gait consistency in low-weight bipedal robots.

---

