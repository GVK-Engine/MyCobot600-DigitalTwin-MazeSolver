# MyCobot600-DigitalTwin-MazeSolver
Digital Twin + AI Vision-Based Maze Solver using MyCobot Pro 600 | MATLAB + Python | RAS545 Project (ASU)
# 🤖 Smart & Accurate Path Planning with MyCobot Pro 600

**Contributors:** Vamshikrishna Gadde 


---

## 🚗 Project Overview
This project demonstrates how **digital twin simulation, computer vision, and robotic control** can be integrated for precise motion planning and execution.  
We connected **Elephant Robotics' MyCobot Pro 600** to its **MATLAB Simscape digital twin**, solving a **4×4 maze** autonomously.

---

## 🧠 Key Components
- **Computer Vision (Python, OpenCV):**  
  Captures maze via AI-Kit camera → Detects walls, start & end blobs → Solves path using Dijkstra algorithm.
- **Kinematic Modeling (MATLAB):**  
  Defined the robot via `rigidBodyTree`, verified with forward kinematics, and validated with the digital twin.
- **Inverse Kinematics & Trajectory Planning:**  
  Converted pixel path → workspace waypoints → joint angles → verified trajectory.
- **Real-Time Control:**  
  Streamed joint angles to robot over TCP sockets for smooth, collision-free motion.

---

## 🧩 Workflow Diagram
1. Maze Capture → 2. Maze Solution → 3. IK Path Generation → 4. Digital-Twin Validation → 5. Real-Robot Execution  

---

## 🧰 Tools & Technologies
| Category | Tools |
|-----------|--------|
| Programming | Python (OpenCV, NumPy), MATLAB |
| Simulation | Simscape Multibody, Robotics Toolbox |
| Hardware | MyCobot Pro 600, AI Kit Camera |
| Control | TCP Socket, URDF-based kinematic model |

---

## 📊 Results
- Successfully solved 4×4 maze and executed precise arm motion.
- Verified predicted vs. actual end-effector paths (<5% deviation).
- Demonstrated strong digital-twin validation loop before deployment.

![Digital Twin Result](images/digital_twin.png)

---

## 🎥 Project Demo
[▶️ Watch Project Video](https://drive.google.com/drive/folders/1pzRG9dZ7iL80BF3FCh8-2rIh7DU6K-nc?usp=sharing)

---

## 📚 References
- [MyCobot Pro 600 Documentation](https://docs.elephantrobotics.com/docs/gitbook-en/2-serialproduct/2.3-mycobot_Pro_600/)
- [MATLAB Robotics Toolbox](https://www.mathworks.com/help/robotics/)
- [OpenCV Dijkstra Maze Solver (Python)](https://docs.opencv.org/)

---

## ⚙️ Keywords
**Digital Twin • Motion Planning • Inverse Kinematics • Python-MATLAB Integration • Autonomous Systems • Real-Time Control • Path Optimization • Robotics System Design • Manufacturing Automation • Simulation Validation**

---
