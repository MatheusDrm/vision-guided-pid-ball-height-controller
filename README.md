# Vision‑Guided PID Ball Height Controller

A complete experimental setup for closed‑loop control of a lightweight ball’s vertical position. This project uses real‑time computer vision (OpenCV) as feedback to a PID controller, which drives a potentiometer‑based actuator regulating rotor speed. By specifying a desired height setpoint, the system automatically adjusts rotor torque to keep the ball hovering at the target position.

---

## 🛠️ Features

- **Real‑time vision feedback**  
  Detects and tracks the ball’s position using OpenCV image processing pipelines.  
- **Tunable PID control**  
  Implements a classic PID loop (Proportional–Integral–Derivative) to minimize height error.  
- **Potentiometer‑based actuator**  
  Converts control voltage to rotor speed adjustments for fine‑grained lift.  
- **Setpoint configuration**  
  Accepts user‑defined height commands via simple serial console.  
- **Data logging & visualization**  
  Records position, control output, and error signals for offline analysis.

---


