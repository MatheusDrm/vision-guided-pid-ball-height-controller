# Vision‑Guided PID Ball Height Controller

A complete experimental setup for closed‑loop control of a lightweight ball’s vertical position. This project uses real‑time computer vision (OpenCV) as feedback to a PID controller, which drives a potentiometer‑based actuator regulating rotor speed. By specifying a desired height setpoint, the system automatically adjusts rotor torque to keep the ball hovering at the target position.

---

## 🛠️ Features

* **Real‑time vision feedback**
  Detects and tracks the ball’s position using OpenCV image processing pipelines.
* **Tunable PID control**
  Implements a classic PID loop (Proportional–Integral–Derivative) to minimize height error.
* **Potentiometer‑based actuator**
  Converts control voltage to rotor speed adjustments for fine‑grained lift.
* **Setpoint configuration**
  Accepts user‑defined height commands via serial interface.
* **Data logging & visualization**
  Records position, control output, and error signals for offline analysis.

---

## 🔄 Project Workflow

This system comprises three coordinated Python modules, each responsible for one stage of the closed‑loop control:

1. **Socket Server (`ServidorThreads.py`)**

   * Hosts a TCP server on a central machine (e.g., PC).
   * Maintains two shared variables: `altura_desejada` (desired height setpoint) and `altura_medida` (measured height).
   * Listens for connections from the vision client and the actuator client, dispatching each into its own thread.

2. **Vision Client (`height_estimator_client.py`)**

   * Captures live video from a USB camera or video file.
   * Applies HSV thresholding and contour detection (via OpenCV) to locate the ball.
   * Converts pixel position to real‑world height (cm) using a calibration factor.
   * Periodically sends the latest `altura_medida` to the socket server.
   * Allows the user to adjust vision parameters at runtime via OpenCV trackbars.

3. **Actuator Client (`soprador.py`)**

   * Runs on a Raspberry Pi, connecting to the same TCP server.
   * Retrieves the current `altura_desejada` and `altura_medida` from the server.
   * Computes a PID control signal to minimize height error.
   * Drives a blower/rotor assembly via GPIOZero PWM—regulated through a potentiometer feedback loop.
   * Optionally reads an onboard ultrasonic sensor for local height fallback or safety checks.
   * Cleans up GPIO on exit to ensure safe shutdown.

These components form a feedback loop:

```text
User Setpoint → Socket Server ←→ Vision Client (measures height)
                         ↓
                   Actuator Client (computes & applies control)
                         ↓
                    Physical Ball Height
                         ↑
                             Feedback
```


## 🚀 Getting Started

1. **Start the Socket Server**

   ```bash
   python ServidorThreads.py
   ```
2. **Launch Vision Client** on your PC:

   ```bash
   python height_estimator_client.py --video 0 --host <SERVER_IP> --port 5000
   ```
3. **Deploy Actuator Client** on Raspberry Pi:

   ```bash
   python soprador.py --host <SERVER_IP> --port 5000
   ```
4. **Adjust Setpoint** via the server console or remote command:

   * Enter desired height in cm; the server propagates this to the actuator.
---

## 📜 License

This project is released under the MIT License. See [LICENSE](LICENSE) for details.
