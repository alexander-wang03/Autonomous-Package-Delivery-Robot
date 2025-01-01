<h1 align="center">Autonomous Package Delivery Robot</h1>

<p align="center">
    <a href="https://www.youtube.com/watch?v=i5FRlQ0tfMQ">
        <img src="https://img.shields.io/badge/YouTube-red?style=flat-square&logo=youtube&logoColor=white" alt="YouTube" align="center" />
    </a>
</p>

<p align="center"><img width=50% alt="" src="/packagebot.jpg" /></p>

An autonomous mail delivery system built on the **TurtleBot 3 Waffle Pi**, combining **Bayesian Localization**, **PID control**, and **ROS** for precise navigation and reliable delivery in a simulated hallway environment.

---

## Features

1. **Line Following**:
   - Implemented using a finely-tuned PID controller to ensure smooth and precise navigation along white tape paths.
   - Dynamically adjusts angular velocity to minimize overshoot and improve tracking accuracy.

2. **Office Recognition**:
   - Detects delivery locations using HSV color thresholds.
   - Adapted to handle challenging lighting conditions by distinguishing colors based on hue and saturation.

3. **Bayesian Localization**:
   - Employs a `BayesLoc` class to predict the robot's state and update probabilities based on observed colors.
   - Achieves convergence on the robot's position with >70% confidence.

4. **Mail Delivery**:
   - Automates delivery maneuvers, including stopping, 90-degree turns, and simulated package drops.
   - Executes actions only when localization confidence exceeds a threshold.

5. **Real-Time Control**:
   - Utilizes ROS for seamless integration of hardware and software, including sensor processing and motor control.

## System Architecture

### Hardware
- **Platform**: TurtleBot 3 Waffle Pi
  - **Microcontroller**: OpenCR1.0 (ARM Cortex-M7)
  - **High-Level Processor**: Raspberry Pi 3
  - **Motors**: Dynamixel XM430-W210 actuators
- **Sensors**:
  - Raspberry Pi Camera Module v2.1 (for line detection and office recognition)

### Software
- **Programming Languages**: Python (main logic), ROS (middleware)
- **Control Algorithm**: PID controller
- **Localization Algorithm**: Bayesian Localization

## Documentation

- Full report: [Project Documentation](./report.pdf)
- Demo video: [YouTube Link](https://www.youtube.com/watch?v=i5FRlQ0tfMQ)

---

## License

This project is licensed under the [MIT License](LICENSE).
