# 🦾 Mechanical Structure Assembly Guide

This directory contains all the necessary resources for building the **mechanical structure of the RAPID hand**, including 3D models, BOM (Bill of Materials), and detailed assembly instructions.

📌 **Goal**: Assemble the full RAPID hand structure and mount the camera module.

---

## 📁 Directory Structure

```txt
mechanical_structure/
│
├─ BOM_and_Assembly_Guide.pdf      # Main reference for mechanical BOM & visual instructions
├─ calibration_board.STL           # STL file for the calibration_board
├─ README.md                       # This file
│
├─ rapidhand_description/          # ROS description package: URDF model, meshes, config & launch files
│
├─ 3D Printed Parts/               # STL files for all 3D printed mechanical parts
│   └─ F1.STL ~ T12.STL            # Fingers, thumb, palm, etc.
│
└─ Model/                          # STEP files for CAD and visualization
    └─ Finger.STEP
        thumb.STEP
        rapidhand.STEP
```

---

## 🧩 Bill of Materials (BOM)

The mechanical assembly consists of three main subassemblies:

### 🔹 1. Finger Module (×4)

| Category              | Name                   | Quantity             |
| --------------------- | ---------------------- | -------------------- |
| Electronic Devices    | Motor                  | 4                    |
|                       | Tactile Sensor         | 1                    |
| 3D Printed Parts      | F1–F9, G1–G5, P1–P2 | 1 each (G1,G2,P1×2) |
| Mechanical Components | Screw M2×4            | 20                   |
|                       | Screw M2×10           | 16                   |
|                       | Screw M2.5×12         | 6                    |
|                       | Screw M3×6            | 3                    |
|                       | Bearing 5×8×2.5      | 4                    |

---

### 🔹 2. Thumb Module (×1)

| Category              | Name                    | Quantity             |
| --------------------- | ----------------------- | -------------------- |
| Electronic Devices    | Motor                   | 4                    |
|                       | Tactile Sensor          | 1                    |
| 3D Printed Parts      | T1–T12, G1–G3, P1–P2 | 1 each (G1,G2,P1×2) |
| Mechanical Components | Screw M2×4             | 16                   |
|                       | Screw M2×10            | 16                   |
|                       | Screw M2.5×12          | 8                    |
|                       | Screw M3×6             | 3                    |
|                       | Bearing 5×8×2.5       | 2                    |
|                       | Bearing 15×24×5       | 1                    |

---

### 🔹 3. Palm & Integration

| Category              | Name           | Quantity          |
| --------------------- | -------------- | ----------------- |
| Electronic Devices    | Camera         | 1                 |
| 3D Printed Parts      | H1–H8         | 1 each (H4,H6×2) |
| Mechanical Components | Screw M2.5×10 | 50                |
|                       | Screw M4×12   | 6                 |
|                       | Screw M3×6    | 2                 |
| Subassemblies         | Finger         | 4                 |
|                       | Thumb          | 1                 |

---

## 🔧 Assembly Instructions

1. 📘 **Refer to `BOM_and_Assembly_Guide.pdf`**

   - This document provides exploded views and labeled diagrams for each module: Finger, Thumb, and Palm.
2. 🧱 **3D Print Components**

   - Print all `.STL` files from `3D Printed Parts/`. Make sure the dimensions are accurate and tolerances are respected.
3. 🆔 **Configure Dynamixel Servo IDs**

   Before assembling motors into the hand, assign each Dynamixel servo a **unique ID**:

   - Connect a **single** motor to the U2D2 controller and the computer via USB.
   - Open [**Dynamixel Wizard 2.0**](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/).
   - In **Option → Preferences**, set:
     - **Protocol:** 2.0
     - **Baudrate:** Select **all available baudrates** for initial detection
     - **Port:** Your serial device (e.g., ttyUSB0 or COMx)
   - Click `Scan` to find connected motors.
   - Assign a new unique ID using the recommended scheme:

   <table>
      <tr>
        <th style="width: 105px; text-align: center;">Finger</th>
        <th style="width: 105px; text-align: center;">MCP-Side</th>
        <th style="width: 105px; text-align: center;">MCP-Forward</th>
        <th style="width: 105px; text-align: center;">PIP</th>
        <th style="width: 105px; text-align: center;">DIP</th>
      </tr>
      <tr>
        <td style="text-align: center;">Index</td>
        <td style="text-align: center;">0</td>
        <td style="text-align: center;">1</td>
        <td style="text-align: center;">2</td>
        <td style="text-align: center;">3</td>
      </tr>
      <tr>
        <td style="text-align: center;">Middle</td>
        <td style="text-align: center;">4</td>
        <td style="text-align: center;">5</td>
        <td style="text-align: center;">6</td>
        <td style="text-align: center;">7</td>
      </tr>
      <tr>
        <td style="text-align: center;">Ring</td>
        <td style="text-align: center;">8</td>
        <td style="text-align: center;">9</td>
        <td style="text-align: center;">10</td>
        <td style="text-align: center;">11</td>
      </tr>
      <tr>
        <td style="text-align: center;">Little</td>
        <td style="text-align: center;">12</td>
        <td style="text-align: center;">13</td>
        <td style="text-align: center;">14</td>
        <td style="text-align: center;">15</td>
      </tr>
      <tr>
        <td style="text-align: center;">Thumb</td>
        <td style="text-align: center;">16</td>
        <td style="text-align: center;">17</td>
        <td style="text-align: center;">18</td>
        <td style="text-align: center;">19</td>
      </tr>
    </table>

   - Also set each motor’s **Bus Baud Rate** to **4,000,000 bps (4Mbps)**.
   - Repeat for all 20 motors before mechanical installation.
4. 🪛 **Assemble Fingers & Thumb**

   - Use the visual guides to install motors, gears, sensors, and linkages.
   - Follow the screw size and placement as annotated.
   - Ensure bearings and gear alignments are precise to avoid mechanical jamming.
5. 🖐️ **Assemble Palm & Integrate Modules**

   - Mount the assembled fingers and thumb onto the palm base (`H1–H8`).
   - Fix the camera unit as shown in the diagrams.
   - Tighten using the correct screw lengths to avoid cracking plastic parts.
6. 🧩 **Fit & Finalize**

   - Once all modules are fixed to the palm, verify smooth motion.
   - Optionally, adjust with lubricant or sanding for tight joints.

<!-- 7. 🎯 **Calibrate Zero Position**

   - Ensure all mechanical installation is complete.
   - Mount the **calibration fixture** and place all fingers in their intended **zero (neutral) pose**.
   - Run:

     ```bash
     python -m utils.motor_init
     ```

   - Verify with:

     ```bash
     python -m utils.motor_init --test
     ```

     This sets all joint angles to zero. Visually confirm proper alignment.

Once all motors have been installed and zeroed, run the following command to test the motor configuration:

```shell
python -m utils.motion_sequence_controller
```

If the robotic hand can perform preset gestures such as finger curling without any collisions or unnatural motion, the assembly and motor configuration are correct. -->

---

## ⚠️ Notes & Precautions

- Ensure all 3D printed parts are clean of support material before assembly.
- Pay close attention to screw lengths—using the wrong length can strip the plastic or block movement.
- Tolerances may vary by printer; minor manual adjustments (filing, drilling) may be necessary.
- Avoid overtightening screws around motors and sensors to prevent damage.
- Validate fit and mechanical movement before powering on the system.

---

## 🔗 Citation

```bibtex
@article{wan2025rapid,
  title={RAPID Hand: A Robust, Affordable, Perception-Integrated, Dexterous Manipulation Platform for Generalist Robot Autonomy},
  author={Wan, Zhaoliang and Bi, Zetong and Zhou, Zida and Ren, Hao and Zeng, Yiming and Li, Yihan and Qi, Lu and Yang, Xu and Yang, Ming-Hsuan and Cheng, Hui},
  journal={arXiv preprint arXiv:2506.07490},
  year={2025}
}
```
