# 🔌 PCB Synchronization Module

This directory contains all design and manufacturing files for reproducing the **PCB sync module** of the RAPID Hand.

📎 **Expected outcome**: A fully soldered and assembled PCB that interfaces with **5 tactile sensors** and a **camera**.

---

## 📁 Directory Structure

```txt
pcb_sync_module/
│
├─ README.md                                # This file
│
└─ pcb_sourcing/                            # Contains all PCB design and production files
    ├─ BOM_Board1_PCB1.xlsx                 # Bill of Materials for PCB components
    └─ SCH_Schematic1.pdf                   # Full schematic of the board
```

---

## 🧩 Connector Guide: Camera and Tactile Sensors

Based on the schematic and board layout:

- **Tactile Sensor Connections**:The PCB includes **five FPC connectors** (FPC5–FPC9), each labeled and routed through `PB6_SCL` and `PB7_SDA`, indicating shared I2C bus operation.
- **Camera Interface**:Located near the right edge, labeled as **Camera**, routed through pin `PB4`.

## ✅ Functionality Test After Assembly

Once all tactile sensors are connected and the PCB is configured:

1. Connect the PCB to your PC via **USB**.
2. Open a serial terminal (e.g., `minicom`, `screen`, `cutecom`).
3. Send a single byte: `0xFF`.
4. If successful, the MCU will respond with:
   - **5 consecutive data frames** — each frame consists of exactly  **96 bytes** , corresponding to one tactile sensor.
   - Each data frame is immediately followed by a **1-byte delimiter** (end marker), indicating the end of that frame.
5. Apply different levels of pressure to test taxel response.

This allows you to **validate sensor communication** and **confirm board-level functionality**.

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
