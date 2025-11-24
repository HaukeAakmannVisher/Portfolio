# Portfolio

Short selection of engineering projects with software + hardware.

> If GitHub ever fails to preview a PDF, use the **Direct download (raw)** link.

---

## 📄 Portfolio PDF

- **View in GitHub:**  
  [`Hauke Aakmann-Visher Portfolio.pdf`](Hauke%20Aakmann-Visher%20Portfolio.pdf)

- **Direct download (raw):**  
  <https://raw.githubusercontent.com/HaukeAakmannVisher/Portfolio/main/Hauke%20Aakmann-Visher%20Portfolio.pdf>

---

## 📦 Projects

### 1) Warehouse Robot Guidance System
- **Folder:**  
  <https://github.com/HaukeAakmannVisher/Portfolio/tree/main/Warehouse%20Robot%20Guidance%20System>
- **Full report (raw):**  
  <https://raw.githubusercontent.com/HaukeAakmannVisher/Portfolio/main/Warehouse%20Robot%20Guidance%20System/warehouse-robot-guidance-system-full-report.pdf>
- **Laptop code (planner + web UI):**  
  <https://github.com/HaukeAakmannVisher/Portfolio/tree/main/Warehouse%20Robot%20Guidance%20System/laptop>
- **Pi code (executor + sensors):**  
  <https://github.com/HaukeAakmannVisher/Portfolio/tree/main/Warehouse%20Robot%20Guidance%20System/pi>  
  • Compensation logic in `AlphaBot2.py`:  
  <https://github.com/HaukeAakmannVisher/Portfolio/blob/main/Warehouse%20Robot%20Guidance%20System/pi/AlphaBot2.py>

> **Hardware note:** Pi code requires the **AlphaBot2** platform (motors/driver, TRSensors, ultrasonic) on a Raspberry Pi with GPIO access.

---

### 2) PV System Efficiency Monitor
- **Folder:**  
  <https://github.com/HaukeAakmannVisher/Portfolio/tree/main/PV%20Efficiency%20Monitor>
- **Full report (view / raw):**  
  View: <https://github.com/HaukeAakmannVisher/Portfolio/blob/main/PV%20Efficiency%20Monitor/pv-efficiency-monitor-full-report.pdf>  
  Raw:  <https://raw.githubusercontent.com/HaukeAakmannVisher/Portfolio/main/PV%20Efficiency%20Monitor/pv-efficiency-monitor-full-report.pdf>
- **STM32 source (key files):**  
  <https://github.com/HaukeAakmannVisher/Portfolio/tree/main/PV%20Efficiency%20Monitor/code>  
  • `main.c` – measurement flow, SP routine, calculations, calibration hooks  
  • `lcd16x2.c/.h` – LCD UI helpers  
  • plus minimal HAL/system stubs used in the project

> **Note:** this isn’t a full CubeIDE project; only the **most relevant** source files are included to keep the repo compact.

---

## 🗂 Repo structure (high level)

├─ Hauke Aakmann-Visher Portfolio.pdf
├─ Warehouse Robot Guidance System/
│  ├─ warehouse-robot-guidance-system-full-report.pdf
│  ├─ laptop/
│  └─ pi/
└─ PV Efficiency Monitor/
   ├─ pv-efficiency-monitor-full-report.pdf
   ├─ code/


---

## ✍️ Author

Hauke Aakmann-Visher  
LinkedIn: <https://www.linkedin.com/in/hauke-aakmann-visher-064064342>  
GitHub: <https://github.com/HaukeAakmannVisher>



