---

## ✅ 📋 **Objective:**

Calculate the **ideal wing dimensions** for a **glider drone**, based on its **total weight**.

---

## ✅ 1. **Essential Aerodynamics Formulas (Glider / Drone Wings)**

### 📌 **Lift** Formula:

$$
L = \frac{1}{2} \cdot \rho \cdot V^2 \cdot S \cdot C_L
$$

| Variable | Meaning                                                           |
| -------- | ----------------------------------------------------------------- |
| **L**    | Lift force (N) → equal to weight for level flight                 |
| **ρ**    | Air density (kg/m³) → approx. 1.225 kg/m³ at sea level            |
| **V**    | Flight speed (m/s)                                                |
| **S**    | Wing area (m²)                                                    |
| **Cₗ**   | Lift coefficient (depends on airfoil profile and angle of attack) |

---

### 📌 **Weight (Gravitational Force)** Formula:

$$
W = m \cdot g
$$

* \$m\$ = total mass of the drone (kg)
* \$g\$ = gravity acceleration (9.81 m/s²)

For stable gliding flight:

$$
L = W
$$

---

### 📌 **Wing Area (Solving the formula):**

$$
S = \frac{2 \cdot W}{\rho \cdot V^2 \cdot C_L}
$$

---

### ✅ 2. **Wing Dimensions Calculation**

Assuming simple rectangular wings (easiest shape to build):

$$
S = b \cdot c
$$

* \$b\$ = wingspan (tip-to-tip distance, in meters)
* \$c\$ = wing chord (width of the wing, in meters)

To define the ideal proportion, we use the aspect ratio \$AR\$:

$$
AR = \frac{b^2}{S}
$$

* Gliders have a **high AR** (8 to 20) → long and thin wings.
* You may choose, for example, **AR = 10**.

---

### 📋 Solving:

1. Calculate \$S\$ using the lift formula.
2. Determine the wingspan:

$$
b = \sqrt{AR \cdot S}
$$

3. Determine the chord:

$$
c = \frac{S}{b}
$$

---

## ✅ 3. **Complete Example**

* Total drone weight (mass): \$m = 3 , kg\$
* Desired flight speed: \$V = 10 , m/s\$
* Average lift coefficient: \$C\_L = 1.0\$ (typical airfoil)
* Aspect ratio: \$AR = 10\$

### Step 1 – Weight:

$$
W = 3 \cdot 9.81 = 29.43 \, N
$$

### Step 2 – Wing Area:

$$
S = \frac{2 \cdot 29.43}{1.225 \cdot 10^2 \cdot 1.0} = \frac{58.86}{122.5} \approx 0.48 \, m^2
$$

### Step 3 – Wingspan:

$$
b = \sqrt{10 \cdot 0.48} \approx \sqrt{4.8} \approx 2.19 \, m
$$

### Step 4 – Chord:

$$
c = \frac{0.48}{2.19} \approx 0.22 \, m
$$

---

### ✅ 🔍 Result:

* **Wing area:** 0.48 m²
* **Wingspan:** 2.19 m
* **Chord:** 0.22 m
* Long and thin wings → ideal for gliders.

---

## ✅ 4. **Summary**

* Use the lift formula to calculate the required wing area.
* Define the desired aspect ratio (usually between 8 and 20).
* Calculate the physical dimensions of the wing (wingspan and chord).

---

## ✅ How it works:

### Interface:

* **Fields**:

  * Drone mass (kg)
  * Flight speed (m/s)
  * Lift Coefficient (CL)
  * Aspect Ratio (AR)
* **Button**: Calculate
* **Result**: Displays wing area, wingspan, and chord

---

## ✅ Compatibility:

* **Windows 10**: Fully supported
* **Ubuntu Linux**: Use `python3-pyqt5` or `pip install pyqt5`
* **macOS**: Install via `pip3 install pyqt5`

---

✅ A **single and complete Qt script** was created, which includes:

### 📦 Features:

* Physics simulations: Force, Torque, Drag, RC Time, Energy, Efficiency
* PID simulator
* Glider drone wing simulator
* MPU6050 sensor readings (in real-time)
* Qt menu with individual windows
* Log writing and reading

---

### ▶️ To run:

1. Install dependencies:

   ```bash
   pip install pyqt5 matplotlib pyserial
   ```

---
