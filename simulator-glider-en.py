# =========================================
# Single Script: Complete Simulator for Robot, Drone, and Glider
# =========================================

import sys
import os
import math
import serial
import threading
from datetime import datetime
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QPushButton, QAction,
    QVBoxLayout, QLabel, QFormLayout, QLineEdit, QTextEdit, QMessageBox
)
from PyQt5.QtCore import QTimer
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as Canvas

LOG_FILE = "simulations.log"

# === Log Functions ===
def write_log(log_type, line):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(LOG_FILE, "a") as f:
        f.write(f"[{timestamp}] {log_type}: {line}\n")

def read_logs(log_type, last_n=10):
    if not os.path.exists(LOG_FILE):
        return []
    with open(LOG_FILE, "r") as f:
        lines = f.readlines()
    filtered = [line.strip() for line in lines if log_type in line]
    return filtered[-last_n:]

# === Physics Simulator ===
class PhysicsSimulator(QWidget):
    def __init__(self, sim_type):
        super().__init__()
        self.sim_type = sim_type
        self.setWindowTitle(f"Simulator: {sim_type}")
        self.layout = QVBoxLayout()
        self.form = QFormLayout()
        self.inputs = {}

        self.fields = {
            "Force": ["Mass (kg)", "Acceleration (m/s²)"],
            "Torque": ["Force (N)", "Radius (m)"],
            "Resistance": ["Resistivity (ρ)", "Length (m)", "Area (m²)"],
            "RC Time": ["Resistance (Ω)", "Capacitance (F)"],
            "Kinetic Energy": ["Mass (kg)", "Velocity (m/s)"],
            "Efficiency": ["Output (W)", "Input (W)"]
        }

        for field in self.fields[sim_type]:
            entry = QLineEdit()
            self.form.addRow(QLabel(field), entry)
            self.inputs[field] = entry

        self.result_label = QLabel("Result: ")
        self.calc_button = QPushButton("Calculate")
        self.calc_button.clicked.connect(self.calculate)

        self.graph_button = QPushButton("Show Graph")
        self.graph_button.clicked.connect(self.show_graph)

        self.old_results = QTextEdit()
        self.old_results.setReadOnly(True)
        self.load_results()

        self.layout.addLayout(self.form)
        self.layout.addWidget(self.calc_button)
        self.layout.addWidget(self.graph_button)
        self.layout.addWidget(self.result_label)
        self.layout.addWidget(QLabel("Last calculations:"))
        self.layout.addWidget(self.old_results)
        self.setLayout(self.layout)

    def calculate(self):
        try:
            if self.sim_type == "Force":
                m = float(self.inputs["Mass (kg)"].text())
                a = float(self.inputs["Acceleration (m/s²)"].text())
                result = m * a
                formula = f"F = m*a = {m}*{a} = {result:.2f} N"

            elif self.sim_type == "Torque":
                f = float(self.inputs["Force (N)"].text())
                r = float(self.inputs["Radius (m)"].text())
                result = f * r
                formula = f"τ = F*r = {f}*{r} = {result:.2f} N·m"

            elif self.sim_type == "Resistance":
                ρ = float(self.inputs["Resistivity (ρ)"].text())
                l = float(self.inputs["Length (m)"].text())
                A = float(self.inputs["Area (m²)"].text())
                result = ρ * l / A
                formula = f"R = ρ*l/A = {ρ}*{l}/{A} = {result:.6f} Ω"

            elif self.sim_type == "RC Time":
                R = float(self.inputs["Resistance (Ω)"].text())
                C = float(self.inputs["Capacitance (F)"].text())
                result = R * C
                formula = f"t = R*C = {R}*{C} = {result:.3f} s"

            elif self.sim_type == "Kinetic Energy":
                m = float(self.inputs["Mass (kg)"].text())
                v = float(self.inputs["Velocity (m/s)"].text())
                result = 0.5 * m * v**2
                formula = f"E = 0.5*m*v² = 0.5*{m}*{v}² = {result:.2f} J"

            elif self.sim_type == "Efficiency":
                output = float(self.inputs["Output (W)"].text())
                input_val = float(self.inputs["Input (W)"].text())
                result = (output / input_val) * 100
                formula = f"η = (Output/Input)*100 = ({output}/{input_val})*100 = {result:.2f}%"

            self.result_label.setText("Result: " + formula)
            write_log(self.sim_type, formula)
            self.load_results()

        except Exception as e:
            QMessageBox.warning(self, "Error", f"Calculation error: {str(e)}")

    def show_graph(self):
        pass  # Simplified in this script

    def load_results(self):
        last = read_logs(self.sim_type, 10)
        self.old_results.setText("\n".join(last))

# === PID Simulator ===
class PIDWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PID Simulator")
        self.layout = QVBoxLayout()
        self.form = QFormLayout()
        self.inputs = {}
        for field in ["Kp", "Ki", "Kd", "Initial Error", "Setpoint"]:
            entry = QLineEdit()
            self.form.addRow(QLabel(field), entry)
            self.inputs[field] = entry
        self.button = QPushButton("Simulate")
        self.button.clicked.connect(self.simulate_pid)
        self.layout.addLayout(self.form)
        self.layout.addWidget(self.button)
        self.setLayout(self.layout)

    def simulate_pid(self):
        try:
            Kp = float(self.inputs["Kp"].text())
            Ki = float(self.inputs["Ki"].text())
            Kd = float(self.inputs["Kd"].text())
            error = float(self.inputs["Initial Error"].text())
            setpoint = float(self.inputs["Setpoint"].text())
            dt = 1
            output, integral = 0, 0
            times, outputs = list(range(50)), []
            for t in times:
                e = setpoint - output
                integral += e * dt
                derivative = (e - error) / dt
                output = Kp * e + Ki * integral + Kd * derivative
                error = e
                outputs.append(output)
            plt.plot(times, outputs, label="PID Output")
            plt.plot(times, [setpoint]*50, '--', label="Setpoint")
            plt.legend(); plt.grid(); plt.tight_layout(); plt.show()
        except Exception as e:
            QMessageBox.warning(self, "Error", str(e))

# === MPU6050 Sensor Reading ===
class SensorPlot(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MPU6050 Sensor Graph")
        self.ax_data, self.ay_data, self.az_data = [], [], []
        self.canvas = Canvas(plt.figure())
        self.ax = self.canvas.figure.add_subplot(111)
        self.line_x, = self.ax.plot([], [], label='Ax')
        self.line_y, = self.ax.plot([], [], label='Ay')
        self.line_z, = self.ax.plot([], [], label='Az')
        self.ax.legend(); self.ax.set_ylim(-20000, 20000)
        layout = QVBoxLayout(); layout.addWidget(self.canvas); self.setLayout(layout)
        self.serial = serial.Serial('COM3', 9600)
        threading.Thread(target=self.read_data, daemon=True).start()
        self.timer = QTimer(); self.timer.timeout.connect(self.update_graph); self.timer.start(100)

    def read_data(self):
        while True:
            try:
                line = self.serial.readline().decode().strip()
                ax, ay, az, *_ = map(int, line.split(','))
                self.ax_data.append(ax); self.ay_data.append(ay); self.az_data.append(az)
                if len(self.ax_data) > 100:
                    self.ax_data.pop(0); self.ay_data.pop(0); self.az_data.pop(0)
            except: continue

    def update_graph(self):
        self.line_x.set_data(range(len(self.ax_data)), self.ax_data)
        self.line_y.set_data(range(len(self.ay_data)), self.ay_data)
        self.line_z.set_data(range(len(self.az_data)), self.az_data)
        self.ax.set_xlim(0, 100); self.canvas.draw()

# === Glider Wing Simulator ===
class GliderSimulator(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Glider Drone Wing Dimensions")
        self.layout = QVBoxLayout(); self.form = QFormLayout(); self.inputs = {}
        fields = [("Drone mass (kg)", "3.0"), ("Flight speed (m/s)", "10.0"),
                  ("Lift coefficient (CL)", "1.0"), ("Aspect ratio (AR)", "10")]
        for field, value in fields:
            entry = QLineEdit(); entry.setText(value)
            self.form.addRow(QLabel(field), entry); self.inputs[field] = entry
        self.result_label = QLabel("Result:")
        self.button = QPushButton("Calculate"); self.button.clicked.connect(self.calculate)
        self.layout.addLayout(self.form); self.layout.addWidget(self.button); self.layout.addWidget(self.result_label)
        self.setLayout(self.layout)

    def calculate(self):
        try:
            g, rho = 9.81, 1.225
            m = float(self.inputs["Drone mass (kg)"].text())
            V = float(self.inputs["Flight speed (m/s)"].text())
            CL = float(self.inputs["Lift coefficient (CL)"].text())
            AR = float(self.inputs["Aspect ratio (AR)"].text())
            W = m * g; S = (2 * W) / (rho * V**2 * CL)
            b = math.sqrt(AR * S); c = S / b
            text = f"Area: {S:.2f} m²\nWingspan: {b:.2f} m\nChord: {c:.2f} m"
            self.result_label.setText("Result:\n" + text)
        except Exception as e:
            QMessageBox.warning(self, "Error", str(e))

# === Main Window ===
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simulations for Drone, Robot, and Glider")
        menu = self.menuBar()
        sim_menu = menu.addMenu("Simulations")
        sensors_menu = menu.addMenu("Sensors")
        for name in ["Force", "Torque", "Resistance", "RC Time", "Kinetic Energy", "Efficiency"]:
            action = QAction(name, self); action.triggered.connect(lambda _, n=name: self.open_simulator(n)); sim_menu.addAction(action)
        sim_menu.addAction(QAction("PID Controller", self, triggered=self.open_pid))
        sim_menu.addAction(QAction("Glider Drone", self, triggered=self.open_glider))
        sensors_menu.addAction(QAction("MPU6050", self, triggered=self.open_sensor))
        exit_button = QPushButton("Exit"); exit_button.clicked.connect(self.close); self.setCentralWidget(exit_button)

    def open_simulator(self, sim_type):
        self.sim = PhysicsSimulator(sim_type); self.sim.show()
    def open_pid(self):
        self.pid = PIDWindow(); self.pid.show()
    def open_sensor(self):
        self.sensor = SensorPlot(); self.sensor.show()
    def open_glider(self):
        self.glider = GliderSimulator(); self.glider.show()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.resize(500, 200)
    window.show()
    sys.exit(app.exec_())
