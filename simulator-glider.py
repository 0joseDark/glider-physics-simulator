# =========================================
# Script único: Simulador Completo de Robô, Drone e Planador
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

LOG_FILE = "simulacoes.log"

# === Funções de Log ===
def gravar_log(tipo, linha):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(LOG_FILE, "a") as f:
        f.write(f"[{timestamp}] {tipo}: {linha}\n")

def ler_logs(tipo, ultimos_n=10):
    if not os.path.exists(LOG_FILE):
        return []
    with open(LOG_FILE, "r") as f:
        linhas = f.readlines()
    filtradas = [linha.strip() for linha in linhas if tipo in linha]
    return filtradas[-ultimos_n:]

# === Simulador Físico ===
class SimuladorFisica(QWidget):
    def __init__(self, tipo):
        super().__init__()
        self.tipo = tipo
        self.setWindowTitle(f"Simulador: {tipo}")
        self.layout = QVBoxLayout()
        self.form = QFormLayout()
        self.inputs = {}

        self.campos = {
            "Força": ["Massa (kg)", "Aceleração (m/s²)"],
            "Torque": ["Força (N)", "Raio (m)"],
            "Resistência": ["Resistividade (ρ)", "Comprimento (m)", "Área (m²)"],
            "Tempo RC": ["Resistência (Ω)", "Capacitância (F)"],
            "Energia Cinética": ["Massa (kg)", "Velocidade (m/s)"],
            "Eficiência": ["Saída (W)", "Entrada (W)"]
        }

        for campo in self.campos[tipo]:
            entrada = QLineEdit()
            self.form.addRow(QLabel(campo), entrada)
            self.inputs[campo] = entrada

        self.resultado = QLabel("Resultado: ")
        self.botao = QPushButton("Calcular")
        self.botao.clicked.connect(self.calcular)

        self.botaoGrafico = QPushButton("Mostrar Gráfico")
        self.botaoGrafico.clicked.connect(self.mostrar_grafico)

        self.resultadosAntigos = QTextEdit()
        self.resultadosAntigos.setReadOnly(True)
        self.carregar_resultados()

        self.layout.addLayout(self.form)
        self.layout.addWidget(self.botao)
        self.layout.addWidget(self.botaoGrafico)
        self.layout.addWidget(self.resultado)
        self.layout.addWidget(QLabel("Últimos cálculos:"))
        self.layout.addWidget(self.resultadosAntigos)
        self.setLayout(self.layout)

    def calcular(self):
        try:
            if self.tipo == "Força":
                m = float(self.inputs["Massa (kg)"].text())
                a = float(self.inputs["Aceleração (m/s²)"].text())
                resultado = m * a
                formula = f"F = m*a = {m}*{a} = {resultado:.2f} N"

            elif self.tipo == "Torque":
                f = float(self.inputs["Força (N)"].text())
                r = float(self.inputs["Raio (m)"].text())
                resultado = f * r
                formula = f"τ = F*r = {f}*{r} = {resultado:.2f} N·m"

            elif self.tipo == "Resistência":
                ρ = float(self.inputs["Resistividade (ρ)"].text())
                l = float(self.inputs["Comprimento (m)"].text())
                A = float(self.inputs["Área (m²)"].text())
                resultado = ρ * l / A
                formula = f"R = ρ*l/A = {ρ}*{l}/{A} = {resultado:.6f} Ω"

            elif self.tipo == "Tempo RC":
                R = float(self.inputs["Resistência (Ω)"].text())
                C = float(self.inputs["Capacitância (F)"].text())
                resultado = R * C
                formula = f"t = R*C = {R}*{C} = {resultado:.3f} s"

            elif self.tipo == "Energia Cinética":
                m = float(self.inputs["Massa (kg)"].text())
                v = float(self.inputs["Velocidade (m/s)"].text())
                resultado = 0.5 * m * v**2
                formula = f"E = 0.5*m*v² = 0.5*{m}*{v}² = {resultado:.2f} J"

            elif self.tipo == "Eficiência":
                saida = float(self.inputs["Saída (W)"].text())
                entrada = float(self.inputs["Entrada (W)"].text())
                resultado = (saida / entrada) * 100
                formula = f"η = (Saída/Entrada)*100 = ({saida}/{entrada})*100 = {resultado:.2f}%"

            self.resultado.setText("Resultado: " + formula)
            gravar_log(self.tipo, formula)
            self.carregar_resultados()

        except Exception as e:
            QMessageBox.warning(self, "Erro", f"Erro no cálculo: {str(e)}")

    def mostrar_grafico(self):
        pass  # Simplificado neste script

    def carregar_resultados(self):
        ultimos = ler_logs(self.tipo, 10)
        self.resultadosAntigos.setText("\n".join(ultimos))

# === Simulador PID ===
class JanelaPID(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simulador PID")
        self.layout = QVBoxLayout()
        self.form = QFormLayout()
        self.inputs = {}
        for campo in ["Kp", "Ki", "Kd", "Erro Inicial", "Setpoint"]:
            entrada = QLineEdit()
            self.form.addRow(QLabel(campo), entrada)
            self.inputs[campo] = entrada
        self.botao = QPushButton("Simular")
        self.botao.clicked.connect(self.simular_pid)
        self.layout.addLayout(self.form)
        self.layout.addWidget(self.botao)
        self.setLayout(self.layout)

    def simular_pid(self):
        try:
            Kp = float(self.inputs["Kp"].text())
            Ki = float(self.inputs["Ki"].text())
            Kd = float(self.inputs["Kd"].text())
            erro = float(self.inputs["Erro Inicial"].text())
            setpoint = float(self.inputs["Setpoint"].text())
            dt = 1
            saida, integral = 0, 0
            tempos, saidas = list(range(50)), []
            for t in tempos:
                e = setpoint - saida
                integral += e * dt
                derivada = (e - erro) / dt
                saida = Kp * e + Ki * integral + Kd * derivada
                erro = e
                saidas.append(saida)
            plt.plot(tempos, saidas, label="Saída PID")
            plt.plot(tempos, [setpoint]*50, '--', label="Setpoint")
            plt.legend(); plt.grid(); plt.tight_layout(); plt.show()
        except Exception as e:
            QMessageBox.warning(self, "Erro", str(e))

# === Leitura Sensor MPU6050 ===
class SensorPlot(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Gráfico Sensor MPU6050")
        self.ax_data, self.ay_data, self.az_data = [], [], []
        self.canvas = Canvas(plt.figure())
        self.ax = self.canvas.figure.add_subplot(111)
        self.line_x, = self.ax.plot([], [], label='Ax')
        self.line_y, = self.ax.plot([], [], label='Ay')
        self.line_z, = self.ax.plot([], [], label='Az')
        self.ax.legend(); self.ax.set_ylim(-20000, 20000)
        layout = QVBoxLayout(); layout.addWidget(self.canvas); self.setLayout(layout)
        self.serial = serial.Serial('COM3', 9600)
        threading.Thread(target=self.ler_dados, daemon=True).start()
        self.timer = QTimer(); self.timer.timeout.connect(self.atualizar_grafico); self.timer.start(100)

    def ler_dados(self):
        while True:
            try:
                linha = self.serial.readline().decode().strip()
                ax, ay, az, *_ = map(int, linha.split(','))
                self.ax_data.append(ax); self.ay_data.append(ay); self.az_data.append(az)
                if len(self.ax_data) > 100:
                    self.ax_data.pop(0); self.ay_data.pop(0); self.az_data.pop(0)
            except: continue

    def atualizar_grafico(self):
        self.line_x.set_data(range(len(self.ax_data)), self.ax_data)
        self.line_y.set_data(range(len(self.ay_data)), self.ay_data)
        self.line_z.set_data(range(len(self.az_data)), self.az_data)
        self.ax.set_xlim(0, 100); self.canvas.draw()

# === Simulador Planador ===
class SimuladorPlanador(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Asas de Drone Planador")
        self.layout = QVBoxLayout(); self.form = QFormLayout(); self.inputs = {}
        campos = [("Massa do drone (kg)", "3.0"), ("Velocidade de voo (m/s)", "10.0"), ("Coef. sustentação (CL)", "1.0"), ("Razão de aspecto (AR)", "10")]
        for campo, valor in campos:
            entrada = QLineEdit(); entrada.setText(valor)
            self.form.addRow(QLabel(campo), entrada); self.inputs[campo] = entrada
        self.resultado = QLabel("Resultado:")
        self.botao = QPushButton("Calcular"); self.botao.clicked.connect(self.calcular)
        self.layout.addLayout(self.form); self.layout.addWidget(self.botao); self.layout.addWidget(self.resultado)
        self.setLayout(self.layout)

    def calcular(self):
        try:
            g, rho = 9.81, 1.225
            m = float(self.inputs["Massa do drone (kg)"].text())
            V = float(self.inputs["Velocidade de voo (m/s)"].text())
            CL = float(self.inputs["Coef. sustentação (CL)"].text())
            AR = float(self.inputs["Razão de aspecto (AR)"].text())
            W = m * g; S = (2 * W) / (rho * V**2 * CL)
            b = math.sqrt(AR * S); c = S / b
            texto = f"Área: {S:.2f} m²\nEnvergadura: {b:.2f} m\nCorda: {c:.2f} m"
            self.resultado.setText("Resultado:\n" + texto)
        except Exception as e:
            QMessageBox.warning(self, "Erro", str(e))

# === Janela Principal ===
class JanelaPrincipal(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simulações Drone, Robô e Planador")
        menu = self.menuBar()
        sim_menu = menu.addMenu("Simulações")
        sensores_menu = menu.addMenu("Sensores")
        for nome in ["Força", "Torque", "Resistência", "Tempo RC", "Energia Cinética", "Eficiência"]:
            acao = QAction(nome, self); acao.triggered.connect(lambda _, n=nome: self.abrir_simulador(n)); sim_menu.addAction(acao)
        sim_menu.addAction(QAction("Controlador PID", self, triggered=self.abrir_pid))
        sim_menu.addAction(QAction("Drone Planador", self, triggered=self.abrir_planador))
        sensores_menu.addAction(QAction("MPU6050", self, triggered=self.abrir_sensor))
        botao = QPushButton("Sair"); botao.clicked.connect(self.close); self.setCentralWidget(botao)

    def abrir_simulador(self, tipo):
        self.sim = SimuladorFisica(tipo); self.sim.show()
    def abrir_pid(self):
        self.pid = JanelaPID(); self.pid.show()
    def abrir_sensor(self):
        self.sensor = SensorPlot(); self.sensor.show()
    def abrir_planador(self):
        self.planador = SimuladorPlanador(); self.planador.show()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    janela = JanelaPrincipal()
    janela.resize(500, 200)
    janela.show()
    sys.exit(app.exec_())
