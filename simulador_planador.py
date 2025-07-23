# =========================================
# Ficheiro: simulador_planador.py
# =========================================

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QFormLayout, QLineEdit, QPushButton, QMessageBox
import math

class SimuladorPlanador(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simulador de Asas para Drone Planador")
        self.layout = QVBoxLayout()
        self.form = QFormLayout()

        self.inputs = {}
        campos = [
            ("Massa do drone (kg)", "3.0"),
            ("Velocidade de voo (m/s)", "10.0"),
            ("Coef. sustentação (CL)", "1.0"),
            ("Razão de aspecto (AR)", "10")
        ]

        for campo, valor_default in campos:
            entrada = QLineEdit()
            entrada.setText(valor_default)
            self.form.addRow(QLabel(campo), entrada)
            self.inputs[campo] = entrada

        self.resultado = QLabel("Resultado:")
        self.botao = QPushButton("Calcular")
        self.botao.clicked.connect(self.calcular)

        self.layout.addLayout(self.form)
        self.layout.addWidget(self.botao)
        self.layout.addWidget(self.resultado)
        self.setLayout(self.layout)

    def calcular(self):
        try:
            g = 9.81  # gravidade
            rho = 1.225  # densidade do ar

            m = float(self.inputs["Massa do drone (kg)"].text())
            V = float(self.inputs["Velocidade de voo (m/s)"].text())
            CL = float(self.inputs["Coef. sustentação (CL)"].text())
            AR = float(self.inputs["Razão de aspecto (AR)"].text())

            W = m * g  # peso
            S = (2 * W) / (rho * V ** 2 * CL)  # área da asa
            b = math.sqrt(AR * S)  # envergadura
            c = S / b  # corda

            texto = (
                f"Área da asa: {S:.3f} m²\n"
                f"Envergadura: {b:.3f} m\n"
                f"Corda (largura): {c:.3f} m"
            )
            self.resultado.setText("Resultado:\n" + texto)

        except Exception as e:
            QMessageBox.warning(self, "Erro", f"Erro no cálculo: {str(e)}")
