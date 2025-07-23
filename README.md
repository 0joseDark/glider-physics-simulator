## ✅ 📋 **Objetivo:**

Calcular as **dimensões ideais das asas** de um **drone planador**, com base no seu **peso total**.

---

## ✅ 1. **Fórmulas Essenciais da Aerodinâmica (Asas de Planadores / Drones)**

### 📌 Fórmula do **Sustentação**:

$$
L = \frac{1}{2} \cdot \rho \cdot V^2 \cdot S \cdot C_L
$$

| Variável | Significado                                                    |
| -------- | -------------------------------------------------------------- |
| **L**    | Força de sustentação (N) → igual ao peso para voo nivelado     |
| **ρ**    | Densidade do ar (kg/m³) → aprox. 1.225 kg/m³ ao nível do mar   |
| **V**    | Velocidade de voo (m/s)                                        |
| **S**    | Área da asa (m²)                                               |
| **Cₗ**   | Coeficiente de sustentação (depende do perfil da asa e ângulo) |

---

### 📌 Fórmula do **Peso (Força Gravitacional)**:

$$
W = m \cdot g
$$

* $m$ = massa total do drone (kg)
* $g$ = aceleração da gravidade (9.81 m/s²)

Para o drone planar em voo estável:

$$
L = W
$$

---

### 📌 **Área da Asa (Resolvendo a fórmula):**

$$
S = \frac{2 \cdot W}{\rho \cdot V^2 \cdot C_L}
$$

---

### ✅ 2. **Cálculo das Dimensões da Asa**

Assumindo asas retangulares simples (forma mais fácil de construir):

$$
S = b \cdot c
$$

* $b$ = envergadura (distância de ponta a ponta da asa, em m)
* $c$ = corda da asa (largura da asa, em m)

Para definir a proporção ideal, usamos a razão de aspecto $AR$:

$$
AR = \frac{b^2}{S}
$$

* Planadores têm **AR** alto (8 a 20) → asas longas e finas.
* Pode escolher, por exemplo, **AR = 10**.

---

### 📋 Resolvendo:

1. Calcula $S$ com a fórmula da sustentação.
2. Determina a envergadura:

$$
b = \sqrt{AR \cdot S}
$$

3. Determina a corda:

$$
c = \frac{S}{b}
$$

---

## ✅ 3. **Exemplo Completo**

* Peso total do drone (massa): $m = 3 \, kg$
* Velocidade de voo desejada: $V = 10 \, m/s$
* Coeficiente de sustentação médio: $C_L = 1.0$ (perfil típico)
* Razão de aspecto: $AR = 10$

### Passo 1 – Peso:

$$
W = 3 \cdot 9.81 = 29.43 \, N
$$

### Passo 2 – Área da Asa:

$$
S = \frac{2 \cdot 29.43}{1.225 \cdot 10^2 \cdot 1.0} = \frac{58.86}{122.5} \approx 0.48 \, m^2
$$

### Passo 3 – Envergadura:

$$
b = \sqrt{10 \cdot 0.48} \approx \sqrt{4.8} \approx 2.19 \, m
$$

### Passo 4 – Corda:

$$
c = \frac{0.48}{2.19} \approx 0.22 \, m
$$

---

### ✅ 🔍 Resultado:

* **Área da asa:** 0.48 m²
* **Envergadura:** 2.19 m
* **Corda:** 0.22 m
* Asas alongadas e finas → ideal para planadores.

---

## ✅ 4. **Resumo**

* Usa a fórmula de sustentação para calcular a área necessária da asa.
* Define o aspecto desejado (normalmente entre 8 e 20).
* Calcula as dimensões físicas da asa (envergadura e corda).

---
O ficheiro `simulador_planador.py` é um **simulador em Qt para Windows 10, Ubuntu e Mac**, que calcula as dimensões ideais das asas (área, envergadura, corda) de um drone planador com base no peso total.

---

## ✅ Como funciona:

### Interface:

* **Campos**:

  * Massa do drone (kg)
  * Velocidade de voo (m/s)
  * Coeficiente de Sustentação (CL)
  * Razão de Aspecto (AR)
* **Botão**: Calcular
* **Resultado**: Exibe área da asa, envergadura e corda

---

## ✅ Compatibilidade:

* **Windows 10**: Totalmente suportado
* **Ubuntu Linux**: Usa `python3-pyqt5` ou `pip install pyqt5`
* **macOS**: Instala via `pip3 install pyqt5`

---
✅ Foi criado um **script único e completo em Qt** que inclui:

### 📦 Funcionalidades:

* Simulações físicas: Força, Torque, Resistência, Tempo RC, Energia, Eficiência
* Simulador PID
* Simulador de asas de drone planador
* Leitura de sensores MPU6050 (em tempo real)
* Menu Qt com janelas individuais
* Gravação e leitura de logs

---

### ▶️ Para executar:

1. Instala dependências:

   ```bash
   pip install pyqt5 matplotlib pyserial
   ```



