import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import TextBox, Button
from collections import deque

# =========================
# SERIAL
# =========================
PORTA_SERIAL = 'COM3'
BAUD_RATE = 115200
TIMEOUT = 1

ser = serial.Serial(PORTA_SERIAL, BAUD_RATE, timeout=TIMEOUT)

# =========================
# BUFFERS
# =========================
max_len = 1000
tempo = deque(maxlen=max_len)
data_1 = deque(maxlen=max_len)
data_2 = deque(maxlen=max_len)
data_3 = deque(maxlen=max_len)

contador = 0

# =========================
# ENVIO DE COMANDO
# =========================
def enviar_cmd(event=None):
    cmd = text_box.text.strip()
    if cmd:
        try:
            ser.write((cmd + '\n').encode())
            print("Enviado:", cmd)
            text_box.set_val('')
        except Exception as e:
            print("Erro envio:", e)

# =========================
# UPDATE DO PLOT
# =========================
def update_plot(frame):
    global contador

    while ser.in_waiting:
        try:
            linha = ser.readline().decode(errors='ignore').strip()
            partes = linha.split('\t')

            if len(partes) == 4:
                _, y1, y2, y3 = map(float, partes)

                tempo.append(contador)
                data_1.append(y1)
                data_2.append(y2)
                data_3.append(y3)

                contador += 1

        except Exception as e:
            print("Erro leitura:", e)

    linha_d1.set_data(tempo, data_1)
    linha_d2.set_data(tempo, data_2)
    linha_d3.set_data(tempo, data_3)

    ax.relim()
    ax.autoscale_view()

    return linha_d1, linha_d2, linha_d3

# =========================
# PLOT
# =========================
fig, ax = plt.subplots(figsize=(10, 6))
plt.subplots_adjust(bottom=0.25)

ax.set_title('Monitor Serial')
ax.set_xlabel('Tempo (amostras)')
ax.set_ylabel('Valor')

linha_d1, = ax.plot([], [], label='P1 [cmH2O]')
linha_d2, = ax.plot([], [], label='P2 [cmH2O]')
linha_d3, = ax.plot([], [], label='P2 [cmH2O]')

ax.legend()
ax.grid()

# =========================
# UI (COMANDO)
# =========================
axbox = plt.axes([0.1, 0.1, 0.6, 0.06])
text_box = TextBox(axbox, 'Comando:')

axbtn = plt.axes([0.75, 0.1, 0.15, 0.06])
btn = Button(axbtn, 'Enviar')

btn.on_clicked(enviar_cmd)
text_box.on_submit(enviar_cmd)

# =========================
# ANIMAÇÃO
# =========================
ani = animation.FuncAnimation(fig, update_plot, interval=50)

plt.show()