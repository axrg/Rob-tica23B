import cv2
import numpy as np
import serial
import time

# Inicializar la comunicación serial con el ESP32
ser = serial.Serial('COMX', 9600)  # Reemplaza 'COMX' con el puerto serial correcto del ESP32

# Función para detectar las figuras geométricas
def detect_shapes(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, threshold = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
        x = approx.ravel()[0]
        y = approx.ravel()[1] - 10

        if len(approx) == 3:
            cv2.drawContours(frame, [approx], 0, (0, 0, 255), 5)  # Triángulo rojo
            send_coordinates(x, y, "Triángulo Rojo")
        elif len(approx) == 4:
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h
            if 0.95 <= aspect_ratio <= 1.05:
                cv2.drawContours(frame, [approx], 0, (255, 0, 0), 5)  # Cuadrado azul
                send_coordinates(x, y, "Cuadrado Azul")
            else:
                cv2.drawContours(frame, [approx], 0, (0, 0, 255), 5)  # Rectángulo azul
                send_coordinates(x, y, "Rectángulo Azul")
        else:
            cv2.circle(frame, (x, y), 6, (0, 0, 255), -1)  # Círculo rojo
            send_coordinates(x, y, "Círculo Rojo")

    return frame

# Función para enviar coordenadas por comunicación serial
def send_coordinates(x, y, shape):
    data = f"{shape}: X={x}, Y={y}\n"
    ser.write(data.encode())
    time.sleep(0.1)  # Esperar un momento para evitar problemas de buffer

# Capturar video desde la cámara
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()

    if not ret:
        break

    frame = detect_shapes(frame)
    cv2.imshow('Figuras Geométricas', frame)

    # Presiona 'q' para salir del bucle
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar la cámara y cerrar las ventanas
cap.release()
cv2.destroyAllWindows()
