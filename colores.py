import cv2
import numpy as np
import serial

# Inicializar el puerto serial
COM = 'com3'
BAUD = 9600
ser = serial.Serial(COM, BAUD)

# Solicitar al usuario que elija el color
color_elegido = input("Por favor, elija un color (rojo/azul/verde/amarillo): ").lower()

while color_elegido not in ['rojo', 'azul', 'verde', 'amarillo']:
    color_elegido = input("Color no válido. Por favor, elija rojo, azul, verde o amarillo: ").lower()

# Definir los rangos de color según la elección del usuario
if color_elegido == 'rojo':
    color_bajo1 = np.array([0, 100, 20], np.uint8)
    color_alto1 = np.array([10, 255, 255], np.uint8)
    color_bajo2 = np.array([160, 100, 20], np.uint8)
    color_alto2 = np.array([180, 255, 255], np.uint8)
elif color_elegido == 'azul':
    color_bajo = np.array([90, 100, 20], np.uint8)
    color_alto = np.array([120, 255, 255], np.uint8)
elif color_elegido == 'verde':
    color_bajo = np.array([35, 100, 20], np.uint8)
    color_alto = np.array([85, 255, 255], np.uint8)
elif color_elegido == 'amarillo':
    color_bajo = np.array([15, 100, 20], np.uint8)
    color_alto = np.array([35, 255, 255], np.uint8)

# Inicializar la cámara
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if ret:
        frame = cv2.flip(frame, 1)
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Determinar la máscara según la elección del usuario
        if color_elegido == 'rojo':
            mask1 = cv2.inRange(frame_hsv, color_bajo1, color_alto1)
            mask2 = cv2.inRange(frame_hsv, color_bajo2, color_alto2)
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(frame_hsv, color_bajo, color_alto)

        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame, contornos, -1, (255, 0, 0), 4)

        for c in contornos:
            area = cv2.contourArea(c)
            if area > 6000:
                M = cv2.moments(c)
                if M["m00"] == 0:
                    M["m00"] = 1
                x = int(M["m10"] / M["m00"])
                y = int(M['m01'] / M['m00'])
                cv2.circle(frame, (x, y), 7, (0, 0, 255), -1)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(frame, '{},{}'.format(x, y), (x + 10, y), font, 1.2, (0, 0, 255), 2, cv2.LINE_AA)

                # Control del dispositivo a través del puerto serial en función de la posición del objeto rojo
                if x < 200:
                    print("Mover a la izquierda 100%")
                    ser.write(b"izq1\n")
                elif 420 > x >= 200:
                    print("Mover a la izquierda 60%")
                    ser.write(b"izq2\n")
                elif 520 > x >= 420:
                    print("Mover a la izquierda 30%")
                    ser.write(b"izq3\n")
                # Mover al centro
                elif 520 <= x < 650:
                    print("Mover al centro")
                    ser.write(b"ctr\n")
                elif 650 <= x < 860:
                    print("Moviendo a la derecha 30%")
                    ser.write(b"der3\n")
                elif 860 <= x < 1080:
                    print("Moviendo a la derecha 60%")
                    ser.write(b"der2\n")
                elif x >= 1080:
                    print("Moviendo a la derecha 100%")
                    ser.write(b"der1\n")
        # Resto del código para controlar el dispositivo según la posición del objeto detectado

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('s'):
            ser.close()
            break

cap.release()
cv2.destroyAllWindows()
