import cv2
import numpy as np
import serial
import time


# Inicializar la cámara
# El servo de abajo va en la posición 10

# Inicializar la cámara
cap = cv2.VideoCapture(0)

# Inicializar la conexión con el puerto serie
ser = serial.Serial('COM3', 9600)  # Ajusta el nombre del puerto y la velocidad según tu configuración

# Inicializar variables de tiempo para seguimiento
start_time_red = None
start_time_blue = None
max_time = 15  # 5 segundos

while True:
    _, frame = cap.read()

    # Convertir a espacio de color HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Definir rangos de color para las figuras
    lower_red = np.array([160, 50, 50])
    upper_red = np.array([180, 255, 255])
    lower_blue = np.array([90, 50, 50])
    upper_blue = np.array([130, 255, 255])

    # Crear máscaras para cada color
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # Encontrar contornos en las máscaras
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Restablecer temporizadores si se detecta una nueva figura
    if contours_red and start_time_red is None:
        start_time_red = time.time()
    elif not contours_red:
        start_time_red = None

    if contours_blue and start_time_blue is None:
        start_time_blue = time.time()
    elif not contours_blue:
        start_time_blue = None

    # Procesar contornos rojos
    if contours_red:
        # Encontrar el contorno más grande
        largest_contour_red = max(contours_red, key=cv2.contourArea)
        area_red = cv2.contourArea(largest_contour_red)

        if area_red > 100:  # Se ha añadido el área mínima aquí también
            # Calcular el centroide de la figura
            M = cv2.moments(largest_contour_red)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0

            # Identificar la figura por su número de vértices
            epsilon = 0.04 * cv2.arcLength(largest_contour_red, True)
            approx = cv2.approxPolyDP(largest_contour_red, epsilon, True)

            if len(approx) == 3:
                shape = "triangulo"
            else:
                shape = "circulo"

            # Dibujar el centroide en la imagen
            cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)

            # Dibujar la figura y el nombre en la imagen
            cv2.drawContours(frame, [largest_contour_red], -1, (0, 255, 0), 2)
            cv2.putText(frame, f"{shape} rojo", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Imprimir las coordenadas debajo del tipo de figura
            cv2.putText(frame, f"Coordenadas: X={cX}, Y={cY}", (cX - 20, cY + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 2)

            # Enviar comandos al brazo robótico
            if cX != 0 and cY != 0:
                # Convertir coordenadas a comandos para el brazo robótico
                # Supongamos que el rango del brazo es de 0 a 180 grados en ambos ejes
                servo_x = int(np.interp(cX, [0, frame.shape[1]], [0, 180]))
                servo_y = int(np.interp(cY, [0, frame.shape[0]], [0, 180]))

                # Enviar comandos al brazo robótico a través del puerto serie
                ser.write(f'X{servo_x}Y{servo_y}\n'.encode())

    # Procesar contornos azules
    if contours_blue:
        # Encontrar el contorno más grande
        largest_contour_blue = max(contours_blue, key=cv2.contourArea)
        area_blue = cv2.contourArea(largest_contour_blue)

        if area_blue > 100:  # Se ha añadido el área mínima aquí también
            # Calcular el centroide de la figura
            M = cv2.moments(largest_contour_blue)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0

            # Identificar la figura por su número de vértices
            epsilon = 0.02 * cv2.arcLength(largest_contour_blue, True)
            approx = cv2.approxPolyDP(largest_contour_blue, epsilon, True)

            if len(approx) == 4:
                # Calcular las razones entre las longitudes de los lados opuestos del rectángulo
                side1 = np.linalg.norm(approx[0] - approx[1])
                side2 = np.linalg.norm(approx[1] - approx[2])
                ratio = side1 / side2  # Razón de longitud de los lados opuestos

                # Verificar si las razones están dentro de un rango para cuadrados o rectángulos
                if 0.85 <= ratio <= 1.15:  # Margen de error de medio centímetro
                    shape = "cuadrado"
                else:
                    shape = "rectangulo"
            else:
                shape = "otro"

            # Dibujar el centroide en la imagen
            cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)

            # Dibujar la figura y el nombre en la imagen
            cv2.drawContours(frame, [largest_contour_blue], -1, (0, 255, 0), 2)
            cv2.putText(frame, f"{shape} azul", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Imprimir las coordenadas debajo del tipo de figura
            cv2.putText(frame, f"Coordenadas: X={cX}, Y={cY}", (cX - 20, cY + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 2)

            # Enviar comandos al brazo robótico
            if cX != 0 and cY != 0:
                # Convertir coordenadas a comandos para el brazo robótico
                # Supongamos que el rango del brazo es de 0 a 180 grados en ambos ejes
                servo_x = int(np.interp(cX, [0, frame.shape[1]], [0, 180]))
                servo_y = int(np.interp(cY, [0, frame.shape[0]], [0, 180]))

                # Enviar comandos al brazo robótico a través del puerto serie
                ser.write(f'X{servo_x}Y{servo_y}\n'.encode())

    # Verificar el tiempo transcurrido y cerrar la cámara si es necesario
    current_time = time.time()

    if start_time_red is not None and current_time - start_time_red > max_time:
        cap.release()
        cv2.destroyAllWindows()
        # Mover el primer servo a 0 grados
        ser.write(b'X90Y180\n')
        time.sleep(2)  # Esperar 2 segundos
        ser.write(b'X90Y90\n')

        break

    if start_time_blue is not None and current_time - start_time_blue > max_time:
        cap.release()
        cv2.destroyAllWindows()
        # Mover el primer servo a 0 grados
        ser.write(b'X90Y0\n')
        time.sleep(2)  # Esperar 2 segundos
        ser.write(b'X90Y90\n')
        break

    # Mostrar la imagen con las figuras detectadas
    cv2.imshow("Image", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
