import cv2
import numpy as np

# Inicializar la cámara
cap = cv2.VideoCapture(0)

while True:
    _, frame = cap.read()
    # Convertir a espacio de color HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Definir rangos de color para las figuras
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    lower_blue = np.array([90, 50, 50])
    upper_blue = np.array([130, 255, 255])

    # Crear máscaras para cada color
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # Encontrar contornos en las máscaras
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours_red:
        # Calcular el centroide de la figura
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0

        # Dibujar el centroide en la imagen
        cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)

        # Identificar la figura por su número de vértices
        epsilon = 0.04 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        if len(approx) == 3:
            shape = "triangulo"
        else:
            shape = "circulo"

        # Dibujar la figura y el nombre en la imagen
        cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)
        cv2.putText(frame, shape + " rojo", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    for cnt in contours_blue:
        # Calcular el centroide de la figura
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0

        # Dibujar el centroide en la imagen
        cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)

        # Identificar la figura por su número de vértices
        epsilon = 0.04 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

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

        # Dibujar la figura y el nombre en la imagen
        cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)
        cv2.putText(frame, shape + " azul", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Mostrar la imagen con las figuras detectadas
    cv2.imshow("Image", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
