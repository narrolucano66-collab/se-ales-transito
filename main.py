import cv2
import numpy as np
import serial
import time

# Definimos los rangos de colores en el espacio HSV
verde_bajo = np.array([40, 50, 50]) # Verde oscuro, con saturación y valor más altos
verde_alto = np.array([90, 255, 255])# Verde claro, aún más específico

amarillo_bajo = np.array([20, 100, 100])# amarillo oscuro, con saturación y valor más altos
amarillo_alto = np.array([40, 255, 255])# Amarilo claro, aún más específico

# Rango ajustado para el color rojo, más restrictivo para evitar detección errónea
rojo_bajo = np.array([0, 150, 150])   # Rojo oscuro, con saturación y valor más altos
rojo_alto = np.array([10, 255, 255])  # Rojo claro, aún más específico

# Configuración del puerto serial para Arduino
ser = serial.Serial('COM6', 9600)  # Ajusta el puerto según sea necesario
time.sleep(2)  # Espera para asegurarse de que la conexión esté establecida

# Abrimos la cámara web
cap = cv2.VideoCapture(0)

# Variable para almacenar el último estado del semáforo y el tiempo de inicio
ultimo_estado = None
tiempo_inicio = None

# Tiempo máximo en segundos para mantener el sonido activado (5 SEGUNDOS)
tiempo_maximo = 5

while True:
    # Capturamos el frame
    ret, frame = cap.read()
    
    # Si no se obtiene un frame, salimos del bucle
    if not ret:
        break

    # Convertimos el frame al espacio de color HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Detectar círculos usando HoughCircles
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (15, 15), 0)  # Reducimos el ruido para detectar mejor los círculos

    # Ajuste de los parámetros de HoughCircles
    # Establecemos un rango para el radio de los círculos
    min_radius = 20  # Radio mínimo para los círculos que se dectecta
    max_radius = 60  # Radio máximo para los círculos que se dectecta

    # Detectamos círculos usando HoughCircles con estos parámetros
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=40, minRadius=min_radius, maxRadius=max_radius)
    
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        
        for (x, y, r) in circles:
            # Dibujamos el círculo detectado con mayor grosor y en color blanco para hacerlo más visible
            cv2.circle(frame, (x, y), r, (255, 255, 255), 5)
            
            # Creamos una máscara para extraer el área dentro del círculo
            mask = np.zeros_like(frame)
            cv2.circle(mask, (x, y), r, (255, 255, 255), -1)  # Máscara blanca en el círculo
            # Extraemos la región dentro del círculo
            masked_frame = cv2.bitwise_and(frame, mask)

            # Convertimos la región extraída a HSV para la detección de colores
            hsv_region = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2HSV)
            
            # Detectamos el color verde
            mask_verde = cv2.inRange(hsv_region, verde_bajo, verde_alto)
            verde = cv2.countNonZero(mask_verde)

            # Detectamos el color amarillo
            mask_amarillo = cv2.inRange(hsv_region, amarillo_bajo, amarillo_alto)
            amarillo = cv2.countNonZero(mask_amarillo)

            # Detectamos el color rojo usando el rango ajustado
            mask_rojo = cv2.inRange(hsv_region, rojo_bajo, rojo_alto)
            rojo = cv2.countNonZero(mask_rojo)

            #  determinar la alerta si cualquier color está presente
            if rojo > 0:  # Si detecta el color rojo
                if ultimo_estado != "rojo":
                    ser.write(b'35')  #  (volumen al 35%)
                    ultimo_estado = "rojo"
                    tiempo_inicio = time.time()  # Iniciar el temporizador para rojo
                cv2.putText(frame, "ALERTA: Semaforo Rojo - PASAR", (x - r, y - r - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                cv2.putText(frame, "Puedes PASAR", (x - r, y - r), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                cv2.circle(frame, (x, y), r, (0, 0, 255), -1)  # Rellenar el círculo con rojo

            elif verde > 0:  # Si detecta el color verde
                if ultimo_estado != "verde":
                    ser.write(b'100')  #  (volumen al 100%)
                    ultimo_estado = "verde"
                    tiempo_inicio = time.time()  # Iniciar el temporizador para verde
                cv2.putText(frame, "ALERTA: Semaforo Verde - NO PASAR", (x - r, y - r - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
                cv2.putText(frame, "No puedes PASAR", (x - r, y - r), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
                cv2.circle(frame, (x, y), r, (0, 255, 0), -1)  # Rellenar el círculo con verde

            elif amarillo > 0:  # Si detecta el color amarillo
                if ultimo_estado != "amarillo":
                    ser.write(b'0')  #  (Volumen al 0%)
                    ultimo_estado = "amarillo"
                    tiempo_inicio = time.time()  # Iniciar el temporizador para amarillo
                cv2.putText(frame, "ALERTA: Semaforo Amarillo - ESPERAR", (x - r, y - r - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)
                cv2.putText(frame, "Espera", (x - r, y - r), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)
                cv2.circle(frame, (x, y), r, (0, 255, 255), -1)  # Rellenar el círculo con amarillo

    else:
        # Si no detectamos círculos, mostramos un mensaje de "Sin deteccion"
        cv2.putText(frame, "Sin deteccion de semaforo", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

    # Apagar el sonido después de 1 minuto
    if tiempo_inicio is not None and time.time() - tiempo_inicio > tiempo_maximo:
        ser.write(b'0')  # Apagar el sonido
        tiempo_inicio = None  # Resetear el temporizador

    # Mostrar la imagen con la información del semáforo
    cv2.imshow('Semaforo Detectado', frame)
    
    # Si presionamos la tecla 'q', se sale del loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cerramos la cámara y todas las ventanas de OpenCV
cap.release()
cv2.destroyAllWindows()
ser.close()
