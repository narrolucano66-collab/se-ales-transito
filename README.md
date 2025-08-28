# 🚦 Sistema de Detección de Semáforos para Personas con Discapacidad Visual

Proyecto de **visión por computadora + Arduino** que, con una **cámara web**, detecta el color del semáforo (rojo/amarillo/verde) y envía una señal por **serial** para accionar un **vibrador** (o buzzer) que guíe al usuario.

> **Nota:** El mapeo de acciones sigue exactamente el código proporcionado:
> - **Rojo** → “Puedes PASAR” → envía `35`
> - **Verde** → “No puedes PASAR” → envía `100`
> - **Amarillo** → “Espera” → envía `0`  
> Ajusta estos comportamientos si quieres alinear con otras reglas.

---

## ✨ Características
- Detección de **círculos** (lentes del semáforo) con `HoughCircles`.
- Clasificación por **color HSV** (rojo/amarillo/verde).
- Señal háptica/sonora vía **Arduino** (puerto serial).
- **Temporizador** de seguridad para desactivar tras `5 s`.
- Vista en tiempo real con overlay de estado.

---

## 🧰 Requisitos

### Hardware
- Cámara web.
- Arduino (UNO u otro compatible).
- Módulo **vibrador** o **buzzer** conectado al Arduino.
- Cable USB.

### Software
- Python 3.8+
- Dependencias:
  ```bash
  pip install opencv-python numpy pyserial
## ESTRUCTURA DEL CODIGO
Proyecto-Semaforo-Invidentes/
├── semaforo_invidentes.py   # Código principal (este repo)
└── README.md                # Este documento
##CONFIGURACION DE PUERTO
#Conecta el Arduino por USB.
#En el código, ajusta el puerto:
ser = serial.Serial('COM6', 9600)
##EJECUCION
python semaforo_invidentes.py
##Ejemplo de lectura en Arduino (orientativo)
// Lee valores "0", "35" o "100" enviados como texto y ajusta salida PWM.
void setup() {
  Serial.begin(9600);
  pinMode(9, OUTPUT); // vibrador/buzzer con PWM
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n'); // o usa parseInt si envías números con separador
    s.trim();
    int val = s.toInt(); // 0, 35, 100
    int pwm = map(val, 0, 100, 0, 255);
    analogWrite(9, pwm);
  }
}
##Parámetros ajustables (en el código)

##Rangos HSV:

verde_bajo = np.array([40, 50, 50]);  verde_alto   = np.array([90, 255, 255])
amarillo_bajo = np.array([20, 100, 100]); amarillo_alto = np.array([40, 255, 255])
rojo_bajo = np.array([0, 150, 150]);  rojo_alto   = np.array([10, 255, 255])
Ajusta según iluminación/cámara.

##Detección de círculos:

min_radius = 20
max_radius = 60

circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1, 20,
                           param1=50, param2=40,
                           minRadius=min_radius, maxRadius=max_radius)


Baja param2 para más sensibilidad (más falsos positivos), súbelo para menos.

##Temporizador de silencio:

tiempo_maximo = 5  # segundos

🧪 Consejos de calibración

Comienza con buena iluminación y coloca la cámara enfocando el semáforo a media distancia.

Ajusta primero rangos HSV (observa la máscara con cv2.inRange si haces depuración).

Luego ajusta HoughCircles para que encuentre un solo círculo por luz.

🧯 Solución de problemas

No abre la cámara: prueba VideoCapture(1), revisa permisos.

Puerto serial ocupado: cierra el monitor serial del IDE de Arduino.

Falsos positivos de color: aumenta S y V mínimos en HSV o usa balance de blancos fijo.

Retrasos: baja resolución de captura (cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640) y alto 480).
