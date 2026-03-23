# 🤖 P4 Adicional — Robot de Seguimiento de Línea con Control PID Concurrente

Proyecto adicional de la **Práctica 4 de FreeRTOS con ESP32**, basado en el diseño de un **robot seguidor de línea** con arquitectura multitarea.  
El sistema se ha desarrollado sobre **ESP32 DevKit v1** usando **PlatformIO + Arduino + FreeRTOS**, separando la lectura de sensores, el control PID, el accionamiento de motores, la detección de obstáculos y la telemetría en tareas concurrentes.

---

## 📌 Descripción

Este proyecto implementa un **robot de seguimiento de línea** utilizando un enfoque basado en **FreeRTOS**, donde cada parte importante del sistema se ejecuta en una tarea diferente.

La idea principal es que el robot sea capaz de:

- Leer un **array de 5 sensores IR**
- Calcular la **posición de la línea**
- Aplicar un **control PID** en tiempo real
- Ajustar la velocidad de los motores mediante **PWM**
- Detectar obstáculos con un **HC-SR04**
- Detenerse de forma segura si hay peligro
- Enviar información de estado por **Serial / Bluetooth**

---

## 🎯 Objetivo

El objetivo de este proyecto es aplicar los conceptos de **sistemas operativos en tiempo real** en un caso práctico de control embebido, utilizando:

- **múltiples tareas concurrentes**
- **prioridades diferenciadas**
- **colas de comunicación**
- **mutex**
- **tareas fijadas a un núcleo**
- **watchdog de aplicación**

---

## 🧰 Hardware utilizado

- **ESP32 DevKit v1**
- **Array de 5 sensores IR**
- **Driver L298N**
- **2 motores DC**
- **Sensor ultrasónico HC-SR04**
- **Bluetooth / BLE para telemetría**

---

## 🧠 Arquitectura del sistema

La aplicación se divide en varias tareas FreeRTOS para separar responsabilidades y asegurar un comportamiento más ordenado y robusto.

### Tareas principales

| Tarea | Prioridad | Núcleo | Función |
|------|-----------:|--------|---------|
| `Task_Sensores_IR` | 4 | No fijado | Leer los sensores IR y calcular la posición de la línea |
| `Task_PID` | 4 | Core 1 | Ejecutar el control PID cada 10 ms |
| `Task_Motores` | 3 | No fijado | Aplicar la consigna PWM al driver L298N |
| `Task_Ultrasonidos` | 3 | No fijado | Detectar obstáculos y detener el robot |
| `Task_Telemetria_BT` | 1 | No fijado | Enviar información de estado por Serial/Bluetooth |
| `Task_Watchdog` | 2 | No fijado | Supervisar heartbeats y stack de las tareas |

---

## 🔄 Funcionamiento general

El flujo principal del sistema es el siguiente:

1. **Task_Sensores_IR** lee los 5 sensores y calcula la posición de la línea.
2. Esa información se envía mediante una **cola** a **Task_PID**.
3. **Task_PID** calcula el error respecto al centro y genera la corrección.
4. La corrección se envía a **Task_Motores**, que controla el **L298N**.
5. **Task_Ultrasonidos** comprueba continuamente la presencia de obstáculos.
6. Si detecta uno, el robot se detiene y se suspende temporalmente el PID.
7. **Task_Telemetria_BT** muestra por Serial/Bluetooth el estado interno del sistema.
8. **Task_Watchdog** supervisa que todas las tareas sigan activas y con stack suficiente.

---

## ⚙️ Elementos FreeRTOS utilizados

Este proyecto utiliza varios mecanismos de FreeRTOS para organizar y sincronizar el sistema:

### ✅ Tareas concurrentes
Cada bloque funcional del robot trabaja en su propia tarea.

### ✅ Colas (`Queue`)
Se usan para comunicar tareas de forma segura:

- `qLineData` → pasa la posición de línea desde sensores al PID
- `qMotorCmd` → pasa la consigna del PID a la tarea de motores

### ✅ Mutex
- `mtxIO` → protege la salida por **Serial / Bluetooth** para evitar conflictos entre tareas

### ✅ Tarea fijada a un núcleo
- `Task_PID` se crea con `xTaskCreatePinnedToCore()` para garantizar un comportamiento más estable al ser la tarea más crítica

### ✅ Watchdog de aplicación
Se supervisan:
- los **heartbeats** de cada tarea
- el **stack libre mínimo** con `uxTaskGetStackHighWaterMark()`

---

## 📍 Pines utilizados

### Sensores IR
| Sensor | GPIO |
|--------|------|
| IR1 | 36 |
| IR2 | 39 |
| IR3 | 34 |
| IR4 | 35 |
| IR5 | 32 |

### Driver L298N

#### Motor izquierdo
| Señal | GPIO |
|------|------|
| ENA | 25 |
| IN1 | 26 |
| IN2 | 27 |

#### Motor derecho
| Señal | GPIO |
|------|------|
| ENB | 14 |
| IN3 | 18 |
| IN4 | 19 |

### Sensor ultrasónico HC-SR04
| Señal | GPIO |
|------|------|
| TRIG | 23 |
| ECHO | 22 |

> ⚠️ **Importante:** la salida `ECHO` del HC-SR04 trabaja a **5V**, por lo que en un montaje real es recomendable usar un **divisor resistivo** o adaptación de nivel antes de conectarlo al ESP32.

---

## 📐 Parámetros de control

El controlador PID se ha planteado con los siguientes valores iniciales:

- **Kp = 45.0**
- **Ki = 8.0**
- **Kd = 20.0**

Otros parámetros importantes:

- **Velocidad base**: `160`
- **Periodo del PID**: `10 ms`
- **Umbral de obstáculo**: `10 cm`

> Estos valores son iniciales y deberían ajustarse experimentalmente en un robot real.

---

## 🛡️ Medidas de seguridad

El sistema incluye varias medidas de protección:

- Parada automática si se detecta un obstáculo
- Parada segura si se pierde la línea
- Suspensión temporal del control PID ante obstáculo
- Watchdog de aplicación para detectar fallos de tareas
- Supervisión del stack para evitar desbordamientos

---

## 🖥️ Salida esperada por monitor serie / Bluetooth

Durante la ejecución se espera observar mensajes relacionados con:

- Inicialización del sistema
- Posición de la línea
- Error del PID
- Corrección aplicada
- Velocidad de motores
- Distancia medida por el HC-SR04
- Estado de obstáculo
- Estado del watchdog

Ejemplo de salida:

```text
[INIT] Robot seguidor de linea con FreeRTOS iniciado.
[INIT] BLE/BT nombre: ESP32-LineFollower
[TEL] pos=1980 err=-0.02 ctrl=-1.30 L=158 R=161 dist=35.2 obst=0
[WDG] Task_PID hb=OK stackMin=3020 words
[ULTRA] Obstaculo detectado. PID suspendido y robot parado.
