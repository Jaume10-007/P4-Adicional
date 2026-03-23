#include <Arduino.h>
#include <BluetoothSerial.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// =========================================================
// CONFIGURACION GENERAL DE HARDWARE
// =========================================================

// ------------ Sensores IR (ADC1 recomendados en ESP32) ------------
// Ajusta estos pines según tu array real de 5 sensores IR.
static const uint8_t IR_PINS[5] = {36, 39, 34, 35, 32};

// Calibración básica de cada sensor IR.
// Debes ajustarla experimentalmente con tus sensores reales.
static const int IR_CAL_MIN[5] = {300, 300, 300, 300, 300};     // blanco aproximado
static const int IR_CAL_MAX[5] = {3200, 3200, 3200, 3200, 3200}; // negro aproximado

// ------------ Driver L298N + motores DC ------------
static const uint8_t MOTOR_L_EN  = 25;  // PWM
static const uint8_t MOTOR_L_IN1 = 26;
static const uint8_t MOTOR_L_IN2 = 27;

static const uint8_t MOTOR_R_EN  = 14;  // PWM
static const uint8_t MOTOR_R_IN1 = 18;
static const uint8_t MOTOR_R_IN2 = 19;

// Canales PWM LEDC
static const uint8_t PWM_CH_L = 0;
static const uint8_t PWM_CH_R = 1;
static const uint32_t PWM_FREQ = 20000;   // 20 kHz
static const uint8_t PWM_RES_BITS = 8;    // 0..255

// ------------ HC-SR04 ------------
static const uint8_t US_TRIG_PIN = 23;
static const uint8_t US_ECHO_PIN = 22;

// ------------ Bluetooth Classic ------------
BluetoothSerial SerialBT;

// =========================================================
// PARAMETROS DE CONTROL Y RTOS
// =========================================================

// Frecuencias / periodos
static const TickType_t T_IR_MS        = pdMS_TO_TICKS(10);    // 100 Hz
static const TickType_t T_PID_MS       = pdMS_TO_TICKS(10);    // 100 Hz
static const TickType_t T_US_MS        = pdMS_TO_TICKS(50);    // 20 Hz
static const TickType_t T_TELEM_MS     = pdMS_TO_TICKS(200);   // 5 Hz
static const TickType_t T_WDG_MS       = pdMS_TO_TICKS(1000);  // 1 Hz

// Umbrales
static const float OBSTACLE_STOP_CM = 10.0f;
static const int LINE_DETECT_SUM_THRESHOLD = 700; // umbral total de línea detectada

// PID (ajuste inicial razonable; debe afinarse en pruebas reales)
static const float KP = 45.0f;
static const float KI = 8.0f;
static const float KD = 20.0f;

// Velocidades
static const int BASE_SPEED = 160;
static const int MAX_PWM = 255;
static const int MIN_PWM = -255;

// Posiciones ponderadas del array (0..4000), centro ideal = 2000
static const int IR_WEIGHTS[5] = {0, 1000, 2000, 3000, 4000};
static const int LINE_CENTER = 2000;

// =========================================================
// ESTRUCTURAS DE DATOS
// =========================================================

struct LineData {
    uint32_t raw[5];
    uint16_t norm[5];      // 0..1000
    int32_t position;      // 0..4000
    bool lineDetected;
};

struct MotorCommand {
    int16_t leftPWM;       // -255..255
    int16_t rightPWM;      // -255..255
    float error;           // error normalizado aprox. [-2..2]
    float control;         // salida PID
    bool lineLost;
    bool obstacleStop;
};

struct TelemetryState {
    uint32_t raw[5];
    uint16_t norm[5];
    int32_t position;
    bool lineDetected;
    float pidError;
    float pidControl;
    int16_t leftPWM;
    int16_t rightPWM;
    float distanceCm;
    bool obstacleDetected;
    bool pidSuspended;
    bool lineLost;
};

// =========================================================
// RECURSOS FreeRTOS
// =========================================================

// Cola sensores -> PID
QueueHandle_t qLineData = NULL;

// Cola PID -> motores
QueueHandle_t qMotorCmd = NULL;

// Mutex para proteger salida de depuración y telemetría compartida
SemaphoreHandle_t mtxIO = NULL;

// Handles de tareas
TaskHandle_t hTaskSensoresIR   = NULL;
TaskHandle_t hTaskPID          = NULL;
TaskHandle_t hTaskMotores      = NULL;
TaskHandle_t hTaskUltrasonidos = NULL;
TaskHandle_t hTaskTelemetria   = NULL;
TaskHandle_t hTaskWatchdog     = NULL;

// =========================================================
// ESTADO GLOBAL COMPARTIDO
// =========================================================

portMUX_TYPE gStateMux = portMUX_INITIALIZER_UNLOCKED;
TelemetryState gState = {};

volatile bool gObstacleDetected = false;
volatile bool gPidSuspended = false;

// Heartbeats por tarea para watchdog de aplicación
volatile TickType_t hbSensoresIR = 0;
volatile TickType_t hbPID = 0;
volatile TickType_t hbMotores = 0;
volatile TickType_t hbUltrasonidos = 0;
volatile TickType_t hbTelemetria = 0;

// =========================================================
// FUNCIONES AUXILIARES
// =========================================================

int clampInt(int value, int minVal, int maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

float clampFloat(float value, float minVal, float maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

// Normaliza lectura ADC usando calibración por sensor.
// Devuelve un valor entre 0 y 1000.
uint16_t normalizeIR(uint32_t raw, int calMin, int calMax) {
    if (calMax <= calMin) return 0;

    long value = ((long)raw - calMin) * 1000L / (calMax - calMin);
    value = constrain(value, 0L, 1000L);
    return (uint16_t)value;
}

// Impresión protegida por mutex hacia Serial y Bluetooth.
// Así evitamos mensajes mezclados de varias tareas.
void safeLog(const String &msg) {
    if (mtxIO == NULL) return;

    if (xSemaphoreTake(mtxIO, portMAX_DELAY) == pdTRUE) {
        Serial.print(msg);
        if (SerialBT.hasClient()) {
            SerialBT.print(msg);
        }
        xSemaphoreGive(mtxIO);
    }
}

// Actualiza el estado global protegido.
void updateStateLine(const LineData &data) {
    portENTER_CRITICAL(&gStateMux);
    for (int i = 0; i < 5; i++) {
        gState.raw[i] = data.raw[i];
        gState.norm[i] = data.norm[i];
    }
    gState.position = data.position;
    gState.lineDetected = data.lineDetected;
    gState.lineLost = !data.lineDetected;
    portEXIT_CRITICAL(&gStateMux);
}

void updateStateMotor(const MotorCommand &cmd) {
    portENTER_CRITICAL(&gStateMux);
    gState.pidError = cmd.error;
    gState.pidControl = cmd.control;
    gState.leftPWM = cmd.leftPWM;
    gState.rightPWM = cmd.rightPWM;
    gState.lineLost = cmd.lineLost;
    portEXIT_CRITICAL(&gStateMux);
}

void updateStateObstacle(float distCm, bool obstacle, bool pidSuspended) {
    portENTER_CRITICAL(&gStateMux);
    gState.distanceCm = distCm;
    gState.obstacleDetected = obstacle;
    gState.pidSuspended = pidSuspended;
    portEXIT_CRITICAL(&gStateMux);
}

TelemetryState getStateSnapshot() {
    TelemetryState copy;
    portENTER_CRITICAL(&gStateMux);
    copy = gState;
    portEXIT_CRITICAL(&gStateMux);
    return copy;
}

// ---------------------------------------------------------
// CONTROL REAL DE MOTORES L298N
// ---------------------------------------------------------

// Inicializa pines y PWM de los dos motores.
void setupMotors() {
    pinMode(MOTOR_L_IN1, OUTPUT);
    pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_R_IN1, OUTPUT);
    pinMode(MOTOR_R_IN2, OUTPUT);

    ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES_BITS);
    ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES_BITS);

    ledcAttachPin(MOTOR_L_EN, PWM_CH_L);
    ledcAttachPin(MOTOR_R_EN, PWM_CH_R);

    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, LOW);
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, LOW);

    ledcWrite(PWM_CH_L, 0);
    ledcWrite(PWM_CH_R, 0);
}

// Aplica velocidad signed a un motor.
// speed > 0: adelante
// speed < 0: atrás
// speed = 0: frenado / paro
void setSingleMotor(int speed, uint8_t in1, uint8_t in2, uint8_t pwmChannel) {
    int duty = abs(speed);
    duty = clampInt(duty, 0, 255);

    if (speed > 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (speed < 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }

    ledcWrite(pwmChannel, duty);
}

// Aplica la consigna a ambos motores.
void setMotors(int leftPWM, int rightPWM) {
    setSingleMotor(leftPWM, MOTOR_L_IN1, MOTOR_L_IN2, PWM_CH_L);
    setSingleMotor(rightPWM, MOTOR_R_IN1, MOTOR_R_IN2, PWM_CH_R);
}

// Parada segura del robot.
void stopMotors() {
    setMotors(0, 0);
}

// ---------------------------------------------------------
// LECTURA REAL DEL HC-SR04
// ---------------------------------------------------------

float readDistanceCm() {
    digitalWrite(US_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(US_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TRIG_PIN, LOW);

    // Timeout ~30 ms => aprox 5 m máx.
    unsigned long duration = pulseIn(US_ECHO_PIN, HIGH, 30000UL);

    if (duration == 0) {
        return 400.0f; // sin eco -> muy lejos
    }

    return (duration * 0.0343f) / 2.0f;
}

// =========================================================
// TAREA 1: SENSORES IR
// Lee el array de 5 sensores IR a 100 Hz y calcula la posición.
// FreeRTOS:
// - periodicidad estable con vTaskDelayUntil
// - productor principal de datos para la cola qLineData
// =========================================================
void Task_Sensores_IR(void *pvParameters) {
    (void)pvParameters;

    TickType_t lastWakeTime = xTaskGetTickCount();

    for (;;) {
        LineData data = {};
        uint32_t weightedSum = 0;
        uint32_t sumNorm = 0;

        for (int i = 0; i < 5; i++) {
            data.raw[i] = analogRead(IR_PINS[i]);
            data.norm[i] = normalizeIR(data.raw[i], IR_CAL_MIN[i], IR_CAL_MAX[i]);

            weightedSum += (uint32_t)data.norm[i] * (uint32_t)IR_WEIGHTS[i];
            sumNorm += data.norm[i];
        }

        data.lineDetected = (sumNorm >= (uint32_t)LINE_DETECT_SUM_THRESHOLD);

        if (data.lineDetected && sumNorm > 0) {
            data.position = (int32_t)(weightedSum / sumNorm); // 0..4000
        } else {
            data.position = LINE_CENTER; // fallback seguro
        }

        // Cola de tamaño 1: nos quedamos siempre con la última muestra.
        xQueueOverwrite(qLineData, &data);

        updateStateLine(data);
        hbSensoresIR = xTaskGetTickCount();

        vTaskDelayUntil(&lastWakeTime, T_IR_MS);
    }
}

// =========================================================
// TAREA 2: PID
// Ejecuta el lazo de control cada 10 ms.
// FreeRTOS:
// - tarea crítica anclada a un núcleo con xTaskCreatePinnedToCore
// - consumidor de qLineData
// - productor de qMotorCmd
// =========================================================
void Task_PID(void *pvParameters) {
    (void)pvParameters;

    TickType_t lastWakeTime = xTaskGetTickCount();
    LineData lineData = {};
    float integral = 0.0f;
    float prevError = 0.0f;

    for (;;) {
        // No bloqueamos demasiado para no romper el periodo fijo.
        xQueuePeek(qLineData, &lineData, 0);

        MotorCommand cmd = {};
        cmd.lineLost = !lineData.lineDetected;
        cmd.obstacleStop = gObstacleDetected;

        if (gObstacleDetected) {
            // Si hay obstáculo, el PID no debe mover el robot.
            integral = 0.0f;
            prevError = 0.0f;
            cmd.leftPWM = 0;
            cmd.rightPWM = 0;
            cmd.error = 0.0f;
            cmd.control = 0.0f;
        } else if (!lineData.lineDetected) {
            // Pérdida de línea: comportamiento seguro => parar robot.
            integral = 0.0f;
            prevError = 0.0f;
            cmd.leftPWM = 0;
            cmd.rightPWM = 0;
            cmd.error = 0.0f;
            cmd.control = 0.0f;
        } else {
            // Error normalizado aproximadamente a [-2..2]
            float error = ((float)lineData.position - (float)LINE_CENTER) / 1000.0f;
            float dt = 0.010f; // 10 ms

            integral += error * dt;
            integral = clampFloat(integral, -1.5f, 1.5f); // anti-windup simple

            float derivative = (error - prevError) / dt;
            float control = KP * error + KI * integral + KD * derivative;

            int left = BASE_SPEED + (int)control;
            int right = BASE_SPEED - (int)control;

            left = clampInt(left, MIN_PWM, MAX_PWM);
            right = clampInt(right, MIN_PWM, MAX_PWM);

            cmd.leftPWM = left;
            cmd.rightPWM = right;
            cmd.error = error;
            cmd.control = control;

            prevError = error;
        }

        xQueueOverwrite(qMotorCmd, &cmd);
        updateStateMotor(cmd);
        hbPID = xTaskGetTickCount();

        vTaskDelayUntil(&lastWakeTime, T_PID_MS);
    }
}

// =========================================================
// TAREA 3: MOTORES
// Recibe el comando del PID y controla el L298N con PWM.
// FreeRTOS:
// - consumidor de qMotorCmd
// - desacopla el control PID del acceso a hardware PWM
// =========================================================
void Task_Motores(void *pvParameters) {
    (void)pvParameters;

    MotorCommand cmd = {};

    for (;;) {
        if (xQueueReceive(qMotorCmd, &cmd, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (cmd.obstacleStop || cmd.lineLost) {
                stopMotors();
            } else {
                setMotors(cmd.leftPWM, cmd.rightPWM);
            }
        } else {
            // Timeout sin comando nuevo => seguridad
            stopMotors();
        }

        hbMotores = xTaskGetTickCount();
    }
}

// =========================================================
// TAREA 4: ULTRASONIDOS
// Lee el HC-SR04 y detiene temporalmente el PID si hay obstáculo.
// FreeRTOS:
// - usa vTaskSuspend/vTaskResume sobre Task_PID
// - genera un comando de parada inmediata al detectar obstáculo
// =========================================================
void Task_Ultrasonidos(void *pvParameters) {
    (void)pvParameters;

    TickType_t lastWakeTime = xTaskGetTickCount();
    bool prevObstacle = false;

    for (;;) {
        float distanceCm = readDistanceCm();
        bool obstacle = (distanceCm < OBSTACLE_STOP_CM);

        gObstacleDetected = obstacle;

        if (obstacle && !gPidSuspended) {
            // Suspende la tarea PID y fuerza parada segura
            gPidSuspended = true;
            vTaskSuspend(hTaskPID);

            MotorCommand stopCmd = {};
            stopCmd.leftPWM = 0;
            stopCmd.rightPWM = 0;
            stopCmd.obstacleStop = true;
            stopCmd.lineLost = false;
            xQueueOverwrite(qMotorCmd, &stopCmd);

            safeLog("[ULTRA] Obstaculo detectado. PID suspendido y robot parado.\n");
        } else if (!obstacle && gPidSuspended) {
            // Reanuda el control PID al liberar el camino
            gPidSuspended = false;
            vTaskResume(hTaskPID);

            safeLog("[ULTRA] Trayectoria libre. PID reanudado.\n");
        }

        if (obstacle != prevObstacle) {
            prevObstacle = obstacle;
        }

        updateStateObstacle(distanceCm, obstacle, gPidSuspended);
        hbUltrasonidos = xTaskGetTickCount();

        vTaskDelayUntil(&lastWakeTime, T_US_MS);
    }
}

// =========================================================
// TAREA 5: TELEMETRIA BT
// Envía el estado general por Serial y Bluetooth.
// FreeRTOS:
// - baja prioridad para no interferir con el lazo de control
// - usa mutex para evitar salidas concurrentes corruptas
// =========================================================
void Task_Telemetria_BT(void *pvParameters) {
    (void)pvParameters;

    TickType_t lastWakeTime = xTaskGetTickCount();

    for (;;) {
        TelemetryState s = getStateSnapshot();

        String msg;
        msg.reserve(256);
        msg += "[TEL] IRraw=[";
        for (int i = 0; i < 5; i++) {
            msg += String(s.raw[i]);
            if (i < 4) msg += ",";
        }
        msg += "] IRn=[";
        for (int i = 0; i < 5; i++) {
            msg += String(s.norm[i]);
            if (i < 4) msg += ",";
        }
        msg += "]";
        msg += " pos=" + String(s.position);
        msg += " line=" + String(s.lineDetected ? 1 : 0);
        msg += " err=" + String(s.pidError, 3);
        msg += " ctrl=" + String(s.pidControl, 2);
        msg += " L=" + String(s.leftPWM);
        msg += " R=" + String(s.rightPWM);
        msg += " dist=" + String(s.distanceCm, 1);
        msg += " obst=" + String(s.obstacleDetected ? 1 : 0);
        msg += " pidSusp=" + String(s.pidSuspended ? 1 : 0);
        msg += " lineLost=" + String(s.lineLost ? 1 : 0);
        msg += "\n";

        safeLog(msg);

        hbTelemetria = xTaskGetTickCount();
        vTaskDelayUntil(&lastWakeTime, T_TELEM_MS);
    }
}

// =========================================================
// TAREA 6: WATCHDOG DE APLICACION
// Comprueba que cada tarea sigue viva y revisa stack mínimo libre.
// FreeRTOS:
// - usa uxTaskGetStackHighWaterMark para diagnosticar stack
// - vigila heartbeats por tarea
// =========================================================
void Task_Watchdog(void *pvParameters) {
    (void)pvParameters;

    TickType_t lastWakeTime = xTaskGetTickCount();

    for (;;) {
        TickType_t now = xTaskGetTickCount();

        struct WatchEntry {
            const char *name;
            TaskHandle_t handle;
            TickType_t heartbeat;
            TickType_t timeout;
            bool canBeSuspended;
        };

        WatchEntry entries[] = {
            {"Task_Sensores_IR",   hTaskSensoresIR,   hbSensoresIR,   pdMS_TO_TICKS(150), false},
            {"Task_PID",           hTaskPID,          hbPID,          pdMS_TO_TICKS(150), true},
            {"Task_Motores",       hTaskMotores,      hbMotores,      pdMS_TO_TICKS(250), false},
            {"Task_Ultrasonidos",  hTaskUltrasonidos, hbUltrasonidos, pdMS_TO_TICKS(300), false},
            {"Task_Telemetria_BT", hTaskTelemetria,   hbTelemetria,   pdMS_TO_TICKS(600), false}
        };

        for (const auto &entry : entries) {
            bool stale = ((now - entry.heartbeat) > entry.timeout);

            // Caso especial: si PID está suspendida por obstáculo, no se considera fallo.
            if (entry.canBeSuspended && gPidSuspended) {
                stale = false;
            }

            UBaseType_t watermarkWords = uxTaskGetStackHighWaterMark(entry.handle);

            String msg = "[WDG] ";
            msg += entry.name;
            msg += " hb=";
            msg += (stale ? "FAIL" : "OK");
            msg += " stackMin=";
            msg += String((uint32_t)watermarkWords);
            msg += " words\n";

            safeLog(msg);

            if (stale) {
                stopMotors();
            }
        }

        vTaskDelayUntil(&lastWakeTime, T_WDG_MS);
    }
}

// =========================================================
// SETUP
// =========================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    // Bluetooth Classic para telemetría
    SerialBT.begin("ESP32-LineFollower");

    // ADC
    analogReadResolution(12);
    for (int i = 0; i < 5; i++) {
        pinMode(IR_PINS[i], INPUT);
        analogSetPinAttenuation(IR_PINS[i], ADC_11db);
    }

    // Ultrasonidos
    pinMode(US_TRIG_PIN, OUTPUT);
    pinMode(US_ECHO_PIN, INPUT);
    digitalWrite(US_TRIG_PIN, LOW);

    // Motores
    setupMotors();
    stopMotors();

    // Crear cola de línea: tamaño 1, nos quedamos con la última medida
    qLineData = xQueueCreate(1, sizeof(LineData));

    // Crear cola de motores: tamaño 1, nos quedamos con el último comando
    qMotorCmd = xQueueCreate(1, sizeof(MotorCommand));

    // Crear mutex para IO/telemetría
    mtxIO = xSemaphoreCreateMutex();

    if (qLineData == NULL || qMotorCmd == NULL || mtxIO == NULL) {
        Serial.println("[ERROR] No se pudieron crear los recursos FreeRTOS.");
        while (true) {
            delay(1000);
        }
    }

    TickType_t t0 = xTaskGetTickCount();
    hbSensoresIR = hbPID = hbMotores = hbUltrasonidos = hbTelemetria = t0;

    // Crear tareas con prioridades diferenciadas
    xTaskCreate(
        Task_Sensores_IR,
        "Task_Sensores_IR",
        4096,
        NULL,
        4,
        &hTaskSensoresIR
    );

    xTaskCreatePinnedToCore(
        Task_PID,
        "Task_PID",
        4096,
        NULL,
        4,
        &hTaskPID,
        1   // Core 1: tarea crítica de control
    );

    xTaskCreate(
        Task_Motores,
        "Task_Motores",
        4096,
        NULL,
        3,
        &hTaskMotores
    );

    xTaskCreate(
        Task_Ultrasonidos,
        "Task_Ultrasonidos",
        4096,
        NULL,
        3,
        &hTaskUltrasonidos
    );

    xTaskCreate(
        Task_Telemetria_BT,
        "Task_Telemetria_BT",
        4096,
        NULL,
        1,
        &hTaskTelemetria
    );

    xTaskCreate(
        Task_Watchdog,
        "Task_Watchdog",
        4096,
        NULL,
        2,
        &hTaskWatchdog
    );

    safeLog("\n[INIT] Robot seguidor de linea con FreeRTOS iniciado.\n");
    safeLog("[INIT] BLE/BT nombre: ESP32-LineFollower\n");
    safeLog("[INIT] Task_PID anclada al Core 1.\n");
}

// =========================================================
// LOOP
// El trabajo lo realizan las tareas FreeRTOS.
// =========================================================
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}