/***************************************************
 * Prueba.ino: Robot balanceador + Bluetooth (HC-05)
 * con librería LMotorController.
 * 
 * Usa los comandos F/f (Forward ON/OFF),
 * B/b (Backward ON/OFF), L/l (Left ON/OFF),
 * R/r (Right ON/OFF).
 ***************************************************/

#include <SoftwareSerial.h>        
#include "PID_v1.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "LMotorController.h"   // <-- tu librería de motor

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

// ========== Pines para Bluetooth (HC-05) =========
#define BT_RX_PIN 4
#define BT_TX_PIN 5
SoftwareSerial BTSerial(BT_RX_PIN, BT_TX_PIN);

// ========== Pines para Motores (según tu .h/.cpp) =========
int ENA = 10;
int IN1 = 6;
int IN2 = 7;
int ENB = 11;
int IN3 = 8;
int IN4 = 9;

// Constantes de calibración motor
double motorAConst = 0.5; 
double motorBConst = 0.5; 

// Creamos el objeto LMotorController
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorAConst, motorBConst);

// Velocidad mínima para que arranque el motor
#define MIN_ABS_SPEED 20  

// ========== Variables y objetos para MPU6050 =========
MPU6050 mpu;
bool dmpReady = false; 
uint8_t mpuIntStatus;  
uint8_t devStatus;     
uint16_t packetSize;   
uint16_t fifoCount;    
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

// ========== PID de balance =========
double originalSetpoint = 160; // Ajusta según tu calibración
double setpoint = originalSetpoint;
double input, output;

// Ajusta tus constantes PID
double Kp = 55;  
double Kd = 0;  
double Ki = 0;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// ========== Banderas para movimiento Bluetooth =========
bool moveForward  = false; 
bool moveBackward = false;
bool turnLeft     = false;
bool turnRight    = false;

// Ajustes para inclinarse y girar
int moveOffset = 3;   // Inclinación extra para avanzar/retroceder
int turnOffset = 30;  // Diferencia de velocidad para girar

// Interrupción MPU
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

// ===========================================================
// ================         SETUP         =====================
// ===========================================================
void setup() {
  Serial.begin(115200);
  
  // Bluetooth
  BTSerial.begin(9600);
  Serial.println("Bluetooth iniciado en 9600 baudios");

  // I2C + MPU6050
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#endif

  Serial.println(F("Inicializando MPU6050..."));
  mpu.initialize();
  Serial.println(
    mpu.testConnection() ? 
    F("MPU6050 conectado correctamente") : 
    F("Error: MPU6050 desconectado")
  );

  // Inicializar DMP
  devStatus = mpu.dmpInitialize();

  // Ajusta tus offsets de giroscopio/acelerómetro:
  mpu.setXGyroOffset(-9);
  mpu.setYGyroOffset(-62);
  mpu.setZGyroOffset(22);
  mpu.setZAccelOffset(-1228);

  // Verificar DMP
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    // Configurar PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);

    Serial.println(F("DMP listo y PID configurado!"));
  }
  else {
    Serial.print(F("Fallo en DMP (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

// ===========================================================
// ================         LOOP         ======================
// ===========================================================
void loop() {
  // 1) Leer comandos Bluetooth
  readBluetoothCommands();

  // 2) Verificar si el DMP está listo
  if (!dmpReady) return;

  // 3) Esperar interrupción o más paquetes
  while (!mpuInterrupt && fifoCount < packetSize) {
    // Calculamos PID (sin actualizar motores todavía)
    pid.Compute();
  }

  // 4) Procesar FIFO cuando haya datos
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount    = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // FIFO overflow
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    // Obtener orientaciones
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // El pitch (inclinación) está en ypr[1]. Lo convertimos a grados y sumamos 180:
    input = ypr[1] * 180.0 / M_PI + 180.0;

    // Ajustar setpoint para avanzar/retroceder
    if (moveForward && !moveBackward) {
      setpoint = originalSetpoint - moveOffset;
    }
    else if (moveBackward && !moveForward) {
      setpoint = originalSetpoint + moveOffset;
    }
    else {
      // Sin moverse adelante/atrás
      setpoint = originalSetpoint;
    }

    


    // Volver a calcular PID con el "nuevo" setpoint
    pid.Compute();

    // Calcular velocidades izquierda/derecha
    int speedLeft  = (int)output;
    int speedRight = (int)output;

    // Aplicar offset de giro
    if (turnLeft && !turnRight) {
      speedLeft  -= turnOffset;
      speedRight += turnOffset;
    }
    else if (turnRight && !turnLeft) {
      speedLeft  += turnOffset;
      speedRight -= turnOffset;
    }

    // Finalmente, mover cada motor usando la librería
    // (leftSpeed, rightSpeed, minAbsSpeed)
    motorController.move(speedLeft, speedRight, MIN_ABS_SPEED);
  }
}

// ===========================================================
// ========== LECTURA DE COMANDOS BLUETOOTH  ================
// ===========================================================
void readBluetoothCommands() {
  while (BTSerial.available() > 0) {
    char command = (char)BTSerial.read();
    Serial.print("Recibido: ");
    Serial.println(command);

    switch (command) {
      // FORWARD
      case 'F': // ON
        moveForward = true;
        break;
      case 'f': // OFF
        moveForward = false;
        break;

      // BACKWARD
      case 'B': // ON
        moveBackward = true;
        break;
      case 'b': // OFF
        moveBackward = false;
        break;

      // LEFT
      case 'L': // ON
        turnLeft = true;
        break;
      case 'l': // OFF
        turnLeft = false;
        break;

      // RIGHT
      case 'R': // ON
        turnRight = true;
        break;
      case 'r': // OFF
        turnRight = false;
        break;

      default:
        // Comando no reconocido
        break;
    }
  }
}
