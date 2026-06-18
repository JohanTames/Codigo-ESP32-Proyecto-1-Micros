#include <AccelStepper.h>
#include <Stepper.h>

// Descomentar la siguiente línea para probar el programa sin la rasp
#define TEST_MOTORES

// UART
#define RXD2 16
#define TXD2 17

// DIP y botón
#define DIP0 32
#define DIP1 33
#define DIP2 25
#define BTN 0

// Sensor Hall
#define HALL_PIN 26

// DRV8825
#define STEP_PIN 14
#define DIR_PIN 27
#define ENABLE_PIN 13

volatile bool botonFlag = false;
volatile bool hallFlancoDetectado = false;

AccelStepper stepperCentral(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
#define STEPS_28BYJ 2048
Stepper motoresPercheros(STEPS_28BYJ, 4, 5, 18, 19);

int perchero_actual = 1;
int posicion_actual[3] = { 0, 0, 0 };

int pasos_por_perchero = 1067;
const int POS_PERCHERO[3] = { -pasos_por_perchero, 0, pasos_por_perchero };
int pasos_por_prenda = 410;
int movimientosCentral = 0;

// ISR
void IRAM_ATTR ISR_boton() {
  botonFlag = true;
}
void IRAM_ATTR ISR_hall() {
  hallFlancoDetectado = true;
}

uint8_t leerDIP() {
  return (digitalRead(DIP2) << 2) | (digitalRead(DIP1) << 1) | digitalRead(DIP0);
}

void enviarCaracteristicas() {
  static uint8_t data[5];
  static int index = 0;
  uint8_t valor = leerDIP();
  Serial.println(valor);
  if (valor <= 5) data[index++] = valor;
  else data[index++] = 0;
  if (index == 5) {
    Serial2.write(0xAA);
    for (int i = 0; i < 5; i++) Serial2.write(data[i]);
    Serial2.write(0x55);
    index = 0;
  }
}

bool homingLocal() {
  Serial.println("Buscando hall...");
  digitalWrite(ENABLE_PIN, LOW);  //Encendido
  int ventana = pasos_por_perchero / 4;
  if (digitalRead(HALL_PIN) == LOW) {
    if (abs(stepperCentral.currentPosition() - POS_PERCHERO[1]) < ventana) {
      stepperCentral.setCurrentPosition(0);
      perchero_actual = 1;
      digitalWrite(ENABLE_PIN, HIGH);
      Serial.println("Ya estaba en el imán, referencia asumida.");
      return true;
    }
  }
  hallFlancoDetectado = false;
  stepperCentral.move(ventana);
  while (stepperCentral.distanceToGo() != 0) {
    stepperCentral.run();
    if (hallFlancoDetectado) {
      stepperCentral.stop();
      //while (stepperCentral.isRunning()) stepperCentral.run();
      stepperCentral.setCurrentPosition(0);
      perchero_actual = 1;
      digitalWrite(ENABLE_PIN, HIGH);
      Serial.println("Referencia encontrada");
      return true;
    }
  }
  hallFlancoDetectado = false;
  stepperCentral.move(-2 * ventana);
  while (stepperCentral.distanceToGo() != 0) {
    stepperCentral.run();
    if (hallFlancoDetectado) {
      stepperCentral.stop();
      // while (stepperCentral.isRunning()) stepperCentral.run();
      stepperCentral.setCurrentPosition(0);
      perchero_actual = 1;
      digitalWrite(ENABLE_PIN, HIGH);
      Serial.println("Referencia encontrada");
      return true;
    }
  }
  
  Serial.println("ERROR: Hall no encontrado");
  digitalWrite(ENABLE_PIN, HIGH);

  stepperCentral.move(ventana);
  while (stepperCentral.distanceToGo() != 0) {
    stepperCentral.run();
    if (hallFlancoDetectado) {
      stepperCentral.stop();
      // while (stepperCentral.isRunning()) stepperCentral.run();
      stepperCentral.setCurrentPosition(0);
      perchero_actual = 1;
      digitalWrite(ENABLE_PIN, HIGH);
      Serial.println("Referencia encontrada");
      return true;
    }
  }
  return false;
}

void moverCentral(int destino) {
  if (movimientosCentral >= 2) {
    Serial.println("Re-homing periódico");
    digitalWrite(ENABLE_PIN, LOW);
    stepperCentral.moveTo(POS_PERCHERO[1]);
    while (stepperCentral.distanceToGo() != 0) stepperCentral.run();
    perchero_actual = 1;
    homingLocal();
    movimientosCentral = 0;
    digitalWrite(ENABLE_PIN, LOW);
  }
  if ((perchero_actual == 1 && destino == 2) || (perchero_actual == 2 && destino == 1)) {
    moverCentral(0);
    moverCentral(destino);
    return;
  }
  digitalWrite(ENABLE_PIN, LOW);
  //Serial.println("Se activo");
  stepperCentral.moveTo(POS_PERCHERO[destino]);
  while (stepperCentral.distanceToGo() != 0) stepperCentral.run();
  perchero_actual = destino;
  movimientosCentral++;
  digitalWrite(ENABLE_PIN, HIGH);
}

void moverPerchero(int perchero, int posicion) {
  int delta = posicion - posicion_actual[perchero];
  int pasos = delta * pasos_por_prenda;
  motoresPercheros.step(pasos);
  for (int i = 0; i < 3; i++) {
    posicion_actual[i] += delta;
    while (posicion_actual[i] >= 5) posicion_actual[i] -= 5;
    while (posicion_actual[i] < 0) posicion_actual[i] += 5;
  }
}

void leerUART() {
  static uint8_t buffer[3];
  static int index = 0;
  static bool leyendo = false;
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (b == 0xAA) {
      leyendo = true;
      index = 0;
      continue;
    }
    if (leyendo) {
      if (index >= 3) {
        leyendo = false;
        index = 0;
        continue;
      }
      buffer[index++] = b;
      if (index == 3) {
        if (buffer[2] == 0x55) {
          int perchero = buffer[0];
          int posicion = buffer[1];
          if (perchero <= 2 && posicion <= 4) {
            moverCentral(perchero);
            moverPerchero(perchero, posicion);
          }
        }
        leyendo = false;
      }
    }
  }
}

#ifdef TEST_MOTORES
void pruebaMotores() {

  Serial.println("INICIANDO PRUEBA DE MOTORES");
  // Mueve del 0 al 1, luego al 2
  for (int p = 0; p < 3; p++) {
    Serial.printf("Moviendo a perchero %d\n", p);
    moverCentral(p);
    delay(1500);
  }
  // Probar rotación de prendas en el perchero actual (perchero 2 tras el bucle anterior)
  Serial.println("Probando rotación de prendas en perchero 2");
  for (int pos = 0; pos < 5; pos++) { 
    Serial.printf("Llevando prenda a posición %d\n", pos);
    moverPerchero(2, pos);
    delay(10000);
    // if (pos==4) pos = 0; // Provoca que sea infinito
  }
  // Regresar a perchero 0, posición 0
  // Serial.println("Regresando a posición inicial (perchero 0, prenda 0)");
  // moverCentral(0);
  //moverPerchero(0, 0);
  Serial.println("PRUEBA DE MOTORES COMPLETADA");
}
#endif

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  pinMode(DIP0, INPUT_PULLUP);
  pinMode(DIP1, INPUT_PULLUP);
  pinMode(DIP2, INPUT_PULLUP);
  pinMode(BTN, INPUT_PULLUP);
  attachInterrupt(BTN, ISR_boton, FALLING);
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(HALL_PIN, ISR_hall, FALLING);

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);

  stepperCentral.setMaxSpeed(1200);
  stepperCentral.setAcceleration(600);
  motoresPercheros.setSpeed(10);

  Serial.println("Presione Boton");
  while (!botonFlag) {}
  botonFlag = false;

  if (!homingLocal()) {
    Serial.println("ADVERTENCIA: homing inicial fallido");
  }

}

void loop() {
  #ifdef TEST_MOTORES
    pruebaMotores();  // Ejecuta la secuencia de prueba una sola vez
  #endif

  static unsigned long ultimoBoton = 0;
  if (botonFlag) {
    Serial.println("Boton presionado");
    botonFlag = false;
    unsigned long ahora = millis();
    if (ahora - ultimoBoton > 200) {
      Serial.println("Acceso");
      ultimoBoton = ahora;
      leerUART();
      enviarCaracteristicas();
    }
  }
}