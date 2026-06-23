// La ESP hace 4 cosas:
// Lee DIP switch y botón
// Envía lo que leyó a la rasp
// Recibe de la rasp la posición a la que se debe mover
// Reinicia posición con sensor efecto Hall

// ESP a Rasp:
// AA [tipo][color][tela][talla][fit] 55

// Rasp a ESP:
// AA [perchero][posición] 55

#include <AccelStepper.h> // Librería para el motor central, controlo aceleración. Si arranca instantáneamente pierde pasos, vibra o se traba.
#include <Stepper.h> // Para motores de los vértices

// UART
#define RXD2 16
#define TXD2 17

// DIP y botón, ahora con pull-up interno (se eliminan resistencias externas)
#define DIP0 32
#define DIP1 33
#define DIP2 25
#define BTN 0 // Botón IO0, no tocar mientras enciende ESP, tiene pull-up interno

// Sensor Hall
#define HALL_PIN 26

// DRV8825
// Configurado en 1/16 microstep
// M0 = LOW Por defecto son LOW, no conectar a nada
// M1 = LOW
// M2 = HIGH

#define STEP_PIN 14
#define DIR_PIN 27
#define ENABLE_PIN 13

// Variables ISR
volatile bool botonFlag = false; // Como se cambia dentro de la ISR, puede tener cambios bruscos fuera del flujo normal

// Detecta el flanco de bajada del sensor Hall
volatile bool hallFlancoDetectado = false;

// Motor central
// Crea objeto perteneciente a librería AccelStepper
// DRIVER dice que es tipo STEP/DIR (el controlador se maneja con pulsos de dirección y luego pasos)
AccelStepper stepperCentral(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// 28BYJ
// Pasos internos del 28BYJ-48
#define STEPS_28BYJ 2048

// Todos los motores de los percheros giran al mismo tiempo
// Se utilizan los mismos pines para los tres ULN2003
// Si gira al revés intercambiar IN2 e IN3
Stepper motoresPercheros(STEPS_28BYJ, 4, 5, 18, 19);

// Variables
// El sistema inicia asumiendo que está cerca del perchero 1
// El Hall corrige la referencia angular real
int perchero_actual = 1;
int posicion_actual[3] = {0, 0, 0};

// NEMA17:
// 200 pasos/rev
// 1/16 microstep
// 3200 microsteps/rev
// Como hay 3 percheros:
// 3200/3 = 1066.67
int pasos_por_perchero = 1067;

// Posiciones absolutas de los percheros respecto al perchero 1
// El perchero 1 ahora es la referencia física del Hall
// Pero el rango sigue siendo respecto al perchero 0
const int POS_PERCHERO[3] = {
    -pasos_por_perchero, // perchero 0 (-120° respecto al Hall)
    0, // perchero 1 (origen físico del Hall)
    pasos_por_perchero // perchero 2 (-240°)
};

// 2048/5 = 409.6
int pasos_por_prenda = 410;

// Cada ciertos movimientos se hace re-homing automático
int movimientosCentral = 0;
int contadorMovPerchero = 0;

// Funciones ISR

// ISR para leer botón
// IRAM_ATTR hace que se almacene en RAM y no en Flash, por eso volatile
void IRAM_ATTR ISR_boton() {
  botonFlag = true;
}

// ISR del sensor Hall
// Se detecta el flanco HIGH a LOW
void IRAM_ATTR ISR_hall() {
  hallFlancoDetectado = true;
}

// Funciones

// DIP
// Retorna número de 3 bits (0 a 7) con ceros a la izquierda
// uint8_t es entero sin signo de 8 bits
uint8_t leerDIP() {
  return (digitalRead(DIP2) << 2) |
         (digitalRead(DIP1) << 1) |
          digitalRead(DIP0);
}

// UART TX
void enviarCaracteristicas() {
  static uint8_t data[5]; // Almacena caracteristicas del dip
  static int index = 0;

  uint8_t valor = leerDIP();
  Serial.printf("Botón presionado, valor dip leido: %d\n", valor);

  // 6 y 7 no se usan, 0 es no elegir.
  if (valor <= 5) data[index++] = valor;
  else data[index++] = 0;

  Serial.print("Carac ");
  Serial.print(index - 1);
  Serial.print(": ");
  Serial.println(data[index - 1]);

  if (index == 5) {
    // UART es un flujo continuo de bytes
    // AA y 55 permiten sincronizar el inicio y final del paquete
    Serial2.write(0xAA);

    for (int i = 0; i < 5; i++) {
      Serial2.write(data[i]);
    }

    Serial2.write(0x55);

    Serial.println("Caracteristicas enviadas");

    index = 0;
  }
}

// Homing local
// Busca el Hall solo cerca de la posición esperada (perchero 1), en una ventana de 30°
// Evita vueltas completas de 360°
bool homingLocal() {
  Serial.println("Buscando hall...");
  digitalWrite(ENABLE_PIN, LOW); // Se activa con LOW

  // Aproximadamente 30 grados
  int ventana = pasos_por_perchero / 4;

  // Si ya detecta el imán y la posición actual está cerca de perchero 1 (por si acaso falso contacto o campo magnético externo)
    if (digitalRead(HALL_PIN) == LOW) {
        // Verificar que no estamos muy lejos del perchero 1
        if (abs(stepperCentral.currentPosition() - POS_PERCHERO[1]) < ventana) {
            stepperCentral.setCurrentPosition(0);
            perchero_actual = 1;
            digitalWrite(ENABLE_PIN, HIGH);
            Serial.println("Ya estaba en el imán, referencia asumida.");
            return true;
        } else {
            // Está LOW pero muy lejos, probablemente es un fallo, hacer búsqueda real
            Serial.println("Imán detectado pero posición incoherente, buscando...");
        }
    }

  hallFlancoDetectado = false;

  // Buscar horario
  stepperCentral.move(ventana);

  while (stepperCentral.distanceToGo() != 0) {
    stepperCentral.run(); // Un paso a la vez, es no bloqueante

    if (hallFlancoDetectado) {
      stepperCentral.stop(); // No se detiene inmediatamente

      // Quito este bucle para que se frene justo cuando detecta el imán, de lo contrario se pasa por la desaceleración
      // while (stepperCentral.isRunning()) { // Espera a que termine de parar
      //   stepperCentral.run();
      // }

      stepperCentral.setCurrentPosition(0);
      perchero_actual = 1; // El imán está en el perchero 1

      Serial.println("Referencia encontrada");
      digitalWrite(ENABLE_PIN, HIGH);
      return true;
    }
  }

  hallFlancoDetectado = false;

  // Buscar antihorario si no encontró en horario
  stepperCentral.move(-2 * ventana);

  while (stepperCentral.distanceToGo() != 0) {
    stepperCentral.run();

    if (hallFlancoDetectado) {
      stepperCentral.stop();

      // while (stepperCentral.isRunning()) {
      //   stepperCentral.run();
      // }

      stepperCentral.setCurrentPosition(0);
      perchero_actual = 1;

      Serial.println("Referencia encontrada");
      digitalWrite(ENABLE_PIN, HIGH);
      return true;
    }
  }

  // Si no detecta regresa a la posición donde debería estar
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

  Serial.println("ERROR: Hall no encontrado");
  digitalWrite(ENABLE_PIN, HIGH);
  return false;
}

// Mover central con posiciones absolutas, evita giro 360°
void moverCentral(int destino) {

  // Re-homing periódico (cada ciertos movimientos del central)
  if (movimientosCentral >= 5) {
    Serial.println("Re-homing periódico");

    digitalWrite(ENABLE_PIN, LOW);

    // Ir a perchero 1 porque ahora es la referencia física
    stepperCentral.moveTo(POS_PERCHERO[1]);

    while (stepperCentral.distanceToGo() != 0) {
      stepperCentral.run();
    }

    perchero_actual = 1;

    bool exito = homingLocal();

    if (!exito) {
      Serial.println("Falló re-homing");
    }

    movimientosCentral = 0;

    // homingLocal() deshabilita el driver, lo vuelvo a habilitar para el movimiento
    digitalWrite(ENABLE_PIN, LOW);
  }

  // No mover directamente entre perchero 1 y 2 para evitar giros de 360°
  if ((perchero_actual == 1 && destino == 2) || (perchero_actual == 2 && destino == 1)) {
    Serial.println("movimiento 1<->2 detectado, pasando por perchero 0 primero");
    moverCentral(0); // ir a perchero 0
    moverCentral(destino);
    return;
  }

  digitalWrite(ENABLE_PIN, LOW);

  stepperCentral.moveTo(POS_PERCHERO[destino]);

  while (stepperCentral.distanceToGo() != 0) {
    stepperCentral.run();
  }

  perchero_actual = destino;
  movimientosCentral++;

  digitalWrite(ENABLE_PIN, HIGH);
}

// Mover perchero (los 28BYJ de los vértices)
// Todos giran simultáneamente porque comparten señales
// Por simplicidad no hago muchos cambios en esta función, anteriormente manejaba los motores por separado
void moverPerchero(int perchero, int posicion) {

  // Calcular cuánto debe girar el perchero solicitado
  int delta = posicion - posicion_actual[perchero];
  int pasos = delta * pasos_por_prenda;

  // Todos los motores reciben la misma señal
  motoresPercheros.step(pasos);

  // Como todos giraron, actualizar todas las posiciones internas
  for (int i = 0; i < 3; i++) {
    posicion_actual[i] += delta;

    // Mantener posiciones entre 0 y 4
    while (posicion_actual[i] >= 5) {
      posicion_actual[i] -= 5;
    }

    while (posicion_actual[i] < 0) {
      posicion_actual[i] += 5;
    }
  }

  // Corrige error
  contadorMovPerchero++;
  if (contadorMovPerchero >= 5) {
    Serial.printf("Corrigiendo error perchero");
    motoresPercheros.step(114);  // Gira el ángulo elegido (20° = 20/360 * 2048 =aprox 114 pasos)
    contadorMovPerchero = 0;
  }
}

// UART RX
void leerUART() {
  static uint8_t buffer[3];
  static int index = 0;
  static bool leyendo = false;

  // Available devuelve cuántos bytes hay almacenados en el buffer UART
  while (Serial2.available()) {
    uint8_t b = Serial2.read(); // Extrae el byte más antiguo y disminuye le número de bytes en el buffer por 1

    // AA y 55 ayudan a sincronizar la comunicación UART
    if (b == 0xAA) {
      leyendo = true;
      index = 0;
      continue;
    }

    if (leyendo) {
      // Protección contra overflow
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

          // Validaciones de seguridad
          if (perchero > 2) {
            Serial.println("Perchero invalido");
            leyendo = false;
            continue;
          }

          if (posicion > 4) {
            Serial.println("Posicion invalida");
            leyendo = false;
            continue;
          }

          // imprimir comando recibido
          Serial.printf("Recibido de rasp: perchero %d, posicion %d\n", perchero, posicion);

          moverCentral(perchero);
          moverPerchero(perchero, posicion);
        }

        leyendo = false;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // DIP (pull-up internos activados)
  pinMode(DIP0, INPUT_PULLUP);
  pinMode(DIP1, INPUT_PULLUP);
  pinMode(DIP2, INPUT_PULLUP);

  // Botón (pull-up interno)
  pinMode(BTN, INPUT_PULLUP);
  attachInterrupt(BTN, ISR_boton, FALLING); // HIGH a LOW

  // Hall (pull-up interno)
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(HALL_PIN, ISR_hall, FALLING);

  // DRV8825
  pinMode(ENABLE_PIN, OUTPUT);

  // LOW = habilitado, HIGH = deshabilitado
  digitalWrite(ENABLE_PIN, HIGH);

  // Configuración motor central
  stepperCentral.setMaxSpeed(1200);
  stepperCentral.setAcceleration(600);

  // Configuración 28BYJ
  motoresPercheros.setSpeed(10);

  // Hace homing inicial con advertencia si falla
  if (!homingLocal()) {
    Serial.println("ADVERTENCIA: homing inicial fallido, el sistema puede estar descalibrado");
  }
}

void loop() {
  // Interrupción botón
  static unsigned long ultimoBoton = 0;

  if (botonFlag) {
    botonFlag = false;

    unsigned long ahora = millis();

    if (ahora - ultimoBoton > 200) { // Debounce, solo guarda el dato si pasaron al menos 200ms desde la última vez que se presionó
      ultimoBoton = ahora;
      enviarCaracteristicas();
    }
  }

  leerUART();
}
