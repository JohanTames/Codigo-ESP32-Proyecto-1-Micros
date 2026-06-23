/*
 * =====================================================
 *  Prueba UART ESP32 <-> Raspberry Pi 5
 *  Protocolo de trama fija de 4 bytes:
 *
 *    [ 0xAA ] [ DATA1 ] [ DATA2 ] [ 0x55 ]
 *      inicio   byte1    byte2     fin
 *
 * CONEXIÓN:
 *   ESP32 GPIO17 (TX2) ---> Raspberry Pi GPIO15 (RX / Pin 10)
 *   ESP32 GPIO16 (RX2) ---> Raspberry Pi GPIO14 (TX / Pin 8)
 *   ESP32 GND          ---> Raspberry Pi GND
 * =====================================================
 */

#define RXD2       16
#define TXD2       17
#define BAUD_RATE  115200
#define LED_PIN    2

// Bytes de inicio y fin de trama
#define BYTE_START  0xAA
#define BYTE_END    0x55
#define FRAME_SIZE  4

// Máquina de estados para la lectura
enum Estado {
  ESPERANDO_INICIO,
  LEYENDO_DATA1,
  LEYENDO_DATA2,
  ESPERANDO_FIN
};

Estado estado = ESPERANDO_INICIO;
uint8_t data1 = 0;
uint8_t data2 = 0;
unsigned long tramasOK    = 0;
unsigned long tramаsError = 0;

// ===================================================
void setup() {
  Serial.begin(BAUD_RATE);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  pinMode(LED_PIN, OUTPUT);

  Serial.println("Enviando datos UART...");
  Serial2.write(0x34);
  Serial2.write(0x00);
  Serial2.write(0x11);
  Serial2.write(0x00);
  Serial2.write(0x12);
  Serial2.write(0x14);

  Serial2.write(0xAA);
  Serial2.write(0x00);
  Serial2.write(0x03);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x55);

  Serial2.write(0x93);
  Serial2.write(0x04);
  Serial2.write(0x11);
  Serial2.write(0x00);
  Serial2.write(0x12);
  Serial2.write(0x14);

  Serial.println("=========================================");
  Serial.println("  ESP32 - Lectura de tramas UART 4 bytes");
  Serial.println("=========================================");
  Serial.println("  Formato: AA [D1] [D2] 55");
  Serial.println("  Esperando tramas de la Raspberry Pi...");
  Serial.println("=========================================\n");
}

// ===================================================
void loop() {


  while (Serial2.available()) {
    uint8_t byteRecibido = Serial2.read();
    Serial.printf("  Recibe: 0x%02X\n", byteRecibido);
    switch (estado) {

      case ESPERANDO_INICIO:
        if (byteRecibido == BYTE_START) {
          estado = LEYENDO_DATA1;
        }
        // Si no es 0xAA, se ignora y se sigue esperando
        break;

      case LEYENDO_DATA1:
        data1 = byteRecibido;
        estado = LEYENDO_DATA2;
        break;

      case LEYENDO_DATA2:
        data2 = byteRecibido;
        estado = ESPERANDO_FIN;
        break;

      case ESPERANDO_FIN:
        if (byteRecibido == BYTE_END) {
          // Trama válida recibida
          tramasOK++;
          digitalWrite(LED_PIN, HIGH);

          Serial.println("---- Trama recibida ----");
          Serial.printf("  Inicio : 0x%02X\n", BYTE_START);
          Serial.printf("  Data 1 : 0x%02X  (%d)\n", data1, data1);
          Serial.printf("  Data 2 : 0x%02X  (%d)\n", data2, data2);
          Serial.printf("  Fin    : 0x%02X\n", BYTE_END);
          Serial.printf("  Tramas OK: %lu\n\n", tramasOK);

          // Responder a la Raspberry Pi con ACK
          uint8_t ack[4] = { BYTE_START, data1, data2, BYTE_END };
          //Serial2.write(ack, 4);

          delay(50);
          digitalWrite(LED_PIN, LOW);

        } else {
          // Byte de fin incorrecto — trama inválida
          tramаsError++;
          Serial.printf("[ERROR] Fin de trama incorrecto: 0x%02X (esperado 0x55) | Errores: %lu\n",
                        byteRecibido, tramаsError);

          // Si el byte inesperado es 0xAA, podría ser inicio de nueva trama
          if (byteRecibido == BYTE_START) {
            estado = LEYENDO_DATA1;
            break;
          }
        }

        estado = ESPERANDO_INICIO;
        break;
    }
  }
}
