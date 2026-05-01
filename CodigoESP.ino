// Pines DIP (3 bits)
#define DIP0 32
#define DIP1 33
#define DIP2 25

// Botón
#define BTN 26

// UART (puedes usar Serial2 si quieres separar debug)
#define RXD2 16
#define TXD2 17

uint8_t caracteristicas[5]; // tipo, color, tela, talla, fit
int index_carac = 0;

bool lastBtnState = HIGH;

uint8_t leerDIP() {
  uint8_t b0 = digitalRead(DIP0);
  uint8_t b1 = digitalRead(DIP1);
  uint8_t b2 = digitalRead(DIP2);

  return (b2 << 2) | (b1 << 1) | b0; // valor 0–7
}

void enviarUART() {
  Serial2.write(0xAA);

  for (int i = 0; i < 5; i++) {
    Serial2.write(caracteristicas[i]);
  }

  Serial2.write(0x55);
}

void setup() {
  pinMode(DIP0, INPUT_PULLUP);
  pinMode(DIP1, INPUT_PULLUP);
  pinMode(DIP2, INPUT_PULLUP);

  pinMode(BTN, INPUT_PULLUP);

  Serial.begin(115200);      // debug
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  Serial.println("Sistema listo");
}

void loop() {
  bool btnState = digitalRead(BTN);

  // detectar flanco de bajada (botón presionado)
  if (lastBtnState == HIGH && btnState == LOW) {
    delay(50); // debounce

    uint8_t valor = leerDIP();

    if (valor <= 5) { // válido
      caracteristicas[index_carac] = valor;
    } else {
      caracteristicas[index_carac] = 0; // ignorar 6 y 7
    }

    Serial.print("Caracteristica ");
    Serial.print(index_carac);
    Serial.print(": ");
    Serial.println(caracteristicas[index_carac]);

    index_carac++;

    if (index_carac == 5) {
      enviarUART();
      Serial.println("Datos enviados por UART");

      index_carac = 0; // reiniciar
    }
  }

  lastBtnState = btnState;
}