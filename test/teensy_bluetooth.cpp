#include <SoftwareSerial.h>

SoftwareSerial BTSerial(2, 3); // RX, TX pins del módulo Bluetooth

void setup() {
  Serial.begin(9600); // Inicializa la comunicación serie a través del USB
  BTSerial.begin(9600); // Inicializa la comunicación serie a través del Bluetooth

  Serial.println("Teensy está listo. Ingresa un mensaje:");
}

void loop() {
  // Leer datos desde el Monitor Serie y enviarlos al Bluetooth
  if (Serial.available() > 0) {
    char receivedChar = Serial.read();
    BTSerial.write(receivedChar);
  }

  // Leer datos desde el Bluetooth y enviarlos al Monitor Serie
  if (BTSerial.available() > 0) {
    char receivedChar = BTSerial.read();
    Serial.print(receivedChar);
  }
}