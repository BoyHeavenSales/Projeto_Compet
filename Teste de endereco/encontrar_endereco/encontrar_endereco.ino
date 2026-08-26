#include <Wire.h>

#define TEMPOLEITURA 50

byte endereco;
byte resultadoleitura = 0;

void setup() {
  Serial.begin(115200);

  Serial.println("LEITURA DE ENDERECO I2C DO MPU6050");
  Serial.println();

  Wire.begin();
}

void loop() {

  for (int i = 0; i < 128; i++) {

    endereco = i;

    Wire.beginTransmission(endereco);
    resultadoleitura = Wire.endTransmission();

    if (resultadoleitura == 0) {
      Serial.print("Endereco encontrado: 0x");
      Serial.println(endereco, HEX);
    }

    delay(TEMPOLEITURA);
  }

  Serial.println("Leitura finalizada.");
  delay(3000);
}