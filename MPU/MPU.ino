#include <Wire.h>
#include "I2Cdev.h"
#include <WiFi.h>
#include <esp_now.h>
#include "MPU6050.h"

MPU6050 mpu;

long gyroXOffset = 0;
long gyroYOffset = 0;
long gyroZOffset = 0;

long accelXOffset = 0;
long accelYOffset = 0;
long accelZOffset = 0;

float roll = 0;
float pitch = 0;

const float ALPHA = 0.96;

const float LIMITE_ATIVACAO = 10.0;
const float LIMITE_DESATIVACAO = 7.0;

unsigned long ultimoTempo;


uint8_t enderecoMAC[6] =  {0xA4,0xF0,0x0F,0x83,0x22,0x78};

typedef enum {
  FRENTE = 0x01,
  TRAS = 0x02,
  DIREITA_FRENTE = 0x05,
  DIREITA_TRAS = 0x06,
  ESQUERDA_FRENTE = 0x07,
  ESQUERDA_TRAS = 0x08,
  PARAR = 0x09
} Codigo;

Codigo ultimoCodigo = PARAR;
Codigo atualCodigo = PARAR;

void calibrarMPU() {

  Serial.println();
  Serial.println("=================================");
  Serial.println("       CALIBRANDO MPU6050");
  Serial.println("=================================");
  Serial.println();

  Serial.println("NAO MOVA O SENSOR!");
  Serial.println("Aguarde...");

  delay(2000);

  const int NUM_AMOSTRAS = 2000;

  long somaGX = 0;
  long somaGY = 0;
  long somaGZ = 0;

  long somaAX = 0;
  long somaAY = 0;
  long somaAZ = 0;

  for (int i = 0; i < NUM_AMOSTRAS; i++) {

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    somaGX += gx;
    somaGY += gy;
    somaGZ += gz;

    somaAX += ax;
    somaAY += ay;
    somaAZ += az;

    delay(2);
  }

  gyroXOffset = somaGX / NUM_AMOSTRAS;
  gyroYOffset = somaGY / NUM_AMOSTRAS;
  gyroZOffset = somaGZ / NUM_AMOSTRAS;

  accelXOffset = somaAX / NUM_AMOSTRAS;
  accelYOffset = somaAY / NUM_AMOSTRAS;
  accelZOffset = somaAZ / NUM_AMOSTRAS;

  Serial.println();
  Serial.println("CALIBRACAO CONCLUIDA!");
  Serial.println();

  Serial.print("Gyro X offset: ");
  Serial.println(gyroXOffset);

  Serial.print("Gyro Y offset: ");
  Serial.println(gyroYOffset);

  Serial.print("Gyro Z offset: ");
  Serial.println(gyroZOffset);

  Serial.println();

  Serial.print("Accel X media: ");
  Serial.println(accelXOffset);

  Serial.print("Accel Y media (deve ficar proximo de ~16384, pois agora Y e o eixo vertical): ");
  Serial.println(accelYOffset);

  Serial.print("Accel Z media: ");
  Serial.println(accelZOffset);

  Serial.println();

  delay(2000);
}

void setup() {

  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("=================================");
  Serial.println("       MPU6050 - CONTROLE");
  Serial.println("=================================");

  Wire.begin();

  mpu.initialize();

  if (!mpu.testConnection()) {

    Serial.println("ERRO: MPU6050 NAO ENCONTRADO!");

    while (1) {
      delay(1000);
    }
  }

  Serial.println("MPU6050 conectado!");

  delay(1000);

  calibrarMPU();

  // Inicializa o tempo
  ultimoTempo = micros();

  Serial.println("Sistema pronto!");
  Serial.println();

  WiFi.mode(WIFI_STA);

  if(esp_now_init() != ESP_OK) {
    Serial.println("ERRO DE COMUNICAÇÂO");
  }

  esp_now_peer_info_t infoMotores = {};

  memcpy(infoMotores.peer_addr, enderecoMAC, 6);

  infoMotores.channel = 0;
  infoMotores.encrypt = false;

  if(esp_now_add_peer(&infoMotores) != ESP_OK) {
    Serial.println("ERRO COMUNICACAO");
  }
}


void loop() {

  int16_t axRaw, ayRaw, azRaw;
  int16_t gxRaw, gyRaw, gzRaw;

  mpu.getMotion6(
    &axRaw,
    &ayRaw,
    &azRaw,
    &gxRaw,
    &gyRaw,
    &gzRaw
  );

  float gx = gxRaw - gyroXOffset;
  float gz = gzRaw - gyroZOffset;

  float ax = axRaw;
  float ay = ayRaw;
  float az = azRaw;

  unsigned long agora = micros();

  float dt = (agora - ultimoTempo) / 1000000.0;

  ultimoTempo = agora;

  if (dt <= 0 || dt > 0.1) {
    dt = 0.01;
  }

  float gyroX = gx / 131.0; 
  float gyroZ = gz / 131.0; 

  float rollAccel =
      atan2(az, ay) * 180.0 / PI;

  float pitchAccel =
      atan2(
        -ax,
        sqrt(ay * ay + az * az)
      )
      * 180.0 / PI;

  roll =
      ALPHA * (roll + gyroX * dt)
      + (1.0 - ALPHA) * rollAccel;

  pitch =
      ALPHA * (pitch + gyroZ * dt)  
      + (1.0 - ALPHA) * pitchAccel;

  String direcao = determinarDirecao(
      pitch,
      roll
  );

  Serial.print("Pitch: ");
  Serial.print(pitch, 1);

  Serial.print(" | Roll: ");
  Serial.print(roll, 1);

  Serial.print(" | Direcao: ");

  Serial.println(direcao);


  delay(30);
}

String determinarDirecao(float pitch, float roll) {

  bool frente;
  bool tras;
  bool direita;
  bool esquerda;

  if (atualCodigo == FRENTE) {
    frente = pitch > LIMITE_DESATIVACAO;
  } 
  else {
    frente = pitch > LIMITE_ATIVACAO;
  }

  if (atualCodigo == TRAS) {
    tras = pitch < -LIMITE_DESATIVACAO;
  } 
  else {
    tras = pitch < -LIMITE_ATIVACAO;
  }

  if (atualCodigo == DIREITA_FRENTE ||
      atualCodigo == DIREITA_TRAS) {
    direita = roll > LIMITE_DESATIVACAO;
  } 
  else {
    direita = roll > LIMITE_ATIVACAO;
  }

  if (atualCodigo == ESQUERDA_FRENTE ||
      atualCodigo == ESQUERDA_TRAS) {
    esquerda = roll < -LIMITE_DESATIVACAO;
  } 
  else {
    esquerda = roll < -LIMITE_ATIVACAO;
  }

  if (frente && direita) {
    atualCodigo = DIREITA_FRENTE;
    enviarCodigo(atualCodigo);
    return "FRENTE DIREITA";
  }

  if (frente && esquerda) {
    atualCodigo = ESQUERDA_FRENTE;
    enviarCodigo(atualCodigo);
    return "FRENTE ESQUERDA";
  }

  if (frente) {
    atualCodigo = FRENTE;
    enviarCodigo(atualCodigo);
    return "FRENTE";
  }

  if (tras && direita) {
    atualCodigo = DIREITA_TRAS;
    enviarCodigo(atualCodigo);
    return "TRAS DIREITA";
  }

  if (tras && esquerda) {
    atualCodigo = ESQUERDA_TRAS;
    enviarCodigo(atualCodigo);
    return "TRAS ESQUERDA";
  }

  if (tras) {
    atualCodigo = TRAS;
    enviarCodigo(atualCodigo);
    return "TRAS";
  }

  atualCodigo = PARAR;
  enviarCodigo(atualCodigo);
  return "PARADO";
  
}
void enviarCodigo(Codigo cod) {

  if (cod == ultimoCodigo) {
    return;
  }

  Codigo dadoEnviado = cod;
  esp_err_t resultadoComunicacao = esp_now_send(enderecoMAC, 
  (uint8_t *)&dadoEnviado, sizeof(dadoEnviado));

  if(resultadoComunicacao == ESP_OK) {
    Serial.print("Dado enviado com sucesso: 0x");
    Serial.println((uint8_t)dadoEnviado, HEX);
  }
  ultimoCodigo = cod;
}