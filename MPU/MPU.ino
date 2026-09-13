#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <WiFi.h>
#include <esp_now.h>
#include <math.h>
#include <string.h>

MPU6050 mpu;

// Offsets do giroscopio, em unidades brutas.
float gyroXXOffset = 0.0f;
float gyroZOffset = 0.0f;

float roll = 0.0f;
float pitch = 0.0f;

const float ALPHA = 0.96f;

const float LIMITE_ATIVACAO = 10.0f;
const float LIMITE_DESATIVACAO = 7.0f;

// Ajuste somente se o gesto comandar o sentido contrario.
// Esses sinais alteram os comandos, sem modificar o filtro.
const float SINAL_FRENTE = 1.0f;
const float SINAL_DIREITA = 1.0f;

unsigned long ultimoTempo = 0;

// MAC da interface Station do ESP32 receptor.
uint8_t enderecoMAC[6] = {
  0xA4, 0xF0, 0x0F, 0x83, 0x22, 0x78
};

// Mantenha esta mesma definicao no receptor.
// Nao altere apenas um dos lados para enum : uint8_t.
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

const unsigned long INTERVALO_ENVIO = 100;
unsigned long ultimoEnvio = 0;
bool primeiroEnvio = true;

// Prototipos explicitos para evitar problemas com tipos
// personalizados no preprocessamento da Arduino IDE.
void interromper(const char *mensagem);
void calibrarMPU();
void inicializarAngulos();
void enviarCodigo(Codigo codigo);
const char *determinarDirecao(float pitchComando, float rollComando);

void interromper(const char *mensagem) {
  Serial.println(mensagem);

  // while (true) {
  //   delay(1000);
  // }
}

void calibrarMPU() {
  Serial.println("\n=================================");
  Serial.println("       CALIBRANDO MPU6050");
  Serial.println("=================================");
  Serial.println("Mantenha o sensor parado.");
  Serial.println("Posicao neutra: Y positivo na vertical.");

  delay(2000);

  const int NUM_AMOSTRAS = 2000;

  int64_t somaGX = 0;
  int64_t somaGZ = 0;

  for (int i = 0; i < NUM_AMOSTRAS; i++) {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    somaGX += gx;
    somaGZ += gz;

    delay(2);
  }

  gyroXOffset = (float)somaGX / NUM_AMOSTRAS;
  gyroZOffset = (float)somaGZ / NUM_AMOSTRAS;

  Serial.println("Calibracao concluida!");

  Serial.print("Gyro X offset: ");
  Serial.println(gyroXOffset, 2);

  Serial.print("Gyro Z offset: ");
  Serial.println(gyroZOffset, 2);
}

void inicializarAngulos() {
  int16_t axRaw, ayRaw, azRaw;
  int16_t gxRaw, gyRaw, gzRaw;

  mpu.getMotion6(
    &axRaw, &ayRaw, &azRaw,
    &gxRaw, &gyRaw, &gzRaw
  );

  float ax = (float)axRaw;
  float ay = (float)ayRaw;
  float az = (float)azRaw;

  roll = atan2f(az, ay) * 180.0f / PI;

  pitch = atan2f(
    -ax,
    sqrtf(ay * ay + az * az)
  ) * 180.0f / PI;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("MPU6050 - CONTROLE DO CARRINHO");

  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    interromper("ERRO: MPU6050 nao encontrado!");
  }

  // Garante a escala usada na conversao do giroscopio.
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  delay(100);
  calibrarMPU();

  if (!WiFi.mode(WIFI_STA)) {
    interromper("ERRO ao ativar Wi-Fi Station!");
  }

  if (esp_now_init() != ESP_OK) {
    interromper("ERRO ao inicializar ESP-NOW!");
  }

  esp_now_peer_info_t infoMotores = {};

  memcpy(infoMotores.peer_addr, enderecoMAC, 6);

  // Usa o canal atual do transmissor.
  // O receptor precisa estar no mesmo canal.
  infoMotores.channel = 0;
  infoMotores.ifidx = WIFI_IF_STA;
  infoMotores.encrypt = false;

  if (esp_now_add_peer(&infoMotores) != ESP_OK) {
    interromper("ERRO ao adicionar o receptor!");
  }

  // Inicializa o filtro e o tempo somente depois do setup.
  inicializarAngulos();
  ultimoTempo = micros();

  enviarCodigo(PARAR);

  Serial.println("Sistema pronto!");
}

void loop() {
  int16_t axRaw, ayRaw, azRaw;
  int16_t gxRaw, gyRaw, gzRaw;

  mpu.getMotion6(
    &axRaw, &ayRaw, &azRaw,
    &gxRaw, &gyRaw, &gzRaw
  );

  unsigned long agora = micros();

  float dt = (agora - ultimoTempo) / 1000000.0f;
  ultimoTempo = agora;

  float ax = (float)axRaw;
  float ay = (float)ayRaw;
  float az = (float)azRaw;

  // Sensibilidade para a escala de +/-250 graus/s.
  float gyroX = ((float)gxRaw - gyroXOffset) / 131.0f;
  float gyroZ = ((float)gzRaw - gyroZOffset) / 131.0f;

  float rollAccel = atan2f(az, ay) * 180.0f / PI;

  float pitchAccel = atan2f(
    -ax,
    sqrtf(ay * ay + az * az)
  ) * 180.0f / PI;

  if (dt <= 0.0f || dt > 0.1f) {
    // Reinicializa o filtro se houver uma pausa longa.
    roll = rollAccel;
    pitch = pitchAccel;
  } else {
    roll =
      ALPHA * (roll + gyroX * dt) +
      (1.0f - ALPHA) * rollAccel;

    // Para as formulas acima, com Y positivo na vertical,
    // pitch usa o sinal negativo do giroscopio Z.
    pitch =
      ALPHA * (pitch - gyroZ * dt) +
      (1.0f - ALPHA) * pitchAccel;
  }

  const char *direcao = determinarDirecao(
    SINAL_FRENTE * pitch,
    SINAL_DIREITA * roll
  );

  // Chamada continua para permitir o heartbeat.
  enviarCodigo(atualCodigo);

  Serial.print("Pitch: ");
  Serial.print(pitch, 1);

  Serial.print(" | Roll: ");
  Serial.print(roll, 1);

  Serial.print(" | Direcao: ");
  Serial.println(direcao);

  delay(10);
}

const char *determinarDirecao(
  float pitchComando,
  float rollComando
) {
  bool estavaFrente =
    atualCodigo == FRENTE ||
    atualCodigo == DIREITA_FRENTE ||
    atualCodigo == ESQUERDA_FRENTE;

  bool estavaTras =
    atualCodigo == TRAS ||
    atualCodigo == DIREITA_TRAS ||
    atualCodigo == ESQUERDA_TRAS;

  bool estavaDireita =
    atualCodigo == DIREITA_FRENTE ||
    atualCodigo == DIREITA_TRAS;

  bool estavaEsquerda =
    atualCodigo == ESQUERDA_FRENTE ||
    atualCodigo == ESQUERDA_TRAS;

  bool frente = pitchComando > (
    estavaFrente ? LIMITE_DESATIVACAO : LIMITE_ATIVACAO
  );

  bool tras = pitchComando < -(
    estavaTras ? LIMITE_DESATIVACAO : LIMITE_ATIVACAO
  );

  bool direita = rollComando > (
    estavaDireita ? LIMITE_DESATIVACAO : LIMITE_ATIVACAO
  );

  bool esquerda = rollComando < -(
    estavaEsquerda ? LIMITE_DESATIVACAO : LIMITE_ATIVACAO
  );

  if (frente && direita) {
    atualCodigo = DIREITA_FRENTE;
    return "FRENTE DIREITA";
  }

  if (frente && esquerda) {
    atualCodigo = ESQUERDA_FRENTE;
    return "FRENTE ESQUERDA";
  }

  if (frente) {
    atualCodigo = FRENTE;
    return "FRENTE";
  }

  if (tras && direita) {
    atualCodigo = DIREITA_TRAS;
    return "TRAS DIREITA";
  }

  if (tras && esquerda) {
    atualCodigo = ESQUERDA_TRAS;
    return "TRAS ESQUERDA";
  }

  if (tras) {
    atualCodigo = TRAS;
    return "TRAS";
  }

  // Inclinacao apenas lateral continua resultando em PARAR.
  atualCodigo = PARAR;
  return "PARADO";
}

void enviarCodigo(Codigo codigo) {
  unsigned long agora = millis();

  // Mudou o comando: envia na proxima passagem do loop.
  // Comando igual: repete apos aproximadamente 100 ms.
  if (!primeiroEnvio &&
      codigo == ultimoCodigo &&
      agora - ultimoEnvio < INTERVALO_ENVIO) {
    return;
  }

  esp_err_t resultado = esp_now_send(
    enderecoMAC,
    reinterpret_cast<const uint8_t *>(&codigo),
    sizeof(codigo)
  );

  if (resultado == ESP_OK) {
    ultimoCodigo = codigo;
    ultimoEnvio = agora;
    primeiroEnvio = false;
  }
}