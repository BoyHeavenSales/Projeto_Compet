#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
#include <string.h>

const uint8_t ENDERECO_MPU = 0x68;  // AD0 baixo; use 0x69 se AD0 alto.
const uint8_t CANAL_WIFI = 1;       // Igual no receptor.
const uint16_t TIMEOUT_I2C = 20;
const uint32_t INTERVALO_ENVIO = 100;
const uint32_t TEMPO_NEUTRO = 500;
const float TAU_FILTRO = 0.24f;     // Equivale a alpha 0,96 com dt=10 ms.
const float LIMITE_ATIVACAO = 10.0f;
const float LIMITE_DESATIVACAO = 7.0f;
const float SINAL_FRENTE = 1.0f;
const float SINAL_DIREITA = -1.0f;
// Led de indicação de erro ou sucesso da comunicação
const int LED = 2;
// MAC Station impresso pelo receptor.
uint8_t enderecoMAC[6] = {0xA4, 0xF0, 0x0F, 0x83, 0x22, 0x78};

typedef enum {
  FRENTE = 0x01, TRAS = 0x02,
  DIREITA_FRENTE = 0x05, DIREITA_TRAS = 0x06,
  ESQUERDA_FRENTE = 0x07, ESQUERDA_TRAS = 0x08,
  PARAR = 0x09
} Codigo;

static_assert(sizeof(Codigo) == 4 && sizeof(int) == 4,
              "Protocolo requer comandos de 4 bytes");

struct Leitura {
  int16_t ax, ay, az, gx, gy, gz;
};

MPU6050 mpu(ENDERECO_MPU);
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;
float roll = 0, pitch = 0, rollNeutro = 0, pitchNeutro = 0;
Codigo atualCodigo = PARAR, ultimoCodigo = PARAR;
uint32_t ultimoTempo = 0, ultimoEnvio = 0, ultimoDiagnostico = 0;
uint32_t ultimaVerificacao = 0, inicioNeutro = 0;
bool radioPronto = false, primeiroEnvio = true, filtroPronto = false;
bool controleLiberado = false, contandoNeutro = false;
bool falhaSensor = false;
const char *erroLeitura = "nenhum";
Leitura ultimaLeituraBruta = {};
bool leituraBrutaDisponivel = false;

void interromper(const char *mensagem);
void enviarCodigo(Codigo codigo);
bool lerRegistradores(uint8_t reg, uint8_t *dados, size_t quantidade);
bool verificarConfiguracao();
bool lerMPU(Leitura &leitura);
int16_t lerInt16(const uint8_t *dados);
void angulosAcelerometro(const Leitura &leitura, float &r, float &p);
bool calibrarMPU();
void aguardarParado(uint32_t duracao);
void mostrarLeituraBruta(const Leitura &leitura);
void bloquearControle(const char *mensagem);
void atualizarFiltro(const Leitura &leitura, float dt);
const char *determinarDirecao(float pitchComando, float rollComando);

void interromper(const char *mensagem) {
  digitalWrite(LED, HIGH);
  Serial.println(mensagem);
  Serial.println("Corrija a causa e reinicie o controle.");
  while (true) {
    enviarCodigo(PARAR);
    delay(10);
  }
}

bool lerRegistradores(uint8_t reg, uint8_t *dados, size_t quantidade) {
  // Verifica endereco, leitura completa e timeout
  Wire.beginTransmission(ENDERECO_MPU);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(ENDERECO_MPU, quantidade, true) != quantidade) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (size_t i = 0; i < quantidade; ++i) {
    int valor = Wire.read();
    if (valor < 0) return false;
    dados[i] = static_cast<uint8_t>(valor);
  }
  return true;
}

bool verificarConfiguracao() {
  uint8_t identidade, energia[2], configuracao[4];
  if (!lerRegistradores(MPU6050_RA_WHO_AM_I, &identidade, 1) ||
      !lerRegistradores(MPU6050_RA_PWR_MGMT_1, energia, 2) ||
      !lerRegistradores(MPU6050_RA_SMPLRT_DIV, configuracao, 4)) return false;

  return (identidade & 0x7E) == 0x68 && energia[0] == 0x01 &&
         energia[1] == 0 && configuracao[0] == 9 &&
         configuracao[1] == MPU6050_DLPF_BW_20 &&
         configuracao[2] == 0 && configuracao[3] == 0;
}

int16_t lerInt16(const uint8_t *dados) {
  int32_t valor = (static_cast<uint16_t>(dados[0]) << 8) | dados[1];
  return static_cast<int16_t>(valor >= 32768 ? valor - 65536 : valor);
}

bool lerMPU(Leitura &leitura) {
  erroLeitura = "nenhum";
  leituraBrutaDisponivel = false;
  uint8_t dados[14];
  if (!lerRegistradores(MPU6050_RA_ACCEL_XOUT_H, dados, sizeof(dados))) {
    erroLeitura = "falha I2C ou leitura incompleta; confira SDA, SCL, GND e alimentacao";
    return false;
  }
  Leitura nova = {lerInt16(dados), lerInt16(dados + 2), lerInt16(dados + 4),
                 lerInt16(dados + 8), lerInt16(dados + 10), lerInt16(dados + 12)};
  ultimaLeituraBruta = nova;
  leituraBrutaDisponivel = true;
  float ax = nova.ax / 16384.0f, ay = nova.ay / 16384.0f;
  float az = nova.az / 16384.0f;
  float norma2 = ax * ax + ay * ay + az * az;
  // Queda livre, impacto forte e saturacao nao sao gestos confiaveis.
  if (norma2 < 0.25f || norma2 > 2.25f) {
    erroLeitura = "aceleracao fora da faixa de 0,5 a 1,5 g";
    return false;
  }
  if (abs(static_cast<int>(nova.gx)) >= 32760 ||
      abs(static_cast<int>(nova.gy)) >= 32760 ||
      abs(static_cast<int>(nova.gz)) >= 32760) {
    erroLeitura = "giroscopio saturado; mantenha o sensor parado e confira o modulo";
    return false;
  }
  leitura = nova;
  return true;
}

void angulosAcelerometro(const Leitura &leitura, float &r, float &p) {
  float ax = leitura.ax, ay = leitura.ay, az = leitura.az;
  r = atan2f(az, ay) * RAD_TO_DEG;
  p = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

void aguardarParado(uint32_t duracao) {
  uint32_t inicio = millis();
  while (static_cast<uint32_t>(millis() - inicio) < duracao) {
    enviarCodigo(PARAR);
    delay(10);
  }
}

void mostrarLeituraBruta(const Leitura &leitura) {
  float ax = leitura.ax / 16384.0f, ay = leitura.ay / 16384.0f;
  float az = leitura.az / 16384.0f;
  Serial.printf("Accel [g]: X=%.2f Y=%.2f Z=%.2f | modulo=%.2f\n",
                ax, ay, az, sqrtf(ax * ax + ay * ay + az * az));
  Serial.printf("Gyro bruto [graus/s]: X=%.2f Y=%.2f Z=%.2f\n",
                leitura.gx / 131.0f, leitura.gy / 131.0f, leitura.gz / 131.0f);
}

bool calibrarMPU() {
  digitalWrite(LED, HIGH);
  controleLiberado = false;
  contandoNeutro = false;
  filtroPronto = false;
  atualCodigo = PARAR;
  enviarCodigo(PARAR);
  Serial.println("Mantenha o sensor imovel na posicao neutra, Y+ para cima.");
  Serial.println("Y+ e o eixo do sensor, nao a face da placa. Esperado: ay perto de +1 g.");
  // Mantem PARAR sendo transmitido inclusive durante a calibracao.
  aguardarParado(2000);
  if (!verificarConfiguracao()) {
    Serial.println("CALIBRACAO: falha I2C ou configuracao do MPU alterada. Confira as ligacoes; se o MPU reiniciou, reinicie o controle.");
    return false;
  }
  const int AMOSTRAS = 400;
  double somaGX = 0, somaGY = 0, somaGZ = 0;
  double quadradoGX = 0, quadradoGY = 0, quadradoGZ = 0;
  double somaRoll = 0, somaPitch = 0, quadradoRoll = 0, quadradoPitch = 0;
  for (int i = 0; i < AMOSTRAS; ++i) {
    Leitura leitura;
    if (!lerMPU(leitura)) {
      Serial.printf("CALIBRACAO: %s (amostra %d de %d).\n", erroLeitura, i + 1, AMOSTRAS);
      if (leituraBrutaDisponivel) mostrarLeituraBruta(ultimaLeituraBruta);
      return false;
    }
    float r, p;
    angulosAcelerometro(leitura, r, p);
    float gx = leitura.gx / 131.0f, gy = leitura.gy / 131.0f;
    float gz = leitura.gz / 131.0f;
    // A montagem deve continuar proxima de Y+. A media compensa pequenos desvios.
    if (fabsf(r) > 25 || fabsf(p) > 25) {
      Serial.printf("CALIBRACAO: POSICAO fora da faixa. Pitch=%.1f, Roll=%.1f; limite +/-25 graus.\n", p, r);
      mostrarLeituraBruta(leitura);
      Serial.println("Aponte Y+ para cima. Se Z estiver perto de +1 ou -1 g, a placa esta deitada para esta montagem.");
      return false;
    }
    if (fabsf(gx) > 15 || fabsf(gy) > 15 || fabsf(gz) > 15) {
      Serial.println("CALIBRACAO: giroscopio acima de 15 graus/s (movimento ou offset elevado).");
      mostrarLeituraBruta(leitura);
      Serial.println("Apoie o sensor sem mover. Se persistir parado, guarde estes valores para diagnostico.");
      return false;
    }
    if (i == 0) {
      Serial.printf("Posicao aceita: Pitch=%.1f, Roll=%.1f. Medindo por cerca de 4 segundos...\n", p, r);
      mostrarLeituraBruta(leitura);
    }
    somaGX += gx; somaGY += gy; somaGZ += gz;
    quadradoGX += gx * gx; quadradoGY += gy * gy; quadradoGZ += gz * gz;
    somaRoll += r; somaPitch += p;
    quadradoRoll += r * r; quadradoPitch += p * p;
    enviarCodigo(PARAR);
    delay(10);
  }
  double gx = somaGX / AMOSTRAS, gy = somaGY / AMOSTRAS;
  double gz = somaGZ / AMOSTRAS, r = somaRoll / AMOSTRAS;
  double p = somaPitch / AMOSTRAS;
  // Rejeita calibracao com variacao maior que 1 grau ou 1 grau/s (desvio padrao).
  if (quadradoGX / AMOSTRAS - gx * gx > 1 ||
      quadradoGY / AMOSTRAS - gy * gy > 1 ||
      quadradoGZ / AMOSTRAS - gz * gz > 1 ||
      quadradoRoll / AMOSTRAS - r * r > 1 ||
      quadradoPitch / AMOSTRAS - p * p > 1) {
    Serial.println("CALIBRACAO: INSTABILIDADE (movimento, vibracao ou ruido). Limite de desvio padrao: 1 grau ou 1 grau/s.");
    Serial.printf("Desvio gyro X/Y/Z: %.2f / %.2f / %.2f graus/s | Roll/Pitch: %.2f / %.2f graus\n",
                  sqrt(fmax(0.0, quadradoGX / AMOSTRAS - gx * gx)),
                  sqrt(fmax(0.0, quadradoGY / AMOSTRAS - gy * gy)),
                  sqrt(fmax(0.0, quadradoGZ / AMOSTRAS - gz * gz)),
                  sqrt(fmax(0.0, quadradoRoll / AMOSTRAS - r * r)),
                  sqrt(fmax(0.0, quadradoPitch / AMOSTRAS - p * p)));
    Serial.println("Apoie o modulo numa superficie firme, mantendo Y+ para cima.");
    return false;
  }
  if (!verificarConfiguracao()) {
    Serial.println("CALIBRACAO: falha I2C ou configuracao alterada durante a medicao; tentativa descartada.");
    return false;
  }
  gyroXOffset = gx * 131.0f; gyroYOffset = gy * 131.0f;
  gyroZOffset = gz * 131.0f;
  rollNeutro = r; pitchNeutro = p;
  Serial.printf("Calibrado. Neutro: pitch %.1f, roll %.1f graus.\n", pitchNeutro, rollNeutro);
  digitalWrite(LED,LOW);
  return true;
}

void bloquearControle(const char *mensagem) {
  digitalWrite(LED, HIGH);
  if (!falhaSensor) Serial.println(mensagem);
  falhaSensor = true;
  controleLiberado = false;
  contandoNeutro = false;
  filtroPronto = false;
  atualCodigo = PARAR;
  enviarCodigo(PARAR);

}

void atualizarFiltro(const Leitura &leitura, float dt) {
  float rAccel, pAccel;
  angulosAcelerometro(leitura, rAccel, pAccel);
  if (!filtroPronto) {
    roll = rAccel; pitch = pAccel;
    filtroPronto = true;
    return;
  }
  float gx = (leitura.gx - gyroXOffset) / 131.0f;
  float gy = (leitura.gy - gyroYOffset) / 131.0f;
  float gz = (leitura.gz - gyroZOffset) / 131.0f;
  float r = roll * DEG_TO_RAD, p = pitch * DEG_TO_RAD;
  // Para Y+ vertical: roll usa -gx e pitch usa -gz perto do neutro.
  float taxaRoll = -gx - tanf(p) * (gy * cosf(r) + gz * sinf(r));
  float taxaPitch = gy * sinf(r) - gz * cosf(r);
  float alpha = TAU_FILTRO / (TAU_FILTRO + dt);
  roll = alpha * (roll + taxaRoll * dt) + (1 - alpha) * rAccel;
  pitch = alpha * (pitch + taxaPitch * dt) + (1 - alpha) * pAccel;
}

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  Serial.begin(115200);
  delay(1000);
  Serial.println("MPU6050 - CONTROLE DO CARRINHO");
  if (!WiFi.mode(WIFI_STA)) interromper("ERRO ao ativar Wi-Fi Station!");
  if (esp_wifi_set_channel(CANAL_WIFI, WIFI_SECOND_CHAN_NONE) != ESP_OK)
    interromper("ERRO ao configurar canal Wi-Fi!");
  if (esp_now_init() != ESP_OK) interromper("ERRO ao inicializar ESP-NOW!");
  esp_now_peer_info_t receptor = {};
  memcpy(receptor.peer_addr, enderecoMAC, 6);
  receptor.channel = CANAL_WIFI;
  receptor.ifidx = WIFI_IF_STA;
  receptor.encrypt = false;
  if (esp_now_add_peer(&receptor) != ESP_OK)
    interromper("ERRO ao adicionar receptor!");
  radioPronto = true;
  enviarCodigo(PARAR);
  Serial.print("MAC Station do controle: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("Canal Wi-Fi: %u\n", CANAL_WIFI);
  if (!Wire.begin()) interromper("ERRO ao iniciar I2C!");
  Wire.setTimeOut(TIMEOUT_I2C);
  I2Cdev::readTimeout = TIMEOUT_I2C;
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);
  mpu.setRate(9);  // 1 kHz / (1 + 9) = 100 Hz com DLPF ligado.
  delay(100);
  if (!verificarConfiguracao()) interromper("ERRO: MPU ausente ou configuracao invalida!");
  while (!calibrarMPU()) {
    Serial.println("Motores bloqueados (PARAR). Ajuste a causa indicada; nova tentativa em 3 segundos.");
    aguardarParado(3000);
  }
  ultimoTempo = micros();
  Serial.println("Aguardando 0,5 s imovel no neutro para liberar comandos.");
}

void loop() {
  uint32_t agoraMs = millis();
  if (static_cast<uint32_t>(agoraMs - ultimaVerificacao) >= 100) {
    ultimaVerificacao = agoraMs;
    if (!verificarConfiguracao()) {
      bloquearControle("FALHA: I2C/configuracao do MPU. PARAR. Se o MPU reiniciou, reinicie o controle.");
      delay(10);
      return;
    }
  }
  Leitura leitura;
  if (!lerMPU(leitura)) {
    bloquearControle("FALHA: leitura do MPU invalida. PARAR; retorne ao neutro apos corrigir.");
    delay(10);
    return;
  }
  uint32_t agora = micros();
  float dt = static_cast<uint32_t>(agora - ultimoTempo) / 1000000.0f;
  ultimoTempo = agora;
  if (dt <= 0 || dt > 0.1f) bloquearControle("PAUSA na leitura. PARAR; retorne ao neutro.");
  float rAccel, pAccel;
  angulosAcelerometro(leitura, rAccel, pAccel);
  // Evita inversoes, singularidades e comandos com o sensor de cabeca para baixo.
  if (fabsf(rAccel) > 65 || fabsf(pAccel) > 65) {
    bloquearControle("Inclinacao excessiva. PARAR; retorne ao neutro.");
    delay(10);
    return;
  }
  atualizarFiltro(leitura, dt);
  if (!isfinite(roll) || !isfinite(pitch) || fabsf(roll) > 70 || fabsf(pitch) > 70) {
    bloquearControle("Angulo fora da faixa. PARAR; retorne ao neutro.");
    delay(10);
    return;
  }
  float pitchComando = SINAL_FRENTE * (pitch - pitchNeutro);
  float rollComando = SINAL_DIREITA * (roll - rollNeutro);
  const char *direcao = "BLOQUEADO: RETORNE AO NEUTRO";
  if (!controleLiberado) {
    bool neutro = fabsf(pitchComando) <= LIMITE_DESATIVACAO &&
                  fabsf(rollComando) <= LIMITE_DESATIVACAO &&
                  fabsf(pAccel - pitchNeutro) <= LIMITE_DESATIVACAO &&
                  fabsf(rAccel - rollNeutro) <= LIMITE_DESATIVACAO &&
                  fabsf(leitura.gx - gyroXOffset) < 1310 &&
                  fabsf(leitura.gy - gyroYOffset) < 1310 &&
                  fabsf(leitura.gz - gyroZOffset) < 1310;
    if (!neutro) contandoNeutro = false;
    else if (!contandoNeutro) {
      contandoNeutro = true;
      inicioNeutro = millis();
    } else if (static_cast<uint32_t>(millis() - inicioNeutro) >= TEMPO_NEUTRO) {
      controleLiberado = true;
      falhaSensor = false;
      Serial.println("Controle liberado.");
    }
    atualCodigo = PARAR;
  } else {
    direcao = determinarDirecao(pitchComando, rollComando);
  }
  enviarCodigo(atualCodigo);
  bool naPosicaoNeutra =
    fabsf(pitchComando) <= LIMITE_DESATIVACAO &&
    fabsf(rollComando) <= LIMITE_DESATIVACAO;

  digitalWrite( LED,(!controleLiberado || naPosicaoNeutra) ? HIGH : LOW);
  if (static_cast<uint32_t>(millis() - ultimoDiagnostico) >= 200) {
    ultimoDiagnostico = millis();
    Serial.printf("Pitch: %.1f | Roll: %.1f | %s\n", pitchComando, rollComando, direcao);
  }
  delay(10);
}

void enviarCodigo(Codigo codigo) {
  if (!radioPronto) return;
  uint32_t agora = millis();
  if (!primeiroEnvio && codigo == ultimoCodigo &&
      static_cast<uint32_t>(agora - ultimoEnvio) < INTERVALO_ENVIO) return;
  esp_err_t resultado = esp_now_send(enderecoMAC,
      reinterpret_cast<const uint8_t *>(&codigo), sizeof(codigo));
  if (resultado == ESP_OK) {
    ultimoCodigo = codigo;
    ultimoEnvio = agora;
    primeiroEnvio = false;
  } else {
    static uint32_t ultimoErro = 0;
    if (static_cast<uint32_t>(agora - ultimoErro) >= 1000) {
      ultimoErro = agora;
      Serial.printf("ERRO ao solicitar envio ESP-NOW: %d\n", static_cast<int>(resultado));
    }
  }
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
