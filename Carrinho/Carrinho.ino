#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

// Arduino-ESP32 3.x. Motor A = lado esquerdo; B = lado direito.
// Confira se estes GPIOs estao livres na sua variante de ESP32.
const uint8_t CANAL_WIFI = 1;  // Igual no transmissor.
const int SINAL_MOTOR_A = 1;   // Use -1 se este motor estiver invertido.
const int SINAL_MOTOR_B = 1;
const int PWM_MAXIMO = 255;
const int PWM_CURVA = PWM_MAXIMO / 2;
static_assert(PWM_MAXIMO > 0 && PWM_MAXIMO <= 255, "PWM deve estar entre 1 e 255");

// Pinos originais.
const int inA1 = 4;
const int inA2 = 16;
const int inB1 = 17;
const int inB2 = 5;
const int enA = 2;
const int enB = 18;

// PWM: valores de 0 a 255.
const int FREQUENCIA = 1000;
const int RESOLUCAO = 8;
const int CANAL_A = 0;
const int CANAL_B = 1;

const unsigned long TIMEOUT_COMUNICACAO = 300;
const unsigned long INTERVALO_RAMPA = 20;
const int PASSO_PWM = 8;

// Mesma definicao usada no transmissor.
typedef enum {
  FRENTE = 0x01,
  TRAS = 0x02,
  DIREITA_FRENTE = 0x05,
  DIREITA_TRAS = 0x06,
  ESQUERDA_FRENTE = 0x07,
  ESQUERDA_TRAS = 0x08,
  PARAR = 0x09
} Codigo;

// Confirma o formato esperado pelo transmissor fornecido.
static_assert(
  sizeof(Codigo) == 4 && sizeof(int) == 4,
  "Protocolo requer comandos de 4 bytes"
);

// Protege os dados compartilhados entre callback e loop.
portMUX_TYPE travaDados = portMUX_INITIALIZER_UNLOCKED;

Codigo comandoRecebido = PARAR;
uint32_t ultimoTempo = 0;
bool recebeuPacote = false;
uint32_t pacotesRecebidos = 0;
bool movimentoLiberado = false;

// Estado dos motores: positivo = frente; negativo = tras.
int pwmAtualA = 0;
int pwmAtualB = 0;
int pwmAlvoA = 0;
int pwmAlvoB = 0;

unsigned long ultimaAtualizacaoPWM = 0;

bool pwmConfiguradoA = false;
bool pwmConfiguradoB = false;

// Prototipos.
bool codigoValido(int codigo);
void receberDados(
  const esp_now_recv_info_t *info,
  const uint8_t *dados,
  int tamanho
);
void decodificar(Codigo codigo);
int aproximarPWM(int atual, int alvo);
void aplicarMotor(int pino1, int pino2, int enable, int pwm);
void atualizarRampa();
void parar();
void interromper(const char *mensagem);
void diagnosticar(Codigo codigo, bool conectado, uint32_t pacotes);

void setup() {
  Serial.begin(115200);

  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  // Mantem as saidas desligadas durante a inicializacao.
  digitalWrite(enA, LOW);
  digitalWrite(enB, LOW);
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, LOW);
  digitalWrite(inB1, LOW);
  digitalWrite(inB2, LOW);

  pwmConfiguradoA = ledcAttachChannel(
    enA, FREQUENCIA, RESOLUCAO, CANAL_A
  );

  pwmConfiguradoB = ledcAttachChannel(
    enB, FREQUENCIA, RESOLUCAO, CANAL_B
  );

  if (!pwmConfiguradoA || !pwmConfiguradoB) {
    interromper("ERRO ao configurar PWM!");
  }

  parar();

  if (!WiFi.mode(WIFI_STA)) {
    interromper("ERRO ao iniciar Wi-Fi Station!");
  }

  if (esp_wifi_set_channel(CANAL_WIFI, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    interromper("ERRO ao configurar canal Wi-Fi!");
  }

  Serial.print("MAC Station do receptor: ");
  Serial.println(WiFi.macAddress());

  Serial.print("Canal Wi-Fi: ");
  Serial.println(WiFi.channel());

  if (esp_now_init() != ESP_OK) {
    interromper("ERRO ao inicializar ESP-NOW!");
  }

  if (esp_now_register_recv_cb(receberDados) != ESP_OK) {
    interromper("ERRO ao registrar recebimento!");
  }

  Serial.println("Receptor pronto. Aguardando comandos...");
}

void loop() {
  Codigo codigo;
  uint32_t recebidoEm, pacotes;
  bool temPacote;
  portENTER_CRITICAL(&travaDados);
  codigo = comandoRecebido;
  recebidoEm = ultimoTempo;
  temPacote = recebeuPacote;
  pacotes = pacotesRecebidos;
  portEXIT_CRITICAL(&travaDados);

  uint32_t agora = millis();
  bool conectado = temPacote &&
      static_cast<uint32_t>(agora - recebidoEm) < TIMEOUT_COMUNICACAO;
  if (!conectado) {
    movimentoLiberado = false;
    parar();
  } else if (codigo == PARAR) {
    // Apos ligar/perder sinal, exige PARAR novo antes de aceitar movimento.
    movimentoLiberado = true;
    parar();
  } else if (!movimentoLiberado) {
    parar();
  } else {
    decodificar(codigo);
    atualizarRampa();
  }
  diagnosticar(codigo, conectado, pacotes);
  delay(1);
}

void diagnosticar(Codigo codigo, bool conectado, uint32_t pacotes) {
  static bool primeiro = true, anteriorConectado = false, anteriorLiberado = false;
  static Codigo anteriorCodigo = PARAR;
  static uint32_t ultimoLog = 0;
  uint32_t agora = millis();
  if (!primeiro && conectado == anteriorConectado &&
      movimentoLiberado == anteriorLiberado && codigo == anteriorCodigo &&
      static_cast<uint32_t>(agora - ultimoLog) < 1000) return;
  primeiro = false;
  anteriorConectado = conectado;
  anteriorLiberado = movimentoLiberado;
  anteriorCodigo = codigo;
  ultimoLog = agora;
  if (!conectado) {
    Serial.println("SEM SINAL: motores desligados. Aguarde sinal e retorne ao neutro.");
  } else if (!movimentoLiberado) {
    Serial.println("SINAL RECEBIDO: retorne ao neutro para liberar os motores.");
  } else {
    Serial.printf("RX: 0x%02X | pacotes: %lu | PWM A: %d B: %d\n",
                  static_cast<unsigned>(codigo), static_cast<unsigned long>(pacotes),
                  pwmAtualA, pwmAtualB);
  }
}

bool codigoValido(int codigo) {
  switch (codigo) {
    case FRENTE:
    case TRAS:
    case DIREITA_FRENTE:
    case DIREITA_TRAS:
    case ESQUERDA_FRENTE:
    case ESQUERDA_TRAS:
    case PARAR:
      return true;

    default:
      return false;
  }
}

void receberDados(
  const esp_now_recv_info_t *info,
  const uint8_t *dados,
  int tamanho
) {
  (void)info;

  if (dados == nullptr ||
      tamanho != static_cast<int>(sizeof(Codigo))) {
    return;
  }

  // Valida como inteiro antes de converter para Codigo.
  int valor;
  memcpy(&valor, dados, sizeof(valor));

  if (!codigoValido(valor)) {
    return;
  }

  unsigned long agora = millis();

  // O callback apenas guarda os dados.
  portENTER_CRITICAL(&travaDados);
  comandoRecebido = static_cast<Codigo>(valor);
  ultimoTempo = agora;
  recebeuPacote = true;
  ++pacotesRecebidos;
  portEXIT_CRITICAL(&travaDados);
}

void decodificar(Codigo codigo) {
  // Preserva as curvas originais, inclusive em marcha a re.
  // Na re, direita indica o lado para onde a traseira descreve a curva.
  switch (codigo) {
    case FRENTE:
      pwmAlvoA = PWM_MAXIMO;
      pwmAlvoB = PWM_MAXIMO;
      break;

    case TRAS:
      pwmAlvoA = -PWM_MAXIMO;
      pwmAlvoB = -PWM_MAXIMO;
      break;

    case DIREITA_FRENTE:
      pwmAlvoA = PWM_MAXIMO;
      pwmAlvoB = PWM_CURVA;
      break;

    case ESQUERDA_FRENTE:
      pwmAlvoA = PWM_CURVA;
      pwmAlvoB = PWM_MAXIMO;
      break;

    case DIREITA_TRAS:
      pwmAlvoA = -PWM_MAXIMO;
      pwmAlvoB = -PWM_CURVA;
      break;

    case ESQUERDA_TRAS:
      pwmAlvoA = -PWM_CURVA;
      pwmAlvoB = -PWM_MAXIMO;
      break;

    default:
      pwmAlvoA = 0;
      pwmAlvoB = 0;
      break;
  }
}

int aproximarPWM(int atual, int alvo) {
  // Antes de inverter, reduz o PWM ate zero.
  if ((atual > 0 && alvo < 0) ||
      (atual < 0 && alvo > 0)) {
    alvo = 0;
  }

  if (atual < alvo) {
    return min(atual + PASSO_PWM, alvo);
  }

  if (atual > alvo) {
    return max(atual - PASSO_PWM, alvo);
  }

  return atual;
}

void aplicarMotor(
  int pino1,
  int pino2,
  int enable,
  int pwm
) {
  if (pwm == 0) {
    // Desliga o PWM antes de alterar a direcao.
    ledcWrite(enable, 0);
    digitalWrite(pino1, LOW);
    digitalWrite(pino2, LOW);
    return;
  }

  digitalWrite(pino1, pwm > 0 ? HIGH : LOW);
  digitalWrite(pino2, pwm < 0 ? HIGH : LOW);

  ledcWrite(enable, abs(pwm));
}

void atualizarRampa() {
  unsigned long agora = millis();

  if (agora - ultimaAtualizacaoPWM < INTERVALO_RAMPA) {
    return;
  }

  ultimaAtualizacaoPWM = agora;

  pwmAtualA = aproximarPWM(pwmAtualA, pwmAlvoA);
  pwmAtualB = aproximarPWM(pwmAtualB, pwmAlvoB);

  aplicarMotor(inA1, inA2, enA, SINAL_MOTOR_A * pwmAtualA);
  aplicarMotor(inB1, inB2, enB, SINAL_MOTOR_B * pwmAtualB);
}

void parar() {
  pwmAtualA = 0;
  pwmAtualB = 0;
  pwmAlvoA = 0;
  pwmAlvoB = 0;

  if (pwmConfiguradoA) {
    ledcWrite(enA, 0);
  } else {
    digitalWrite(enA, LOW);
  }

  if (pwmConfiguradoB) {
    ledcWrite(enB, 0);
  } else {
    digitalWrite(enB, LOW);
  }

  digitalWrite(inA1, LOW);
  digitalWrite(inA2, LOW);
  digitalWrite(inB1, LOW);
  digitalWrite(inB2, LOW);

  ultimaAtualizacaoPWM = millis();
}

void interromper(const char *mensagem) {
  parar();
  Serial.println(mensagem);

  while (true) {
    delay(1000);
  }
}
