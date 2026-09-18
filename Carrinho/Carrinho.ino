#include <WiFi.h>
#include <esp_now.h>
#include <string.h>

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
  sizeof(Codigo) == sizeof(int),
  "O enum deve ter o mesmo tamanho de int"
);

// Protege os dados compartilhados entre callback e loop.
portMUX_TYPE travaDados = portMUX_INITIALIZER_UNLOCKED;

Codigo comandoRecebido = PARAR;
unsigned long ultimoTempo = 0;
bool recebeuPacote = false;

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
  unsigned long recebidoEm;
  bool temPacote;

  // Copia comando e horario juntos, de forma protegida.
  portENTER_CRITICAL(&travaDados);
  codigo = comandoRecebido;
  recebidoEm = ultimoTempo;
  temPacote = recebeuPacote;
  portEXIT_CRITICAL(&travaDados);

  unsigned long agora = millis();

  if (!temPacote ||
      codigo == PARAR ||
      agora - recebidoEm >= TIMEOUT_COMUNICACAO) {
    parar();
    delay(1);
    return;
  }

  decodificar(codigo);
  atualizarRampa();

  delay(1);
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
  portEXIT_CRITICAL(&travaDados);
}

void decodificar(Codigo codigo) {
  // Preserva o mapeamento de motores do codigo original.
  switch (codigo) {
    case FRENTE:
      pwmAlvoA = 255;
      pwmAlvoB = 255;
      break;

    case TRAS:
      pwmAlvoA = -255;
      pwmAlvoB = -255;
      break;

    case DIREITA_FRENTE:
      pwmAlvoA = 255;
      pwmAlvoB = 127;
      break;

    case ESQUERDA_FRENTE:
      pwmAlvoA = 127;
      pwmAlvoB = 255;
      break;

    case DIREITA_TRAS:
      pwmAlvoA = -255;
      pwmAlvoB = -127;
      break;

    case ESQUERDA_TRAS:
      pwmAlvoA = -127;
      pwmAlvoB = -255;
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

  aplicarMotor(inA1, inA2, enA, pwmAtualA);
  aplicarMotor(inB1, inB2, enB, pwmAtualB);
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