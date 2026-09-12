#include <WiFi.h>
#include <esp_now.h>

int led = 2;

void recebeuDados(
  const esp_now_recv_info_t *info,
  const uint8_t *dados,
  int tamanho
) {

  if (tamanho == sizeof(int)) {

    int numero;

    memcpy(&numero, dados, sizeof(numero));

    Serial.print("Numero recebido: ");
    Serial.println(numero);

    if (numero == 1) {

      digitalWrite(led, HIGH);

    }

    if (numero == 0) {
      digitalWrite(led, LOW)
    }
  }
}

void setup() {

  Serial.begin(115200);

  Serial.println("Iniciando receptor!");

  pinMode(led, OUTPUT);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {

    Serial.println("Erro ao se comunicar!");
    return;
  }

  esp_now_register_recv_cb(recebeuDados);

  Serial.println("Receptor pronto!");
}

void loop() {

}