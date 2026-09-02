#include <WiFi.h>
#include <esp_now.h>

uint8_t enderecoMAC[]= {0xA4,0xF0,0x0F,0x83,0x22,0x78};

int but = 16;

void setup() {
 Serial.begin(115200);
 Serial.println("Iniciando codigo!");
 pinMode(but, INPUT_PULLUP);

 WiFi.mode(WIFI_STA);

 if(esp_now_init() != ESP_OK) {
  Serial.println("Erro ao se comunicar!");
 }

 esp_now_peer_info_t infoParceiro = {};

 memcpy(infoParceiro.peer_addr, enderecoMAC, 6);

 infoParceiro.channel = 0;
 infoParceiro.encrypt = false;

 if(esp_now_add_peer(&infoParceiro) != ESP_OK) {
  Serial.println("ALGO DEU ERRADO NA COMUNICACAO");
  return;
 }
 Serial.println("PARCEIRO CONECTADO KRLH");
}

void loop() {
    int leituraBotao = digitalRead(but);
    Serial.print("Valor but: ");
    Serial.println(leituraBotao);

    if(leituraBotao == LOW) {
      int numero = 1;
      esp_err_t resultadoComunicacao = esp_now_send(enderecoMAC, (uint8_t *)&numero, sizeof(numero));

      if(resultadoComunicacao == ESP_OK) {
        Serial.print("Dados enviado com sucesso       ");
        Serial.println(numero);
      } else { Serial.println("ERRO DE COMUNICACAO!");}
    } else {
      int numero = 0;
      esp_err_t resultadoComunicacao = esp_now_send(enderecoMAC, (uint8_t *)&numero, sizeof(numero));

      if(resultadoComunicacao == ESP_OK) {
        Serial.print("Dados enviado com sucesso       ");
        Serial.println(numero);
      } else { Serial.println("ERRO DE COMUNICACAO!");}
    }

    delay(500);
}