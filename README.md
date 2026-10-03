# Projeto_Compet
# Plataforma Robótica Móvel Controlada por Gestos

Projeto desenvolvido para a comPETção 2026, utilizando dois ESP32 e um sensor MPU-6050 para controlar uma plataforma robótica por movimentos, com comunicação sem fio via ESP-NOW.

## Funcionamento

- **Primeiro ESP32:** lê os dados do MPU-6050 e transmite os comandos.
- **Segundo ESP32:** recebe os comandos e controla os motores da plataforma.

## Componentes principais

- 2 microcontroladores ESP32
- Sensor MPU-6050
- 2 motores DC com caixa de redução e rodas
- Módulo de acionamento dos motores
- Chassi 2WD com roda de apoio
- Sistema de alimentação

## Utilização

1. Instale as bibliotecas utilizadas nos códigos.
2. Configure o endereço MAC do ESP32 receptor no código do transmissor.
3. Confira as conexões e os pinos definidos nos programas.
4. Grave o código correspondente em cada ESP32.
5. Ligue os dispositivos e teste os comandos com as rodas suspensas.

## Objetivo

Incentivar o aprendizado de programação, eletrônica e robótica por meio da construção de uma plataforma controlada por gestos.

## Desenvolvimento

O projeto está sujeito a melhorias de funcionamento, montagem e documentação.
