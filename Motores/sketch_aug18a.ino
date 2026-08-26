const int inA1 = 21;
const int inA2 = 19;
const int inB1 = 5;
const int inB2 = 18;
const int enA = 17;
const int enB = 22;

const int but1  = 15;
const int but2 = 4;

const int frequencia = 1000;
const int resolucao = 8;
const int canal_A = 0;
const int canal_B = 1;

void frente(int incl);
void tras(int incl);

typedef enum {
  FRENTE = 0x01,
  TRAS = 0x02,
  DIREITA_FRENTE = 0x05,
  DIREITA_TRAS = 0x06,
  ESQUERDA_FRENTE = 0x07,
  ESQUERDA_TRAS = 0x08, 
} Codigo;

void setup() {
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);
  pinMode(but1, INPUT_PULLUP);
  pinMode(but2, INPUT_PULLUP);

  ledcAttachChannel(enA, frequencia, resolucao, canal_A);
  ledcAttachChannel(enB, frequencia, resolucao, canal_B);

  parar();

  ledcWrite(enA, 0);
  ledcWrite(enB, 0);

  Serial.begin(115200);
  Serial.println("Inicio/n/n");
}

void loop() {
  

}
void decodificar(Codigo codigo){
  switch (codigo) {
  case FRENTE:
    frente(0);
    break;
  case TRAS:
    tras(0);
    break;
  case DIREITA_FRENTE: 
    frente(1);
    break;
  case ESQUERDA_FRENTE:
    frente(-1);
    break;
  case DIREITA_TRAS:
    tras(1);
    break;
  case ESQUERDA_TRAS:
    tras(-1);
    break;
  }
}
void frente(int incl) {
  switch (incl) {
    case 0:
      digitalWrite(inA1, HIGH);
      digitalWrite(inA2, LOW);
      digitalWrite(inB1, HIGH);
      digitalWrite(inB2, LOW);

      ledcWrite(enA, 255);
      ledcWrite(enB, 255);
      break;

    case 1:
      digitalWrite(inA1, HIGH);
      digitalWrite(inA2, LOW);
      digitalWrite(inB1, HIGH);
      digitalWrite(inB2, LOW);

      ledcWrite(enA, 255);
      ledcWrite(enB, 127);
      break;
    
    case -1:
      digitalWrite(inA1, HIGH);
      digitalWrite(inA2, LOW);
      digitalWrite(inB1, HIGH);
      digitalWrite(inB2, LOW);

      ledcWrite(enA, 127);
      ledcWrite(enB, 255);
      break;
  }

}

void tras(int incl) {
  switch (incl) {
    case 0:
      digitalWrite(inA1, LOW);
      digitalWrite(inA2, HIGH);
      digitalWrite(inB1, LOW);
      digitalWrite(inB2, HIGH);
      
      ledcWrite(enA, 255);
      ledcWrite(enB, 255);
      break;

    case 1:
      digitalWrite(inA1, LOW);
      digitalWrite(inA2, HIGH);
      digitalWrite(inB1, LOW);
      digitalWrite(inB2, HIGH);

      ledcWrite(enA, 255);
      ledcWrite(enB, 127);
      break;

    case -1:
      digitalWrite(inA1, LOW);
      digitalWrite(inA2, HIGH);
      digitalWrite(inB1, LOW);
      digitalWrite(inB2, HIGH);

      ledcWrite(enA, 127);
      ledcWrite(enB, 255);
      break;
  }
}

void parar() {
    digitalWrite(inA1, LOW);
    digitalWrite(inA2, LOW);
    digitalWrite(inB1, LOW);
    digitalWrite(inB2, LOW);

    ledcWrite(enA, 0);
    ledcWrite(enB, 0);
}