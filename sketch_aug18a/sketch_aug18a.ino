const int inA1 = 21;
const int inA2 = 19;
const int inB1 = 5;
const int inB2 = 18;

const int but1  = 15;
const int but2 = 4;


void frente();
void tras();

void setup() {
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);
  pinMode(but1, INPUT_PULLUP);
  pinMode(but2, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println("Inicio/n/n");
}

void loop() {
  int leitura_botao1 = digitalRead(but1);
  int leitura_botao2 = digitalRead(but2);

  Serial.print(leitura_botao1);
  Serial.print("      ");
  Serial.println(leitura_botao2);


  if (leitura_botao1 == LOW) {
    frente();
  } 
  if (leitura_botao2 == LOW) {
    tras();
  } 

  if (leitura_botao1 == leitura_botao2) {
    parar();
  }

}

void frente() {
  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, LOW);
  digitalWrite(inB1, HIGH);
  digitalWrite(inB2, LOW);
}

void tras() {
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, HIGH);
  digitalWrite(inB1, LOW);
  digitalWrite(inB2, HIGH);
}

void parar() {
    digitalWrite(inA1, HIGH);
    digitalWrite(inA2, HIGH);
    digitalWrite(inB1, HIGH);
    digitalWrite(inB2, HIGH);
}