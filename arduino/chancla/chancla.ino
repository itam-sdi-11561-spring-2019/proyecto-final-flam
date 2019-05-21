int encoderDer = 42;
int encoderIzq = 44;
int counterEncDer = 0;
int counterEncIzq = 0;
int t0=0;

void setup(){
  pinMode(encoderDer, INPUT);
  pinMode(encoderIzq, INPUT);
  Serial.begin(9600);
}

void loop(){
  if(t0 == 0){
    t0 = millis();
    counterEncDer = 0;
    counterEncIzq = 0;
  }

  if(digitalRead(encoderDer) == HIGH){
    counterEncDer++;
  }

  if(digitalRead(encoderIzq) == HIGH){
    counterEncIzq++;
  }

  if(millis()-t0 == 1000){
    t0 = 0;
    Serial.print("\nenc derecho\n");
    Serial.print(counterEncDer);
    
    Serial.print("\nenc izq\n");
    Serial.print(counterEncDer);
  }

}
