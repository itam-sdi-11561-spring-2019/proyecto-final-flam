int encoderDer = 42;
int encoderIzq = 44;
int counterEncDer = 0;
int counterEncIzq = 0;
unsigned long t0 = 0;

int encDerVal = 0;
int encIzqVal = 0;

int currVal = 0;

// Motor 1 es der
int motor1_red = 11;
int motor1_white = 12;

// Motor 2 es izq
int motor2_red = 5;
int motor2_white = 2;

// Velocidad en encoder ticks/seg
double targetR = 0;
double targetL = 0;

// Cada cuanto revizar velocidad (en milis)
int sample_time = 100;

double kpr = 1.2;
double kir = 1.2;
double kdr = 0.2;

double kpl = 1;
double kil = 1.4;
double kdl = 0.2;

double err_r = 0;
double sum_err_r = 0;
double prev_err_r = 0;

double err_l = 0;
double sum_err_l = 0;
double prev_err_l = 0;

double pid_r = 0;
double pid_l = 0;

int out_r = 0;
int out_l = 0;

void setup(){
  pinMode(encoderDer, INPUT);
  encDerVal = digitalRead(encoderDer);
  pinMode(encoderIzq, INPUT);
  encIzqVal = digitalRead(encoderIzq);
  
  pinMode(motor1_red, OUTPUT);
  pinMode(motor1_white, OUTPUT);
  
  analogWrite(motor1_red, 0);
  analogWrite(motor1_white, 0);
  
  pinMode(motor2_red, OUTPUT);
  pinMode(motor2_white, OUTPUT);
  
  analogWrite(motor2_red, 0);
  analogWrite(motor2_white, 0);
  
  t0 = millis();
  
  Serial.begin(9600);
}

void loop(){

  // Contar encoder derecho
  currVal = digitalRead(encoderDer);
  if(encDerVal == HIGH){
    if(currVal == LOW){
      counterEncDer++;
    }
  }
  encDerVal = currVal;

  // Contar encoder izquierdo
  currVal = digitalRead(encoderIzq);
  if(encIzqVal == HIGH){
    if(currVal == LOW){
      counterEncIzq++;
    }
  }
  encIzqVal = currVal;
  
  // Cada sample_time ms calcula velocidad y PID
  if(millis() >= t0 + sample_time){
    
    Serial.print("\nTiempo: ");
    Serial.print(sample_time);
    Serial.print("\nenc derecho\n");
    Serial.print(counterEncDer);
    
    Serial.print("\nenc izq\n");
    Serial.print(counterEncIzq);
    
    //PID
    
    // Error actual es target - velocidad en ticks/segundo
    err_r = targetR - (sample_time/1000)*counterEncDer;
    err_l = targetL - (sample_time/1000)*counterEncIzq;

    // Error integral
    sum_err_r += err_r;
    sum_err_l += err_l;

    // PID completo
    pid_r = err_r*kpr + (err_r - prev_err_r)*kdr + sum_err_r*kir;
    pid_l = err_l*kpl + (err_l - prev_err_l)*kdl + sum_err_l*kil;

    // Actualizar error previo
    prev_err_r = err_r;
    prev_err_l = err_l;

    // Ajustar salida
    out_r = min(abs(pid_r), 255);
    out_l = min(abs(pid_l), 255);
    
    // Por ahora, solo hay targets positivos.
    if (pid_r >= 0){
      analogWrite(motor1_red, out_r);
      analogWrite(motor1_white, 0);
    }else{
      analogWrite(motor1_red, 0);
      analogWrite(motor1_white, out_r);
    }
    
    // Por ahora, solo hay targets positivos
    if (pid_l >= 0){
      analogWrite(motor2_red, out_l);
      analogWrite(motor2_white, 0);
    }else{
      analogWrite(motor2_red, 0);
      analogWrite(motor2_white, out_l);
    }

    // Volver a empezar tiempo para calcular velocidad
    t0 = millis();
    counterEncDer = 0;
    counterEncIzq = 0;
  }

}
