int encoderDer = 42;
int encoderIzq = 44;
int counterEncDer = 0;
int counterEncIzq = 0;
unsigned long t0=0;

int encDerVal = 0;
int encIzqVal = 0;

int currVal = 0;

//Motor 1 es der
int motor1_red = 11;
int motor1_white = 12;

//Motor 2 es izq
int motor2_red = 5;
int motor2_white = 2;

int targetR = 0;
int targetL = 0;

double kpr = 0.6;
double kir = 0.6;
double kdr = 0.1;

double kpl = 0.5;
double kil = 0.7;
double kdl = 0.1;

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
  
  currVal = digitalRead(encoderDer);
  
  if(encDerVal == HIGH){
    if(currVal == LOW){
      encDerVal = currVal;
      counterEncDer++;
    }
  }
  
  encDerVal = currVal;

  currVal = digitalRead(encoderIzq);
  
  if(encIzqVal == HIGH){
    if(currVal == LOW){
      encIzqVal = currVal;
      counterEncIzq++;
    }
  }
  
  encIzqVal = currVal;
  
  // Cada 500 ms resetea
  if(millis() >= t0 + 500){
    
    Serial.print("\nenc derecho\n");
    Serial.print(counterEncDer);
    
    Serial.print("\nenc izq\n");
    Serial.print(counterEncIzq);
    
    //PID
    err_r = targetR - counterEncDer;
    err_l = targetL - counterEncIzq;
    
    sum_err_r += err_r;
    sum_err_l += err_l;
    
    pid_r = err_r*kpr + (err_r - prev_err_r)*kdr + sum_err_r*kir;
    pid_l = err_l*kpl + (err_l - prev_err_l)*kdl + sum_err_l*kil;
    
    prev_err_r = err_r;
    prev_err_l = err_l;
    
    out_r = min(abs(pid_r), 255);
    out_l = min(abs(pid_l), 255);
    
    
    if (pid_r >= 0){
      analogWrite(motor1_red, out_r);
      analogWrite(motor1_white, 0);
    }else{
      analogWrite(motor1_red, 0);
      analogWrite(motor1_white, out_r);
    }
    
    
    if (pid_l >= 0){
      analogWrite(motor2_red, out_l);
      analogWrite(motor2_white, 0);
    }else{
      analogWrite(motor2_red, 0);
      analogWrite(motor2_white, out_l);
    }
    
    t0 = millis();
    counterEncDer = 0;
    counterEncIzq = 0;
  }

}
