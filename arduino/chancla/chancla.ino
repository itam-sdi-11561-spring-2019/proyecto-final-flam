
#include <SoftwareSerial.h>
int TX = 50;
int RX = 51;
int bauds = 9600;
int cmd_right, cmd_left;
SoftwareSerial xbee(RX, TX);

int encoderDer = 42;
int encoderIzq = 44;
int counterEncDer = 0;
int counterEncIzq = 0;
unsigned long t0 = 0;

int encDerVal = 0;
int encIzqVal = 0;

int currVal = 0;

// Direccion (1 adelante, -1 atras)
int dir_1 = 1;
int dir_2 = 1;

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
int sample_time = 250;

double kpr = .7;
double kir = .7;
double kdr = 0.15;

double kpl = .9;
double kil = .6;
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
  //xbee
  xbee.begin(bauds);
  
  
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
  
  //Serial.begin(9600);
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
    
    //xbee.print("Tiempo: ");
    //xbee.println(sample_time);
    //xbee.print("enc azul: ");
    //xbee.println(counterEncDer);
    
    //xbee.print("enc rojo: ");
    //xbee.println(counterEncIzq);
    
    //PID
    
    // Error actual es target - velocidad en ticks/segundo
    err_r = targetR - (1000/sample_time)*counterEncDer*dir_1;
    err_l = targetL - (1000/sample_time)*counterEncIzq*dir_2;

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
      dir_1 = 1;
    }else{
      analogWrite(motor1_red, 0);
      analogWrite(motor1_white, out_r);
      dir_1 = -1;
    }
    
    // Por ahora, solo hay targets positivos
    if (pid_l >= 0){
      analogWrite(motor2_red, out_l);
      analogWrite(motor2_white, 0);
      dir_2 = 1;
    }else{
      analogWrite(motor2_red, 0);
      analogWrite(motor2_white, out_l);
      dir_2 = -1;
    }

    // Volver a empezar tiempo para calcular velocidad
    t0 = millis();
    counterEncDer = 0;
    counterEncIzq = 0;
  }
  
  if(xbee.available()>1){
    xbee.write(0xFF);
    
    cmd_right = xbee.read();
    dir_1 = xbee.read() == 0 ? -1 : 1;
    cmd_left = xbee.read();
    dir_2 = xbee.read() == 0 ? -1 : 1;
    
    xbee.write(cmd_right);
    xbee.println(dir_1);
    xbee.write(cmd_left);
    xbee.println(dir_2);
    
    targetR = (int) cmd_right;
    targetL = (int) cmd_left;
    
    xbee.println(targetR);
    xbee.println(targetL);
    
  }

}
