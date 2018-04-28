const byte interruptPin = 2;
float wheelSpeed = 0;
unsigned long lastMesureTime = 0;

const float coefP = 0.02;
const float coefI = 0;
float sumSpeedError = 0.0;
float wheelSpeedReference = 3000.0;
int motorPin = 3;
unsigned long lastControlTime = 0;
const unsigned long controlPeriod = 100; //ms

unsigned short nbIt = 0;
const unsigned short nbItMax = 10;


void setup() {
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(motorPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), speedMesure, RISING);
  Serial.begin(9600);
}

void loop() {
  unsigned long currentControlTime = millis(); 
  if(currentControlTime - lastControlTime >= controlPeriod) {
    sumSpeedError += wheelSpeedReference - wheelSpeed;
    int motorPwm = coefP*(wheelSpeedReference - wheelSpeed) + coefI*sumSpeedError;
    if(motorPwm > 255){
      motorPwm = 255;
    }
    else if(motorPwm < 10){
      motorPwm = 0;
    }
    analogWrite(motorPin, motorPwm);
    Serial.print("reference speed: ");
    Serial.print(wheelSpeedReference);
    Serial.print(" pwm speed: ");
    Serial.print(motorPwm);
    Serial.print(" speed: ");
    Serial.println(wheelSpeed);
    wheelSpeed = 0.0;
    lastControlTime = currentControlTime;
  }
}

void speedMesure() {
  nbIt++;
  if(nbIt >= nbItMax){
    unsigned long currentMesureTime = micros();
    wheelSpeed = 10000000.0*float(nbIt)/float(currentMesureTime-lastMesureTime); // 2*pi/10*1000/(t-t_last) 100000.0*float(nbIt)*60.0
    lastMesureTime = currentMesureTime;
  
    nbIt = 0;
  }
}
