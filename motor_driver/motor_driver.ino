/*
 * Arduino nano old bootloader.
*/
#include <util/atomic.h>

// Coltrol Values
long controlTime = 0;
long controlTimeOld = 0;
long sendTime = 0;
long sendTimeOld = 0;
long time_start = 0;

double kpR = 0.01;
double kpL = 0.01;
double kiR = 0.01;
double kiL = 0.01;

// Not really useful
double kdR = 0.0; //0.0002;
double kdL = 0.0; //0.0002;

double PR = 0;
double PL = 0;
double IR = 0;
double IL = 0;
double DR = 0;
double DL = 0;

double etR = 0.0;
double etL = 0.0;
double etR_integral = 0.0;
double etL_integral = 0.0;
double etR_derivate = 0.0;
double etL_derivate = 0.0;
double etR_ant = 0.0;
double etL_ant = 0.0;


double p2rpm = 0.0;

// encoders R config

const int RchannelPinA = 2;
const int RchannelPinB = 12;

long RtimeCounter = 0;
volatile long RISRtime_new = 0;
long Rtime_new = 0;
long Rtime_ant = 0;

// Encoder de 663 pulsos/vuelta pero lectura precision doble
const uint16_t maxSteps = 1328;
volatile int16_t RISRCounter = 0;
int16_t Rcounter = 0;
int16_t RcounterOld = 0;
int16_t Rvueltas = 0;
double wr = 0.0;

bool RIsCW = true;

// encoders L config

const int LchannelPinA = 3;
const int LchannelPinB = 4;

long LtimeCounter = 0;
volatile long LISRtime_new = 0;
long Ltime_new = 0;
long Ltime_ant = 0;

volatile int16_t LISRCounter = 0;
int16_t Lcounter = 0;
int16_t LcounterOld = 0;
int16_t Lvueltas = 0;
double wl = 0;

bool LIsCW = true;


// Wheel L
int enable_L = 5;
int frontL = 8; // front
int backL = 9; // backwards

// Wheel R
int enable_R = 6;
int frontR = 10; // front
int backR = 11; // backwards

long totalRrpm = 0;
long totalRcounter = 0;
int totalAdds = 0;
double speedR = 0.0;
int dirR = 0;
int dirL = 0;
long totalLrpm = 0;
long totalLcounter = 0;
int speedL = 0;

String serial_read()
{
  if (Serial.available() > 0) {
    //Se crea una variable que servirá como buffer
    String bufferString = "";
    //int n=0;
    char a='0';
    a=Serial.read();
    delay(2);
    if (a=='X'){
      a=Serial.read();
      while(a!='Y'){
        delay(2);
        bufferString += a;
        a=Serial.read();
      }
      return bufferString;
    }
  }
  return "";
}

void copyRCounter()
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    Rcounter = RISRCounter;
  }
}
void copyRtime()
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    Rtime_new = RISRtime_new;
  }
}
void setRCounter(uint16_t value)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    RISRCounter = value;
  }
}

void copyLCounter()
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    Lcounter = LISRCounter;
  }
}
void copyLtime()
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    Ltime_new = LISRtime_new;
  }
}
void setLCounter(uint16_t value)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    LISRCounter = value;
  }
}


void doREncode()
{
  if (digitalRead(RchannelPinA) == digitalRead(RchannelPinB))
  {
     RIsCW = true;
     RISRCounter++;
  }
  else
  {
     RIsCW = false;
     RISRCounter++;
  }
  RISRtime_new = millis();
}

void doLEncode()
{
  if (digitalRead(LchannelPinA) == digitalRead(LchannelPinB))
  {
     LIsCW = true;
     LISRCounter++;
  }
  else
  {
     LIsCW = false;
     LISRCounter++;
  }
  LISRtime_new = millis();
}

void setup()
{
  Serial.begin(9600);
  pinMode (frontL, OUTPUT);    // Input4 conectada al pin 4 
  pinMode (backL, OUTPUT);    // Input3 conectada al pin 5
  pinMode (frontR, OUTPUT);    // Input4 conectada al pin 4 
  pinMode (backR, OUTPUT);    // Input3 conectada al pin 5

  pinMode(RchannelPinA, INPUT_PULLUP);
  pinMode(RchannelPinB, INPUT_PULLUP);
  pinMode(LchannelPinA, INPUT_PULLUP);
  pinMode(LchannelPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RchannelPinA), doREncode, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LchannelPinA), doLEncode, CHANGE);

  digitalWrite (frontR, dirR);
  digitalWrite (frontL, dirL);

  p2rpm = 60000.0/(663.0*2);

  controlTime = millis();
  controlTimeOld = controlTime;
  time_start = millis();
}
void loop()
{
  String s = serial_read();
  if (s!="")
  {
    int index = s.indexOf(";");
    dirR = s.substring(0,index).toInt();
    s = s.substring(index+1);
    
    index = s.indexOf(";");
    speedR = s.substring(0,index).toInt();
    s = s.substring(index+1);
    
    index = s.indexOf(";");
    dirL = s.substring(0,index).toInt();
    speedL = s.substring(index+1).toInt();

    if (dirR)
    {
      digitalWrite (frontR, HIGH);
      digitalWrite (backR, LOW);
    }
    else
    {
      digitalWrite (backR, HIGH);
      digitalWrite (frontR, LOW);
    }
    if (dirL)
    {
      digitalWrite (frontL, HIGH);
      digitalWrite (backL, LOW);
    }
    else
    {
      digitalWrite (backL, HIGH);
      digitalWrite (frontL, LOW);
    }
  }

  controlTime = millis();
  if ((controlTime - controlTimeOld) >= 50)
  {
    copyRCounter(); // copy RISRCounter into RCounter
    copyRtime();
    if (Rtime_ant >= Rtime_new) wr = 0;
    else wr = ((double)Rcounter / (double)(Rtime_new - Rtime_ant))*p2rpm; // rpm

    // PID Control Loop WheelR
    // Compute the error
    etR = speedR - wr;
    // We saturate the error at a 30% of the setpoint to avoid inestavility
    if (etR > (speedR*0.3)) etR = speedR*0.3;

    // We apply the Proporcional control
    PR = kpR * etR;

    // We apply the Integral part
    etR_integral += etR*0.1;//(Rtime_new - Rtime_ant)/1000; // integral
    IR = kiR * (etR_integral);

    // We apply the Derivative part
    etR_derivate = (etR-etR_ant)/((double)(Rtime_new - Rtime_ant)/1000);
    DR = kdR * etR_derivate;

    // We convert the output of the PID into PWM signal
    double voltageR = 146.0/12.0 * (PR+IR+DR);
    int pwmR = 255/12 * voltageR;

    // We write the PWM signail into the motors
    if (speedR == 0) analogWrite(enable_R, 0);
    else analogWrite(enable_R, pwmR);

    //Serial.print(wr); // to see the variable in the Serial Ploter
    //Serial.print(",");
    totalRrpm += Rcounter;
    
    // Reset values
    setRCounter(0);
    Rcounter = 0;

    Rtime_ant = millis();
    etR_ant = etR;

    // --------------------
    
    copyLCounter(); // copy RISRCounter into RCounter
    copyLtime();
    if (Ltime_ant >= Ltime_new) wl = 0;
    else wl = ((double)Lcounter / (double)(Ltime_new - Ltime_ant))*p2rpm; // rpm

    // PID Control Loop WheelL
    // Compute the error
    etL = speedL - wl;
    // We saturate the error at a 30% of the setpoint to avoid inestavility
    if (etL > (speedL*0.3)) etL = speedL*0.3;

    // We apply the Proporcional control
    PL = kpL * etL;

    // We apply the Integral part
    etL_integral += etL*0.1; // integral
    IL = kiL * (etL_integral);//*(Ltime_new - Ltime_ant)/1000;

    // We apply the Derivative part
    etL_derivate = (etL-etL_ant)/((double)(Ltime_new - Ltime_ant)/1000);
    DL = kdL * etL_derivate;


    // We convert the output of the PID into PWM signal
    double voltageL = 146.0/12.0 * (PL+IL+DL);
    int pwmL = 255/12 * voltageL;

    // We write the PWM signail into the motors
    if (speedL == 0) analogWrite(enable_L, 0);
    else analogWrite(enable_L, pwmL);

    //Serial.println(wl); // to see the variable in the Serial Ploter
    //Serial.println(Lcounter); // send variable feedback
    totalLrpm += Lcounter;
    totalAdds += 1;

    setLCounter(0);
    Lcounter = 0;

    Ltime_ant = millis();
    etL_ant = etL;

    controlTimeOld = controlTime;
  }
  sendTime = millis();
  if ((sendTime - sendTimeOld) >= 128)
  {
    String bufferString = "X" + String(dirR) + ";" + String(totalRrpm/totalAdds) + ";" + String(dirL) + ";" + String(totalLrpm/totalAdds) + ";" + String(sendTime - sendTimeOld)+ "Y";
    //String bufferString = "X" + String(dirR) + ";" + String(totalRcounter) + ";" + String(dirL) + ";" + String(totalLcounter) + ";" + String(sendTime - sendTimeOld)+ "Y";
    Serial.print(bufferString);
    totalRrpm = 0;
    totalRcounter = 0;
    totalLrpm = 0;
    totalLcounter = 0;
    totalAdds = 0;
    sendTimeOld = sendTime;
  }
  delay(10);
}
