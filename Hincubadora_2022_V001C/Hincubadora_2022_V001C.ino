

#include "TramasMicros2.h"


//---------------------------------------------------------------------------------



#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#include "Pantalla.h"
//#include "PID.h"
//---------------------------------------------------------------------------------

bool lecturaPar = false;

double celsius_1 = 20;
int ThermistorPin = 0;
const int TemperaturaRedundante = 60;
int Vout_a[TemperaturaRedundante];



float logR2, R2, Temperature1;
int VoutR2;
//float R1 = 10000;
//float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
//http://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm
float R1 = 95000;
float c1 = 2.114990448e-03, c2 = 0.383281228e-04, c3 = 5.228061052e-07;

//---------------------------------------------------------------------------------


#include <PID_v1.h>

#define PIN_INPUT 0
#define PIN_ReleS1 11
#define PIN_Led 13

//#include "PID.h"

//Define Variables we'll be connecting to
double Setpoint = 21;
double Output;
unsigned long TiempoAlto;

//Specify the links and initial tuning parameters  P = propocion al error , I = incremento al pulso , D = rectifica al mobimiento "como, cuidado que biene"
double Kp=8.0, Ki=3.75, Kd=3.00;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

PID myPID(&celsius_1, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

double gap = 0;
//---------------------------------------------------------------------------------


void setup() {

  //inputString.reserve(200);
  bitWrite(ADCSRA, ADPS2, 1);
  bitWrite(ADCSRA, ADPS1, 0);
  bitWrite(ADCSRA, ADPS0, 1);
  //ADPS2 - ADPS1 - ADPS0 - Division Factor
  //0        - 0       - 0        ->2
  //0        - 0       - 1        ->2
  //0        - 1       - 0        ->4
  //0        - 1       - 1        ->8
  //1        - 0       - 0        ->16
  //1        - 0       - 1        ->32
  //1        - 1       - 0        ->64
  //1        - 1       - 1        ->128

  //---------------------------------------------------------------------------------
  
  Serial.begin(115200);

  //---------------------------------------------------------------------------------

  pinMode(PIN_ReleS1, OUTPUT);  
  pinMode(PIN_Led, OUTPUT);  
  //pinMode(PIN_INPUT, INPUT);  
  //---------------------------------------------------------------------------------
  
  //initialize the variables we're linked to
  myPID.SetOutputLimits(0, 1000);
  //myPID.SampleTime(2000);

  
  //---------------------------------------------------------------------------------


  //turn the PID on
  myPID.SetMode(AUTOMATIC);


//---------------------------------------------------------------------------------

  setupPantalla() ;


  for (int indice_1 = TemperaturaRedundante - 1; indice_1 > -1; indice_1--) {
    Vout_a[indice_1] = analogRead(ThermistorPin);
    display.setCursor(2 * indice_1 + 1, 10);
    display.print(".");
    display.display();  
    delay(250);
      
  }
  
}



//---------------------------------------------------------------------------------


TramaTiempo blink_LecturaSensores = TramaTiempo(70111, LecturaSensores);
TramaTiempo blink_PID = TramaTiempo(2000009, CumputePID);
TramaTiempo blink_PidGap = TramaTiempo(4010701, CumputePidGap);
TramaTiempo blink_DisplayOLED = TramaTiempo(1000105, DisplayOLED);

TramaTiempo blink_Rele1 = TramaTiempo(3146055, Rele1);

//---------------------------------------------------------------------------------


void LecturaSensores() {
  
  VoutR2 = analogRead(ThermistorPin);

  
  float Vout_Media = 0;
  for (int indice_1 = 0; indice_1 < TemperaturaRedundante; indice_1++) {   
    Vout_Media += Vout_a[indice_1] ;
  }
  Vout_Media = Vout_Media / TemperaturaRedundante;

  //R2 = R1 * (1023.0 / (( ( (float)Vout1) + (float)Vout2 + (float)Vout3 + (float)Vout4 + (float)Vout5 + (float)Vout6 + (float)Vout7 + (float)Vout8 + (float)Vout9 + (float)Vout10) / 10) - 1.0);
  R2 = R1 * (1023.0 / Vout_Media - 1) ;

   
  logR2 = log(R2);
  Temperature1  = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2)); // kelvin
  celsius_1 = Temperature1 - 273.15; // Centigrados
  Temperature1 = (Temperature1 * 9.0)/ 5.0 + 32.0;  // Farenheit

  for (int indice_1 = (TemperaturaRedundante - 1); indice_1 > 0; indice_1--) {
    Vout_a[indice_1] = Vout_a[indice_1 - 1];

  }
  Vout_a[0] = VoutR2;
  if (gap > 2) {
    if ( lecturaPar == false ){
      digitalWrite(PIN_Led,LOW);
      lecturaPar = true;
    } else {
      digitalWrite(PIN_Led,HIGH);
      lecturaPar = false;
    }
  } else {digitalWrite(PIN_Led,LOW);}

}

void CumputePID() {
  

  myPID.Compute();


  Serial.print("Temperature: "); 
  Serial.print(Temperature1);
  Serial.println(" F"); 
  
  Serial.print("Out: "); 
  Serial.print(VoutR2);
  Serial.println(" "); 
  
}

void CumputePidGap(){
  
  gap = abs(Setpoint-celsius_1); //distance away from setpoint
  if (gap < 0.5)
  {  
    //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(Kp, Ki, Kd);
    display.setCursor(60, 12);
    display.print(gap);
    display.println(".");
  }
  else if ((gap < 1) && (celsius_1 > Setpoint) ) {
    myPID.SetTunings(Kp + 200, Ki + 10.50, Kd + 5.00);
    display.setCursor(60, 12);
    display.print(gap);
    display.println("Z.");
  }
  else if (gap < 1) {
    myPID.SetTunings(Kp + 20, Ki + 2.75, Kd + 2.00);
    display.setCursor(60, 12);
    display.print(gap);
    display.println("o");
  }
  else if ((gap < 2) && (celsius_1 > Setpoint) ) {
    myPID.SetTunings(Kp + 300, Ki + 15.00, Kd + 1.00);
    display.setCursor(60, 12);
    display.print(gap);
    display.println("O.");
  }
  else if (gap < 2) {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(Kp + 45.2, Ki + 4.25, Kd + 4.00);
     display.setCursor(60, 12);
     display.print(gap);
     display.println("D");}
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(Kp + 90.2, Ki + 5.50, Kd + 4.25);
     display.setCursor(60, 12);
     display.print(gap);
     display.println("X");
  }
  
  display.display();
  
  //yield();
}


void Rele1(){

  
  unsigned long TiempoBajo_S1;
  unsigned long TiempoAlto_S1;

  TiempoAlto_S1 = ( (Output * 20000) + TiempoAlto ) / 2 ;
  TiempoBajo_S1 = 10000000 - TiempoAlto_S1;

  
  if ((bool)digitalRead(PIN_ReleS1))
  {
    digitalWrite(PIN_ReleS1, LOW);
    if (TiempoBajo_S1 > 0 ){
      blink_Rele1.setInterval(TiempoBajo_S1) ;
    } else {
      digitalWrite(PIN_ReleS1, HIGH);
      blink_Rele1.setInterval(TiempoAlto_S1) ;
    }

  }
  else
  {
    
    if (TiempoAlto_S1 > 0 ) {
      digitalWrite(PIN_ReleS1, HIGH);//digitalWrite(PinLedS1, HIGH);
      blink_Rele1.setInterval(TiempoAlto_S1) ;
    } else {
      digitalWrite(PIN_ReleS1, LOW);//digitalWrite(PinLedS1, LOW);
      blink_Rele1.setInterval(TiempoBajo_S1) ;
    }
    
  }
  
}


void loop() {

  blink_LecturaSensores.check();

//---------------------------------------------------------------------------------

  blink_PID.check();

//---------------------------------------------------------------------------------

  blink_DisplayOLED.check();
 
//---------------------------------------------------------------------------------

  blink_PidGap.check();

//---------------------------------------------------------------------------------

  blink_Rele1.check();
  
}


void DisplayOLED() {
  
  display.clearDisplay(); 

  display.setCursor(10, 0);
  display.print(celsius_1);
  display.println("C");

  display.drawBitmap( 70 , 0 , temperature, 8, 8, 1);
  display.setCursor(80, 0);
  display.print(Temperature1);
  display.println("F");

//  display.setCursor(80, 24);
//  display.print(Temperature1);
//  display.println("F");


//  int Vcc = 5;
//  float A = 1.11492089e-3;
//  float B = 2.372075385e-4;
//  float C = 6.954079529e-8;
//  float K = 2.5; //factor de disipacion en mW/C
//
//  float V =  Vo / 1024 * Vcc;
//  float R = (R1 * V ) / (Vcc - V);
//  
//  float logR  = log(R);
//  float R_th = 1.0 / (A + B * logR + C * logR * logR * logR );
//  float kelvin = R_th - V*V/(K * R)*1000;
//  float celsius = kelvin - 273.15;

//  float fahrenheit = ((Temperature1 * 9) + 3) / 5 + 32;
//  celsius_1 = ((Temperature1 - 32)* 5) /9;
//  celsius_1 = celsius_1 - 1;




  display.drawBitmap( 0 , 24 , solar, 8, 8, 1);
  display.setCursor(10, 24);
  display.print(Output);
  display.println("PMW");
  
  display.display();
}
