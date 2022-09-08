 /**
  ******************************************************************************
  * @file    vr_sample_control_led.ino
  * @author  JiapengLi
  * @brief   This file provides a demostration on 
              how to control led by using VoiceRecognitionModule
  ******************************************************************************
  * @note:
        voice control led
  ******************************************************************************
  * @section  HISTORY 
    
    2013/06/13    Initial version.
  */
  
#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"
#include <Servo.h> // Incluímos la librería para poder controlar el servo

#include <LiquidCrystal_I2C.h> // Para usar el LCD conectado al dispositivo I2C
#include <Wire.h> // Nos permite la comunicación con el dispositivo I2C

/**        
  Connection
  Arduino    VoiceRecognitionModule
   2   ------->     TX
   3   ------->     RX
*/


VR myVR(2,9);    // 2:RX 3:TX, you can choose your favourite pins.

uint8_t records[7]; // save record
uint8_t buf[64];
 
int led = 13;

#define onRecord    (0)
#define offRecord   (1) 

Servo servoMotor; // Servo de la papelera 1

/**
  @brief   Print signature, if the character is invisible, 
           print hexible value instead.
  @param   buf     --> command length
           len     --> number of parameters
*/
void printSignature(uint8_t *buf, int len)
{
  int i;
  for(i=0; i<len; i++){
    if(buf[i]>0x19 && buf[i]<0x7F){
      Serial.write(buf[i]);
    }
    else{
      Serial.print("[");
      Serial.print(buf[i], HEX);
      Serial.print("]");
    }
  }
}

/**
  @brief   Print signature, if the character is invisible, 
           print hexible value instead.
  @param   buf  -->  VR module return value when voice is recognized.
             buf[0]  -->  Group mode(FF: None Group, 0x8n: User, 0x0n:System
             buf[1]  -->  number of record which is recognized. 
             buf[2]  -->  Recognizer index(position) value of the recognized record.
             buf[3]  -->  Signature length
             buf[4]~buf[n] --> Signature
*/
void printVR(uint8_t *buf)
{
  Serial.println("VR Index\tGroup\tRecordNum\tSignature");

  Serial.print(buf[2], DEC);
  Serial.print("\t\t");

  if(buf[0] == 0xFF){
    Serial.print("NONE");
  }
  else if(buf[0]&0x80){
    Serial.print("UG ");
    Serial.print(buf[0]&(~0x80), DEC);
  }
  else{
    Serial.print("SG ");
    Serial.print(buf[0], DEC);
  }
  Serial.print("\t");

  Serial.print(buf[1], DEC);
  Serial.print("\t\t");
  if(buf[3]>0){
    printSignature(buf+4, buf[3]);
  }
  else{
    Serial.print("NONE");
  }
  Serial.println("\r\n");
}

//-------------------------- LCD -----------------------------//

LiquidCrystal_I2C lcd(0x27, 16, 2); // Creamos objeto que representa el LCD: dirección 0x27 y 16 columnas x 2 filas


void setup()
{
  /** initialize */
  delay(2000);
  myVR.begin(9600);
  
  Serial.begin(115200);
  //Serial.println("Elechouse Voice Recognition V3 Module\r\nControl LED sample");
  
  pinMode(led, OUTPUT);

  // Iniciamos el servo correspondiente a la papelera 1 con el pin 7
  servoMotor.attach(7);
  //servoMotor.write(0);
  servoMotor.write(50);
    
  if(myVR.clear() == 0){
   // Serial.println("Recognizer cleared.");
  }else{
    //Serial.println("Not find VoiceRecognitionModule.");
    //Serial.println("Please check connection and restart Arduino.");
    while(1);
  }
  
  if(myVR.load((uint8_t)onRecord) >= 0){
    //Serial.println("onRecord loaded");
  }
  
  if(myVR.load((uint8_t)offRecord) >= 0){
    //Serial.println("offRecord loaded");
  }

// Se inicializa el LCD (y el I2C)
 lcd.init();

  // Se enciende la luz de fondo del LCD
  lcd.backlight();

  // Escribimos en el LCD
  lcd.print("abrir:cerrado");
  lcd.setCursor(0, 1); // Ubicamos el cursor del LCD en la primera posición (columna:0) de la segunda línea (fila:1)
  lcd.print("");

  
}


void loop()
{
  servoMotor.write(50);
  delay(0);
  int ret;
  ret = myVR.recognize(buf, 50);
  if(ret>0){
    switch(buf[1]){ 
      case onRecord:
        /** turn on LED */
        digitalWrite(led, HIGH);
        printCountdown(0, 0, "Cerrando en:", 3); // Llamamos a la función que actualizará el LCD
       servoMotor.write(0);
        delay(1000);
        printCountdown(0, 0, "Cerrando en:", 2); // Llamamos a la función que actualizará el LCD
        delay(1000);
        printCountdown(0, 0, "Cerrando en:", 1); // Llamamos a la función que actualizará el LCD
        delay(1000);
        printCountdown(0, 0, "Cerrando en:", 0); // Llamamos a la función que actualizará el LCD
        delay(2000); // Tiempo de seguridad
        printCountdown(0, 0, "Basurero:", -1); // Llamamos a la función que actualizará el LCD
        servoMotor.write(50);
        digitalWrite(led, LOW);
        break;

       case offRecord:

        /** turn on LED */
        digitalWrite(led, HIGH);  
        printCountdown(0, 0, "Cerrando en:", 3); // Llamamos a la función que actualizará el LCD
       servoMotor.write(0);
        delay(1000);
        printCountdown(0, 0, "Cerrando en:", 2); // Llamamos a la función que actualizará el LCD
        delay(1000);
        printCountdown(0, 0, "Cerrando en:", 1); // Llamamos a la función que actualizará el LCD     
        delay(1000);
        printCountdown(0, 0, "Cerrando en:", 0); // Llamamos a la función que actualizará el LCD
        delay(2000); // Tiempo de seguridad
        printCountdown(0, 0, "Basurero:", -1); // Llamamos a la función que actualizará el LCD
        servoMotor.write(50);
        /** turn off LED*/
        digitalWrite(led, LOW);
        break;
       default:
        //Serial.println("Record function undefined");
        break;

      
    }
    /** voice recognized */
   // printVR(buf);
  }
 
}

void printCountdown(int col, int row, String typeText, int countdown) {
  /*
    Escribimos en el LCD cada vez que tengamos que actualizar alguna información.
    - En caso de que "countdown" sea menor a 0, en el LCD ponemos que la papelera
      correspondiente está cerrada.
    - En caso de que "countdown" sea mayor a 0, [1,3] ponemos ese valor como la cuenta atrás
      en el LCD.
  */
  lcd.setCursor(col, row);
  lcd.print("");
  lcd.setCursor(col, row);
  String lcdText = "";
  if (countdown < 0) {
    lcdText = typeText + "cerrado";
  } else {
    lcdText = typeText + " " + countdown + " sg";
  }
  lcd.print(lcdText);
}
