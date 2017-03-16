
/*
Arduino Duemilanove and Tri axis senzor ADIS16354
Datasheet:
http://www.analog.com/static/imported-files/data_sheets/ADIS16354.pdf

pin 10               CS -     SS - vo?ba SPI zariadenia            
pin 11               DIN -    MOSI - Master-out, Slave-in - digitálny vstup, slúži na zápis dát z Arduina do senzora  
pin 12               DOUT -   MISO - Master-in, Slave out - digitálny výstup, slúži na ?ítanie dát zo senzora do Arduina
pin 13               SCK -    SCLK - Serial clock - nastavenie hodín a frekvencie
*/
#include <SPI.h>


#define SUPPLY_OUT     0X02
#define XGYRO_OUT   0X04
#define YGYRO_OUT     0X06
#define ZGYRO_OUT     0X08
#define XACCL_OUT     0X0A
#define YACCL_OUT     0X0C
#define ZACCL_OUT     0X0E
#define XTEMP_OUT   0X10
#define YTEMP_OUT     0X12
#define ZTEMP_OUT     0X14
#define AUX_ADC     0X16
// tieto adresy registrov slúžia na ?ítanie hodnôt z jednotlivých senzorov

#define XGYRO_OFF   0X1A
#define YGYRO_OFF   0X1C
#define ZGYRO_OFF   0X1E
#define XACCL_OFF   0X20
#define YACCL_OFF   0X22
#define ZACCL_OFF   0X24
#define ALM_MAG1    0X26
#define ALM_MAG2    0X28
#define ALM_SMPL1   0X2A
#define ALM_SMPL2   0X2C
#define ALM_CTRL    0X2E
#define AUX_DAC     0X30
// nastavenie a kalibrácia senzora, po zápise hodnôt (pozri datasheet senzora) sa zmenia nastavenia senzora

#define GPIO_CTRL   0X32
#define MSC_CTRL    0X34
#define SMPL_PRD    0X36    // TS = TB × (NS + 1)

#define SENS_AVG    0X38
#define SLP_CNT     0X3A
#define STATUS      0X3C
#define COMMAND     0X3E

#define TWOCOMP14(x) (((int16_t)(x)<0x2000)?(int16_t)(x):-(0x4000-(int16_t)(x)))

int ss=10; // vyberieme SPI zariadenie, kedže sa používa iba jedno bude stále  táto hodnota nastavená na 10


uint8_t hi_ax;
uint8_t lo_ax;
uint8_t hi_ay;
uint8_t lo_ay;
uint8_t hi_az;
uint8_t lo_az;

uint8_t hi_gx;
uint8_t lo_gx;
uint8_t hi_gy;
uint8_t lo_gy;
uint8_t hi_gz;
uint8_t lo_gz;

void setup()
{
 Serial.begin (230400);
 pinMode(ss, LOW); // používa sa pin 10
 SPI.begin(); // inicialuzuje sa SPI
 SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
}

void write_register(uint8_t reg, uint8_t hi, uint8_t lo)
{
  reg |= 0x80;
  digitalWrite(ss,LOW);
  delayMicroseconds(160);
  SPI.transfer(reg);
  delayMicroseconds(160);
  SPI.transfer(lo);
  delayMicroseconds(160);
  hi = SPI.transfer(reg + 1);
  delayMicroseconds(160);
  lo = SPI.transfer(hi);
  delayMicroseconds(160);
   digitalWrite(ss,HIGH);
   
}

void read_register(uint8_t reg)
{
  uint8_t hi = 0,lo = 0;
  digitalWrite(ss,LOW);
  delayMicroseconds(160);
  SPI.transfer(reg);
  delayMicroseconds(160);
  SPI.transfer(reg - 1);
  delayMicroseconds(160);
  hi = SPI.transfer(reg);
  delayMicroseconds(160);
  lo = SPI.transfer(reg - 1);
  delayMicroseconds(160);
   digitalWrite(ss,HIGH);
 Serial.write(hi);
 Serial.write(lo);   
}

void read_data()
{
  
  digitalWrite(ss,LOW);
  delayMicroseconds(160);
  SPI.transfer(XGYRO_OUT);
  delayMicroseconds(160);
  SPI.transfer(XGYRO_OUT - 1);
  delayMicroseconds(160);
  hi_gx = SPI.transfer(YGYRO_OUT);
  delayMicroseconds(160);
  lo_gx = SPI.transfer(YGYRO_OUT - 1);
  delayMicroseconds(160);
  hi_gy = SPI.transfer(ZGYRO_OUT);
  delayMicroseconds(160);
  lo_gy = SPI.transfer(ZGYRO_OUT - 1);
  delayMicroseconds(160);
  hi_gz = SPI.transfer(XACCL_OUT);
  delayMicroseconds(160);
  lo_gz = SPI.transfer(XACCL_OUT - 1);
  delayMicroseconds(160);
  hi_ax = SPI.transfer(YACCL_OUT);
  delayMicroseconds(160);
  lo_ax = SPI.transfer(YACCL_OUT - 1);
  delayMicroseconds(160);
  hi_ay = SPI.transfer(ZACCL_OUT);
  delayMicroseconds(160);
  lo_ay = SPI.transfer(ZACCL_OUT - 1);
  delayMicroseconds(160);
  hi_az = SPI.transfer(0x00);
  delayMicroseconds(160);
  lo_az = SPI.transfer(0x00);
  delayMicroseconds(160);
  digitalWrite(ss,HIGH);
  

  //Serial.println("gyro");
Serial.write(hi_gx);
Serial.write(lo_gx);
Serial.write(hi_gy);
Serial.write(lo_gy);
Serial.write(hi_gz);
Serial.write(lo_gz);

//Serial.println("akcel");
Serial.write(hi_ax);
Serial.write(lo_ax);
Serial.write(hi_ay);
Serial.write(lo_ay);
Serial.write(hi_az);
Serial.write(lo_az);
}

void read_temperature()
{

uint8_t hi_x;
uint8_t lo_x;
uint8_t hi_y;
uint8_t lo_y;
uint8_t hi_z;
uint8_t lo_z;
  
  digitalWrite(ss,LOW);
  delayMicroseconds(160);
  SPI.transfer(XTEMP_OUT);
  delayMicroseconds(160);
  SPI.transfer(XTEMP_OUT - 1);
  delayMicroseconds(160);
  hi_x = SPI.transfer(YTEMP_OUT);
  delayMicroseconds(160);
  lo_x = SPI.transfer(YTEMP_OUT - 1);
  delayMicroseconds(160);
  hi_y = SPI.transfer(ZTEMP_OUT);
  delayMicroseconds(160);
  lo_y = SPI.transfer(ZTEMP_OUT - 1);
  delayMicroseconds(160);
  hi_z = SPI.transfer(0x00);
  delayMicroseconds(160);
  lo_z = SPI.transfer(0x00);
  delayMicroseconds(160);
  digitalWrite(ss,HIGH);

//Serial.println("temp");
Serial.write(hi_x);
Serial.write(lo_x);
Serial.write(hi_y);
Serial.write(lo_y);
Serial.write(hi_z);
Serial.write(lo_z);
}

uint8_t incomingByte[4] = {0x00,0x00,0x00,0x00};

void loop()
{


if (Serial.available() > 0)  {
                // read the incoming byte:
                incomingByte[0] = Serial.read();
}

switch (incomingByte[0])
{
  case 0x01:
  while (Serial.available() <= 2); 
   incomingByte[1] = Serial.read();
   incomingByte[2] = Serial.read();
   incomingByte[3] = Serial.read();
   write_register(incomingByte[1],incomingByte[2],incomingByte[3]);
   break;
   
  case 0x02: 
  while (Serial.available() <= 0); 
  incomingByte[1] = Serial.read();
   read_register(incomingByte[1]);
  break;
  case 0x03: read_data();
          break;
  case 0x04: read_temperature();
        break;
    
}
incomingByte[0] = 0;


}
