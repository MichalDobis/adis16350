
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
bool flag;
int ss=10; // vyberieme SPI zariadenie, kedže sa používa iba jedno bude stále  táto hodnota nastavená na 10
int ax, ay, az;
float aax, aay, aaz;
byte hix;
byte lox;
byte hiy;
byte loy;
byte hiz;
byte loz;

void setup()
{
 Serial.begin (9600);
 pinMode(ss, LOW); // používa sa pin 10
 SPI.begin(); // inicialuzuje sa SPI
 SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
 // senzor posle najskor MSB (most significant byte) bit ako prvý
//digitalWrite (SMPL_PRD, 0x0A);
//SPI.transfer(0x0a);  // set the IMU to read so the next command will get valid data
//SPI.transfer(0x0b);
}



void loop()
{
/*
digitalWrite(ss,LOW);
delayMicroseconds(160);
SPI.transfer(0x0B);
delayMicroseconds(160);
SPI.transfer(0x0A);
delayMicroseconds(160);
hi = SPI.transfer(0x00);
delayMicroseconds(160);
lo = SPI.transfer(0x00);
delayMicroseconds(160);
digitalWrite(ss,HIGH);

delayMicroseconds(160);

digitalWrite(ss,LOW);
SPI.transfer(0x0D);
delayMicroseconds(160);
SPI.transfer(0x0C);
delayMicroseconds(160);
Ba = SPI.transfer(0x00);
delayMicroseconds(160);
Bb = SPI.transfer(0x00);
delayMicroseconds(160);
digitalWrite(ss,HIGH);

delayMicroseconds(160);

digitalWrite(ss,LOW);
delayMicroseconds(160);
SPI.transfer(0x0F);
delayMicroseconds(160);
SPI.transfer(0x0E);
delayMicroseconds(160);
Bc = SPI.transfer(0x00);
delayMicroseconds(160);
Bd = SPI.transfer(0x00);
delayMicroseconds(160);
digitalWrite(ss,HIGH);
*/

digitalWrite(ss,LOW);
delayMicroseconds(160);
SPI.transfer(0x0B);
delayMicroseconds(160);
SPI.transfer(0x0A);
delayMicroseconds(160);
hix = SPI.transfer(0x0D);
delayMicroseconds(160);
lox = SPI.transfer(0x0C);
delayMicroseconds(160);
hiy = SPI.transfer(0x0F);
delayMicroseconds(160);
loy = SPI.transfer(0x0E);
delayMicroseconds(160);
hiz = SPI.transfer(0x00);
delayMicroseconds(160);
loz = SPI.transfer(0x00);
delayMicroseconds(160);
digitalWrite(ss,HIGH);

ax = ((hix & 0b00111111) *256)+lox;

if(ax&0b100000000000)
ax=-1*(~ax+1)-16384;
else
ax=-(~ax+1);
aax = ax*0.002522*9.81;

ay = ((hiy & 0b00111111) *256)+loy;
if(ay&0b100000000000)
ay=-1*(~ay+1)-16384;
else
ay=-(~ay+1);
aay =ay*0.002522*9.81;

az = ((hiz & 0b00111111) *256)+loz;
if(az&0b100000000000)
az=-1*(~az+1)-16384;
else
az=-(~az+1);
aaz =az*0.002522*9.81;/*
Serial.println("Byty hi  low  1   2   3   4");
Serial.print("    ");
Serial.print(hi, HEX);
Serial.print("  ");
Serial.print(lo, HEX);
Serial.print("  ");
Serial.print(Ba, HEX);
Serial.print("  ");
Serial.print(Bb, HEX);
Serial.print("  ");
Serial.print(Bc, HEX);
Serial.print("  ");
Serial.println(Bd, HEX);*/
Serial.println((float)(aax),3);
Serial.println((float)(aay),3);
Serial.println((float)(aaz),3);/*
Serial.println(ax,DEC);
Serial.println(ay,DEC);
Serial.println(az,DEC);*/
delay(300);

}
