#include <SPI.h>
#include "config.h"

// using two incompatible SPI devices, A and B. Incompatible means that they need different SPI_MODE

const int slaveAPin = 21;
int send = 0;
unsigned long actual;
unsigned long ant;
unsigned long millis();
const int tiempo = 15000;


SPISettings settingsA(8000000, MSBFIRST, SPI_MODE1);


AdafruitIO_Feed *secondsf = io.feed("secondsf");
AdafruitIO_Feed *minutesf = io.feed("minutesf");
AdafruitIO_Feed *hoursf = io.feed("hoursf");
AdafruitIO_Feed *buttf = io.feed("buttf");
AdafruitIO_Feed *tempf = io.feed("tempf");
AdafruitIO_Feed *potf = io.feed("potf");




void setup() {

  // set the Slave Select Pins as outputs:

  pinMode (slaveAPin, OUTPUT);
  SPI.begin();
  Serial.begin(115200);
  // initialize SPI:

  //conectar a WiFi 

  Serial.print("Connecting to WiFi");
  WiFi.begin("UVG","");
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println(" Connected!");

  // wait for serial monitor to open
  while(! Serial);
  Serial.print("Connecting to Adafruit IO");
  // connect to io.adafruit.com
  io.connect();
  // wait for a connection
    while(io.status() < AIO_CONNECTED) {
    Serial.println(io.statusText());
    delay(500);
  }
  
  Serial.println();
  Serial.println(io.statusText());
 

}

uint8_t hours, minutes, seconds, pot, butt, temp;

void loop() {
  
  io.run();

  SPI.beginTransaction(settingsA);
  
  digitalWrite (slaveAPin, HIGH);
  delay(10);
  digitalWrite (slaveAPin, LOW);
  delay(10);
  hours = SPI.transfer(1);
  delay(10);
 
  digitalWrite (slaveAPin, HIGH);
  delay(10);
  digitalWrite (slaveAPin, LOW);
  delay(10);
  minutes = SPI.transfer(2);
  delay(10);

  digitalWrite (slaveAPin, HIGH);
  delay(10);
  digitalWrite (slaveAPin, LOW);
  delay(10);
  seconds = SPI.transfer(3);
  delay(10);

  digitalWrite (slaveAPin, HIGH);
  delay(10);
  digitalWrite (slaveAPin, LOW);
  delay(10);
  pot = SPI.transfer(4);
  delay(10);

  digitalWrite (slaveAPin, HIGH);
  delay(10);
  digitalWrite (slaveAPin, LOW);
  delay(10);
  butt = SPI.transfer(5);
  delay(10);

  digitalWrite (slaveAPin, HIGH);
  delay(10);
  digitalWrite (slaveAPin, LOW);
  delay(10);
  temp = SPI.transfer(0);
  delay(10);

 
  
  SPI.endTransaction();
  


   actual = millis();
  if (actual - ant >= tiempo) {
    ant = actual;
    send = 1;
  }

  if (send == 1) {
    Serial.print("sending seconds -> ");
    Serial.println(seconds);
    secondsf->save(seconds);
    Serial.print("sending minutes -> ");
    Serial.println(minutes);
    minutesf->save(minutes);
    Serial.print("sending hours -> ");
    Serial.println(hours);
    hoursf->save(hours);
    Serial.print("sending temp -> ");
    Serial.println(temp);
    tempf->save(temp);
    Serial.print("sending butt -> ");
    Serial.println(butt);
    buttf->save(butt);
    Serial.print("sending pot -> ");
    Serial.println(pot);
    potf->save(pot);
    send = 0;
  }
}
