// ArduCAM Mini demo (C)2018 Lee
// Web: http://www.ArduCAM.com
// This program is a demo of how to use the enhanced functions
// This demo was made for ArduCAM_Mini_2MP_Plus.
// It can  continue shooting and store it into the SD card  in JPEG format
// The demo sketch will do the following tasks
// 1. Set the camera to JPEG output mode.
// 2. Capture a JPEG photo and buffer the image to FIFO
// 3.Write the picture data to the SD card
// 5.close the file
//You can change the FRAMES_NUM count to change the number of the picture.
//IF the FRAMES_NUM is 0X00, take one photos
//IF the FRAMES_NUM is 0X01, take two photos
//IF the FRAMES_NUM is 0X02, take three photos
//IF the FRAMES_NUM is 0X03, take four photos
//IF the FRAMES_NUM is 0X04, take five photos
//IF the FRAMES_NUM is 0X05, take six photos
//IF the FRAMES_NUM is 0X06, take seven photos
//IF the FRAMES_NUM is 0XFF, continue shooting until the FIFO is full
//You can see the picture in the SD card.
// This program requires the ArduCAM V4.0.0 (or later) library and ArduCAM_Mini_2MP_Plus
// and use Arduino IDE 1.6.8 compiler or above

#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include <SD.h>
#include "memorysaver.h"
#include <Adafruit_BMP280.h>
#include <Adafruit_I2CDevice.h>
#include <RTClib.h>

//This demo can only work on OV5640_MINI_5MP_PLUS or OV5642_MINI_5MP_PLUS platform.
#if !(defined (OV2640_MINI_2MP_PLUS))
#error Please select the hardware platform and camera module in the ../libraries/ArduCAM/memorysaver.h file
#endif
#define   FRAMES_NUM    0x06

//Accelerameter pins
#define accel_xInput (A0)
#define accel_yInput (A1)
#define accel_zInput (A2)

const int sampleSize = 5;




//Temp and Pressure Sensor Pins
//#define BMP_SCK  (A5)
//#define BMP_MISO (12)
//#define BMP_MOSI (A4)
#define BMP_CS   (6)

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

File BMP_Data;

//solenoid pin
#define solenoidPin (4)

//led pin
#define ledPin (2)

// set pin 7 as the slave select for the digital pot:
const int CS = 7;
#define SD_CS 10
bool is_header = false;
int total_time = 0;
#if defined (OV2640_MINI_2MP_PLUS)
ArduCAM myCAM( OV2640, CS );
#endif
uint8_t read_fifo_burst(ArduCAM myCAM);

RTC_DS1307 rtc; 

void setup() {
 //Set all nonused pins to high so that they act as 5v voltage sources
pinMode(5, OUTPUT);
pinMode(8, OUTPUT);
//pinMode(9, OUTPUT);
digitalWrite(5, HIGH);
digitalWrite(8, HIGH);
//digitalWrite(9, HIGH);
  
  
  //For accelerameter to reference. Don't know if this is needed because connected to external 9v instead of 5v but think it still works
  analogReference(EXTERNAL);
  // put your setup code here, to run once:
  uint8_t vid, pid;
  uint8_t temp;
#if defined(__SAM3X8E__)
  Wire1.begin();
#else
  Wire.begin();
#endif
  Serial.begin(115200);
  Serial.println(F("ArduCAM Start!"));
  // set the CS as an output:
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  // initialize SPI:
  SPI.begin();
  //Reset the CPLD
myCAM.write_reg(0x07, 0x80);
delay(100);
myCAM.write_reg(0x07, 0x00);
delay(100);
  while (1) {
    //Check if the ArduCAM SPI bus is OK
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    temp = myCAM.read_reg(ARDUCHIP_TEST1);
    if (temp != 0x55)
    {
      Serial.println(F("SPI interface Error!"));
      delay(1000); continue;
    } else {
      Serial.println(F("SPI interface OK.")); break;
    }
  }
#if defined (OV2640_MINI_2MP_PLUS)
  while (1) {
    //Check if the camera module type is OV2640
    myCAM.wrSensorReg8_8(0xff, 0x01);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
    if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 ))) {
      Serial.println(F("ACK CMD Can't find OV2640 module!"));
      delay(1000); continue;
    }
    else {
      Serial.println(F("ACK CMD OV2640 detected.")); break;
    }
  }
#endif
  //Initialize SD Card
  while (!SD.begin(SD_CS))
  {
    Serial.println(F("SD Card Error!")); delay(1000);
  }
  Serial.println(F("SD Card detected."));
  //Change to JPEG capture mode and initialize the OV5640 module
  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  myCAM.clear_fifo_flag();
  myCAM.write_reg(ARDUCHIP_FRAMES, FRAMES_NUM);


//temp sensor start
unsigned status;
Serial.println("got here 1");
status = bmp.begin();
Serial.println("got here 2");
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);

    pinMode(solenoidPin, OUTPUT);
    pinMode(BMP_CS, OUTPUT);
  }

  /* Default settings from datasheet. */
  //bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  //Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  //Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  //Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  //Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */




//Set LED pin to output
pinMode(2, OUTPUT);

//rtc
  rtc.begin();

 //rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

}


void loop(){
  //rtc
   DateTime now = rtc.now();
   int time[6] = {now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second()};
  pictures(time); 

//convert analog input to m/s^2: (x - 512) * (.05753)
  int accel_x = (ReadAxis(accel_xInput)-512)*(.05753);
  int accel_y = (ReadAxis(accel_yInput)-512)*(.05753);
  int accel_z = (ReadAxis(accel_zInput)-512)*(.05753);
  int accel_total = sqrt((accel_x ^ 2) + (accel_y ^ 2) + (accel_z ^ 2));

 File AccelDat;
 
  AccelDat = SD.open("AccelDat.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (SD.exists("AccelDat.txt")) {
    Serial.print("Writing to AccelDat.txt...");
    for(int i = 0; i<6; i++){
    AccelDat.print(time[i]);
    AccelDat.print(",");
    }
    AccelDat.println("");
    AccelDat.println("accel_x = " + String(accel_x) + "m/s^2");
    AccelDat.println("accel_y = " + String(accel_y) + "m/s^2");
    AccelDat.println("accel_z = " + String(accel_z) + "m/s^2");
    AccelDat.println("accel_total = " + String(accel_total));
    // close the file:
    AccelDat.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening AccelDat.txt");
  }
    digitalWrite(BMP_CS, HIGH);
    long temperature = bmp.readTemperature();
    long pressure = bmp.readPressure();
    long altitude = bmp.readAltitude(1031.25);
    Serial.print(F("Temperature = "));
    Serial.print(temperature);/* Adjusted to local forecast! */
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(pressure);
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(altitude); 
    Serial.println(" m");

    Serial.println();
    delay(2000);

      //print to SD card
    File BMP_Data = SD.open("BMP_Data.txt", FILE_WRITE);
    for(int i = 0; i<6; i++){
    BMP_Data.print(time[i]);
    BMP_Data.print(",");
    }
    BMP_Data.println("");
    BMP_Data.print(bmp.readTemperature());
    BMP_Data.print(",");
    BMP_Data.print(bmp.readPressure());
    BMP_Data.print(",");
    BMP_Data.println(bmp.readAltitude(1013.25));
    //close the file
    BMP_Data.close();


//print to SD card solenoid status
    File solenoid = SD.open("solenoid.txt", FILE_WRITE);
    for(int i = 0; i<6; i++){
    solenoid.print(time[i]);
    solenoid.print(",");
    }
    solenoid.println("");

    //Condition makes sure we are at least within 2000m of altitude and the acceleration is either close to 0, 9.8 or -9.8. need to calibrate the accelerameter to see which one occurs
    if(altitude > 7000 && ((accel_total > -1 && accel_total < 1) || (accel_total < 10.8 && accel_total > 8.8) || (accel_total > -10.8 && accel_total < -8.8))){ 
      digitalWrite(solenoidPin,HIGH);
      solenoid.print("solenoid is: open");
      solenoid.close();
    }
    else {
      digitalWrite(solenoidPin,LOW);
      solenoid.print("solenoid is: closed");
      solenoid.close();
    }
 
}



//Camera Function that takes pictures and puts it on the SD card
void pictures(DateTime x) {
  //turn light on
  digitalWrite(ledPin, HIGH);

  //camera function
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
#if defined (OV2640_MINI_2MP_PLUS)
  myCAM.OV2640_set_JPEG_size(OV2640_1600x1200);
#endif
  //Start capture
  myCAM.start_capture();
  Serial.println(F("start capture."));
  total_time = millis();
  while ( !myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
  Serial.println(F("CAM Capture Done."));
  total_time = millis() - total_time;
  Serial.print(F("capture total_time used (in miliseconds):"));
  Serial.println(total_time, DEC);
  total_time = millis();
  read_fifo_burst(myCAM);
  total_time = millis() - total_time;
  Serial.print(F("save capture total_time used (in miliseconds):"));
  Serial.println(total_time, DEC);
  //Clear the capture done flag
  myCAM.clear_fifo_flag();
  delay(5000);
}

//Camera function necessary for ^^ to run
uint8_t read_fifo_burst(ArduCAM myCAM)
{
  uint8_t temp = 0, temp_last = 0;
  uint32_t length = 0;
  static int i = 0;
  static int k = 0;
  char str[16];
  File outFile;
  byte buf[256];
  length = myCAM.read_fifo_length();
  Serial.print(F("The fifo length is :"));
  Serial.println(length, DEC);
  if (length >= MAX_FIFO_SIZE) //8M
  {
    Serial.println("Over size.");
    return 0;
  }
  if (length == 0 ) //0 kb
  {
    Serial.println(F("Size is 0."));
    return 0;
  }
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();//Set fifo burst mode
  i = 0;
  while ( length-- )
  {
    temp_last = temp;
    temp =  SPI.transfer(0x00);
    //Read JPEG data from FIFO
    if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
    {
      buf[i++] = temp;  //save the last  0XD9
      //Write the remain bytes in the buffer
      myCAM.CS_HIGH();
      outFile.write(buf, i);
      //Close the file
      outFile.close();
      Serial.println(F("OK"));
      is_header = false;
      myCAM.CS_LOW();
      myCAM.set_fifo_burst();
      i = 0;
    }
    if (is_header == true)
    {
      //Write image data to buffer if not full
      if (i < 256)
        buf[i++] = temp;
      else
      {
        //Write 256 bytes image data to file
        myCAM.CS_HIGH();
        outFile.write(buf, 256);
        i = 0;
        buf[i++] = temp;
        myCAM.CS_LOW();
        myCAM.set_fifo_burst();
      }
    }
    else if ((temp == 0xD8) & (temp_last == 0xFF))
    {
      is_header = true;
      myCAM.CS_HIGH();
      //Create a avi file
      k = k + 1;
      itoa(k, str, 10);
      strcat(str, ".jpg");
      //Open the new file
      outFile = SD.open(str, O_WRITE | O_CREAT | O_TRUNC);
      if (! outFile)
      {
        Serial.println(F("File open failed"));
        while (1);
      }
      myCAM.CS_LOW();
      myCAM.set_fifo_burst();
      buf[i++] = temp_last;
      buf[i++] = temp;
    }
  }
  myCAM.CS_HIGH();

  //turn LED off
  digitalWrite(ledPin, LOW);

  
  return 1;
}

// Accelerameter Function
int ReadAxis(int axisPin) {
  long reading = 0;
  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++)
  {
    reading += analogRead(axisPin);
  }
  return reading / sampleSize;
}
