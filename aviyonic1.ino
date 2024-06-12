#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <Wire.h>
//#include <TinyGPS.h>
#include <TinyGPS++.h>
#include <Adafruit_MPU6050.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#define INTERRUPT_PIN 2

MPU6050 mpu;
Adafruit_MPU6050 mpu2;
Adafruit_BMP280 bmp;
//TinyGPS gps;
//SoftwareSerial pins(17,16); //rx tx
TinyGPSPlus gps;
//Mpu6050 sensör başlangıç
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

struct Orientation {
  double Yaw;
  double Pitch;
  double Roll;
  bool Error;
};

struct Orientation prevOrientation;

void initializeIMU() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(-4062);
  mpu.setYAccelOffset(-2219);
  mpu.setZAccelOffset(1622);
  mpu.setXGyroOffset(-42);
  mpu.setYGyroOffset(-20);
  mpu.setZGyroOffset(4);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
  }
}

struct Orientation getIMUOrientation() {
  if (!dmpReady || !mpu.testConnection()) {
    struct Orientation o;
    o.Yaw = 0;
    o.Pitch = 0;
    o.Roll = 0;
    o.Error = true;
    return o;
  }

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    struct Orientation o;
    o.Yaw = ypr[0] * 180 / M_PI;
    o.Pitch = ypr[1] * 180 / M_PI;
    o.Roll = ypr[2] * 180 / M_PI;
    o.Error = false;

    prevOrientation.Yaw = o.Yaw;
    prevOrientation.Pitch = o.Pitch;
    prevOrientation.Roll = o.Roll;

    return o;
  } else {
    return prevOrientation;
  }
}
//Mpu6050 sensör son

float temel_basinc, irtifa;
float irtifa_;

//Kurtarma pinleri tanımlaması
int k1 = A1;
int k2 = A2;
int buzzer = 6;
//kurtarma durumları false olarak ayarlanır.
bool k1_durum = false;
bool k2_durum = false;
bool buzzerDurum = false;

unsigned long ilk_zaman = 0;
const long ara_zaman = 500;
unsigned long anlik_zaman;
byte kurtarma_durum = 0;
byte kurtarma_durum2 = 0;

String veri_paket = "";

float bmp_180_irtifa_bulma(float ilk_basinc, float guncel_basinc)
{
  float irtifa = 44330.0 * (1 - pow((float)guncel_basinc / (float)ilk_basinc, 1 / 5.255));
  return irtifa;  
}

bool apogee_nokta(float irtifa1,float irtifa2)
{
  float fark = irtifa2 - irtifa1;
  delay(60);
  if (irtifa2>5)//if ((fark <= -10) && (irtifa2 < irtifa1))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  bmp.begin();
  pinMode(k1,OUTPUT);
  pinMode(k2,OUTPUT);
  pinMode(buzzer,OUTPUT);
  Serial.println("BMP180 test");
  
  if(!mpu2.begin())
  {
    Serial.println("MPU6050 Sensörü Algılanmadı! Kontrolleri Yapınız.");
    while(!mpu2.begin());
    Serial.println("MPU6050 Agılandı.");
  }
  else
  {
    Serial.println("MPU6050 Agılandı.");
  }
  if(!bmp.begin()) //Sensör algılama testi
  {
    Serial.println("BMP280 Sensörü Algılanamadı! Kontrolleri Yapınız.");
    while(!bmp.begin());
    Serial.println("BMP280 Algılandı."); 
  }
  else
  {
    Serial.println("BMP280 Algılandı.");
  }
  //MPU6050
  initializeIMU();
  temel_basinc = bmp.readPressure(); 

  //kalman filtrelemesi
}

void loop() {
  gpsBekleme(1000);
  String paket;
  float enlem,boylam,gps_irtifa;
  bool gpsDurum;
  //unsigned long age;
  if(buzzerDurum == false)
  {
    digitalWrite(buzzer,HIGH);
    delay(2000);
    digitalWrite(buzzer,LOW);
    buzzerDurum = true;
  }
  float anlik_basinc = bmp.readPressure();
  float sicaklik = bmp.readTemperature();
  irtifa = irtifa_;
  irtifa_ = bmp_180_irtifa_bulma(temel_basinc, anlik_basinc);
  //Mpu6050 roll,pitch,yaw
  struct Orientation o = getIMUOrientation();
  
  //GPS verilerini çekme
  //gps.f_get_position(&enlem, &boylam, &age);
  gpsDurum = gps.location.isValid();
  if(gpsDurum)
  {
    enlem = gps.location.lat(); //enlem
    boylam = gps.location.lng(); //boylam
    gps_irtifa = gps.altitude.meters();
  }

  sensors_event_t a, g, temp_mpu;
  mpu2.getEvent(&a, &g, &temp_mpu);
  
  anlik_zaman = millis();
  //Aşağıdaki kod bloğu mpu6050 ile kurtarma tetiklemesi içindir.
  /*if(o.Roll >= 0)
  {
    if(kurtarma_durum == 0)
    {
      kurtarma_durum = 255;
    }
    analogWrite(k1,kurtarma_durum);
    kurtarma_durum == 0 ? k1_durum = false : k1_durum = true;
  }*/

  if(irtifa_ >= 0)
  {
    if (apogee_nokta(irtifa,irtifa_))
    {
      unsigned long anlik_zaman = millis();
      if(!k1_durum)
      {
        if (anlik_zaman - ilk_zaman >= ara_zaman)
        {
          ilk_zaman = anlik_zaman;
          if(kurtarma_durum == 0)
          {
            kurtarma_durum = 255;
          }
          else {kurtarma_durum == 0;}
          analogWrite(k1,kurtarma_durum);
          k1_durum = true;
          //kurtarma_durum == 0 ? k1_durum = false : k1_durum = true;
        }
      }   
    }
  }

  //delay(10);
  
  if((irtifa_ <500.0) && (k1_durum) && (!k2_durum))
  {
    delay(15);
    if(kurtarma_durum2 == 0)
    {
      kurtarma_durum2 = 255;
    }
    else{kurtarma_durum2 = 0;}
    analogWrite(k2,kurtarma_durum2);
    k2_durum = true;  
  }
  
  String kurtarma1 = !k1_durum ? "0" : "1";
  String kurtarma2 = !k2_durum ? "0" : "1";
  String genel_kurtarma = kurtarma1 + kurtarma2;
  //delay(50);
  paket += String(enlem,6);
  paket += ",";
  paket += String(boylam,6);
  paket += ",";
  paket += String(long(anlik_basinc));
  paket += ",";
  paket += String(irtifa);
  paket += ",";
  paket += String(sicaklik);
  paket += ",";
  paket += String(int(g.gyro.x));
  paket += ",";
  paket += String(int(g.gyro.y));
  paket += ",";
  paket += String(int(g.gyro.z));
  paket += ",";
  paket += String(a.acceleration.x);
  paket += ",";
  paket += String(a.acceleration.y);
  paket += ",";
  paket += String(a.acceleration.z);
  paket += ",";
  paket += String(int(o.Roll));
  paket += ",";
  paket += String(int(o.Yaw));
  paket += ",";
  paket += String(int(o.Pitch));
  paket += ",";
  paket += genel_kurtarma;
  paket += ",";
  paket += String(gps_irtifa);
  Serial.println(paket);
  delay(500);
}

static void gpsBekleme(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (Serial2.available())
      gps.encode(Serial2.read());
  } while (millis() - start < ms);
}
