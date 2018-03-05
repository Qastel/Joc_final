# Joc_final

Proiect - aspirator de camera

Aspiratorul trebuie sa ocoleasca obiecte, sa nu se loveasca de ziduri si sa aspire comparabil cu un aspirator adevarat.
Ca sa acopere toata incaperea trebuie sa mapeze, sa stie unde nu a fost si unde se afla.


///IMU comment

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialiactiveation code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialiactiveation working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, active] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet siactivee (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, active]         quaternion container
VectorInt16 aa;         // [x, y, active]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, active]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, active]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, active]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}




///MOTORS
int enableStang = 11;
int in1Stang = 10;
int in2Stang = 9;
int in1Drept = 6;
int in2Drept = 5;
int enableDrept = 3;
int speed = 255;
boolean reverse = 1;


///ULTRASONICC SENSORS
const int trigLeftFront = 13; //era 12
const int echoLeftFront  = 12; // era 13
const int trigRightFront = 8;
const int echoRightFront = 7;
const int trigMiddleFrontRight = 17; // era 4
const int echoMiddleFrontRight = 4; //era 2
const int trigMiddleFrontLeft = 14;
const int echoMiddleFrontLeft = 15;
long durationLeftFront;
int distanceLeftFront;
long durationRightFront;
int distanceRightFront;
long durationMiddleFrontRight;
int distanceMiddleFrontRight;
long durationMiddleFrontLeft;
int distanceMiddleFrontLeft;


///MAP
int mat[20][20];
int a=10;
int b=10;

///BUTTON
int switchPin = 16;

///VARIABLES
unsigned long pMil = 0; 
int prim2=0;
int ok2=0;
unsigned long pM=0;
int inclusivStart = 2000;
int x=0;
int y=1;
int ok3 = 0;

int var = 0;
int prim3 = 0;
float deviation = 0;
float straight = 0;

int buttonPushCounter = 1;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0; 
unsigned long  wait = 0;
float dif = 0; 
float angle = 0;

enum Turning{ first, second, third, zero};
enum ChangeTurn{set, notSet};
enum Dual {enable = 1 , disable = 0};

ChangeTurn changeTurn;
Turning turn = first;
Dual turnOnGoing = disable;
Dual zMode = disable;
Dual firstInitializeStraight = disable;
Dual firstInitializeTime = disable;
Dual maping = disable;

int timeImuStabilization = 20000;

enum Button{ stay = 1, maping = 2, z = 3};

void setup()
{
    Serial.begin(115200);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHactive I2C clock (200kHactive if CPU is 8MHactive)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
  
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
   // while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) 
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else 
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")")); 
    }
 
  pinMode(in1Stang, OUTPUT);
  pinMode(in2Stang, OUTPUT);
  pinMode(enableStang, OUTPUT);
  pinMode(in1Drept, OUTPUT);
  pinMode(in2Drept, OUTPUT);
  pinMode(enableDrept, OUTPUT);
  pinMode(trigLeftFront, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoLeftFront, INPUT); // Sets the echoPin as an Input
  pinMode(trigRightFront, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoRightFront, INPUT); // Sets the echoPin as an Input
  pinMode(trigMiddleFrontRight, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoMiddleFrontRight, INPUT); 
  pinMode(trigMiddleFrontLeft, OUTPUT); 
  pinMode(echoMiddleFrontLeft, INPUT); 
  pinMode(switchPin, INPUT);
 
}



void loop()
{
  unsigned long cMil_IMU = millis();
  
  setIMU();
  
  if(cMil_IMU > timeImuStabilization)
  {

    readButto();
    
    if (buttonPushCounter % 3 == 1 )
    {
       stopMotors();
    }
  
    else
    if (buttonPushCounter % 3 == 2) 
    {
      zMode = disable ; 
      readSensors();
     
      unsigned long cMil = millis();

      detectObject();
    }
    else
    if (buttonPushCounter % 3 == 0) 
    {
     zMode = enable ; 
     zModeFunc();
    }
      
  }
}


void detectObject()
{
  
  if(distanceLeftFront <= 5 || distanceRightFront <= 5 || distanceMiddleFrontRight <= 5 || distanceMiddleFrontLeft <= 5 || turnOnGoing == enable) /// daca ok2 == 1 sau distanta mai mica trebuie sa intoarca
  {
          
        if(firstInitializeTime == disable)
        {
          pMil = cMil;
          firstInitializeTime = enable;
          turnOnGoing = enable; 
        }

        angle = ypr[0] * 180/M_PI;
        if((straight + 360 - angle) > 360)
          dif = straight + 360 - angle - 360;
        else 
          dif = straight + 360 - angle;
        
        if(dif > 300)
          dif = 0;
       
             
        if(cMil - pMil < 2750)//dif  < 77.20 ) // val < angle && val>-180 && val<0 && angle>-180 && angle<0  || val < angle && val<180 &&val>0 && angle<180 &&angle>0 || val>angle &&val<180 &&val>0 &&angle>-180 && angle<0 || val < angle && val>-180 && val<0 && angle<180 && angle>0   )  /// intoarce la 90 timp de 2800 , straight - angle) < 74 
        { 
                
           turnLeftMotor(speed, reverse);
           mapingDirection();         
        }
               
        else if(firstInitializeTime == enable)  /// daca a depasit timpul se reinitializeaza
        {   
          stopMotors();
          reinitializeVar();
                 
          /*
            for(int i = 0; i <= 19; i++)  // afisare aici intarzie intoarcerea
                 { 
                    for(int j=0; j <= 19; j++)
                    {  
                      Serial.print(mat[i][j]);
                      Serial.print(" ");
                    }
                  Serial.println();
                 }                 
          */

         //WAIT 1 sec after turning
         wait = cMil;      
       }
  }
    
    
  if(turnOnGoing == disable && cMil - wait > 1000)  /// se poate misca inainte
  {
    setForwardMotor(speed, reverse) 
  }
       
}


void reinitializeVar()
{
  firstInitializeTime = disable;
  turnOnGoing = disable;
  changeTurn = notSet;
  firstInitializeStraight = disable;
  dif = 0;
  angle = 0;
}


void mapingDirection()
{
  if(turn == first && changeTurn == notSet)   /// pentru mapare
    {  
       x = -1;
       y = 0;
       turn = second;
       changeTurn = set;
    }
    if(turn == second && changeTurn == notSet)
    {
      x = 0;
      y = -1;
      turn = third;
      changeTurn = set;
    }
    if(turn == third && changeTurn == notSet)
    {
      x = 1;
      y = 0;
      turn = zero; 
      changeTurn = set;
    }
    if(turn == zero && changeTurn == notSet)
    {
      x = 0;
      y = 1;
      turn = first;
      changeTurn = set; 
    }
}

void readButo()
{
  buttonState = digitalRead(switchPin);
    // compare the buttonState to its previous state
    if (buttonState != lastButtonState) 
    {
      // if the state has changed, increment the counter
      if (buttonState == HIGH) 
      {
          // if the current state is HIGH then the button went from off to on:
          buttonPushCounter++;
          //  Serial.println("on");
          //  Serial.print("number of button pushes: ");
          //  Serial.println(buttonPushCounter);
      } 
      else 
      {
        // if the current state is LOW then the button went from on to off:
        //  Serial.println("off");
      }
      // Delay a little bit to avoid bouncing
      //delay(50);
    }
     // save the current state as the last state, for next time through the loop
    lastButtonState = buttonState;
}

/// SENZORI
void readSensors()
{ 
  
  /// ULTRASONIC SENSORS
  digitalWrite(trigLeftFront, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigLeftFront, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigLeftFront, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationLeftFront = pulseIn(echoLeftFront, HIGH);
  // Calculating the distance
  distanceLeftFront = (durationLeftFront*.0343)/2;

  digitalWrite(trigRightFront, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigRightFront, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigRightFront, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationRightFront = pulseIn(echoRightFront, HIGH);
  // Calculating the distance
  distanceRightFront = (durationRightFront*.0343)/2;

  
  digitalWrite(trigMiddleFrontRight, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigMiddleFrontRight, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigMiddleFrontRight, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationMiddleFrontRight = pulseIn(echoMiddleFrontRight, HIGH);
  // Calculating the distance
  distanceMiddleFrontRight = (durationMiddleFrontRight*.0343)/2;

  digitalWrite(trigMiddleFrontLeft, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigMiddleFrontLeft, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigMiddleFrontLeft, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationMiddleFrontLeft = pulseIn(echoMiddleFrontLeft, HIGH);
  // Calculating the distance
  distanceMiddleFrontLeft = (durationMiddleFrontLeft*.0343)/2;  
  
}


///STOP
void stopMotors()
{
   analogWrite(enableStang, 0);
   analogWrite(enableDrept, 0);  
}





///FORWARD
void setForwardMotor(int speed, boolean reverse)
{
    
  if(firstInitializeStraight == disable) /// retine prima valoare cand merge in linie dreapta, ca sa o pastreze
  {
     straight = ypr[0] * 180/M_PI; //Serial.println(straight); Serial.println("daaaaa");
     firstInitializeStraight = enable;
  }

    
  deviation = ypr[0] * 180/M_PI;
  if(abs(straight - deviation) > 0.2)
  {
    if((straight - deviation) < 0)
    { 
      analogWrite(enableStang, 180); // 83
      analogWrite(enableDrept,255);

    }
    else
    {
      analogWrite(enableStang, 255); 
      analogWrite(enableDrept, 180); //100
  
      }
  }

  else
  {  
    analogWrite(enableStang, 255);
    digitalWrite(in1Stang, ! reverse);
    digitalWrite(in2Stang, reverse);
                
    analogWrite(enableDrept, 255);
    digitalWrite(in1Drept, !reverse);
    digitalWrite(in2Drept, reverse);  
  }
     
  unsigned long cMil = millis();
  if(maping == disable && zMode == disable)  /// reactualizare mapare
  {
       pM = cMil - inclusivStart;
       maping = enable;
  }
  
   if(cMil - pM >= 2000 && zMode == 0) /// mapeaza la fiecare 2 s
   { 
           a = a + x;
           b = b + y;
           
           if( mat[a][b] == 1 ) /// daca a fost parcurs traseul deja
           {
              turnOnGoing = enable; 
              a = a - x; //de unde a venit
              b = b -y;
           } 

          if(mat[a][b] == 1 && mat[a-y][b+x] == 1 && mat[a+x][b+y] == 1)  /// daca patratul de parcurgere se termina , verifica in fata si stanga pentru cele 4 pozitii, sus, stanga, jos, dreapta
          {
            buttonPushCounter ++ ;  
            buttonPushCounter ++ ; 
          }
           
           
          if(b != 20 || a != -1 || b != -1 || a != 20)  /// daca nu trece de limitele patratului mapeaza
               mat[a][b] = 1;
          
          maping = disable;  /// dupa 2 s se poate initializa pM pentru a astepta o noua initializare la 2 s 
          inclusivStart = 0;
   }


   /// daca a trecut de limitele patratului
        
   if(b == 20)
   {
        turnOnGoing = 1;  /// sa intre pe ramura de intoarcere
        b = 19;    /// sa ramana pe limita patratului
   }
   else if(a == -1  )
   {
         turnOnGoing = 1;
         a = 0;
   }
   else if(b == -1 )
   {
        turnOnGoing = 1;
        b = 0;   
   }
   else if( a == 20)
   {
        turnOnGoing = 1;
        a = 19;
   }
          
}




///TURN LEFT
void turnLeftMotor(int speed, boolean reverse)
{
  analogWrite(enableStang, speed);
  digitalWrite(in1Stang,  reverse);
  digitalWrite(in2Stang, !reverse);
  
  analogWrite(enableDrept, speed);
  digitalWrite(in1Drept, !reverse);
  digitalWrite(in2Drept, reverse);
}

void turnRightMotor(int speed, boolean reverse)
{
  analogWrite(enableStang, speed);
  digitalWrite(in1Stang,  reverse);
  digitalWrite(in2Stang, !reverse);
  
  analogWrite(enableDrept, speed);
  digitalWrite(in1Drept, !reverse);
  digitalWrite(in2Drept, reverse);
}



///IMU
void setIMU(){

  // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
         
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif
    }
  
  
  }









///TRACKING Z
int turnRight135 = 4200; // in time
int turnLeft45 = 1400;
enum directio {left = 1, right = -1 };
directio dirPrio;

 void zModeFunc()
{
   readSensors();
   unsigned long cMil = millis();
   if(distanceLeftFront <= 5 || distanceRightFront <= 5 || distanceMiddleFrontRight <= 5 || distanceMiddleFrontLeft <= 5 || ok2 == 1) /// daca ok2 == 1 sau distanta mai mica trebuie sa intoarca
   {
    
      if(firstInitializeTime == 0)
      {
          pMil = cMil;
          firstInitializeTime = 1;
          turnOnGoing = 1; 
      }

         
      if(cMil-pMil  < turnRight135 && dirPrio == right  ) // val < angle && val>-180 && val<0 && angle>-180 && angle<0  || val < angle && val<180 &&val>0 && angle<180 &&angle>0 || val>angle &&val<180 &&val>0 &&angle>-180 && angle<0 || val < angle && val>-180 && val<0 && angle<180 && angle>0   )  /// intoarce la 90 timp de 2800 , straight - angle) < 74 
      { 
            
          turnRightMotor(speed, !reverse);
             
      }
      else 
      if(cMil-pMil  < turnLeft45 && dirPrio == left)
          turnRightMotor(speed, reverse);
      else 
      if(firstInitializeTime == enable)  /// daca a depasit timpul se reinitializeaza
      {   
          stopMotors();
          firstInitializeTime = disable;
          turnOnGoing = disable;
          firstInitializeStraight = 0; 
          dirPrio = dirPrio * (-1);
      }
           
         wait = cMil;
        
   }
  
   if(ok2 == 0 && cMil - wait > 1000 )  /// se poate misca inainte
   {
       
      setForwardMotor(speed, reverse);
   }

    
} 



