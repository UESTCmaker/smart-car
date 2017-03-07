#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include <LSM303.h>
#include <NewPing.h>

/*******************Args Configuration***************************/
float LeftCountDisAgs = 1;
float RighCountDisAgs = 1;
float LeftTurnDisAgs = 0.8;
float RightTurnDisAgs = 0.85;

/****************Sonar Configuration****************************/
#define SONAR_NUM     3 // Number of sensors.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define TRIGGER3_PIN  16  // Arduino pin tied to trigger pin son the ultrasonic sensor.
#define ECHO3_PIN     15  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER2_PIN  14
#define ECHO2_PIN     13
#define TRIGGER1_PIN  12
#define ECHO1_PIN     11
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM] = {0};         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
volatile int Sonarflag = 0;              // Confirm whether a cycle of sonar detection has been processed
NewPing sonar[SONAR_NUM] = {
  NewPing(TRIGGER1_PIN, ECHO1_PIN, MAX_DISTANCE), // NewPing setup of pins and maximum distance.
  NewPing(TRIGGER2_PIN, ECHO2_PIN, MAX_DISTANCE),
  NewPing(TRIGGER3_PIN, ECHO3_PIN, MAX_DISTANCE)
};// NewPing setup of pins and maximum distance.

/****************Motor Encoder Configuration*********************/
const byte interruptPinMotorLeft = 2;
const byte interruptPinMotorRight = 3;
volatile int InterruptNumLeft = 0;
volatile int InterruptNumRight = 0;
float CircleNumLeft = 0.0;
float CircleNumRight = 0.0;

/****************Motor Control Configuration*********************/
volatile int speedflag = 0;

/****************Compass Configuration***************************/
LSM303 compass;
int heading = 1;
volatile int initflag = 0;

/****************Serial Configuration***************************/
byte command = 0x00;
byte ack = 0x11;
int angle = 0;

/****************Function Declaration***************************/
void InterruptDisLeft(void);
void InterruptDisRight(void);
void printInfo(void);
void turnleft(int angle, int signalx);
void turnright(int angle, int signalx);
void startrun(int signalx);
void stoprun(int signalx);
void backrun(void);
void echoCheck(void);
void oneSensorCycle(void);
void serialcheck(void);
void faceNorth(void);
void ObstacleAvoidance(int oa);

void SonarDetectionTask( void *pvParameters );
void CompassTask( void *pvParameters );
void SerialReadTask( void *pvParameters );
void ObstacleAvoidanceTask(void *pvParameters);
void SerialPrintTask( void *pvParameters );

void setup() {

  Serial.begin(9600);
  Serial1.begin(9600);
  /*Compass init*/
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>) {
    -1340,  -1837,  -1810
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +3134,  +2508,  +2451
  };
  /*Motor init*/
  for (int i = 4; i <= 7; i++) {//Pin 4 to 7 are used
    pinMode(i, OUTPUT);
  }
  /*Motor Encoder init*/
  pinMode(interruptPinMotorLeft, INPUT_PULLUP);
  pinMode(interruptPinMotorRight, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinMotorLeft), InterruptDisLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(interruptPinMotorRight), InterruptDisRight, FALLING);

  // Now set up two tasks to run independently.
  xTaskCreate(SonarDetectionTask,  (const portCHAR *)"SonarDetectionTask",  128,  NULL,  4,  NULL);
  xTaskCreate(CompassTask,  (const portCHAR *) "CompassTask",  128 ,  NULL,  3,  NULL );
  xTaskCreate(SerialReadTask,  (const portCHAR *) "SerialReadTask",  128 ,  NULL,  5,  NULL );
  xTaskCreate(ObstacleAvoidanceTask,  (const portCHAR *)"ObstacleAvoidanceTask",  128,  NULL,  6,  NULL);
  //xTaskCreate(SerialPrintTask,  (const portCHAR *) "SerialPrintTask",  128 ,  NULL,  2,  NULL );

  // The RTOS scheduler is automatically started when
  // setup() finishes

  //faceNorth();
  serialcheck();
  Serial.println("Start now!");

}

void loop()
{
  //Empty. The tasks contains all processing
}

void ObstacleAvoidanceTask(void *pvParameters) {
  while (1) {
    if (Sonarflag = 1) {
      unsigned char oa = 0x0;
      for (uint8_t i = 0; i < SONAR_NUM; i++) {
        oa  = oa << 1;
        if (cm[i] >= 10 && cm[i] <= 80) {
          oa |= 1;
        }
      }
      Serial.print("case: ");
      Serial.println(oa);
      ObstacleAvoidance(oa);
      Sonarflag = 0;
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void SonarDetectionTask(void *pvParameters)  // This is a task.
{
  while (1) {
    if (Sonarflag == 0) {
      for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
        currentSensor = i;                          // Sensor being accessed.
        cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
        cm[i] = sonar[i].ping_cm();
        vTaskDelay(40 / portTICK_PERIOD_MS);
      }
      if (currentSensor == SONAR_NUM - 1) {
        Sonarflag = 1; // Sensor ping cycle complete, turn flag to 1.
      }
    }
  }
}

void CompassTask(void *pvParameters)  // Another task
{
  while (1)  {
    compass.read();
    heading = compass.heading();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
void SerialReadTask(void *pvParameters) {
  while (1) {
    if (Serial1.available()) {//1
      command = Serial1.read();//1
      //Serial.println(command);
      if (command == 0x21) {
        byte headingdata[2];
        if (heading > 255) {
          headingdata[0] = 255;
          headingdata[1] = heading % 255;
        }
        else {
          headingdata[0] = 0;
          headingdata[1] = heading;
        }
        Serial1.write(headingdata, 2);
        command = 0x00;
      }
      if (command == 0x22) {
        for (uint8_t i = 0; i < SONAR_NUM; i++) {
          Serial1.write(cm[i]);//1
        }
        command = 0x00;
      }
      if (command == 0x23) {
        int disleft;
        byte disleftbyte[2];
        CircleNumLeft = float(InterruptNumLeft / 192.0);
        disleft = CircleNumLeft * 20.7 * LeftCountDisAgs;
        if (disleft > 255) {
          disleftbyte[0] = disleft / 256;
          disleftbyte[1] = disleft % 256;
        }
        else {
          disleftbyte[0] = 0;
          disleftbyte[1] = disleft;
        }
        Serial1.write(disleftbyte, 2);
        command = 0x00;
      }
      if (command == 0x24) {
        int disright ;
        byte disrightbyte[2];
        CircleNumRight = float(InterruptNumRight / 192.0);
        disright = CircleNumRight * 20.7 * RighCountDisAgs;
        if (disright > 255) {
          disrightbyte[0] = disright / 256;
          disrightbyte[1] = disright % 256;
        }
        else {
          disrightbyte[0] = 0;
          disrightbyte[1] = disright;
        }
        Serial1.write(disrightbyte, 2);
        command = 0x00;
      }
      if (command == 0x26) {
        InterruptNumLeft = 0;
        InterruptNumRight = 0;
        Serial1.write(ack);
        command = 0x00;
      }
      if (command == 0x41) {
        stoprun(1);
        command = 0x00;
      }
      if (command == 0x42) {
        while (angle == -1 || angle == 0) {
          angle = Serial1.read();  //1
          delay(2);
        }
        turnleft(angle, 1);
        angle = 0;
        command = 0x00;
      }
      if (command == 0x43) {
        while (angle == -1 || angle == 0) {
          angle = Serial1.read();  //1
          delay(2);
        }
        turnright(angle, 1);
        angle = 0;
        command = 0x00;
      }
      if (command == 0x44) {
        //speedflag += 51;
        speedflag = 255;
        startrun(1);
        command = 0x00;
      }
      //      if (command == 0x45) {
      //        //speedflag -= 51;
      //        speedflag = -255;
      //        startrun();
      //        command = 0x00;
      //
      //      }
    }
    vTaskDelay(40 / portTICK_PERIOD_MS);
  }
}

void SerialPrintTask( void *pvParameters ) {
  vTaskDelay(200 / portTICK_PERIOD_MS);
  while (1) {
    int disleft ;
    int disright ;
    CircleNumLeft = float(InterruptNumLeft / 192.0);
    CircleNumRight = float(InterruptNumRight / 192.0);
    disleft = CircleNumLeft * 20.7;
    disright = CircleNumRight * 20.7;
    Serial.print("H: ");
    Serial.print(heading);
    Serial.print("degree LD: ");
    Serial.print(disleft);
    Serial.print("cm RD: ");
    Serial.print(disright);
    Serial.print("cm ");
    //    if (Sonarflag == 1) {
    oneSensorCycle();
    //      Sonarflag = 0;
    //    }
    Serial.print("\n");
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}


void faceNorth() {
  compass.read();
  heading = compass.heading();
  while (heading < 355 && heading > 5) {
    compass.read();
    heading = compass.heading();
    if (heading >= 180 && heading < 355) {
      digitalWrite(4, 1);
      digitalWrite(7, 1);
      analogWrite(5, 255);
      analogWrite(6, 0);
    }
    else if (heading < 180 && heading > 5 ) {
      digitalWrite(4, 1);
      digitalWrite(7, 1);
      analogWrite(5, 0);
      analogWrite(6, 255);
    }
  }
  stoprun(0);
}


void InterruptDisLeft() {
  InterruptNumLeft++;
}

void InterruptDisRight() {
  InterruptNumRight++;
}

void startrun(int signalx) {
  digitalWrite(4, 1);
  digitalWrite(7, 1);
  analogWrite(5, 255);
  analogWrite(6, 248);
  if (signalx == 1) {
    Serial1.write(ack);
  }
}

void backrun() {
  digitalWrite(4, 0);
  digitalWrite(7, 0);
  analogWrite(5, 255);
  analogWrite(6, 248);
}

void turnleft(int angle, int signalx) {
  int a = signalx;
  int increase = float(angle) * 2 * 3.14 * 18 * 192 / 360 / 20.7 * LeftTurnDisAgs;
  int hopeInterruptNumRight = InterruptNumRight + increase;
  while (InterruptNumRight < hopeInterruptNumRight) {
    digitalWrite(4, 1);
    digitalWrite(7, 1);
    analogWrite(5, 0);
    analogWrite(6, 255);
    vTaskDelay( 50 / portTICK_PERIOD_MS);
  }
  //if(a==1){
  stoprun(a);
  // }
}

void turnright(int angle, int signalx) {
  int a = signalx;
  int increase = float(angle) * 2 * 3.14 * 18 * 192 / 360 / 20.7 * RightTurnDisAgs;
  int hopeInterruptNumLeft = InterruptNumLeft + increase;
  while (InterruptNumLeft < hopeInterruptNumLeft) {
    digitalWrite(4, 1);
    digitalWrite(7, 1);
    analogWrite(5, 255);
    analogWrite(6, 0);
    vTaskDelay( 50 / portTICK_PERIOD_MS);
  }
  // if(a==1){
  stoprun(a);
  //  }
}

void stoprun(int signalx) {
  digitalWrite(4, 1);
  digitalWrite(7, 1);
  analogWrite(5, 0);
  analogWrite(6, 0);
  if (signalx == 1) {
    Serial1.write(ack);
  }
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print("SO");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(cm[i]);
    Serial.print("cm ");
  }
}

void serialcheck() {
  byte data = 0x00;
  while (data != 0x11) {
    if (Serial1.available()) {
      data = byte( Serial1.read());
      if ( data == 0x12 ) {
        Serial.print("Arduino: Hello received.\n");
        Serial1.write(ack);
        Serial.print("Arduino: Ack sended.\n");
      }
    }
  }
  if ( data == 0x11 ) {
    Serial.print("Arduino: Ack received.\n");
  }
}
void ObstacleAvoidance(int oa) {
  switch (oa) {
    case 0: break;
    case 1: turnright(15, 0); break;
    case 2: ; break;
    case 3: turnright(30, 0); break;
    case 4: turnleft(15, 0); break;
    case 5: break;
    case 6: turnleft(30, 0); break;
    case 7: backrun(); delay(500); turnright(10, 0); break;
  }
}
