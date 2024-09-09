// NANENG 512 - SELF BALANCING ROBOT - Interrupt subroutine Code - FALL22

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

#define leftMotorPWMPin   3
#define leftMotorDirPin   9 
#define leftMotorDirPin2   8

#define rightMotorPWMPin  11
#define rightMotorDirPin  13
#define rightMotorDirPin2  12


#define Kp  14.5
#define Kd  0.003
#define Ki  36
#define sampleTime  0.003
#define targetAngle -0.6

int16_t gyroY;
volatile int MotorPWM, gyroRate;
volatile float gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;


void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  if(leftMotorSpeed >= 0) {
    analogWrite(leftMotorPWMPin, leftMotorSpeed);
    digitalWrite(leftMotorDirPin, HIGH);
    digitalWrite(leftMotorDirPin2, LOW);
  }
  else {
    analogWrite(leftMotorPWMPin, 255 + leftMotorSpeed);
    digitalWrite(leftMotorDirPin, LOW);
    digitalWrite(leftMotorDirPin2, HIGH);
  }
  if(rightMotorSpeed >= 0) {
    analogWrite(rightMotorPWMPin, rightMotorSpeed);
    digitalWrite(rightMotorDirPin, HIGH);
    digitalWrite(rightMotorDirPin2, LOW);
  }
  else {
    analogWrite(rightMotorPWMPin, 255 + rightMotorSpeed);
    digitalWrite(rightMotorDirPin, LOW);
    digitalWrite(rightMotorDirPin2, HIGH);
  }
}

void init_PID() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  

  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin, OUTPUT);
  pinMode(leftMotorDirPin2, OUTPUT);

  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotorDirPin, OUTPUT);
  pinMode(rightMotorDirPin2, OUTPUT);
  // set the status LED to output mode 
  // initialize the MPU6050 and set offset values
  // initialize PID sampling loop
  init_PID();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  /* Get new sensor events with the readings */
  gyroY = (float)g.gyro.y;
  MotorPWM = constrain(MotorPWM, -255, 255);
  setMotors(MotorPWM, MotorPWM);
}

// The ISR will be called every 5 milliseconds
ISR(TIMER1_COMPA_vect)
{

  // Angle calculation
  gyroRate = map(gyroY, -5, 5, -250, 250);
  gyroAngle = (float)gyroRate*sampleTime;  
  currentAngle = 0.9936*(prevAngle + gyroAngle);
  Serial.print(currentAngle);
  Serial.print(" <- Angle \n");

  // PID
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);
  //calculate output from P, I and D values
  MotorPWM = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
  prevAngle = currentAngle;
  // toggle the led on pin13 every second
  count++;
  if(count == 200)  {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }
}
