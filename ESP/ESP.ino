/*
  PINS USED:
  Sensor PINS USED:     +5, GND, 2, 3, 4, 5, 7, 8, 10, 11, 12
  The sensor pin names VCC, GND, 0, 1, 2, 3, 4,  5,  6,  7,  8
  NeoPixel LED Data pin 14
  Motor driver 6, 9, 13, 15
  Type L298N Motor driver. Enable pins are jumpered LOW
   6 //M1 Direction Control int 1
   13 //M1 Direction Control int 2
   9 //M2 Direction Control int 3
   15 //M2 Direction Control int 4
*/
#include "QTRSensors.h";

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   0     // emitter is controlled by digital pin 2

// sensors 0 through 7 are connected to digital pins 3 through 12, respectively
// I moved 6 and 9 for the motor driver

QTRSensorsRC qtrrc((unsigned char[]) { 2, 4, 7, 8, 10, 11, 12, 13 }, NUM_SENSORS, TIMEOUT, QTR_NO_EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

unsigned int sensors[8];
// unsigned int sensors[NUM_SENSORS];////////////problem
int position = 0;
int error = 0;
int m1Speed = 0;
int m2Speed = 0;
int motorSpeed = 0;

/////////////////////////////OUTPUT PINS////////////////
// Jumpered M1 Speed Control ENA
//Jumpered M2 Speed Control ENB
//maybe jump 2 pins to GND to reduce wire count?

int M1A = 3; //M1 Direction Control int 1
int M1B = 5; //M1 Direction Control int 2
int M2A = 6; //M2 Direction Control int 3
int M2B = 9;//M2 Direction Control int 4
boolean Freeze = 0; // pin high or low to stop motor????
///////////////////////PID TUNING//////////////
int lastError = 0;
float KP = 0.9;			 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
float KD = 9;	// experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)
int M1 = 0;  //base motor speeds
int M2 = 0;  //base motor speeds
int M1max = 255;  //max motor speeds
int M2max = 255;  //max motor speeds
int M1min = 0;  //max motor speeds
int M2min = 0;  //max motor speeds

void setup() {
	Serial.begin(9600);

	/*Serial.println("90° turn left");
	analogWrite(M1A, -255);
	digitalWrite(M1B, HIGH);
	analogWrite(M2A, 255);
	digitalWrite(M2B, HIGH);
	delay(4000);
	Serial.println("90° turn right");
	analogWrite(M1A, 255);
	digitalWrite(M1B, HIGH);
	analogWrite(M2A, -255);
	digitalWrite(M2B, HIGH);
	delay(4000);*/

	Serial.println("Calibrating sensors 10secs");
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(M1A, OUTPUT); //M1 Speed Control int 1
	pinMode(M1B, OUTPUT); //M1 Direction Control int 2
	pinMode(M2A, OUTPUT); //M2 Speed Control int 3
	pinMode(M2B, OUTPUT); //M2 Direction Control int 4
	digitalWrite(M1A, Freeze); // stop the motor
	digitalWrite(M2A, Freeze); // stop the motor
	digitalWrite(M1B, HIGH);    // set the second directional pins LOW
	digitalWrite(M2B, HIGH);    // set the second directional pins LOW

	for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
	{
		delay(5);
		qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
	}

	// print the calibration minimum values measured when emitters were on

	for (int i = 0; i < NUM_SENSORS; i++)
	{
		Serial.print(qtrrc.calibratedMinimumOn[i]);
		Serial.print(' ');
	}
	Serial.println();

	// print the calibration maximum values measured when emitters were on
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		Serial.print(qtrrc.calibratedMaximumOn[i]);
		Serial.print(' ');
	}
	Serial.println();
}

void loop()
{
	position = qtrrc.readLine(sensors);

	Serial.print("position ");
	Serial.print(position);
	Serial.println();

	// compute our "error" from the line position
	// error is zero when the middle sensor is over the line,
	error = position - 3750;   /// using an 8 sensor array targeting middle of position 4 to 5

	Serial.print("error ");
	Serial.print(error);
	Serial.println();

	// set the motor speed based on proportional and derivative PID terms
	motorSpeed = KP * error + KD * (error - lastError);
	lastError = error;
	/////////CHANGE THE - OR + BELOW DEPENDING ON HOW YOUR DRIVER WORKS

	m1Speed = M1 + motorSpeed; // M1 and M2 are base motor speeds.
	m2Speed = M2 - motorSpeed;
	Serial.print("m1Speed ");
	Serial.print(m1Speed);
	Serial.print(" ");
	Serial.print("m2Speed ");
	Serial.println(m2Speed);
	Serial.println();

	if (m1Speed < M1min)  //keep speeds to 0 or above
		m1Speed = M1min;
	if (m2Speed < M2min)
		m2Speed = M2min;
	if (m1Speed > M1max) //maximum allowed value
		m1Speed = M1max;
	if (m2Speed > M2max) //maximum allowed value
		m2Speed = M2max;
	// set motor speeds using the two motor speed variables above 255

	/*if (position > 6000) {
		Serial.println("90° turn left");
		analogWrite(M1A, -255);
		digitalWrite(M1B, HIGH);
		analogWrite(M2A, 255);
		digitalWrite(M2B, HIGH);
		return;
	}
	if (position < 500) {
		Serial.println("90° turn right");
		analogWrite(M1A, 255);
		digitalWrite(M1B, HIGH);
		analogWrite(M2A, -255);
		digitalWrite(M2B, HIGH);
		return;
	}*/
	forward();  // run the motors
}

////////////////////////STOP//////////////
void stopit(void)
{
	digitalWrite(M1A, Freeze);
	digitalWrite(M1B, LOW);
	digitalWrite(M2A, Freeze);
	digitalWrite(M2B, LOW);
}

//////////////////////FORWARD////////
void forward()
{
	analogWrite(M1A, m1Speed);
	digitalWrite(M1B, HIGH);
	analogWrite(M2A, m2Speed);
	digitalWrite(M2B, HIGH);
}