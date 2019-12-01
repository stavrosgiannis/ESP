
/* Define motor controll inputs */
const int motorRPin1 = 2; // signal pin 1 for the right motor, connect to IN1               
const int motorRPin2 = 3;  // signal pin 2 for the right motor, connect to IN2
const int motorRPWM = 5; // enable PWM for the right motor

const int motorLPin1 = 4; // signal pin 1 for the left motor, connect to IN3           
const int motorLPin2 = 7; // signal pin 2 for the left motor, connect to IN4
const int motorLPWM = 6; // enable PWM for the left motor

const int irPins[8] = {A0, A1, A2, A3, A4, A5 , 11 , 12};
int irSensorDigital[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//Variable werden 2 Bytes jeweils 0 Bits gesetzt
// the 0b(B) prefix indicates a binary constant
int irSensors = B00000000; 

int motorLSpeed = 255;
int motorRSpeed = 255;
int error = 145;   // 145 best 200  //  normal 255  // mad 0 

void setup() {
  
  Serial.begin(9600);
  
  pinMode(motorLPin1,OUTPUT);        
  pinMode(motorLPin2,OUTPUT);
  pinMode(motorLPWM,OUTPUT);
  
  pinMode(motorRPin1,OUTPUT);        
  pinMode(motorRPin2,OUTPUT);
  pinMode(motorRPWM,OUTPUT);
   
  /* Set-up IR sensor pins as input */
  for (int i = 0; i <= 7; i++) 
  {pinMode(irPins[i], INPUT);}

}

void loop() {
     scan_data();
     check_data(); 
}     
     
void check_data( ) 
{    
     switch (irSensors) {
       case B00000000: // on white paper 
         turn_right();
         break;
       case B10000000: // leftmost sensor on the line
         turn_right();
         break;
       case B01000000:
         turn_right();
         break;  
       case B00100000: 
         turn_right();
         break;
       case B00010000: 
         turn_right();
         break; 
       case B00001000: 
         turn_left();
         break;     
       case B00000100: 
         turn_left();
         break;
       case B00000010: 
         turn_left();
         break;
       case B00000001: 
         turn_left();
         break;       
       case B11000000:
         turn_right();
         break;
       case B01100000:
         turn_right();
         break;
       case B00110000:
         turn_right(); 
         break;
       case B00011000: 
         forward();
         break;          
       case B00001100:
         turn_left();
         break; 
       case B00000110:
         turn_left();
         break;   
       case B00000011:
         turn_left();
         break;          
       case B11100000:
         turn_right();   
         break;
       case B01110000:
         turn_right();
         break;
       case B00111000:
         turn_right();
         break;
       case B00011100:
         turn_left();
         break;  
       case B00001110:
         turn_left();
         break; 
       case B00000111:
         turn_left();
         break;   
       case B11110000:
         turn_right(); 
         break; 
       case B01111000:
         turn_right();       
         break;
       case B00111100:
         forward();
         break;  
       case B00011110:
         turn_left();
         break;  
       case B00001111:
         turn_left();
         break;  
       case B11111000:
         turn_right(); 
         break; 
       case B01111100:
         turn_right(); 
         break;
       case B00111110:
         turn_left();
         break;
       case B00011111:
         turn_left();
         break;
       case B11111100:
         turn_right(); 
         break; 
       case B01111110:
         forward();
         break;
       case B00111111:
         turn_left();
         break;
       case B11111110:
         turn_right(); 
         break; 
       case B01111111:
         turn_left();
         break;  
       case B11100111:
         forward();        
         break;  
       default:
         Serial.println("Unhandled case: ");   
  }
 
}

void turn_right() 
{
     Serial.println("                         right motor forward (right spin)");
     analogWrite(motorRPWM, motorRSpeed);
     digitalWrite(motorRPin1, LOW);
     digitalWrite(motorRPin2, HIGH);
     
     analogWrite(motorLPWM, motorLSpeed-error);
     digitalWrite(motorLPin1, HIGH);
     digitalWrite(motorLPin2, LOW);
  
}

void turn_left()  //turn left
{
     Serial.println("                         left  motor forward (left spin)");
     analogWrite(motorRPWM, motorRSpeed-error);
     digitalWrite(motorRPin1, HIGH);
     digitalWrite(motorRPin2, LOW);
     
     analogWrite(motorLPWM, motorLSpeed);
     digitalWrite(motorLPin1, LOW);
     digitalWrite(motorLPin2, HIGH);
  
}

void forward()
{
    Serial.println("                         forward ");
     analogWrite(motorRPWM, motorRSpeed);
     digitalWrite(motorRPin1, LOW);
     digitalWrite(motorRPin2, HIGH);
     
     analogWrite(motorLPWM, motorLSpeed);
     digitalWrite(motorLPin1, LOW);
     digitalWrite(motorLPin2, HIGH);
  
}

void stop()
{
     Serial.println("                         stop");
     analogWrite(motorRPWM, motorRSpeed);
     digitalWrite(motorRPin1, LOW);
     digitalWrite(motorRPin2, LOW);
     
     analogWrite(motorLPWM, motorLSpeed);
     digitalWrite(motorLPin1, LOW);
     digitalWrite(motorLPin2, LOW);
  
}

void scan_data()
{
  for ( byte count = 0; count < 8;count++ )
    {
	  //Writes a bit of a numeric variable.
      bitWrite(irSensors, count, !digitalRead( irPins[count] ));
    } 
}
