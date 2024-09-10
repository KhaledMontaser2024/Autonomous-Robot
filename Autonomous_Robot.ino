

/////////////////////// IR /////////////////////////////////////////// 
#define F_IR 11             // Front infrared sensor pin
#define L_IR 12             // Left infrared sensor pin
#define R_IR 13             // Right infrared sensor pin
//////////////////////////////////////////////////////////////////////

////////////////////////// Ultrasonic /////////////////////////////////
#define SET_DISTANCE 5     // Threshold distance for obstacle detection

#define US_F_echo A1        // Front ultrasonic sensor echo pin
#define US_F_trigger 2      // Front ultrasonic sensor trigger pin

#define US_R_echo A2        // Right ultrasonic sensor echo pin
#define US_R_trigger 3      // Right ultrasonic sensor trigger pin

#define US_L_echo A3        // Left ultrasonic sensor echo pin
#define US_L_trigger 4      // Left ultrasonic sensor trigger pin
///////////////////////////////////////////////////////////////////////

//////////////////// For Motors and Motion ////////////////////////////
#define R_M_SpeedPin 9      // Right Motor PWM pin
#define R_M_DirectionPin 8  // Right Motor Direction pin

#define L_M_SpeedPin 10     // Left Motor PWM pin
#define L_M_DirectionPin 11 // Left Motor Direction pin

#define MaxSpeed 160
#define StartSpeed 50 
#define SpeedMargin 5		  // how much speed to increase or decrease (0 - 255) 
#define DiffrentialSpeed 50
#define TimeMargin 100 		// time between two speed changes  (in ms)

#define RightMotor 1
#define LeftMotor 2
#define AllMotors 3 
////////////////////////////////////////////////////////////////////////


long Ultrasonic_read(int trigger, int echo);
void check_Ultrasonics();

void Accelerate(int TargetSpeed = MaxSpeed , int TargetMotor = AllMotors);
void Decelerate(int TargetSpeed = StartSpeed, int TargetMotor = AllMotors);
void forward();
void backward();
void RotateRight();
void RotateLeft();
void SteerLeft();
void SteerRight();

unsigned int speed = StartSpeed;
unsigned long int TimeStamp =0;
char lastDirection = 'S';

int distance_L, distance_F, distance_R;


void setup() 
{
  Serial.begin(9600); // Start serial communication
  TimeStamp = millis();
  // Setup pins
  pinMode(R_M_SpeedPin,OUTPUT);
  pinMode(L_M_SpeedPin,OUTPUT);
  pinMode(R_M_DirectionPin,OUTPUT);
  pinMode(L_M_DirectionPin,OUTPUT);

  pinMode(US_F_trigger, OUTPUT);
  pinMode(US_R_trigger, OUTPUT);
  pinMode(US_L_trigger, OUTPUT);
  pinMode(US_F_echo, INPUT);
  pinMode(US_R_echo, INPUT);
  pinMode(US_L_echo, INPUT);
  pinMode(L_IR, INPUT);
  pinMode(R_IR, INPUT);
  pinMode(F_IR, INPUT);

  //Initial ultrasonic readings
  distance_F = Ultrasonic_read(US_F_trigger, US_F_echo);
  distance_R = Ultrasonic_read(US_R_trigger, US_R_echo);
  distance_L = Ultrasonic_read(US_L_trigger, US_L_echo);

  delay(500); 
}

void loop() 
{

  check_Ultrasonics();
  // Line following logic
  if ( digitalRead(R_IR) == LOW && digitalRead(F_IR) == HIGH && digitalRead(L_IR) == LOW ) 
  {
    lastDirection = 'F';
    forward();
  } 
  else if (( digitalRead(R_IR) == HIGH && digitalRead(F_IR) == LOW && digitalRead(L_IR) == LOW ) || (digitalRead(R_IR) == HIGH && digitalRead(F_IR) == HIGH && digitalRead(L_IR) == LOW) )
  {
    lastDirection = 'R';
    RotateRight();
  } 
  else if ((digitalRead(R_IR) == LOW && digitalRead(F_IR) == LOW && digitalRead(L_IR) == HIGH) || (digitalRead(R_IR) == LOW && digitalRead(F_IR) == HIGH && digitalRead(L_IR) == HIGH)) 
  {
    lastDirection = 'L';
    RotateLeft();
  } 
  else if (digitalRead(R_IR) == HIGH &&  digitalRead(F_IR) == HIGH && digitalRead(L_IR) == HIGH) 
  {
    Serial.println("Zone Changed");
    lastDirection = 'F';
    forward();
  }
  else if (digitalRead(R_IR) == LOW &&  digitalRead(F_IR) == LOW && digitalRead(L_IR) == LOW) 
  {
        
    switch (lastDirection)
    {
      case 'R':
        RotateLeft();
        delay(1000);
      break;

      case 'L':
        RotateRight();
        delay(1000);
      break;

      case 'F':
        Decelerate();
      break;

      default:
      break;
    }

  }
  delay(10);
}

void check_Ultrasonics()
{
  distance_F = Ultrasonic_read(US_F_trigger, US_F_echo);
  distance_R = Ultrasonic_read(US_R_trigger, US_R_echo);
  distance_L = Ultrasonic_read(US_L_trigger, US_L_echo);
  delay(500);

  if (distance_F <= SET_DISTANCE && distance_R >= SET_DISTANCE && distance_L >= SET_DISTANCE)   // obstacle in front  
  {
    Decelerate();
    backward();
    delay(1000);
    Decelerate();

  }
  else if (distance_F >= SET_DISTANCE && distance_R <= SET_DISTANCE && distance_L >= SET_DISTANCE) // obstacle on the right  
  {
    Decelerate();
    RotateLeft();
    delay(1000);
    Decelerate();
  }
  else if (distance_F >= SET_DISTANCE && distance_R >= SET_DISTANCE && distance_L <= SET_DISTANCE) // obstacle on the Left 
  {
    
    Decelerate();
    RotateRight();
    delay(1000);
    Decelerate();
  }

 return;
}

long Ultrasonic_read(int trigger, int echo) 
{
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  long time = pulseIn(echo, HIGH);
  return time / 29 / 2;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Accelerate(int TargetSpeed, int TargetMotor)
{
	if (millis() - TimeStamp >= TimeMargin && speed < TargetSpeed)
  {
    TimeStamp = millis();
    speed += SpeedMargin;
  }
  switch (TargetMotor)
  {	
    case AllMotors:
      Serial.print("All Motors Speed:");
      Serial.println(speed);
      analogWrite(R_M_SpeedPin,speed);
      analogWrite(L_M_SpeedPin,speed);
    break;
    
    case LeftMotor:
      Serial.print("Left Motor Speed:");
      Serial.println(speed);
      analogWrite(L_M_SpeedPin,speed);
    break;
    
    case RightMotor:
      Serial.print("Right Motor Speed:");
      Serial.println(speed);
      analogWrite(R_M_SpeedPin,speed);
    break;
  
    default:
    break;
  }
  
  return;
}


void Decelerate(int TargetSpeed , int TargetMotor)
{
	if (millis() - TimeStamp >= TimeMargin && (speed > TargetSpeed && speed<= MaxSpeed))
  {
    TimeStamp = millis();
    speed -= SpeedMargin;
  }
		
  switch (TargetMotor)
  {	
    case AllMotors:
      // Serial.print("All Motors Speed:");
      // Serial.println(speed);       // debug
      analogWrite(R_M_SpeedPin,speed);
      analogWrite(L_M_SpeedPin,speed);
    break;
    
    case LeftMotor:
      // Serial.print("Left Motor Speed:");
      // Serial.println(speed);   /// debug
      analogWrite(L_M_SpeedPin,speed);
    break;
    
    case RightMotor:
      // Serial.print("Right Motor Speed:");
      // Serial.println(speed);      //debug
      analogWrite(R_M_SpeedPin,speed);
    break;
  
    default:
    break;
  }
  return;
}

void SteerRight()
{
  static int TargetSteeringSpeed = speed - DiffrentialSpeed;
  switch (lastDirection)
  {
  	case 'r':
      // Serial.println("Already Steering Right!"); //debug
    break;
    
    case 'S':
      RotateRight();
    break;

    case 'B':
    	// Serial.println(TargetSteeringSpeed);
    	Decelerate(TargetSteeringSpeed ,LeftMotor);
    	speed == TargetSteeringSpeed? lastDirection = 'r' : lastDirection = lastDirection ;
    break;
    
    default :
    	// Serial.println(TargetSteeringSpeed);
    	Decelerate(TargetSteeringSpeed ,RightMotor);
    	speed == TargetSteeringSpeed? lastDirection = 'r' : lastDirection = lastDirection ;
    break;
  }
  return;
}

void SteerLeft()
{
  static int TargetSteeringSpeed = speed - DiffrentialSpeed;
	switch (lastDirection)
  {
  	case 'l':
      // Serial.println("Already Steering Left!");
    break;
    
    case 'S':
      RotateLeft();
    break;

    case 'B':
    	// Serial.println(TargetSteeringSpeed);
    	Decelerate(TargetSteeringSpeed ,RightMotor);
    	speed == TargetSteeringSpeed? lastDirection = 'r' : lastDirection = lastDirection ;
    break;
    
    default :
    	// Serial.println(TargetSteeringSpeed);
    	Decelerate(TargetSteeringSpeed ,LeftMotor);
    	speed == TargetSteeringSpeed? lastDirection = 'l' : lastDirection = lastDirection ;
    break;
  }
  return;
}



void forward() 
{
  switch (lastDirection)
  {
  	case 'F':
      // Serial.println("Already going Forward!"); //debug
    break;
    
    case 'S':
      digitalWrite(R_M_DirectionPin, HIGH);
      digitalWrite(L_M_DirectionPin, LOW);
		  Accelerate();											// default acceleration will accelerate till MaxSpeed
    	speed == MaxSpeed ? lastDirection = 'F' : lastDirection = 'S';
    break;
    
    default :
    	if (speed > StartSpeed) 
      {
        Decelerate(); 
      }
      else if (speed = StartSpeed) 
      {
        lastDirection = 'S'; 
      }
    break;
  }
  
  return;
}

void backward() 
{
 
  switch (lastDirection)
  {
  	case 'B':
      // Serial.println("Already going Backward!");
    break;
    
    case 'S':
      digitalWrite(R_M_DirectionPin, LOW);
    	digitalWrite(L_M_DirectionPin, HIGH);
		  Accelerate();											// default acceleration will accelerate till MaxSpeed
    	speed == MaxSpeed ? lastDirection = 'B' : lastDirection = 'S';
    break;
    
    default :
    	if (speed > StartSpeed) 
      {
        Decelerate(); 
      }
      else if (speed = StartSpeed) 
      {
        lastDirection = 'S'; 
      }
    break;
  }
  
  
  
  
  
  return;
}

void RotateRight()  
{
  switch (lastDirection)
  {
  	case 'R':
      // Serial.println("Already going Right!");
    break;
    
    case 'S':
      digitalWrite(R_M_DirectionPin, HIGH);
    	digitalWrite(L_M_DirectionPin, HIGH);
		  Accelerate();											// default acceleration will accelerate till MaxSpeed
    	speed == MaxSpeed ? lastDirection = 'R' : lastDirection = 'S';
    break;
    
    default :
    	if (speed > StartSpeed) 
      {
        Decelerate(); 
      }
      else if (speed = StartSpeed) 
      {
        lastDirection = 'S'; 
      }
    break;
  }
  
  return;
}

void RotateLeft()
{
  switch (lastDirection)
  {
  	case 'L':
      // Serial.println("Already going Left!");
    break;
    
    case 'S':
      digitalWrite(R_M_DirectionPin, LOW);
      digitalWrite(L_M_DirectionPin, LOW);
      Accelerate();											// default acceleration will accelerate till MaxSpeed
    	speed == MaxSpeed ? lastDirection = 'L' : lastDirection = 'S';
    break;
    
    default :
    	if (speed > StartSpeed) 
      {
        Decelerate(); 
      }
      else if (speed = StartSpeed) 
      {
        lastDirection = 'S'; 
      }
    break;
  }
  
  return;
}
