

/////////////////////// IR /////////////////////////////////////////// 
#define F_IR 11             // Front infrared sensor pin
#define L_IR 12             // Left infrared sensor pin
#define R_IR 13             // Right infrared sensor pin
//////////////////////////////////////////////////////////////////////

////////////////////////// Ultrasonic /////////////////////////////////
#define SET_DISTANCE 5     // Threshold distance for obstacle detection

#define US_F_echo 5        // Front ultrasonic sensor echo pin
#define US_F_trigger 6     // Front ultrasonic sensor trigger pin

#define US_R_echo 3        // Right ultrasonic sensor echo pin
#define US_R_trigger 4     // Right ultrasonic sensor trigger pin

#define US_L_echo 2        // Left ultrasonic sensor echo pin
#define US_L_trigger 1     // Left ultrasonic sensor trigger pin
///////////////////////////////////////////////////////////////////////

//////////////////// For Motors and Motion ////////////////////////////
#define R_M_SpeedPin 9      // Right Motor PWM pin
#define R_M_DirectionPin 8  // Right Motor Direction pin

#define L_M_SpeedPin 10     // Left Motor PWM pin
#define L_M_DirectionPin 7 // Left Motor Direction pin

#define MaxSpeed 160
#define StartSpeed 50 
#define SpeedMargin 5		  // how much speed to increase or decrease (0 - 255) 
#define DiffrentialSpeed 50
#define TimeMargin 100 		// time between two speed changes  (in ms)

#define RightMotor 1
#define LeftMotor 2
#define AllMotors 3 
////////////////////////////////////////////////////////////////////////


void check_Ultrasonics();
void Trigger_UltraSonic();
void US_R_ISR();
void US_L_ISR();


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


int R_duration, R_distance , L_duration, L_distance , F_duration, F_distance;
long int US_R_time=0;
long int US_L_time=0;

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

  
  attachInterrupt(0,US_L_ISR,CHANGE);
  attachInterrupt(1,US_R_ISR,CHANGE);
  
}
//////////////////////////////////////////////////////////////////////////////////////////
void loop() 
{

  check_Ultrasonics();

  if ( digitalRead(R_IR) == LOW && digitalRead(F_IR) == HIGH && digitalRead(L_IR) == LOW ) 
  {
    forward();
  } 
  else if (( digitalRead(R_IR) == HIGH && digitalRead(F_IR) == LOW && digitalRead(L_IR) == LOW ) || (digitalRead(R_IR) == HIGH && digitalRead(F_IR) == HIGH && digitalRead(L_IR) == LOW) )
  {
    RotateRight();
  } 
  else if ((digitalRead(R_IR) == LOW && digitalRead(F_IR) == LOW && digitalRead(L_IR) == HIGH) || (digitalRead(R_IR) == LOW && digitalRead(F_IR) == HIGH && digitalRead(L_IR) == HIGH)) 
  {
    RotateLeft();
  } 
  else if (digitalRead(R_IR) == HIGH &&  digitalRead(F_IR) == HIGH && digitalRead(L_IR) == HIGH) 
  {
    Serial.println("Zone Changed");
    forward();
  }
  else if (digitalRead(R_IR) == LOW &&  digitalRead(F_IR) == LOW && digitalRead(L_IR) == LOW) 
  {
        
    switch (lastDirection)
    {
      case 'R':
        RotateLeft();
      break;

      case 'L':
        RotateRight();
      break;

      case 'F':
        Decelerate();
      break;

      default:
        forward();
      break;
    }

  }
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
    
    case 'r':
      Decelerate(MaxSpeed ,RightMotor);
      // Serial.println("Already going Forward!"); //debug
      speed == MaxSpeed ? lastDirection = 'F' : lastDirection = 'r';
    break;
    
    case 'l':
      Decelerate(MaxSpeed ,LeftMotor);
      // Serial.println("Already going Forward!"); //debug
      speed == MaxSpeed ? lastDirection = 'F' : lastDirection = 'l';
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

/////////////////////////////////////////////////////////////////////////////////////

void check_Ultrasonics()
{
  Trigger_UltraSonic();
  
  if (F_distance <= SET_DISTANCE && R_distance >= SET_DISTANCE && L_distance >= SET_DISTANCE)   // obstacle in front  
  {
    backward();
  }
  else if (F_distance >= SET_DISTANCE && R_distance <= SET_DISTANCE && L_distance >= SET_DISTANCE) // obstacle on the right  
  {
    SteerLeft();
  }
  else if (F_distance >= SET_DISTANCE && R_distance >= SET_DISTANCE && L_distance <= SET_DISTANCE) // obstacle on the Left 
  {
    SteerRight();
  }

 return;
}

void Trigger_UltraSonic()
{
  digitalWrite(US_R_trigger, LOW);
  digitalWrite(US_F_trigger, LOW);
  digitalWrite(US_L_trigger, LOW);
  delayMicroseconds(2);
  
  digitalWrite(US_R_trigger, HIGH);
  digitalWrite(US_F_trigger, HIGH);
  digitalWrite(US_L_trigger, HIGH);
  delayMicroseconds(10);
  
  digitalWrite(US_R_trigger, LOW);
  digitalWrite(US_F_trigger, LOW);
  digitalWrite(US_L_trigger, LOW);
  
  
  F_duration = pulseIn(US_F_echo, HIGH);
  F_distance = (F_duration/29)/2;
 
}


void US_R_ISR()
{
  static bool state = false;
  switch (state)
  {
  	case false:
    	US_R_time = micros();
    	state = true;
    break;
    
    case true:
    	R_duration = (micros()-US_R_time);
    	R_distance = (R_duration/27.5)/2;
    	state = false;
    break;
  	default:
    break;
  }
  
}
void US_L_ISR()
{
  static bool state = false;
  switch (state)
  {
  	case false:
    	US_L_time = micros();
    	state = true;
    break;
    
    case true:
    	L_duration = (micros()-US_L_time);
    	L_distance = (L_duration/27.5)/2;
    	state = false;
    break;
  	default:
    break;
  }
  
}