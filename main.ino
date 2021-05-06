#include "Servo.h"
#include "math.h"
#include "string.h"

#define RIGHT_DIR 3
#define LEFT_DIR 4
#define RIGHT_WHEELS 5
#define LEFT_WHEELS 6

#define SERVO 2
#define TRIG_0 7
#define ECHO_0 8
#define TRIG_1 11
#define ECHO_1 12
#define TRIG_2 10
#define ECHO_2 13


#define US_GAP 6
#define TOL_PCENT 0.3
#define TOL_ABS 5
#define MAX_FIND 10
#define FORWARD_THRESH 10

Servo servo;


static int result[72][2];

static int centre_offset;
static int init_ref[72][2];
bool detect_flag = 0;
bool return_flag = false;
float distance_travelled = 0;
int num = 0;
float MAX_DIST;
float travel_step;

typedef struct 
{
  String type;
  int start_index;
  int end_index;
  float hole_dist;
  
} hole_bump_t;

hole_bump_t  hole_bump[MAX_FIND];


void setup() {
  pinMode(RIGHT_WHEELS,OUTPUT);
  pinMode(LEFT_WHEELS,OUTPUT);
  pinMode(LEFT_DIR,OUTPUT);
  pinMode(RIGHT_DIR,OUTPUT);  
  pinMode(TRIG_0,OUTPUT);
  pinMode(ECHO_0,INPUT);
  pinMode(TRIG_1,OUTPUT);
  pinMode(ECHO_1,INPUT);
  servo.attach(SERVO);
  Serial.begin(9600);

 digitalWrite(RIGHT_DIR,1);
 digitalWrite(LEFT_DIR,0);

}

void loop() 
{
  user_intf();
  init_scan();
    delay(100);
  standard_scan(MAX_DIST, travel_step);
    delay(50);
  dir_flip();   
  travel(0.65, distance_travelled);
  user_data();
  
    while(1);
}

void travel(float speed_in, float rover_distance) 
{
  
  if((rover_distance > 0.5) || ((int(rover_distance*20)%1) != 0))
  {
    Serial.print("error incorrect rover distance");
    return;
  }
  
  const float calibration[2][10] = 
  {
    {0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5},
    {2.05, 1.62, 1.38, 1.31, 1.22, 1.18, 1.15, 1.11, 1.09, 1.07}
  };
  
  float PWM_speed = speed_in - 0.1503;
  PWM_speed = PWM_speed / 0.0041;
  float time_duration = rover_distance / speed_in;
  time_duration = time_duration * 1000;
  time_duration = time_duration * calibration[1][int(rover_distance*20)-1];
  analogWrite(RIGHT_WHEELS, PWM_speed);
  analogWrite(LEFT_WHEELS, PWM_speed);
  delay(int(time_duration));
  analogWrite(RIGHT_WHEELS, 0);
  analogWrite(LEFT_WHEELS, 0);
  distance_travelled = distance_travelled + rover_distance;
}

void init_scan ()
{
  static int diameter = -1;
  servo.write(0);
  delay(500);
  int topVal = sonicReading(TRIG_0, ECHO_0, 10);
  int bottomVal = sonicReading(TRIG_1, ECHO_1, 10);  
  diameter = topVal + bottomVal + US_GAP;
  centre_offset = bottomVal - (diameter/2);
  Serial.print("Diameter/Height (cm) = ");
  Serial.print(diameter);
  Serial.print("\n");
  Serial.print("Offset (cm) = ");
  Serial.print(centre_offset);
  Serial.print("\n\n");
  
  for (int i = 0; i < 36; i++)
    {
       
       servo.write(i*5);
       init_ref[i][0] = sonicReading(TRIG_0, ECHO_0, 5);
       init_ref[i][1] = i*5;
      
       init_ref[i+36][0] = sonicReading(TRIG_1, ECHO_1, 5);
       init_ref[i+36][1] = 180 + i*5;

    }

  
}


void standard_scan(float total_distance, float travel_step)
{
  int count = total_distance/travel_step;

  for(int i = 0; i < count ; i++)
  {
    travel(0.65,travel_step);
    delay(100);
    forward_scan;
    scan_360();
   
    for(int j = 0; j < 72; j++)
    {
      if(result[j][0] > (init_ref[j][0] + (init_ref[j][0] * TOL_PCENT) + TOL_ABS))
      {
        hole_bump[num].type = "Hole";
        detect_flag = true;
        
      }
      else if (result[j][0] < (init_ref[j][0] - (init_ref[j][0] * TOL_PCENT) - TOL_ABS))
      {
        hole_bump[num].type = "Bump";
        detect_flag = true;
       
      }
    }
   if(detect_flag)    //if detect flag is true/set
   {
    detect_flag = 0;
    angle_cal();
   }
   if(return_flag)
    return;
  }
  return;
}

void angle_cal()
{
  bool start_flag = 0;
  for(int j = 0; j < 72; j++)
  {
    bool bigger_tol = result[j][0] > (init_ref[j][0] + (init_ref[j][0] * TOL_PCENT) +   TOL_ABS);
    bool smaller_tol = result[j][0] < (init_ref[j][0] - (init_ref[j][0] * TOL_PCENT) - TOL_ABS);
    if(bigger_tol || smaller_tol)
    {
      if(start_flag)
      {
        hole_bump[num].end_index = j * 5;
        hole_bump[num].hole_dist = distance_travelled;
        
      }
      else
      {
        start_flag = 1;
        hole_bump[num].start_index = j * 5;
        hole_bump[num].end_index = j * 5;
        hole_bump[num].hole_dist = distance_travelled;
     
      }

   
    } 
    
  }
  num++;

  
  return;
       
}
//------------------------------ This function is responsible for taking a 360 degree scan of the surrounding area, using the US sensors.
void scan_360()
{
    servo.write (0);
    for (int i = 0; i < 36; i++)  //This loop controls the servo and takes measurements at 5 degree intervals.
    {
       
       servo.write(i*5);          // Rotates servo by 5 degrees.
       result[i][0] = sonicReading(TRIG_0, ECHO_0, 4);  // Takes reading from Sensor 1, and sorts the value in an array.
       result[i][1] = i*5;   //Stores the angle of the measurement in the 2nd column of the array.
      
       result[i+36][0] = sonicReading(TRIG_1, ECHO_1, 4); // Takes reading from Sensor 2, and sorts the value in an array.
       result[i+36][1] = 180 + i*5; 

    }

}


//--------------------------------------
int sonicReading(uint8_t trig_i, uint8_t echo_i, int average_num)
{
  const int add_cal [3] = {2, 0, 0};
  const int div_cal [3] = {58, 58, 58};
  int cal_index;
  
  switch (trig_i) 
  {
    case TRIG_0:
      cal_index = 0;
      break;
    case TRIG_1:
      cal_index = 1;
      break;
    case TRIG_2:
      cal_index = 2;
      break;
    default:
      Serial.print("code_error");
  }
      
  unsigned long time_value;
  unsigned long average = 0;
  for(int i=0; i<average_num; i++)
  {
    delay(70);
    time_value = 0;
    digitalWrite(trig_i,HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_i,LOW);
    time_value = pulseIn(echo_i, HIGH);
    
    if (time_value == 0)
    {
      i--;
      
    }
    else
    {
    
      time_value = time_value + add_cal[cal_index];
      time_value = time_value / div_cal[cal_index];
      average = average + time_value;
  
    }
  
 
  }
   average = average / average_num;
   return average;
}

void user_data()

{  
 
  if(num == 0)
  {
    Serial.print("No flaws detected.");
  }
  else
  {
    for(int i=0; i < num; i++)
    {
    Serial.print("Detect number: ");
    Serial.print(i);
      Serial.print("\n");
      
    Serial.print("Type: ");
    Serial.print(hole_bump[i].type);
      Serial.print("\n");
      
    Serial.print("Detected at:\n");
    Serial.print("Distance (cm): ");
    Serial.print(hole_bump[i].hole_dist);
      Serial.print("\n");
      
    Serial.print("Start Angle: ");
    Serial.print(hole_bump[i].start_index);
      Serial.print("\n");
      
    Serial.print("End Angle: ");
    Serial.print(hole_bump[i].end_index);
      Serial.print("\n \n \n"); 
    }
      return;
  }
  
}

//---------------------This function flips the original motor orientation, meaning the motors will reverse.
void dir_flip()
{
  digitalWrite(RIGHT_DIR,0);
  digitalWrite(LEFT_DIR,1);
  return;
}


void forward_scan()
{
   int dist = sonicReading(TRIG_2, ECHO_2, 3);

   if(dist < FORWARD_THRESH)
    return_flag = true;

  return;
}


void user_intf()
{
  uint8_t buffer[4] = {0, 0, 0, 0};
  int temp;
  int temp_2;
  Serial.print("Hello, Welcome to your pipe scanning rover, I'm Ardy!\n\n");
  Serial.print("Please enter your maximum travel distance in cm: \n");
  
  while(!(Serial.available() > 0))
  {
  }
  Serial.readBytes(buffer, 3);
  temp = (((buffer[0]- 48) * 100) + ((buffer[1]- 48) *10) + (buffer[2]- 48));
  MAX_DIST = float(temp)/100;
   Serial.print("Maximum travel distance (cm) = ");
   Serial.print(temp);
   Serial.flush();
   Serial.print("\n\n");
   Serial.print("Please enter the incremental distance in cm: \n");
   

  while(!(Serial.available() > 4))
  {
  }
   Serial.readBytes(buffer, 4);
   
   temp_2 = (((buffer[1]- 48) * 100) + ((buffer[2]- 48) *10) + (buffer[3]- 48));
   travel_step = float(temp_2)/100;
   Serial.print("Incremental distance (cm) = ");
   Serial.print(temp_2);
   Serial.flush();
   Serial.print("\n \n");

  

  
}
