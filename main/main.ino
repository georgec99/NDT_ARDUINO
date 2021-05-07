//Title: NDT  Arduino Rover 
//Author:George T Chettle
//Date:07/05/2021
//Version: Dev.1
//Info: This is a program for a rover for non destructive testing 
//it uses three ultrasonic sensor 
//

#include "Servo.h" //Library inclusions
#include "math.h"
#include "string.h"

#define RIGHT_DIR 3      //pre-complier macros allow easy change of parameters 
#define LEFT_DIR 4       //such as pin numbers 
#define RIGHT_WHEELS 5
#define LEFT_WHEELS 6

#define SERVO 2
#define TRIG_0 7
#define ECHO_0 8
#define TRIG_1 11
#define ECHO_1 12
#define TRIG_2 10
#define ECHO_2 13


#define US_GAP 6            //distance between ultrasonic sensors
#define TOL_PCENT 0.3       //percent tolerance allowed before hole/bump detected
#define TOL_ABS 5           //absolute tolerance, an addition to the tolerence to help shorted distances
#define MAX_FIND 10         //the maximumm number of holes/bumps it will detect
#define FORWARD_THRESH 10   //how close a blockage should be before triggering a blockage detection

Servo servo;                //servo instance using the servo library


static int result[72][2];     //anlge and distance stored in here and reset each scan

static int centre_offset;     //how far the sensor is ofset from the center point of the pipe 
static int init_ref[72][2];    //reference values for comparison
bool detect_flag = false;               //flag to trigger a hole/bump has been detected
bool return_flag = false;            //flag to signal the rover must now return
float distance_travelled = 0;        //to keep track of the total distance traveled for returning
int num_of_hole_bumps = 0;      //itterator to keep track of holes/bumps 
float MAX_DIST;           
float travel_step;

typedef struct              //structure type for storing all the information and 
{                           //characteristics of the hole or bump
  String type;       
  int start_index;
  int end_index;
  float hole_dist;
  
} hole_bump_t;

hole_bump_t  hole_bump[MAX_FIND];        //creating MAX_FIND instances of hole_bump_t stored as an array to store info


void setup() {
  pinMode(RIGHT_WHEELS,OUTPUT);          //initialise pinmode 
  pinMode(LEFT_WHEELS,OUTPUT);
  pinMode(LEFT_DIR,OUTPUT);
  pinMode(RIGHT_DIR,OUTPUT);  
  pinMode(TRIG_0,OUTPUT);
  pinMode(ECHO_0,INPUT);
  pinMode(TRIG_1,OUTPUT);
  pinMode(ECHO_1,INPUT);
  servo.attach(SERVO);          //init servo library to the correct pin
  Serial.begin(9600);                  //setting the serial baud rate to 9600

 digitalWrite(RIGHT_DIR,1);       //setting the default travel direction
 digitalWrite(LEFT_DIR,0);

}

void loop() 
{
  user_intf();            //set-up envroment variables (global) from user input
  init_scan();            //initial scan to get reference values
    delay(100);
  standard_scan(MAX_DIST, travel_step);     //standard scan to start the main program
    delay(50);
  dir_flip();                          //on returning flip motor direction and travel back 
  travel(0.65, distance_travelled);
  user_data();                        //report info back to user
  
    while(1);
}

void travel(float speed_in, float rover_distance) 
{
  
  if((rover_distance > 0.5) || ((int(rover_distance*20)%1) != 0))    //values outside the lookup table index range 
  {                                                                  //could cause a read overflow, return to protect against this
    Serial.print("error incorrect rover distance");
    return;
  }
  
  const float calibration[2][10] =                                //lookup table as a const 2d array (faster than a map)
  {
    {0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5},
    {2.05, 1.62, 1.38, 1.31, 1.22, 1.18, 1.15, 1.11, 1.09, 1.07}
  };
  
  float PWM_speed = speed_in - 0.1503;                    //speed calibration
  PWM_speed = PWM_speed / 0.0041;
  float time_duration = rover_distance / speed_in;        //t = s/v
  time_duration = time_duration * 1000;                                        //step is 0.05 multiply by 20 to get increments of 1
  time_duration = time_duration * calibration[1][int(rover_distance*20)-1];    //subtract 1 to start at index 0
  analogWrite(RIGHT_WHEELS, PWM_speed);                       //set PWM to turn wheels
  analogWrite(LEFT_WHEELS, PWM_speed);
  delay(int(time_duration));                                 //wait for time duration
  analogWrite(RIGHT_WHEELS, 0);                             //switch off wheels by setting pwm to 0
  analogWrite(LEFT_WHEELS, 0);
  distance_travelled = distance_travelled + rover_distance;    //add to global distance_travelled to keep track
}

void init_scan ()
{
  static int diameter = -1;                       //default to -1 for error
  servo.write(0);
  delay(500);
  int topVal = sonicReading(TRIG_0, ECHO_0, 10);        //read top and bottom sensor in position 0 for diameter
  int bottomVal = sonicReading(TRIG_1, ECHO_1, 10);  
  diameter = topVal + bottomVal + US_GAP;               //diameter is sensor reading plus dist between sensors
  centre_offset = bottomVal - (diameter/2);
  Serial.print("Diameter/Height (cm) = ");
  Serial.print(diameter);
  Serial.print("\n");
  Serial.print("Offset (cm) = ");
  Serial.print(centre_offset);
  Serial.print("\n\n");
  
  for (int i = 0; i < 36; i++)             //take a full scan and store in init ref: itterate through 36 times 360/(5*2)
    {
       
       servo.write(i*5);
       init_ref[i][0] = sonicReading(TRIG_0, ECHO_0, 5);
       init_ref[i][1] = i*5;
      
       init_ref[i+36][0] = sonicReading(TRIG_1, ECHO_1, 5);
       init_ref[i+36][1] = 180 + i*5;

    }

  return; //return from function
}


void standard_scan(float total_distance, float travel_step)
{
  int count = total_distance/travel_step;      //calculate number of itterations to reach distance

  for(int i = 0; i < count ; i++)
  {
    travel(0.65,travel_step);             //move rover forward by step
    delay(100);
    forward_scan;                       //forward scan to check for blockages ahead
    scan_360();                       //perform rotary scan (values stored in results
   
    for(int j = 0; j < 72; j++)
    {
      if(result[j][0] > (init_ref[j][0] + (init_ref[j][0] * TOL_PCENT) + TOL_ABS))  //if above tolerence
      {
        hole_bump[num_of_hole_bumps].type = "Hole";         //set to hole and set detect flag for angle_cal
        detect_flag = true;
        
      }
      else if (result[j][0] < (init_ref[j][0] - (init_ref[j][0] * TOL_PCENT) - TOL_ABS))  //if below tolerence
      {
        hole_bump[num_of_hole_bumps].type = "Bump";     //set to bump and set detect flag for angle_cal
        detect_flag = true;
       
      }
    }
   if(detect_flag)    //if detect flag is true/set
   {
    detect_flag = 0;   //reset for next itteration
    angle_cal();       // call angle cal
   }
   if(return_flag)   //if return flag is set the return from function so rover can return to user
    return;
  }
  return;
}

void angle_cal()
{
  bool start_flag = 0;           //start flag to indicate the first out of tol dist detected
  for(int j = 0; j < 72; j++)
  {
    bool bigger_tol = result[j][0] > (init_ref[j][0] + (init_ref[j][0] * TOL_PCENT) +   TOL_ABS);
    bool smaller_tol = result[j][0] < (init_ref[j][0] - (init_ref[j][0] * TOL_PCENT) - TOL_ABS);
    if(bigger_tol || smaller_tol)   //if bigger tol OR smaller tol
    {
      if(start_flag)              //if start flag is set (an out of tol measurement has already been found and start index set)
      {
        hole_bump[num_of_hole_bumps].end_index = j * 5;       //set new end index
        hole_bump[num_of_hole_bumps].hole_dist = distance_travelled;     //set distance travelled
        
      }
      else              //if start flag not set (this is the start of the hole/bump)
      {
        start_flag = 1;               //set flag
        hole_bump[num_of_hole_bumps].start_index = j * 5;     //set start index
        hole_bump[num_of_hole_bumps].end_index = j * 5;             //set end index incase no other values are found
        hole_bump[num_of_hole_bumps].hole_dist = distance_travelled;
     
      }

   
    } 
    
  }
  num_of_hole_bumps++;     //increment for next hole/bump

  
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
int sonicReading(uint8_t trig_i, uint8_t echo_i, int average_num)     //input arguments for pins and number of averages
{
  const int add_cal [3] = {2, 0, 0};          //calbration arrays for each sensor
  const int div_cal [3] = {58, 58, 58};
  int cal_index;
  
  switch (trig_i)                 //switch case to select calibration index
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
      
  unsigned long time_value;        //the pulse length returned from the echo
  unsigned long average = 0;
  for(int i=0; i<average_num; i++)       //iterrate through and take average num of measurements
  {
    delay(70);
    time_value = 0;
    digitalWrite(trig_i,HIGH);            //send 10uS pulse to trigger
    delayMicroseconds(10);
    digitalWrite(trig_i,LOW);
    time_value = pulseIn(echo_i, HIGH);    //measure pulse length of echo HIGH start time LOW stop times
    
    if (time_value == 0)        //if no data from measurement do another one
    {
      i--;
      
    }
    else        //otherwise store value by summing to average
    {
    
      time_value = time_value + add_cal[cal_index];
      time_value = time_value / div_cal[cal_index];
      average = average + time_value;
  
    }
  
 
  }
   average = average / average_num;      //calculate average and return value
   return average;
}

void user_data()

{  
 
  if(num_of_hole_bumps == 0)       //if no holes detected
  {
    Serial.print("No flaws detected.");
  }
  else
  {
    for(int i=0; i < num_of_hole_bumps; i++)     //for each hole/bump
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
  digitalWrite(RIGHT_DIR,0);       //flip direction bits
  digitalWrite(LEFT_DIR,1);
  return;
}


void forward_scan()
{
   int dist = sonicReading(TRIG_2, ECHO_2, 3);    //take measurement 

   if(dist < FORWARD_THRESH)            //if outside threshehold set return flag to return to user
    return_flag = true;

  return;
}


void user_intf()
{
  uint8_t buffer[4] = {0, 0, 0, 0};   //buffer array to store character values
  int temp;   //temp variable to store user input
  int temp_2;
  Serial.print("Hello, Welcome to your pipe scanning rover, I'm Ardy!\n\n");
  Serial.print("Please enter your maximum travel distance in cm: \n");
  
  while(!(Serial.available() > 0))    //is there any data in the serial buffer
  {
  }
  Serial.readBytes(buffer, 3);       //read 3 bytes from the serial bufffer
  temp = (((buffer[0]- 48) * 100) + ((buffer[1]- 48) *10) + (buffer[2]- 48));     //convert ascii to int, multiply by sigfig and sum
  MAX_DIST = float(temp)/100;                          //cast as float and divide by 100 to convert to meters
   Serial.print("Maximum travel distance (cm) = ");
   Serial.print(temp);                               //relay back to user
   Serial.flush();         //clear serial buffer register (does not work futher investigation required)
   Serial.print("\n\n");
   Serial.print("Please enter the incremental distance in cm: \n");
   

  while(!(Serial.available() > 4))            //wait until new bytes arrive in serial buffer register
  {
  }
   Serial.readBytes(buffer, 4);
   
   temp_2 = (((buffer[1]- 48) * 100) + ((buffer[2]- 48) *10) + (buffer[3]- 48));   //convert to int
   travel_step = float(temp_2)/100;           //convert to meters
   Serial.print("Incremental distance (cm) = ");
   Serial.print(temp_2);
   Serial.flush();
   Serial.print("\n \n");

  

  
}
