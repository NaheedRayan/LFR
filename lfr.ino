
/*************************************************************************
* Sensor Upper and Lower Threshold
*************************************************************************/
int sensor_threshold_u = 900;
int sensor_threshold_l = 200;

int parity = 1 ;

/*************************************************************************
* Sensor Array object initialisation 
*************************************************************************/
uint16_t sensors_sum=0;
uint16_t sensors_average=0;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount] = {0,0,0,0,0,0,0,0};

const uint16_t threshold = 500; // adjustable - can take values between 0 and 1000
/*************************************************************************
* PID control system variables 
*************************************************************************/
float Kp = 0.06; //related to the proportional control term;(ex: 0.07). 
float Ki = 0.0008; //related to the integral control term; (ex: 0.0008).
float Kd = 0.6; //related to the derivative control term; (ex: 0.6).
int P;
int I;
int D;

/*************************************************************************
* Global variables
*************************************************************************/
int lastError = 0;
boolean onoff = false;

/*************************************************************************
* Motor speed variables (choose between 0 - no speed, and 255 - maximum speed)
*************************************************************************/
const uint8_t maxspeeda = 150;
const uint8_t maxspeedb = 150;
const uint8_t basespeeda = 100;
const uint8_t basespeedb = 100;
/*If your robot can't take tight curves, you can set up the robot
  to revolve around the base (and not around one of the wheels) by
  setting the minspeed to a negative value (-100), so the motors will go 
  forward and backward. Doing this the motors can wear out faster. 
  If you don't want to do this, set the minspeed to 0, so the motors 
  will only go forward.
*/
const int minspeeda = 0;
const int minspeedb = 0;
// const int minspeeda = -100;
// const int minspeedb = -100;


/*************************************************************************
* l298 GPIO pins declaration
*************************************************************************/
// motor one
int enA = 5;//D5
int in1 = 2;//D2
int in2 = 3;//D3
// motor two
int enB = 6;//D6
int in3 = 7;//D7
int in4 = 4;//D4

/*************************************************************************
* Buttons pins declaration
*************************************************************************/
int buttoncalibrate = 11; //D11
int buttonstart     = 12; //D12

/*************************************************************************
* Function Name: setup
**************************************************************************
*************************************************************************/
void setup() {
 
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  Serial.begin(38400);// Read in baud 9600 if using arduino nano

  pinMode(LED_BUILTIN, OUTPUT);
  boolean Ok = false;
  // while (Ok == false) { // the main function won't start until the robot is calibrated
  //   if(digitalRead(buttoncalibrate) == HIGH) {
  //     Serial.println("calibrate clicked");
  //     // calibration(); //calibrate the robot for 10 seconds
  //     // Ok = true;
  //   }
  // }
  Ok = true ;
  delay(1000);

  forward_brake(0, 0); //stop the motors
}



/*************************************************************************
* Function Name: loop
**************************************************************************
* Summary:
* This is the main function of this application. When the start button is
* pressed, the robot will toggle between following the track and stopping.
* When following the track, the function calls the PID control method. 
* 
* Parameters:
*  none
* 
* Returns:
*  none
*************************************************************************/
void loop() {

  if(digitalRead(buttonstart) == HIGH) {
    Serial.println("start clicked");
    onoff =! onoff;
    if(onoff = true) {
      delay(1000);//a delay when the robot starts
    }
    else {
      delay(50);
    }
  }
  if (onoff == true) robot_control();
  else forward_brake(0,0); //stop the motors
  
}

/*************************************************************************
* Function Name: forward_brake
**************************************************************************
* Summary:
* This is the control interface function of the motor driver. As shown in
* the Pololu's documentation of the DRV8835 motor driver, when the MODE is 
* equal to 1 (the pin is set to output HIGH), the robot will go forward at
* the given speed specified by the parameters. The phase pins control the
* direction of the spin, and the enbl pins control the speed of the motor.
* 
* A warning though, depending on the wiring, you might need to change the 
* aphase and bphase from LOW to HIGH, in order for the robot to spin forward. 
* 
* Parameters:
*  int posa: int value from 0 to 255; controls the speed of the motor A.
*  int posb: int value from 0 to 255; controls the speed of the motor B.
* 
* Returns:
*  none
*************************************************************************/


void forward_brake(int posa, int posb) {
  //set the appropriate values for aphase and bphase so that the robot goes straight
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, posa);
  analogWrite(enB, posb);
}

//go right
void left_brake(int posa, int posb) {
  // digitalWrite(aphase, LOW);
  // digitalWrite(bphase, LOW);
  // analogWrite(aenbl, posa);
  // analogWrite(benbl, posb);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, posa);
  analogWrite(enB, posb);
}


//go left
void right_brake(int posa, int posb) {
  // digitalWrite(aphase, HIGH);
  // digitalWrite(bphase, HIGH);
  // analogWrite(aenbl, posa);
  // analogWrite(benbl, posb);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, posa);
  analogWrite(enB, posb);
}




/*************************************************************************
* Function Name: PID_control
**************************************************************************
* Summary: 
* This is the function of the PID control system. The distinguishing 
* feature of the PID controller is the ability to use the three control 
* terms of proportional, integral and derivative influence on the controller 
* output to apply accurate and optimal control. This correction is applied to
* the speed of the motors, which should be in range of the interval [0, max_speed],
* max_speed <= 255. 
* 
* Parameters:
* none
* 
* Returns:
*  none
*************************************************************************/


//for reading the sensor value and determining if black or white
int bw(int a)
{
  if(a > sensor_threshold_l && a < sensor_threshold_u)
    return parity;//if black
  // else if(a >= sensor_threshold_u )
  //   return !parity;// if white
  else return !parity;
}


void debugging(String s){
  Serial.print(sensorValues[0]);
  Serial.print(sensorValues[1]);
  Serial.print(sensorValues[2]);
  Serial.print(sensorValues[3]);
  Serial.print(sensorValues[4]);
  Serial.print(sensorValues[5]);
  Serial.print(sensorValues[6]);
  Serial.print(sensorValues[7]);

  Serial.print(s);
  Serial.println("");
}

void read_sensor_data()
{
  for (int i = 0; i < 8; i++) 
    sensorValues[i] = bw(analogRead(7-i)); 

}


int e = 0 ;
void robot_control() {
  
  sensors_average = 0; 
  sensors_sum = 0;
 
  int cnt = 0;
  int sum = 0;
  
  
  for (int i = 0; i < 8; i++) 
  {

    //since we have taken sensor input as inverted so 7-i
    sensorValues[i] = bw(analogRead(7-i)); 

    cnt += sensorValues[i];/////////////////

    sensors_average += sensorValues[i] * i * 1000   ;  
    sensors_sum += sensorValues[i];

    sum = sensors_sum ;/////////////////////

    

  }
  



  double position = (sensors_average / sensors_sum); //read the current position
  int error = 3500 - position; //3500 is the ideal position (the centre)
  
  // changing the default value if the robot is out of line
  // in accordance to the last error polarity

  if(position > 3500 && position <= 7000)e = -1 ;
  else if(position >= 0 && position <= 3500) e = 1 ;
  if(e == -1 && sensors_sum == 0)
    // error = -3500;
    error = -3000;
  if(e == 1 && sensors_sum == 0)
    // error = 3500;
    error = 3000;

  if(cnt == 8){
    debugging("--> intersection ");
    PID(0);
    // delay(500);

  }
  else if((sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) && (cnt != 8 && cnt != 0)){
    parity = !parity ;
    debugging("--> change ");
  }
  // //left 90  
  // else if(
  //   (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1) 
  // ||(sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 1 && sensorValues[3] == 1)){

  //   debugging("--> left 90 ");
  //   // right_brake(80, 80);

  //   do{
  //     read_sensor_data();//updating data
  //     // forward_brake(0,110);
  //     right_brake(40, 100);

  //   }while(sensorValues[3] != 1);

  //   debugging("--> left 90 done ");


  // }
  // //right 90  
  // else if(
  //   (sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1) 
  // ||(sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 1 && sensorValues[7] == 1)){
    
  //   debugging("--> right 90 ");

  //   do{
  //     read_sensor_data();//updating data
  //     // forward_brake(110,0);
  //     left_brake(100, 40);

  //   }while(sensorValues[5] != 1);

  //   debugging("--> right 90 done ");



  // }
  //left 30
  else if(
    // (sensorValues[0] == 1 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 1 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0) ||
    // (sensorValues[0] == 1 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0) ||
    // (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 0 && sensorValues[3] == 1 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 0)
    (sensorValues[0] == 1 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 1 ) ||
    (sensorValues[0] == 1 && sensorValues[1] == 0 && sensorValues[2] == 1 && sensorValues[3] == 1 ) ||
    (sensorValues[0] == 1 && sensorValues[1] == 1 && sensorValues[2] == 0 && sensorValues[3] == 1 )

    ){

    debugging(" --> left 30 ");
  
  
    do{
      read_sensor_data();//updating data
      debugging(" --> UPDATE ");

      int cnt = 0 ;      
      for(int i = 0 ; i < 8 ; i++)
        cnt += sensorValues[i] ;

      forward_brake(100,100);
      if(cnt == 0)
        break ;
    }while(1);
    
    debugging(" --> left 30 step ");
    do{
      read_sensor_data();//updating data
      right_brake(100, 100);
      // forward_brake(0,110);
    }while(sensorValues[3] != 1);

    debugging("--> left 30 done ");

  }
  //right 30
  else if(
    // (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 1 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 1) ||
    // (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 1 && sensorValues[5] == 0 && sensorValues[6] == 1 && sensorValues[7] == 1) ||
    // (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 1)
    (sensorValues[4] == 1 && sensorValues[5] == 0 && sensorValues[6] == 0 && sensorValues[7] == 1) ||
    (sensorValues[4] == 1 && sensorValues[5] == 0 && sensorValues[6] == 1 && sensorValues[7] == 1) ||
    (sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6] == 0 && sensorValues[7] == 1)
    ){

    debugging(" --> right 30 ");

    do{
      read_sensor_data();//updating data
      debugging(" --> UPDATE ");

      int cnt = 0 ;
      for(int i = 0 ; i < 8 ; i++)
        cnt += sensorValues[i] ;

      forward_brake(100,100);
      if(cnt == 0)
        break ;
    }while(1);
    
    debugging(" --> right 30 step ");
    do{
      read_sensor_data();//updating data
      left_brake(100, 100);
      // forward_brake(0,110);
    }while(sensorValues[4] != 1);

    debugging("--> right 30 done ");

  }
  else
  {
    debugging("--> PID ");
    PID(error);
 
  }

  
}

void PID(int error){
  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd; //calculate the correction
                                       //needed to be applied to the speed
  
  int motorspeeda = basespeeda - motorspeed;
  int motorspeedb = basespeedb + motorspeed;
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < minspeeda) {
    motorspeeda = minspeeda;
  }
  if (motorspeedb < minspeedb) {
    motorspeedb = minspeedb;
  }
  //Serial.print(motorspeeda); Serial.print(" "); Serial.println(motorspeedb);
  speedcontrol(motorspeeda, motorspeedb);
}


void speedcontrol(int mota, int motb) {
  if (mota >= 0 && motb >= 0) {
    forward_brake(mota, motb);
  }
  if (mota < 0 && motb >= 0) {
    //dreapta
    mota = 0 - mota;
    right_brake(mota, motb);
     
  }
  if (mota >= 0 && motb < 0) {
    //stanga
    motb = 0 - motb;
    left_brake(mota, motb);
  }
}
