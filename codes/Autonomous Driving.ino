#include <Servo.h>      //define the servo library
#include <Math.h>


 // Front Sensor
#define fsTrigPin 9
#define fsEchoPin 10   

// Left Sensor
#define lsTrigPin 6     
#define lsEchoPin 7

// Steering port and motor port
#define steerCom 12       
#define motorCom 11


Servo ssm; //create an instance of the servo object called ssm
Servo esc; //create an instance of the servo object called esc


  // Set the max throttle for forwards or backwards, change when inside vs outside
  int vMin = 0;
  int vMax = 0; 
  int outside = 1;                                 // Variable to use to determine inside/outside variable used
  int checkDist = 200;                             // Distance to check to know if braking is needed
  float est_FS = 0;                                // est_x variable for the corrected state, output
  float kg_FS;                                     // kalman_gain variable for the kalman gain K
  float predFS;                                    // predX variable for the prediction of the state
  float kf_errorFS;                                // error variable for the predicted error of the state
  float RFS = 9.93;                                // R variable of the sensor, calculated offline
  float PFS = 0;                                   // P for the error variance
  int initializeFS = 0;                            // initialize varaible to know if the kalman filter initial conditions have been set
  int QFS = 1;                                     // Q value for kalman filter, front sensor
  int steering=90;                                 // neutral values for steering and throttle
  int throttle=1500;                               //defining global variables to use later
  float currTime=0;                               // Values for steering PID
  float diffTime=0;
  float preTime=0;
  float rateErr=0;
  float eS = 0, eS1 = 0, eS2 = 0;

  
  int greater800 = 0;                             // Variables used to determine if there is a new object and if the distance to an object
  int newObject = 0;                              // is greater than 900 (was 800 original)

  // PID variables for steering
  float KpS=.5;                              //2
  float KiS=2;                              //5
  float KdS=.75;                            // was 1
  int corrected = 0;                       // Variable to know if they motor has been corrected

  // Values for throttle PID                                             
  float Kp= 5.5;                                 //1.5; // was 2   was 5
  float Ki= 3;                                   //.5; // was 1.5    // was 2
  float Kd= 2;                                   // was 1
  float e=0, e1 = 0, e2 = 0;

  // Values for calculating PID for throttle                                         
  float k1 = Kp + Ki + Kd;
  float k2 = -Kp - (2*Kd);
  float k3 = Kd;

  // PID output variables output for the controller                                              
  float u = 0;
  float uS = 0;                    
  int moveForward = 0;                               // Variable to know if the vehicle is moving forward
  int initialize = 0;                               // Variable to know if the vehicle has been initialized
  float setDist = 0;                               // Used to set the distance to the wall based on the first reading from the sensor
  int setForwardDist = 0;                          // Variable used to change the distance to the object in front. Seems to change daily :|


void setup() {
  //Serial.begin(9600); //start serial connection. Uncomment for PC

  pinMode(fsTrigPin, OUTPUT);                  // Set the trigger pin as an output for the front sensor
  pinMode(fsEchoPin, INPUT);                   // Set the echo pin as an input for the front sensor

  pinMode(lsTrigPin, OUTPUT);                 // Set the trigger pin as an output for the left sensor
  pinMode(lsEchoPin, INPUT);                  // Set the echo pin as an input for the left sensor
  
  ssm.attach(steerCom);          // Set up the ports for steering and motor //define that ssm is connected at pin 10
  esc.attach(motorCom);       //define that esc is connected at pin 12
  pinMode(LED_BUILTIN, OUTPUT);

  // Set the values in the set vehicle function for the throttle control
  // This will change the values depending on it the car is outside or inside
  if (outside == 1){
    vMin = 1420;                   // test out 1400
    vMax = 1565;
    setForwardDist = 420;         // test out 420
  }
  else{
    vMin = 1420;                  //1500 is neutral
    vMax = 1575;
    setForwardDist = 380;       
  }


  
}

void loop() {
   //control logic
  // Variable for determining the average distnace to the wall on the left side
  // Just the cummulative distance before it is averaged.
   float TotDistl = 0;
  
   float newSteer = 0;   // Variable for determining the average distnace to the wall on the left side
  
  float newThrot = 0;   // Variable for the additional throttle value

  float steerTo = 0;   // Variable to know where to steer to
  
  float durationFS, distanceFS;     // Duration and distance for the front sensor

  int n = 0;         // Variable for the for loop for averaging the left sensor readings.

  // Initialize the motor and the side sensor distance
  if (initialize == 0){
    //setVehicle(steering, throttle);

    // Set initialize to 1 so that it doesn't run through this section again until the 
    // Arduino has been turned off or reset. 
    initialize = 1;


    // Sent the motor forward slightly, then brake, then go to neutral, and then backwards
    // to initialize the motor for either forward or backward reverse. 
    
    //Serial.println("Forwared");
    esc.writeMicroseconds(1575); delay(50);
    //Serial.println("Brake");
    esc.writeMicroseconds(1425); delay(50);
    //Serial.println("Neutral");
    esc.writeMicroseconds(1500); delay(50);
    //Serial.println("Backwards");
    esc.writeMicroseconds(1400); delay(50);
  
    // Set the vehicle status to no steering, no throttle and no error 
    setVehicle(steering, throttle, 0);

    // Variable for setting the distance from the left side wall, for the left side sensor.
    float durationInit = 0;
    //Serial.print("init");


    // Set the trigger on for sensor 1 to low for 2 microseconds to ensure the trigger is not on
    digitalWrite(lsTrigPin, LOW);
    delayMicroseconds(2);
  
    // Set the trigger on sensor 1 high for 10 microseconds per the data sheet to send an 8 cycle sonic
    // burst
    digitalWrite(lsTrigPin, HIGH);
    delayMicroseconds(10);
  
    // Set the trigger for sensor 1 back to low since the 8 cycle sonic burst has been sent. 
    digitalWrite(lsTrigPin, LOW);
  
    // Wait for the pulse to be recieved back, the pulseIn() function calculates the duration that it takes to
    // recieve the pulse back.
    durationInit = pulseIn(lsEchoPin, HIGH, 10000);

    // Calculate the distance to the wall from the left side, use this as the setpoint for the distance
    // and calculating the error. 
    setDist = ((0.1751*durationInit) - 2.1751);
   // ((0.1751*est_LS) - 2.1751);
  }



  // Loop for averaging the distance for the side sensor. 
  for(n = 0; n<=2; n++){

    // Duaration and distance value for the left srnsor readings
    float durationl = 0;
    float distancel = 0;

  
    // Set the trigger on for sensor 1 to low for 2 microseconds to ensure the trigger is not on
    digitalWrite(lsTrigPin, LOW);
    delayMicroseconds(2);
  
    // Set the trigger on sensor 1 high for 10 microseconds per the data sheet to send an 8 cycle sonic
    // burst
    digitalWrite(lsTrigPin, HIGH);
    delayMicroseconds(10);
  
    // Set the trigger for sensor 1 back to low since the 8 cycle sonic burst has been sent. 
    digitalWrite(lsTrigPin, LOW);
  
    // Wait for the pulse to be recieved back, the pulseIn() function calculates the duration that it takes to
    // recieve the pulse back.
    durationl = pulseIn(lsEchoPin, HIGH, 10000);

    if (durationl < 5){
      durationl = 10000+random(0,5);
    }

    // Calculate the distance that sensor 1 is measuring to mm, add one to offset and help correct the distance for testing day
    distancel = ((0.1751*durationl) - 2.1751);

    // Took this delay out 11/19/2019 at 11:23 am
    //delay(10);

    // Cummulative distance
    TotDistl = TotDistl + distancel;
  }

  // Get the current time
  currTime = millis();

  // Checked to see how much time has gone by since the previous time 
  // and the current time for the integral and derivative. 
  diffTime = currTime - preTime;

  // Calculate the average distance to steer for
  steerTo = TotDistl/n;

  // Calculate the steering error. 
  eS = 300 - steerTo;

  // If the steering error is less than 20 then don't steer yet. This should change, likely to be 10. 
  // Should try that
  // Steer if the error is more than 1, if it is less than 1 then the error is 0.
  if(abs(eS) < 1){
    eS=0;
  }

  // Get the error rate for computing the derivative
  rateErr = (eS - eS1)/diffTime;

  // Calculate the change needed for the controller for steering
  uS = (KpS * eS) + (KdS * rateErr);

  // Set the last error to the current error
  eS1 = eS;

  // Set the previous time to the current time. 
  preTime = currTime;

  // delay for 100 ms
  // took this delay out 11/19/2019 at 11:15 am
  delay(100);

  // Calculate the new steering angle
  newSteer = steering + uS;

  // Kalman Filter Front Sensor
  if (initializeFS == 0){

    // Create variable for the inital duration to be saved in, only needed for the init, no need for use 
    // elsewhere
    float initDurationFS = 0;

    // Set the trigger on the sensor low for 2 microseconds to ensure the trigger is not on
    digitalWrite(fsTrigPin, LOW);
    delayMicroseconds(2);

    // Set the trigger on the sensor high for 10 microseconds per the data sheet to send an 8 cycle sonic
    // burst
    digitalWrite(fsTrigPin, HIGH);
    delayMicroseconds(10);

    // Set the trigger back to low since the 8 cycle sonic burst has been sent. 
    digitalWrite(fsTrigPin, LOW);

    // Wait for the pulse to be recieved back, the pulseIn() function calculates the duration that it takes to
    // recieve the pulse back. 
    initDurationFS = pulseIn(fsEchoPin, HIGH, 12000);

    // Set the inital conditions for the prediction and the error. Set the prediction to the duration value
    // that was just measured and the error to R. Then set the initalize variable to 1 since the initalization
    // is completed.
    predFS = initDurationFS;
    kf_errorFS = RFS;
    initializeFS = 1;
  
  }
  else{
    // Set the prediction to the previous corrected state and set the error to the previous corrected error
    // (variance).
    predFS = est_FS;
    kf_errorFS = PFS + QFS;
  }

  // Set the trigger on the sensor low for 2 microseconds to ensure the trigger is not on
  digitalWrite(fsTrigPin, LOW);
  delayMicroseconds(2);

  // Set the trigger on the sensor high for 10 microseconds per the data sheet to send an 8 cycle sonic
  // burst
  digitalWrite(fsTrigPin, HIGH);
  delayMicroseconds(10);

  // Set the trigger back to low since the 8 cycle sonic burst has been sent. 
  digitalWrite(fsTrigPin, LOW);

  // Wait for the pulse to be recieved back, the pulseIn() function calculates the duration that it takes to
  // recieve the pulse back.
  durationFS = pulseIn(fsEchoPin, HIGH, 12000);

  // If the duration is less than 5, then it is likely out of range 
  // since 0 was returned. In order to prevent a zero from propigating 
  // through the kalman filter, then set it to 12000 (the cut off) plus 
  // a random number from 0 to 5, this will help the filter from just 
  // using the same constant over and over again.
  if (durationFS < 5){
    durationFS = 12000+random(0,5);
  }
  
  // Correcting steps:
  // Calculate the kalman gain for the front sensor
  kg_FS = kf_errorFS / (kf_errorFS + RFS);

  // Calculate the correct output for the front sensor
  est_FS = predFS + (kg_FS*((durationFS) - predFS));

  // Calculate the corrected error (variance) for the front sensor
  PFS = (1 - kg_FS)*kf_errorFS;

  // Calculate the filtered distance for the front sernsor
  distanceFS = ((0.176*est_FS) - 1.8021+1);

  // Calculate the error for the front sensor. May need to change the value 
  // based on inside or outside
  e = distanceFS - setForwardDist;

  // If the error is less than 10mm, then just let it be,
  // it is close enough. Just set the error to 0 manually. 
  if (abs(e) < 10){
    e = 0;
  }

  // Calculate the throttle value! 
  u = (k1*e) + (k2*e1) + (k3*e2);

  // calculate the new throttle value based on the throttle + u
  // which is the output from the controller. 
  newThrot = throttle + u;
    
  // If the vehicle has a throttle command that is negative and the vehicle
  // is moving forward, then set the moving forward variable to 0 and then 
  // brake the vehicle to make sure that it slows down accordingly. 
  // May want to make the braking force higher (closer to 1500). See how this
  // parameter works. 
  if(u < 0 && moveForward == 1){
    moveForward = 0;
    digitalWrite(LED_BUILTIN, LOW);
    //Serial.println("Brake");
    esc.writeMicroseconds(1350); delay(100);
    //Serial.println("Neutral");
    esc.writeMicroseconds(1500); delay(100);
    esc.writeMicroseconds(1425); delay(200);
  }

  // If the throttle command is less than zero and the vehicle has not been
  // moving by checking the current error and the previous error, if 
  // subtracting them from one another and the difference is less than 1, then 
  // the car needs to move in reverse but it cannot. So, check the make sure 
  // the car has not been corrected already by moving forward and verify that
  // the distance to the object is less than 200 mm (probably want to make 
  // this more like 300 later if need be), then set the variable corrected to 1
  // to make sure that it does not enter this code without going forward again.
  // So move the car forward and then brake, and then go to neutral and then
  // go to reverse to make sure the car can reverse. This corrects any motion
  // that stops the motor and prevents the car from moving backwards for 
  // some reason. This was an issue in a lot of our testing of the vehicle
  // as it is hard to determine the state of the vehicle without the proper
  // sensors. 
  else if(u < 0 && (abs(e) - abs(e2) < 1)){

    if (corrected == 0 || distanceFS < checkDist){
      //digitalWrite(LED_BUILTIN, LOW);
  
      corrected = 1;
      //Serial.println("Forward");
      esc.writeMicroseconds(1575); delay(100);
      //Serial.println("Brake");
      esc.writeMicroseconds(1425); delay(200);
      //Serial.println("Neutral");
      esc.writeMicroseconds(1500); delay(100);
      esc.writeMicroseconds(1425); delay(100);

    }
  }

  // If the errors are greater than zero, then the vehicle is moving forward,
  // so set the variable moveForward to 1 to be used in other parts of the code
  // to know if the vehicle is moving forward. 
  // Set the variable to correct to 0 since the vehicle will need to brake or 
  // possibly correct itself if for some reason it will not move. 

  
  else if (e1 > 0 && e > 0){
    moveForward = 1;
    corrected = 0;
    //digitalWrite(LED_BUILTIN, LOW);
    //Serial.print("Moving Forward");
  }

  // Send the throttle and steering command. 
  // may need to do this after the braking, but not sure.
  // Leave here for now, might need to change to be later
  setVehicle(newSteer, newThrot, e);

  // If the distance is greater than 800 and a new object has not been
  // detected (picked up from 800 mm to 300 mm) and the vehicle is moving forward,
  // then don't do anything. But, if the object distance is between 800 and 300 mm, 
  // and the vehicle is moving forward, then set the variable newObject to 1 to ensure
  // that this code is not executed again UNLESS a new object is detected that is 800
  // to 300 mm away and the vehicle is moving forward. This is just error checking to 
  // prevent sudden and unnesscary acceleration backwards since this is meant for braking.
  // Then send the throttle command for neutral. 
  // Later changed 800 to 900 since it worked better
  if (newObject == 0 && greater800 == 1){
    greater800 = 0;
    if(distanceFS < 900 && distanceFS > 300 && moveForward == 1){
      digitalWrite(LED_BUILTIN, HIGH);
      newObject == 1;
      moveForward == 0;
      Serial.print("Brake for object, ");
      esc.writeMicroseconds(1400);
      delay(100);
      
  
      // may want to make the steering command something other than just the straight 
      // wheels command. 
      setVehicle(steering, throttle, e);
      
    }
  }

      // If the distance to an object is ever determined to be greater than 800 mm, then 
  // set this variable to 1. This is used to know that the car has gained momentum
  // and will need to be braked more aggressively in order to prevent hitting the object
  // that is ahead that has been detected. One small issue with this is that if the car
  // goes too wide and picks up say the edge of a box, it will set this variable to 1, 
  // and slow the car down too early. This isn't necessarily bad since the car doesn't
  // gain enough momentum to be a problem after this. One way around this is to check
  // for a false alarm by seeing is the next distance reading is below 800 or above 800
  // and by how much. It might be even more reliable to check another reading before 
  // "approving" the change of the variable to 1. Should do this in the next 
  // iteration of the code - accomplish by:
  // At the top check for a false alarm by checking the next reading when greater800 is 1,
  // if it is less than 800, leave it as 1, if it is greater than 800, like say 1000 
  // or so then set back to 0. This is basicallt error checking which won't let the above 
  // if statement execute since the variable will be 0. 
  // Actually based on the above code of setting it back to 0, not sure if that will actually
  // happen anymore since it is alreadying checking to see if the newest data is less than 800
  // So may be good. If it pops up again, then will add. 
  // Later changed 800 to 900 since it worked better
  // Need to check this after the previous if statement because it will prevent it from working.
  // This is looking to see if the current distance is more than 900 and the previous is looking to
  // see if it is less than 900, so need to check current data to see if its great than 900, then
  // if it is the next iteration of the void loop will check if it is less than 900 since it was 
  // previously greater than 900. 
  // All change the check distance to 200 since the car is now moving. 
  if (distanceFS > 900){
    greater800 = 1;

    checkDist = 200;
  }



  // Print the front sensor filtered data and print the PID controller throttle value
 // Serial.print(distanceFS);
//  Serial.print(", ");
//  Serial.println(u);

  // Set the previous throttle error to the previous, previous error for the controller
  e2 = e1;

  // Set the current throttle error to the previous error for the controller
  e1 = e;

  // Set the previous steering error to the previous, previous error for the controller
  eS2 = eS1;

  // Set the current steering error to the previous error for the controller
  eS1 = eS;

}

//********************** Vehicle Control **********************//
//***************** Do not change below part *****************//
void setVehicle(int s, int v, float err) 
{
  s=min(max(85,s),95);  //saturate steering command
  ssm.write(s); //write steering value to steering servo

  // Added for error checking in the throttle, this is used to ensure that the car does
  // not move when there is no error. The code in the main function should handle this
  // correctly, this is just an extra measure. 

  // If the error is not zero, then alllow a throttle command, else just send neutral
  // which is always a safe command. 
  if (err != 0){
      v=min(max(vMin,v),vMax); //saturate velocity command
      esc.writeMicroseconds(v); //write velocity value to the ESC unit
  }
  else{
    esc.writeMicroseconds(1500); //write velocity value to the ESC unit
  }
}
//***************** Do not change above part *****************//
