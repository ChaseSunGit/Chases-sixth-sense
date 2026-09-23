#include <stdio.h>
#include <arduino.h>
#include <IntervalTimer.h>

#include "Tmotor_driver.h"
#include "MegaStrain5000.h"
#include "ADS1115_MF3000.h"
#include "low_level_controller.h"

#define CONTROL_LOOP_FREQUENCY 400 //This is the frequency of the low level control loop and is actuated by back IMU interrupt as the master timer
#define IMU_INTERRUPT_PIN 2 //This is the int pin connected to the IMU for timing the control loop via ISR

MotorOutData motor_L_Data; //position velocity and torque for left motor
MotorOutData motor_R_Data; //position velocity and torque for right motor

MotorControllerData motor_L_ControlData; //Controller data used to feed into the low level controller
MotorControllerData motor_R_ControlData; //Controller data used to feed into the low level controller


bool verbose_Serial = true; //This determines if we are printing messages to the serial port. Should be turned off during normal operation
bool plotting_Serial = false; //Simple csv for serial plotting
//Define some controller related stuff
const float ForceTarget_slack = 6;// During slack, we maintain constant 2 Newtons
const float force_maxAllowed = 60; // Max 60N on the strain gauge

//PID constants

float Ki_admittance = 1;//convert velocity to position
float pos_L_cmd = 0.0;
float pos_R_cmd = 0.0;


//PID constants
//There is also a low level controller on the motor
//These gains are directly provided and Epically-Powerful defaults are used
float kp_motor = 0;
float kd_motor = 0.25;//Place holders
//No I controller, PD only

//Some constants for biotorque scaling
float percent_biotorque = 0.2;
float sbj_mass_kg = 80.0;
#define KNEE_GEAR_RATIO 5 //Ratio to convert knee biotorque to measured force at back
//With a pully radius of 0.1m and being a double pulley, KNEE_GEAR_RATIO = 1/(R*2) = 5
//force = biotorque * KNEE_GEAR_RATIO

//Variables to record initial angle of imus as offsets before using them for actuation
float IMU_offset[5][3] = {0};
int loop_counter = 0;
#define NUM_LOOP_BEFORE_AVG 800 // number of iterations to average to get zero angles
#define NUM_LOOP_BEFORE_ADD 400
bool actuation_enable_angle = false; //Boolean value to turn off controller to not actuate motor when taking offsets

//A global variable to record state of FSM
int FSM_state = 0;

//A variable to keep track of current time
unsigned long current_us;
unsigned long current_us_isr;
unsigned long last_us_isr = 0;
//--------------------------------------------------

//Function prototype for computing motor cmd given mode
float compute_motor_cmd(int controller_mode, float MLtorqueOut);


volatile bool IMU_ISR_Flag = false; // Flag set by ISR

// ISR for MPU6050 INT pin
void imuISR() {
  IMU_ISR_Flag = true;
  current_us_isr = micros()-last_us_isr;
  last_us_isr =  micros();
}

void setup(void)
{
  //-------------------Initializing Serial----------------------------
  //In this block, initialize serial communication to jetson orin nano to receive
  //200Hz torque outputs if in ML mode. Alternatively, local BNOs can be used to run
  //a state machine based impedance controller

  Serial.begin(912600);
  
  //-------------------Initializing I2C----------------------------
  //Initialize all 3 busses of the I2C

  //Set 12C bus 0. Wire is 0, Wire1 is 1, and Wire2 is 2
  Wire.begin();              // Bus 0 (pins SDA=18, SCL=19)
  Wire.setClock(1000000);     // 400 kHz
  Wire.setTimeout(400);

  Wire1.begin();              // Bus 1 (pins SDA=17, SCL=16)
  Wire1.setClock(1000000);     // 400 kHz
  Wire1.setTimeout(400);

  Wire2.begin();              // Bus 2 (pins SDA=25, SCL=24)
  Wire2.setClock(1000000);     // 1 MHz. Bus 2 is reserved for chips on board, which can use highspeed +
  Wire2.setTimeout(400);

  //Bus0 will be 2X MPU6050 on left shank and thigh. Possibly one additional ADC for passive L strain gauge
  //Bus1 will be 2X MPU6050 on right shank and thigh. Possibly one additional ADC for passive R strain gauge
  //Bus2 will be 1x MPU6050 on back and 2x active strain gauge ADC on left/right leg and 2x passive strain gauge ADC


  //-------------------Initializing IMU----------------------------
  //In this block, MS5Ks are initialized
  
  MegaStrain_bulk_initialization(CONTROL_LOOP_FREQUENCY);

  //Enable the interrupt on the back IMU. If IMU mode not used, disable
  pinMode(IMU_INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), imuISR, FALLING);//Attach the int pin of the IMU to the ISR
  
  ICM_bulk_calibration(400, IMU_ISR_Flag);
  //Calibrate accel. This line will block code from advancing forward so comment out for normal operation
  //mpu6050_factory_accel_calibration(2,400,IMU_ISR_Flag);
  

  //-------------------Initializing Strain Gauge----------------------------
  //Begin I2C communication on 2x strain gauge ADCs and configure them
  //Strain gauge must be initialized after IMUs since it relies on IMU ISR to time
  //If IMU mode is not used, just give a 200hz clock signal

  Serial.println("Initializing ADCs");
  ADS1115_bulk_initialization(CONTROL_LOOP_FREQUENCY);
  ADS1115_bulk_calibration(400,IMU_ISR_Flag);//Calibrate all strain gauges at once

  //-------------------Define controller states----------------------------
  //Quick and easy section to create 3 controller states - ML, Impedance, Power

  //Todo: take an input and set the MCU to one of 3 controller states

  //-------------------Initializing Motors----------------------------
  //Initialize AK80-6 motors through enable, motor mode, zero, etc
  
  motor_L_Data.motorID = motor_L_ID;//Motor IDs defined in Tmotor_driver.h
  motor_R_Data.motorID = motor_R_ID;

  CANInitialize(); //Initialize can bus 0 for both motors


  motorInitialize(motor_L_ID);//Left
  motorInitialize(motor_R_ID);//Right
  current_us = micros();

  
}

void loop(void)
{
  int time_since_last_trigger =  micros()-last_us_isr;
  //The loop will run at the set frequency as the flag will be raised every _ us based on set frequency (default 200Hz)
  //The following if statement will trigger when the flag is raised
  if (IMU_ISR_Flag) {//If interrupt flag is triggered
    int waited_since_isr = micros() - current_us_isr;
    //-------------------Clear interrupt falg----------------------------
    //noInterrupts();//Disable interrupt to set flag
    IMU_ISR_Flag = false;
    //interrupts();//Enable interrupts again

    //Used for checking timing
    unsigned long current_ms = millis();
    int loop_total_elapsed = micros() - current_us;//Subtract from the last time measurement to get total loop time
    current_us = micros();
    float dt = (float)loop_total_elapsed/1000000.0;//us to s

    //-------------------Start ADC read----------------------------
    //Due to the nature of the ADC, we start a read first
    //Once the IMUs are read, read ADC data because data conversion takes time

    
    request_conversion_all_ADC();//Non blocking request for read, ~10us



    int loop_elapsed_ADC_request = micros()-current_us;

    //-------------------Read IMU----------------------------
    //Make a read of all available IMUs
    
    //Now make a read

    read_all_MS5K();

    int loop_elapsed_IMU_read = micros()-current_us;

    blocking_read_all_ADC();//Read the actual adc data

    

    //Kalman filter all IMUs
    Kalman_batch_MS5K(1); //The IMU structs will now be populated by roll and pitch
    int loop_elapsed_IMU_kalman = micros()-current_us;


    //-------------------Calculate joint angles---------------------
    //Review of IMUs: 0:back, 1:L_shank, 2:L_thigh, 3:R_shank, 4:R_thigh
    //knee and hip angle simply calculated from IMU pitch in upright mode

    //We need to store the true zero reference when the subject is first standing still
    if (loop_counter <= NUM_LOOP_BEFORE_ADD){
      loop_counter ++;
    }
    else if (loop_counter > NUM_LOOP_BEFORE_ADD && loop_counter < NUM_LOOP_BEFORE_AVG){
      loop_counter++;
      //Take offsets
      for (int i = 0; i<(int)NUM_MS5K;i++){
        if(MS5K_Data_Holder[i].valid_flag == 1){
          IMU_offset[i][0] += MS5K_Data_Holder[i].roll;
          IMU_offset[i][1] += MS5K_Data_Holder[i].pitch;
          IMU_offset[i][2] += MS5K_Data_Holder[i].yaw;
        }
      }
    }
    else if (loop_counter == NUM_LOOP_BEFORE_AVG){//Take the average
      loop_counter++;
        for (int i = 0; i<(int)NUM_MS5K;i++){
        if(MS5K_Data_Holder[i].valid_flag == 1){
          IMU_offset[i][0] = IMU_offset[i][0] / (float)(NUM_LOOP_BEFORE_AVG-NUM_LOOP_BEFORE_ADD);
          IMU_offset[i][1] = IMU_offset[i][1] / (float)(NUM_LOOP_BEFORE_AVG-NUM_LOOP_BEFORE_ADD);
          IMU_offset[i][2] = IMU_offset[i][2] / (float)(NUM_LOOP_BEFORE_AVG-NUM_LOOP_BEFORE_ADD);
        }
      }
    }
    float angle_knee_L = (MS5K_Data_Holder[1].pitch-IMU_offset[1][1]) - (MS5K_Data_Holder[2].pitch-IMU_offset[2][1]); //We add because thigh pitch is negative, zero means fully straight
    float angle_knee_R = (MS5K_Data_Holder[3].pitch-IMU_offset[3][1]) - (MS5K_Data_Holder[4].pitch-IMU_offset[4][1]);

    float angle_hip_L = (MS5K_Data_Holder[0].pitch-IMU_offset[0][1]) - (MS5K_Data_Holder[2].pitch-IMU_offset[2][1]);
    float angle_hip_R = (MS5K_Data_Holder[0].pitch-IMU_offset[0][1]) - (MS5K_Data_Holder[4].pitch-IMU_offset[4][1]);

    //For hip rotation, we take the average of the left and right shank yaw and subtract the back yaw

    float angle_lumb_rotation = ((MS5K_Data_Holder[3].yaw-IMU_offset[3][2])+(MS5K_Data_Holder[1].yaw-IMU_offset[1][2]))/2.0 - (MS5K_Data_Holder[0].yaw-IMU_offset[0][2]);
    //float angle_lumb_rotation = (MS5K_Data_Holder[3].yaw-IMU_offset[3][2]) - (MS5K_Data_Holder[0].yaw-IMU_offset[0][2]);
    

    //-----------------------------Calculate joint velocities----------------------------------
    //This step is used to calculate the velocities of each joint of actuation for the feed-forward term
    //Velocity feedforward will account for deflection due to motion
    //Feedforward can be a simple time derivative of the joint angles, or directly taken from gyro.
    //We take directly from the gyro since its much cleaner

    //Joint velocity calculation from raw gyro with no filter
    float vel_knee_L_gyro = -1.0*(MS5K_Data_Holder[1].gx+MS5K_Data_Holder[2].gy);
    float vel_knee_R_gyro = -1.0*(MS5K_Data_Holder[3].gx+MS5K_Data_Holder[4].gy);

    float vel_hip_L_gyro = -1.0*(MS5K_Data_Holder[0].gx+MS5K_Data_Holder[2].gy);
    float vel_hip_R_gyro = -1.0*(MS5K_Data_Holder[0].gx+MS5K_Data_Holder[4].gy);

    int loop_elapsed_IMU_calc = micros()-current_us;


    //-----------------------------------------------Make ADC read-----------------------------
    //Wait for ADC to complete conversion and read out forces on strain gauges
    

    
    int loop_elapsed_ADC_read = micros()-current_us;

    LPF_force_conversion_batch_ADC();
    int loop_elapsed_ADC_convert = micros()-current_us;

    float force_L = ADC_Data_Holder[0].force_N;
    float force_R = ADC_Data_Holder[1].force_N;

    //-------------------Calculate force requirement----------------------------
    //Calculate desired force output based on angles
    //This is the midlevel controller - if ML mode is used, this is where biotorque commands are read and converted to required force
    

    //kd is 10 times larger than kp

    // Control equation: Eff = kp(error) - kd(velocity) + ki(integral(error))

    float setpoint_L = ForceTarget_slack;//For now, only run slack mode
    float setpoint_R = ForceTarget_slack;//For now, only run slack mode
    //float total_time = (float)current_ms/1000.0;
    //float frequency = 0.2;
    //float setpoint_0 = ForceTarget_slack + (sin(total_time * 2.0 * 3.14159 *frequency) * 10+10);
 
    //PID for Motor A

    //--------------------------------------------------------------------Controller------------------------------------------
    
    /*
    admittance_controller(motor_L_ControlData, setpoint_L, force_L, dt);
    pos_L_cmd = pos_L_cmd + Ki_admittance * dt * motor_L_ControlData.current_Vel_cmd; //simple integration to get position from velocity
    float vel_L_cmd = motor_L_ControlData.current_Vel_cmd;
    
    admittance_controller(motor_R_ControlData, setpoint_R, force_R, dt);
    pos_L_cmd = pos_R_cmd + Ki_admittance * dt * motor_R_ControlData.current_Vel_cmd; //simple integration to get position from velocity
    float vel_R_cmd = motor_R_ControlData.current_Vel_cmd;

    int loop_elapsed_ADM_calc = micros()-current_us;
    */
    //------------------------------------------------------------------torque cap------------------------------------------------


    //-------------------Motor cmd----------------------------
    //Send CAN commands by applying lowlevel controller to convert requested force into cmd signal
    //This is where the low level controller is applied
    //As soon as motor cmd is sent, the motor will return its current state which we can read

    //send_motor_message(motor_L_ID, pos_L_cmd, 0, motor_L_ControlData.Torque_ff, kp_motor, kd_motor);   
    //send_motor_message(motor_R_ID, pos_R_cmd, 0, motor_R_ControlData.Torque_ff, kp_motor, kd_motor);   

    //send_motor_message(motor_L_ID, 0, motor_L_ControlData.current_Vel_cmd, motor_L_ControlData.Torque_ff, kp_motor, kd_motor);   
    //send_motor_message(motor_R_ID, 0, -motor_R_ControlData.current_Vel_cmd, motor_R_ControlData.Torque_ff, kp_motor, kd_motor); 

    send_motor_message(motor_L_ID, 0, 0, 0, 1, 0);   
    send_motor_message(motor_R_ID, 0, 0, 0, 1, 0);   
    
    int motor_read_status = readAllMotorData(motor_L_Data, motor_R_Data);
    

    int loop_active_elapsed = micros()-current_us;

    float loop_active_elapsed_ms = (float)loop_active_elapsed / 1000.0;

    //-------------------Print values for serial plotting----------
    if (plotting_Serial){
      //Print comma separated values
      //Serial.printf("%.4f,%.4f,%4f,%.4f,%.4f,%4f,%.4f,%.4f,%.4f\n",MS5K_Data_Holder[0].ax,MS5K_Data_Holder[0].ay,MS5K_Data_Holder[0].az,MS5K_Data_Holder[0].gx,MS5K_Data_Holder[0].gy,MS5K_Data_Holder[0].gz,MS5K_Data_Holder[0].pitch,MS5K_Data_Holder[0].roll,loop_active_elapsed_ms);
      Serial.printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",current_us_isr,time_since_last_trigger,loop_elapsed_ADC_request,loop_elapsed_IMU_read,loop_elapsed_IMU_kalman,loop_elapsed_IMU_calc,loop_elapsed_ADC_read,loop_elapsed_ADC_convert,loop_active_elapsed,loop_total_elapsed);
      //Serial.printf("%d,%.4f,%.4f,%.4f\n",loop_total_elapsed,MS5K_Data_Holder[0].az,MS5K_Data_Holder[0].gx,MS5K_Data_Holder[0].pitch);
    }
    
    //-------------------Print debug to serial port----------------
    if (verbose_Serial){

      Serial.printf("\n\n\n\n\nLoop start time: %d\n",current_ms);
      //Print IMU, ADC, and motor information as well as timing information
      
      for (int i = 0; i<(int)NUM_ADCS;i++){
      if(ADC_Data_Holder[i].valid_flag == 1){
        Serial.printf("ADC %d volt: %.4f\t filtered: %.4f\t force: %.4f\t filtered %.4f\n",i,ADC_Data_Holder[i].voltageADC,ADC_Data_Holder[i].voltage_filtered,ADC_Data_Holder[i].force_N,ADC_Data_Holder[i].force_N_filtered);
        }
      }

      for (int i = 0; i<(int)NUM_MS5K;i++){
        if(MS5K_Data_Holder[i].valid_flag == 1){
          Serial.printf("IMU %d ax: %.4f\t ay %.4f\t az %.4f\t gx %.4f\t gy %.4f\t gz %.4f\n",i,MS5K_Data_Holder[i].ax,MS5K_Data_Holder[i].ay,MS5K_Data_Holder[i].az,MS5K_Data_Holder[i].gx,MS5K_Data_Holder[i].gy,MS5K_Data_Holder[i].gz);
          Serial.printf("IMU %d roll: %.4f\t pitch %.4f\t yaw %.4f\n",i,MS5K_Data_Holder[i].roll,MS5K_Data_Holder[i].pitch,MS5K_Data_Holder[i].yaw);
        }
      }
      //Printing joint angles
      Serial.printf("Joint angles: Left knee: %.4f\t Right knee: %.4f\t Left hip: %.4f\t Right hip: %.4f\t Lumbar rotation: %.4f\n",angle_knee_L, angle_knee_R, angle_hip_L, angle_hip_R, angle_lumb_rotation);
      Serial.printf("Joint velocity from gyro:   Left knee: %.4f\t Right knee: %.4f\t Left hip: %.4f\t Right hip: %.4f\n",vel_knee_L_gyro, vel_knee_R_gyro, vel_hip_L_gyro, vel_hip_R_gyro);
      //Serial.printf("IMU offsets R thigh pitch %.4f\t R shank pitch %.4f\n",IMU_offset[4][1],IMU_offset[3][1]);
      //Printing controller and motor:
      //Serial.printf("Admittance controller output L: Current force error: %.4f\t velocity CMD: %.4f\t angle CMD: %.4f\t current angle: %.4f\t torque FF: %.4f\n",motor_L_ControlData.current_Force_error,motor_L_ControlData.current_Vel_cmd,pos_L_cmd,motor_L_Data.position,motor_L_ControlData.Torque_ff);
      //Serial.printf("Admittance controller output R: Current force error: %.4f\t velocity CMD: %.4f\t angle CMD: %.4f\t current angle: %.4f\t torque FF: %.4f\n",motor_R_ControlData.current_Force_error,motor_R_ControlData.current_Vel_cmd,pos_R_cmd,motor_R_Data.position,motor_R_ControlData.Torque_ff);

      //Serial.printf("Motor %d (L) data: Position: %.4f\t velocity: %.4f\t torque: %.4f\n",motor_L_Data.motorID,motor_L_Data.position,motor_L_Data.velocity,motor_L_Data.torque);
      //Serial.printf("Motor %d (R) data: Position: %.4f\t velocity: %.4f\t torque: %.4f\n",motor_R_Data.motorID,motor_R_Data.position,motor_R_Data.velocity,motor_R_Data.torque);

      
      Serial.printf("Loop active elapsed: %d, loop total elapsed: %d\n",loop_active_elapsed, loop_total_elapsed);

      
      
    }
    //IMU_ISR_Flag = false; //This is a safeguard against double reads
  }

}

float compute_motor_cmd(int controller_mode, float MLtorqueOut){
  //This function will contain BNO reads and determine FSM mode and torque output
    //Inputs:
    //  -int controller_mode: 0 -> FSM; 1 -> ML
    //  -float MLtorqueOut: only used for ML controller, direct scaling
    //Outputs:
    //  -float motor_cmd: commanded velocity
  float out = 0;

  //------------------------------FSM Controller------------------------------
  if (controller_mode == 1){
    //Using fsm
    //Todo - add state
    //State 0 = follower
    if(FSM_state == 0){
      out = ForceTarget_slack;
    }
    else{
      //Invalid state, go to follower mode
      out = ForceTarget_slack;
    }
  }
  //------------------------------ML Controller------------------------------
  else if(controller_mode == 2){
    //Scale output to command
    out = MLtorqueOut * KNEE_GEAR_RATIO;
  }
  else{//Invalid mode
    return 0;
  }
  //------------------------------Limiting output------------------------------
  if (out < ForceTarget_slack){
    //If the force command is less than the slack force command, turn it to the slack force command
    out = ForceTarget_slack;
  }
  else if(out >= force_maxAllowed){
    //Do not allow force to be larger than max allowed (default 30N)
    out = force_maxAllowed;
  }

  return out;
}