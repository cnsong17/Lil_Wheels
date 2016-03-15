/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include <device.h>
#include <stdio.h>

// ------------- Speed Control Globals ----------------

int mag_counter = 0; // magnet counter
double prevTime; // previous timer time
double period; // timer period
double cmPerSec; // centimeters per revolution
double timePerRev; // time per revolution
const double Kp = 2.8; // proportional constant 0.1, 1.65
const double Ki = 1.1; // integral constant 0.05 , 0.5
const double Kd = 0.1; // derivative constant 0.01 , 0.01
double currErr = 0; // the error term
double TargetSpeed = 121.92 * (3/4); // Target speed
double prevErr = 0; // previous error
double windupGuard = 0; // integral windup guard
double pwm = 70; // the PID gain for the PWM
double intErr; // integral error

double speed_PID(void);

// ------------- Navigation Globals ----------------

int line_counter = 0; // line counter
int frame_counter = 0; // frame counter
int modLine = 20; // read once every 10 lines
double cam_prevTime = 0; // start time @ line read
double cam_currTime = 0; // stop time @ black line detected
double timeDiff = 0; // delta t
double AvgTimeDiff; // average delta t
double diff; // for testing
double cam_testCurr = 0; // for testing
double cam_testPrev = 0; // for testing
const double Kp_nav = 0.01; // proportional constant 0.1, 1.65
const double Ki_nav = 0.0; // integral constant 0.05 , 0.5
const double Kd_nav = 0.0; // derivative constant 0.01 , 0.01

double nav_PID(void); 

// ======================== SPEED CONTROL =============================

CY_ISR(inter)
{
    double distPerRev = 19.669/5; //centimeters
    double currTime; // current timer time
    char speed[17]; // speed string
    char pwmString[17]; // pwm string
 
    currTime = SpeedControl_Timer_ReadCounter(); //read timer (timer counts down)
    if(prevTime < currTime) // overflow of the timer period has occured
    {
        timePerRev = (prevTime + (period - currTime))/10000;
        cmPerSec = distPerRev/timePerRev;
    }
    else
    {
        timePerRev = (prevTime - currTime)/10000;
        cmPerSec = distPerRev/timePerRev;
    }
     
    prevTime = currTime;
    mag_counter++;
    
    // run the PID control
    pwm = speed_PID();
    if(pwm > 999) pwm = 999;
    if(pwm < 0) pwm = 0;
    Speed_PWM_WriteCompare(pwm); 
    
    if (mag_counter == 6) 
    {
       /* mag_counter = 1;
        LCD_Position(0,0);    
        sprintf(speed, "%f", cmPerSec);
        LCD_PrintString(speed);
        
        LCD_Position(1,0);
        sprintf(pwmString, "%f", pwm);
        LCD_PrintString(pwmString);  */
    }
}

// PID controller for speed control
double speed_PID(void)
{
    double diffErr; // differential error
    double pTerm; // proportional term
    double iTerm; // intergral term
    double dTerm; // derivative term
    
    // the error term
    currErr = TargetSpeed - cmPerSec; 
    
    // integral response with windup protection
    intErr+=(currErr*timePerRev);
    /*if (intErr < -windupGuard)
        intErr = -windupGuard;
    else if (intErr > windupGuard)
        intErr = windupGuard;
    */
    // differential
    diffErr = (currErr - prevErr)/timePerRev;
    
    // scaling
    pTerm = Kp*currErr; // proportional response
    iTerm = Ki*intErr; // integral response
    dTerm = Kd*diffErr; // differential response
    
    prevErr = currErr; //save the current error
    
    return pTerm + iTerm + dTerm;
}

// ======================== NAVIGATION =============================

// interrupt for beginning of burst
CY_ISR(burst_inter)
{ 
    int nav_period = 65536;
    
    // every modLine lines, read the comparator
    if (line_counter == modLine)
    {
        // start comparator (DAC reference)
        Cam_Comparator_Start();
    
        cam_prevTime = Nav_Timer_ReadCounter();
        
        /*cam_testCurr = Nav_Timer_ReadCounter();
        if (cam_testPrev < cam_testCurr) // overflow of the timer period has occured
            diff = (cam_testPrev + (nav_period - cam_testCurr));
    
        else diff = (cam_testPrev - cam_testCurr);
            cam_testPrev = cam_testCurr;
        */
        
        line_counter = 0;
    }
    else line_counter++; 
}

// interrupt for beginning of line
CY_ISR(line_inter)
{

}

// interrupt for black line detection
CY_ISR(cam_inter)
{
    int nav_period = 65536;
 
    cam_currTime = Nav_Timer_ReadCounter();
    
    if (cam_prevTime < cam_currTime) // overflow of the timer period has occured
        timeDiff = (cam_prevTime + (nav_period - cam_currTime));
    
    else timeDiff = (cam_prevTime - cam_currTime);
    
    /*
      cam_testCurr = Nav_Timer_ReadCounter();
        if (cam_testPrev < cam_testCurr) // overflow of the timer period has occured
            diff = (cam_testPrev + (nav_period - cam_testCurr));
    
        else diff = (cam_testPrev - cam_testCurr);
            cam_testPrev = cam_testCurr;
    */    
    
    // stop comparator (DAC reference)
    Cam_Comparator_Stop();

}

// interrupt for end of frame
CY_ISR(frame_inter)
{
    char timeString[17];
    double servoVal;
    int AvgWindow = 3; // avg every three frames
       
    // take moving average
    if (AvgTimeDiff == 0)
        AvgTimeDiff = timeDiff;
        
    AvgTimeDiff *= (AvgWindow - 1)/AvgWindow;
    AvgTimeDiff += timeDiff/AvgWindow;
        
    if (frame_counter == 32)
    {       
        LCD_Position(0,0);
        sprintf(timeString, "time diff: %f", timeDiff);
        LCD_PrintString(timeString); 
        frame_counter = 0;
    }
    
    else frame_counter++;
    
    servoVal = nav_PID();
    
    // boundary conditions for servo
    if (servoVal > 0.47)
        servoVal = 0.47;
    if (servoVal < -0.43)
        servoVal = -0.43;
    
    Steering_PWM_WriteCompare((1.52 - servoVal)*100);
}

// PID controller for nav control
double nav_PID(void)
{
    double diffErr_nav; // differential error
    double currErr_nav;
    double pTerm_nav; // proportional term
    double iTerm_nav; // intergral term
    double dTerm_nav; // derivative term
    double intErr_nav;
    double prevErr_nav;
    
    double targetTime = 330.0; // when the black line is centered in the frame ------------------------------------------------
    
    
    // the error term
    currErr_nav = targetTime - timeDiff; 
    
    // integral response 
    intErr_nav += currErr_nav;

    // differential
    diffErr_nav = currErr_nav - prevErr_nav;
    
    // scaling
    pTerm_nav = Kp_nav*currErr_nav; // proportional response
    iTerm_nav = Ki_nav*currErr_nav; // integral response
    dTerm_nav = Kd_nav*currErr_nav; // differential response
    
    prevErr_nav = currErr_nav; //save the current error
    
    return pTerm_nav + iTerm_nav + dTerm_nav;
}

void main()
{  
    //enable global interrupts
    CyGlobalIntEnable;
    
    //start the LCD screen
    LCD_Start();
    
    //---------------------- SPEED CONTROL -------------------------------
    
    // start magnet interrupt
    Magnet_Inter_Start();
    Magnet_Inter_SetVector(inter);
    
    // start PWM
    Speed_PWM_Start();
    Speed_PWM_WriteCompare(0);
    
    // start timer
    SpeedControl_Timer_Start();
    period = SpeedControl_Timer_ReadPeriod();
    
    LCD_Position(1,0);
    LCD_PrintNumber(pwm);
    Speed_PWM_WriteCompare(pwm);
    
    //---------------------- NAVIGATION ----------------------------------
    /* read camera_line_pin - every modLine lines
       read a line, start timer & comparator
       comparator interrupts at black line - stop timer
       then take the time diff */
    
    //start nav timer
    Nav_Timer_Start();
        
    // start DAC
    Comp_Ref_DAC_Start();
    
    // start comparator (DAC reference)
    //Cam_Comparator_Start();
    
    //start interrupts for frame, line, and burst
    Cam_Line_Inter_Start();
    Cam_Line_Inter_SetVector(line_inter);
    Cam_Frame_Inter_Start();
    Cam_Frame_Inter_SetVector(frame_inter);
    Cam_Burst_Inter_Start();
    Cam_Burst_Inter_SetVector(burst_inter);
    
    // start camera interrupt (when black line detected)
    Cam_Inter_Start();
    Cam_Inter_SetVector(cam_inter);
    
    // start steering_PWM
    Steering_PWM_Start();
    Steering_PWM_WriteCompare(152);
    
    
    for(;;)
    {
        
    }
}

/* [] END OF FILE */