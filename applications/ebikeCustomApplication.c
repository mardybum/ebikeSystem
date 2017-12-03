/*
	Copyright 2017 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout
#include "commands.h"

//Change between different bike setups (e.g. crank sensor type)
#define mode1
//#define mode2

static THD_FUNCTION(example_thread, arg);
static THD_WORKING_AREA(example_thread_wa, 2048); // 2kb stack for this thread

//Timers to define the time between two high flanks at each sensor
static systime_t timerSpeedSensor;
static systime_t timerCrankSensor;

//Timers to store the start of a high flank
static systime_t highStart;

//Triggers for Semaphore
static int highTriggerStart;
static int highTriggerEnd;

//Variable to store the time difference
static float differenceHigh;

//Timer for a low flank
static systime_t lowStart;

//Semaphores for low flank
static int lowTriggerStart;
static int lowTriggerEnd;

//Difference between two low flanks
static float differenceLow;

//Are the pedals going in the righ direction?
static bool pedalingForward;

//Treshold in milli seconds for non rotation of the crank (Remember that for some reson 3000 is 0.3 seconds)
static const float noRotatingTreshold = 3000.0;

//timer in milli seconds for not pedaling
static const float noPedalingTreshold = 8000.0;

//Number of magnets that have to be triggered at the crank to set isPedaling true
static const int tresholdMagnet = 1;

//timer ifor not pedaling
static const float tresholdTimePedaling = 15000.0;

//Min speed to start the engine
static const float minSpeed = 6.0;

//TImers for on and off time pedaling
static systime_t firstPedalStroke;
static systime_t timeOnPedaling;
static systime_t timeOffPedaling;
static systime_t timeStartPedaling;

//Timer for the duration of a different current that will be applied (Startup routine)
static const float startUpRoutineEnd = 1000;

//Semaphore for the startup
static int startUpTrigger = 0;

//for the state transition => in what state are we?
static bool startUpRoutine = true;

//Determination if the pedals are turining as desired
static bool isPedaling = false;

//Number of pedal counts
static int pedalCount = 0;

//Velocity of the bike
static float vehicleVelocity;

void settingMotorCurrent(void) {
    // Set the UART TX pin as an input with pulldown
    palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLDOWN);
    
    // Start the example thread
    chThdCreateStatic(example_thread_wa, sizeof(example_thread_wa),
                      NORMALPRIO, example_thread, NULL);
}

static THD_FUNCTION(example_thread, arg) {
    (void)arg;
    
    //TODO How can this be changed?
    chRegSetThreadName("APP_EXAMPLE");
    
    //Get the current Motor Config test
    const volatile mc_configuration *mcconf = mc_interface_get_configuration();
    
    //current for the motor
    float setPointCurrent;
    
    //RPM of the speed sensor at the wheel
    float speedSensorWheel = 0.0;
    
    //Definition of hall sensors
    int hallSensorBack;
    int hallSensorCrank;
    
    //Set initial value for old timer
    timerSpeedSensor = 0;
    
    
    //Semaphores for both speed sensors
    int semaphoreWheelSensor = 1;
    int semaphoreCrankSensor = 1;
    
    float timeDiff;
    
    //Endless looop
    for(;;) {
        // Read the pot value and scale it to a number between 0 and 1 (see hw_46.h)
        
        //Read value of the poti
        float pot = (float)ADC_Value[ADC_IND_EXT];
        pot /= 4095.0;
        
        
        //Read in Hall Sensor state
        hallSensorBack = READ_HALL2();
        hallSensorCrank = READ_HALL1();
        
        //Determine rotational speed if the hall sensor is triggered
        if (hallSensorBack == 1 && semaphoreWheelSensor == 1) {
            
            speedSensorWheel = (1.0 / ((float)ST2MS(chVTTimeElapsedSinceX(timerSpeedSensor))/1000.0)) * 60.0;
            
            //Calculate velocity based on 28Zoll tires (km/h)
            vehicleVelocity = (3.14*0.0007112 * speedSensorWheel*60.0);
            
            semaphoreWheelSensor = 0;
            timerSpeedSensor = chVTGetSystemTimeX();
            
        } else if (hallSensorBack == 0 && semaphoreWheelSensor == 0)  {
            
            semaphoreWheelSensor = 1;
        } else if (hallSensorBack == 0 && (float)ST2MS(chVTTimeElapsedSinceX(timerSpeedSensor)) > noRotatingTreshold) {
            
            speedSensorWheel = 0.0;
            vehicleVelocity = 0.0;
        }
        
        
        //Determine rotational speed if the hall sensor is triggered
        if (hallSensorCrank == 1 && semaphoreCrankSensor == 1) {
            
            if (pedalCount == 0) {
                
                firstPedalStroke = chVTGetSystemTimeX();
            }

            //Check if pedaling
            if(isPedaling == false) {
                
                if(pedalCount >= tresholdMagnet && (float)ST2MS(chVTTimeElapsedSinceX(firstPedalStroke)) < tresholdTimePedaling) {
                    
                    //Check if we are pedaling in the right direction and if we have a minimumn speed
                    if (pedalingForward == true && vehicleVelocity > minSpeed) {
                        
                        isPedaling = true;
                    } else {
                        
                        isPedaling = false;
                    }
                //Check if the amount of magnets specified have been passed and if the treshold has been exceeded
                } else if (pedalCount >= tresholdMagnet && (float)ST2MS(chVTTimeElapsedSinceX(firstPedalStroke)) > tresholdTimePedaling) {
                    
                    pedalCount = 0;
                    
                } else {
                    
                    pedalCount = pedalCount + 1;
                }
            }
            
            semaphoreCrankSensor = 0;
            timerCrankSensor = chVTGetSystemTimeX();
            
            
        } else if (hallSensorCrank == 0 && semaphoreCrankSensor == 0)  {
            
            semaphoreCrankSensor = 1;
        }
        
        
        //This routine is for the detection of the rotating direction of the crank
        if (hallSensorCrank == 1) {
            
            //Store a timer at the beginning and and the end of each flank
            if(lowTriggerEnd == 0) {
                
                lowTriggerEnd = 1;
                
                differenceLow = (float)ST2MS(chVTTimeElapsedSinceX(lowStart));
            }
            
            if(highTriggerStart == 0) {
                
                highStart = chVTGetSystemTimeX();
                highTriggerStart = 1;
            }
            
            lowTriggerStart = 0;
            highTriggerEnd = 0;
            
            timeOnPedaling = chVTGetSystemTimeX();
           
        } else if (hallSensorCrank == 0) {
            
            if(lowTriggerStart == 0) {
                
                lowStart = chVTGetSystemTimeX();
                lowTriggerStart = 1;
            }
            
            if(highTriggerEnd == 0) {
                
                highTriggerEnd = 1;
                
                differenceHigh = (float)ST2MS(chVTTimeElapsedSinceX(highStart));
            }

            lowTriggerEnd = 0;
            highTriggerStart = 0;
            
            timeOffPedaling = chVTGetSystemTimeX();
        }

        
        //TODO ABS()?
        if(abs(differenceLow > differenceHigh)) {
            
            pedalingForward = true;
        } else {
            
            pedalingForward = false;
        }
        
        
        //Check the timers of the high and low flank and dtermine in which direction we are pedaling
        //Check specificaiton of the hall sensors for the direction of travel
        if (timeOnPedaling >= timeOffPedaling) {
            
            timeDiff = (float) (timeOnPedaling - timeOffPedaling);
        } else {
            timeDiff = (float) (timeOffPedaling - timeOnPedaling);
        }
        
        
        //If not a sufficient amount of sensors at the crank have been passed, then stop the engine
        if (isPedaling == true && timeDiff > noPedalingTreshold) {
            
            isPedaling = false;
            pedalCount = 0;
        }
        
        if (pedalingForward == false) {
            
            isPedaling = false;
        }
        
        //If we are pedaling, then first a startup roune is activated in order to limit the motor current (Currently 4 amps)
        if(isPedaling == true) {
            
            if(startUpRoutine == true) {
                
                if(startUpTrigger == 0) {
                    timeStartPedaling = chVTGetSystemTimeX();
                    startUpTrigger = 1;
                }
                
                if((float)ST2MS(chVTTimeElapsedSinceX(timeStartPedaling)) > startUpRoutineEnd) {
                    
                    startUpRoutine = false;
                }
                
                setPointCurrent = (float) 4.0;
                mc_interface_set_current(setPointCurrent);
                
            } else {
                
                //At least 10 Amps and additionaly the amoount of the max motor current times poti value
                setPointCurrent = ((float) mcconf->lo_current_motor_max_now * pot) + 10.0;
                mc_interface_set_current(setPointCurrent);
            }
            
        } else {
            
            //If we are not pedaling then the motor is released
            mc_interface_release_motor();
            startUpTrigger = 0;
            startUpRoutine = true;
        }
        
        commands_printf("isPedaling: %f \n", poti);
        
        // Run this loop at 500Hz
        chThdSleepMilliseconds(2);
        
        // Reset the timeout
        timeout_reset();
    }
}


