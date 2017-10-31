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


// Example thread
static THD_FUNCTION(example_thread, arg);
static THD_WORKING_AREA(example_thread_wa, 2048); // 2kb stack for this thread

static systime_t lastTimeSensor01;
static systime_t lastTimeSensor02;

static float vehicleVelocity;

void app_example_init(void) {
    // Set the UART TX pin as an input with pulldown
    palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLDOWN);
    
    // Start the example thread
    chThdCreateStatic(example_thread_wa, sizeof(example_thread_wa),
                      NORMALPRIO, example_thread, NULL);
    
    commands_printf("Init INIT INIT \n");
}

static THD_FUNCTION(example_thread, arg) {
    (void)arg;
    
    chRegSetThreadName("APP_EXAMPLE");
    
    //Get the current Motor Config
    const volatile mc_configuration *mcconf = mc_interface_get_configuration();
    float setPointCurrent;
    float rotationalSpeedSensor01 = 0.0;
    float rotationalSpeedSensor02 = 0.0;
    
    int hallSensorState01;
    int hallSensorState02;
    int hallSensorState03;
    
    //Set initial value for old timer
    lastTimeSensor01 = 0;
    
    int hallTrigger01 = 1;
    int hallTrigger02 = 1;
    
    
    for(;;) {
        // Read the pot value and scale it to a number between 0 and 1 (see hw_46.h)
        
        float pot = (float)ADC_Value[ADC_IND_EXT];
        pot /= 4095.0;
        
        
        //Read in Hall Sensor state
        hallSensorState01 = READ_HALL1();
        hallSensorState02 = READ_HALL2();
        hallSensorState03 = READ_HALL3();
        
        
        //Determine rotational speed if the hall sensor is triggered
        if (hallSensorState01 == 0 && hallTrigger01 == 1) {
            
            rotationalSpeedSensor01 = (1.0 / ((float)ST2MS(chVTTimeElapsedSinceX(lastTimeSensor01))/1000.0)) * 60.0;
            
            hallTrigger01 = 0;
            lastTimeSensor01 = chVTGetSystemTimeX();
            
            
        } else if (hallSensorState01 == 1 && hallTrigger01 == 0)  {
            
            hallTrigger01 = 1;
        }
        
        //Determine rotational speed if the hall sensor is triggered
        if (hallSensorState02 == 0 && hallTrigger02 == 1) {
            
            rotationalSpeedSensor02 = (1.0 / ((float)ST2MS(chVTTimeElapsedSinceX(lastTimeSensor02))/1000.0)) * 60.0;
            
            hallTrigger02 = 0;
            lastTimeSensor02 = chVTGetSystemTimeX();
            
            
        } else if (hallSensorState02 == 1 && hallTrigger02 == 0)  {
            
            hallTrigger02 = 1;
        }
        
        //Calculate velocity based on 28Zoll tires (km/h)
        vehicleVelocity = (0.7112 * rotationalSpeedSensor01 / 1000.0) / (1/60.0);
        
        
        commands_printf("velocity : %f \n", vehicleVelocity);
        //commands_printf("rotation Speed Sensor 1: %f \n", rotationalSpeedSensor01);
        //commands_printf("HallSensor State: %d \n", hallSensorState01);
        //commands_printf("HallSensor State: %d \n", hallSensorState02);
        //commands_printf("HallSensor State: %d \n", hallSensorState03);
        //commands_printf("lastTimeSensor01: %f \n", (float) (lastTimeSensor01));
        //commands_printf("-------------------------\n");
        
        //If the poti is off, then the motor shall be released
        if (pot > 0.01) {

            setPointCurrent = (float) mcconf->lo_current_motor_max_now * pot;
            mc_interface_set_current(setPointCurrent);
            
        } else {
            // If the button is not pressed, release the motor
            mc_interface_release_motor();
        }
        
        // Run this loop at 500Hz
        chThdSleepMilliseconds(2);
        
        // Reset the timeout
        timeout_reset();
    }
}


