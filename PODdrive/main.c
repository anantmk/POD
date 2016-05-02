#include <stdio.h>
#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>
#include "PODdrive.h"
#include "i2clcd.h"
#include "i2cmaster.h"
#include <util/atomic.h>		// need "--std=c99"
//#include "lcd_menu.h"

//Counts overflovs
volatile uint32_t   T1Overflow = 0;
//Variables holding two timestamps
volatile uint32_t   Capt1 = 0;
volatile uint32_t   Capt2 = 0;
volatile uint32_t   tFrame = 0;
//capture Flag
volatile uint8_t    State = 0;
volatile uint8_t    PPM_Frame = 0;
volatile uint8_t    PPM_Signal = 0;
volatile uint8_t    Flag = 0;
volatile uint32_t   PPM_Channel[9];
volatile uint8_t    PPM_Ch = 0;
volatile uint8_t    PPM_Read = 0;

uint16_t podVelocity = 0;
uint16_t podDirection = 0;
uint16_t maxHyp = 0;
uint8_t linked = 0;
float trange = 0.0;
unsigned char calcDrives = 0;

char sTestPPM[5];

unsigned char PODConfig = 0;


//FCPU?

//I2C
unsigned char POD_LCD = 1;
unsigned char i2c_r_tmp;

volatile uint8_t key_state;                                // debounced and inverted key state:
// bit = 1: key pressed
volatile uint8_t key_press;                                // key press detect

volatile uint8_t key_rpt;                                  // key long press and repeat


//temp!!!
unsigned char sector = 0;
unsigned char percent1 = 0;
unsigned char percent2 = 0;
uint8_t servodrivecmd = 0;
unsigned char run_mode = 0;

//temporary data read out and stored in EEPROM
//POD_CHANNEL InChannel[8];
//POD_CHANNEL OutChannel[8];
//POD_MODE    POD_Config;
//POD_STEPPER POD_StepperConf[4];

//main routine
int main(void)
{
    // initialize I2C library
    i2c_init();
    
    //LCD connected
    POD_LCD = i2c_start(LCD_I2C_DEVICE);
    i2c_stop();
    
    //initalize LCD if connected
    lcd_init();
    
    _delay_ms(100);
    
    //*put all in and out configurations to an subroutine
    //POD_PWMCH_PORT all Pins as output
    //POD_PWMCH_DDR = 0xFF;
    
    DDRB |= (1 << PB6) | (1 << PB7);
    
    //LED_PORT &= ~((1 << LED0) |(1 << LED1)|(1 << LED2));
    //LED_DDR = 0xFF;
    LED_DDR |= (1 << LED0) | (1 << LED1) | (1 << LED2);
    
    //all LEDs on
    LED_PORT |= (1 << LED0) | (1 << LED1) | (1 << LED2);
    
    
    //set STEPPER port Pins as output
    POD_STEP_DDR = 0xFF;
    
    
    //set Hall sensor input
    POD_HALLSENS_DDR &= ~((1 << POD_HALLSENS_1) | (1 << POD_HALLSENS_2) | (1 << POD_HALLSENS_3) | (1 << POD_HALLSENS_4));
    //set pullups
    POD_HALLSENS_PORT |= (1 << POD_HALLSENS_1) | (1 << POD_HALLSENS_2) | (1 << POD_HALLSENS_3) | (1 << POD_HALLSENS_4);

    
    //LCD splah screen

    lcd_command(LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKINGOFF);
    lcd_light(true);
    lcd_printlc_P(1,1, PSTR("              |     "));
    lcd_printlc_P(2,1, PSTR("PODdrive     ====|  "));
    lcd_printlc_P(3,1, PSTR("                    "));
    lcd_printlc_P(4,1, PSTR("V0.5 alpha       amk"));
    
    _delay_ms(1000);
    
    //all LEDs off
    LED_PORT &= ~((1 << LED0) |(1 << LED1)|(1 << LED2));
    
    //LCD clear
    lcd_command(LCD_CLEAR);
    
    _delay_ms(500);
    
    //intitialize timers
    InitTimer1();
    StartTimer1();
    
    //initialize debouncing
    InitDebounce();
    
    //enable global interrutps
    sei();
    
    _delay_ms(200);
    
    //configure?
    if (get_key_press(1<<KEY_Enter))
    {
        //enter Comfig menu
        browse_menu();
        if (POD_LCD == 0) lcd_command(LCD_CLEAR);
        
        lcd_printlc_P(1,1, PSTR("RUN"));
    }
    
    //POD initialization
    POD_INIT();
    
    _delay_ms(2000);
    
    //endless loop
    while (1)
    {
        // map PPM to PWM
        if (PPM_Signal == 1)
        {
            //LED rot aus
            LED_PORT &= ~(1 << LED0);

            PPM_Read = 1;
            
            for (uint8_t tmp = 0; tmp < 9; tmp = tmp + 1)
            {
                CH_in[tmp] = PPM_Channel[tmp];
            }
            
            PPM_Read = 0;
        }
        else
        {
            //LED rot ein
            LED_PORT |= (1 << LED0);
        }
    
        //copy values from input to work
        for (uint8_t tmp = 0; tmp < 9; tmp++)
        {
            CH_work[tmp] = CH_in[tmp];
        }
        
        //PODS linked?
        if (POD_DriveConig.PODs_Linked == 1)
        {
            linked = 1;
            LED_PORT |= (1 << LED1);
        }
        else if (POD_DriveConig.PODs_Linked == 2)
        {
            if (CH_work[5] >= 35000)
            {
                linked = 1;
                LED_PORT |= (1 << LED1);
            }
            else
            {
                linked = 0;
                LED_PORT &= ~(1 << LED1);
            }
        }
        else
        {
            linked = 0;
            LED_PORT &= ~(1 << LED1);
        }
        
        //ToDo
        if (CH_work[6] >= 35000)
        {
            servodrivecmd = 111;
        }
        else
        {
            servodrivecmd = 0;
        }
        
        //DO
        if (POD_DriveConig.DO_Channel > 0)
        {
            if (CH_work[POD_DriveConig.DO_Channel] >= 35000)
            {
                run_mode = DOUT;
            }
            else
            {
                run_mode = RUN;
            }
        }
        
        switch (run_mode)
        {
            case RUN:
                
                //calculate drives
                POD_calc_Drive();
                
                //set Servo outputs
                POD_set_PWM_Channels();
                
                //write ServoDrive
                POD_set_ServoDrive();
                
                //to be deleted!!!
                //Drive Stpper
                //POD_drive_Stepper();

                break;
                
            case DOUT:
                
                POD_set_DO();
                
                _delay_ms(750);
                break;
                
            default:
                break;
        }

        _delay_ms(10);
        
    }

	return 0; // never reached
}

void POD_set_PWM_Channels (void)
{
    //channel1 POD1
    ServoDriveOUT[0] = 2500 + (PODStepper[0].Speed_Set * 25);
    
    //channel2 POD2
    if (POD_DriveConig.PODs_No > 1) ServoDriveOUT[1] = 2500 + (PODStepper[1].Speed_Set * 25);
    
    //channel3
    if (((POD_DriveConig.PODs_No == 1) || (POD_DriveConig.PODs_Linked == 1)) && (POD_DriveConig.DO_Channel != 3) && (POD_DriveConig.ControlMode != 3))
        ServoDriveOUT[2] = CH_in[3]/8;
    
    //channel4
    if (((POD_DriveConig.PODs_No == 1) || (POD_DriveConig.PODs_Linked == 1)) && (POD_DriveConig.DO_Channel != 4) && (POD_DriveConig.ControlMode != 4))
        ServoDriveOUT[3] = CH_in[4]/8;
    
    //channel5
    if ((POD_DriveConig.PODs_Linked != 2) && (POD_DriveConig.DO_Channel != 5) && (POD_DriveConig.ControlMode != 5))
        ServoDriveOUT[4] = CH_in[5]/8;
    
    //channel6
    if ((POD_DriveConig.DO_Channel != 6) && (POD_DriveConig.ControlMode != 6))
        ServoDriveOUT[5] = CH_in[6]/8;
    
    //channel7
    if ((POD_DriveConig.DO_Channel != 7) && (POD_DriveConig.ControlMode != 7))
        ServoDriveOUT[6] = CH_in[7]/8;
    
    //channel8
    if ((POD_DriveConig.DO_Channel != 8) && (POD_DriveConig.ControlMode != 8))
        ServoDriveOUT[7] = CH_in[8]/8;
}

void POD_set_DO (void)
{
    uint16_t setpoint = 0;
    
    setpoint = 90 * (POD_DriveConig.MaxSpeed / 100.0);
    
    if (POD_DriveConig.TurningRange > 0)
    {
        if (setpoint <= POD_DriveConig.TurningRange)
        {
            setpoint = 0;
        }
        else
        {
            trange = 100.0 / (100 - POD_DriveConig.TurningRange);
            
            setpoint = (setpoint - POD_DriveConig.TurningRange) * trange;
        }
    }
    
    POD_clac_Sticks(1);
    
    //OUT1
    if ((podVelocity > setpoint) && ((podDirection > 340) || (podDirection < 10)))
    {
        PCF1_OUTB ^= (1 << PIN0);
    }
    
    //OUT2
    if ((podVelocity > setpoint) && (podDirection > 15) && (podDirection < 65))
    {
        PCF1_OUTB ^= (1 << PIN1);
    }
    
    //OUT3
    if ((podVelocity > setpoint) && (podDirection > 70) && (podDirection < 110))
    {
        PCF1_OUTB ^= (1 << PIN2);
    }
    
    //OUT4
    if ((podVelocity > setpoint) && (podDirection > 115) && (podDirection < 155))
    {
        PCF1_OUTB ^= (1 << PIN3);
    }
    
    //OUT5
    if ((podVelocity > setpoint) && (podDirection > 160) && (podDirection < 200))
    {
        PCF1_OUTB ^= (1 << PIN4);
    }
    
    //OUT6
    if ((podVelocity > setpoint) && (podDirection > 205) && (podDirection < 245))
    {
        PCF1_OUTB ^= (1 << PIN5);
    }
    
    //OUT7
    if ((podVelocity > setpoint) && (podDirection > 250) && (podDirection < 290))
    {
        PCF1_OUTB ^= (1 << PIN6);
    }
    
    //OUT8
    if ((podVelocity > setpoint) && (podDirection > 295) && (podDirection < 335))
    {
        PCF1_OUTB ^= (1 << PIN7);
    }
    
    itoa(podDirection, sTestPPM, 10);
    lcd_printlc(1, 1, sTestPPM);
    itoa(podVelocity, sTestPPM, 10);
    lcd_printlc(2, 1, sTestPPM);
    
    
    POD_clac_Sticks(2);
    
    //OUT1
    if ((podVelocity > setpoint) && ((podDirection > 340) || (podDirection < 10)))
    {
        PCF2_OUTB ^= (1 << PIN0);
    }
    
    //OUT2
    if ((podVelocity > setpoint) && (podDirection > 15) && (podDirection < 65))
    {
        PCF2_OUTB ^= (1 << PIN1);
    }
    
    //OUT3
    if ((podVelocity > setpoint) && (podDirection > 70) && (podDirection < 110))
    {
        PCF2_OUTB ^= (1 << PIN2);
    }
    
    //OUT4
    if ((podVelocity > setpoint) && (podDirection > 115) && (podDirection < 155))
    {
        PCF2_OUTB ^= (1 << PIN3);
    }
    
    //OUT5
    if ((podVelocity > setpoint) && (podDirection > 160) && (podDirection < 200))
    {
        PCF2_OUTB ^= (1 << PIN4);
    }
    
    //OUT6
    if ((podVelocity > setpoint) && (podDirection > 205) && (podDirection < 245))
    {
        PCF2_OUTB ^= (1 << PIN5);
    }
    
    //OUT7
    if ((podVelocity > setpoint) && (podDirection > 250) && (podDirection < 290))
    {
        PCF2_OUTB ^= (1 << PIN6);
    }
    
    //OUT8
    if ((podVelocity > setpoint) && (podDirection > 295) && (podDirection < 335))
    {
        PCF2_OUTB ^= (1 << PIN7);
    }
    
    itoa(podDirection, sTestPPM, 10);
    lcd_printlc(1, 10, sTestPPM);
    itoa(podVelocity, sTestPPM, 10);
    lcd_printlc(2, 10, sTestPPM);
    
    POD_write_DO();
}

void POD_write_DO (void)
{
    i2c_start_wait(PCF1+I2C_WRITE);
    i2c_write(0);
    i2c_write(PCF1_OUTB);
    i2c_stop();
    
    i2c_start_wait(PCF2+I2C_WRITE);
    i2c_write(0);
    i2c_write(PCF2_OUTB);
    i2c_stop();
}

void POD_set_ServoDrive(void)
{
    i2c_start_wait(SERVODRIVE+I2C_WRITE);
    i2c_write(0);
    i2c_write(servodrivecmd);
    for (uint8_t tmp = 0; tmp <= 7; tmp++)
    {
        i2c_write(LOW_BYTE(ServoDriveOUT[tmp]));
        i2c_write(HIGH_BYTE(ServoDriveOUT[tmp]));
    }
    
    i2c_stop();
}

void POD_Dirve_Stepper_PWM (void)
{
    //POD1
    if (POD_DriveConig.PODs_No >= 1)
    {
        if (PODStepper[0].podStepRun != 0)
        {
            //set direction
            if (PODStepper[0].podStepDir == CW)
            {
                POD_STEP_PORT |= (1 << 0);
            }
            else if (PODStepper[0].podStepDir == CCW)
            {
                POD_STEP_PORT &= ~(1 << 0);
            }
            
            POD_STEP_PORT &= ~(1 << 4);
            
            //decrease steps to to
            PODStepper[0].podStepRun--;
            
            if (PODStepper[0].podStepDir == CW)
            {
                PODStepper[0].Steps_Is++;
                
                if (PODStepper[0].Steps_Is == PODStepper[0].StepsFullCircle)
                {
                    PODStepper[0].Steps_Is = 0;
                }
            }
            else
            {
                if (PODStepper[0].Steps_Is == 0)
                {
                    PODStepper[0].Steps_Is = PODStepper[0].StepsFullCircle;
                }
                
                PODStepper[0].Steps_Is--;
            }
        }
    }
    
    //POD2
    if (POD_DriveConig.PODs_No >= 2)
    {
        if (PODStepper[1].podStepRun != 0)
        {
            //set direction
            if (PODStepper[1].podStepDir == CW)
            {
                POD_STEP_PORT |= (1 << 1);
            }
            else if (PODStepper[1].podStepDir == CCW)
            {
                POD_STEP_PORT &= ~(1 << 1);
            }
            
            //POD1
            POD_STEP_PORT &= ~(1 << 5);
            
            //decrease steps to to
            PODStepper[1].podStepRun--;
            
            if (PODStepper[1].podStepDir == CW)
            {
                PODStepper[1].Steps_Is++;
                
                if (PODStepper[1].Steps_Is == PODStepper[1].StepsFullCircle)
                {
                    PODStepper[1].Steps_Is = 0;
                }
            }
            else
            {
                if (PODStepper[1].Steps_Is == 0)
                {
                    PODStepper[1].Steps_Is = PODStepper[1].StepsFullCircle;
                }
                
                PODStepper[1].Steps_Is--;
            }
        }
    }
    
    //POD3
    if (POD_DriveConig.PODs_No >= 3)
    {
        if (PODStepper[2].podStepRun != 0)
        {
            //set direction
            if (PODStepper[2].podStepDir == CW)
            {
                POD_STEP_PORT |= (1 << 2);
            }
            else if (PODStepper[2].podStepDir == CCW)
            {
                POD_STEP_PORT &= ~(1 << 2);
            }
            
            //POD1
            POD_STEP_PORT &= ~(1 << 6);
            
            //decrease steps to to
            PODStepper[2].podStepRun--;
            
            if (PODStepper[2].podStepDir == CW)
            {
                PODStepper[2].Steps_Is++;
                
                if (PODStepper[2].Steps_Is == PODStepper[2].StepsFullCircle)
                {
                    PODStepper[2].Steps_Is = 0;
                }
            }
            else
            {
                if (PODStepper[2].Steps_Is == 0)
                {
                    PODStepper[2].Steps_Is = PODStepper[2].StepsFullCircle;
                }
                
                PODStepper[2].Steps_Is--;
            }
        }
    }
    
    //POD4
    if (POD_DriveConig.PODs_No == 4)
    {
        if (PODStepper[3].podStepRun != 0)
        {
            //set direction
            if (PODStepper[3].podStepDir == CW)
            {
                POD_STEP_PORT |= (1 << 3);
            }
            else if (PODStepper[3].podStepDir == CCW)
            {
                POD_STEP_PORT &= ~(1 << 3);
            }
            
            //POD1
            POD_STEP_PORT &= ~(1 << 7);
            
            //decrease steps to to
            PODStepper[3].podStepRun--;
            
            if (PODStepper[3].podStepDir == CW)
            {
                PODStepper[3].Steps_Is++;
                
                if (PODStepper[3].Steps_Is == PODStepper[3].StepsFullCircle)
                {
                    PODStepper[3].Steps_Is = 0;
                }
            }
            else
            {
                if (PODStepper[3].Steps_Is == 0)
                {
                    PODStepper[3].Steps_Is = PODStepper[3].StepsFullCircle;
                }
                
                PODStepper[3].Steps_Is--;
            }
        }
    }
}

void POD_drive_Stepper(void)
{
    uint16_t StepDuration = 0;
    
    //maxiumum steps
    for (uint8_t tmp = 0; tmp < POD_DriveConig.PODs_No; tmp++)
    {
        if (StepDuration < PODStepper[tmp].podStepRun) StepDuration = PODStepper[tmp].podStepRun;
    }
    
    //loop to drive PODs
    while (StepDuration != 0)
    {
        //POD1
        if (POD_DriveConig.PODs_No >= 1)
        {
            if (PODStepper[0].podStepRun != 0)
            {
                POD_Step(0,PODStepper[0].podStepDir);
                PODStepper[0].podStepRun--;
            }
        }
        
        //POD2
        if (POD_DriveConig.PODs_No >= 2)
        {
            if (PODStepper[1].podStepRun != 0)
            {
                POD_Step(1,PODStepper[1].podStepDir);
                PODStepper[1].podStepRun--;
            }
        }
        
        //POD3
        if (POD_DriveConig.PODs_No >= 3)
        {
            if (PODStepper[2].podStepRun != 0)
            {
                POD_Step(2,PODStepper[2].podStepDir);
                PODStepper[2].podStepRun--;
            }
        }
        
        //POD4
        if (POD_DriveConig.PODs_No == 4)
        {
            if (PODStepper[3].podStepRun != 0)
            {
                POD_Step(3,PODStepper[3].podStepDir);
                PODStepper[3].podStepRun--;
            }
        }
        
        StepDuration--;
        
//        while (POD_DriveConig.RotSpeed--)
//        {
//            _delay_us(1);
//        }
    }
}

void POD_calc_Drive(void)
{
    calcDrives = 1;
    
    if ((POD_DriveConig.PODs_No == 1) || (linked == 1))
    {
        POD_clac_Sticks(1);
        
        //set Speed and direction for all PODs
        //calculate all PODs
        for (uint8_t tmp = 0; tmp < POD_DriveConig.PODs_No; tmp++)
        {
            PODStepper[tmp].Speed_Set = podVelocity;
            PODStepper[tmp].podStepDir = podDirection;
            POD_calc_Stepper(tmp);
        }
    }
    else if ((POD_DriveConig.PODs_No > 1) && (linked == 0))
    {
        POD_clac_Sticks(1);
        
        //set Speed and direction for POD1
        //calculate POD1
        PODStepper[0].Speed_Set = podVelocity;
        PODStepper[0].podStepDir = podDirection;
        POD_calc_Stepper(0);
        
        //if set
        //set Speed and direction for POD3
        //calculate POD3
        if (POD_DriveConig.PODs_No >= 3)
        {
            PODStepper[2].Speed_Set = podVelocity;
            PODStepper[2].podStepDir = podDirection;
            POD_calc_Stepper(2);
        }
        
        POD_clac_Sticks(2);

        //set Speed and direction for POD2
        //calculate POD2
        PODStepper[1].Speed_Set = podVelocity;
        PODStepper[1].podStepDir = podDirection;
        POD_calc_Stepper(1);
        
        //if set
        //set Speed and direction for POD4
        //calculate POD4
        if (POD_DriveConig.PODs_No == 4)
        {
            PODStepper[3].Speed_Set = podVelocity;
            PODStepper[3].podStepDir = podDirection;
            POD_calc_Stepper(3);
        }
    }
    
    calcDrives = 0;
}

void POD_calc_Stepper(uint8_t PODno)
{
    // direction in steps
    PODStepper[PODno].Steps_Set = PODStepper[PODno].podStepDir / PODStepper[PODno].StepAngle;
    
    // calculation of number of steps
    PODStepper[PODno].podStepRun = PODStepper[PODno].Steps_Set - PODStepper[PODno].Steps_Is;
    
    // calculation of stepper dirction
    if (PODStepper[PODno].podStepRun > (PODStepper[PODno].StepsFullCircle / 2))
    {
        PODStepper[PODno].podStepDir = CCW;
        
        PODStepper[PODno].podStepRun = PODStepper[PODno].StepsFullCircle - PODStepper[PODno].podStepRun;
    }
    else
    {
        if (PODStepper[PODno].podStepRun >= 0)
        {
            PODStepper[PODno].podStepDir = CW;
        }
        else
        {
            PODStepper[PODno].podStepRun = abs(PODStepper[PODno].podStepRun);
            
            if (PODStepper[PODno].podStepRun > (PODStepper[PODno].StepsFullCircle / 2))
            {
                PODStepper[PODno].podStepDir = CW;
                
                PODStepper[PODno].podStepRun = PODStepper[PODno].StepsFullCircle - PODStepper[PODno].podStepRun;
            }
            else
            {
                PODStepper[PODno].podStepDir = CCW;
            }
        }
    }
}

//calculating direction and speed
void POD_clac_Sticks(uint8_t StickNo)
{

    sector = 0;
    
    //POD_Drive_Set

    if (StickNo == 1)
    {
        //to big?
        if (CH_work[1] >= InChannel[CH1].max) CH_work[1] = InChannel[CH1].max;
        
        if (CH_work[1] <= InChannel[CH1].min) CH_work[1] = InChannel[CH1].min;
        
        //to big?
        if (CH_work[2] >= InChannel[CH2].max) CH_work[2] = InChannel[CH2].max;
        
        if (CH_work[2] <= InChannel[CH2].min) CH_work[2] = InChannel[CH2].min;

        //Channel 1
        if (CH_work[1] > InChannel[0].mid)
        {
            //sector
            sector++;
            
            //percent of channel
            CH_work[1] = CH_work[1] - InChannel[0].mid;
    
            percent1 = (unsigned char)((CH_work[1] * 100)/(InChannel[0].max - InChannel[0].mid));
        }
        else if (CH_work[1] < InChannel[0].mid)
        {
            //percent of channel
            CH_work[1] = InChannel[0].mid - CH_work[1];
            
            percent1 = (unsigned char)((CH_work[1] * 100)/(InChannel[0].mid - InChannel[0].min));
        }
        else if (CH_work[1] == InChannel[0].mid)
        {
            //sector
            sector++;
            
            percent1 = 0;
        }
        
        //Channel2
        
        if (CH_work[2] > InChannel[1].mid)
        {
            //sector
            sector = sector +10;
            
            //percent of channel
            CH_work[2] = CH_work[2] - InChannel[1].mid;
            
            percent2 = (unsigned char)((CH_work[2] * 100)/(InChannel[1].max - InChannel[1].mid));
        }
        else if (CH_work[2] < InChannel[1].mid)
        {
            //percent of channel
            CH_work[2] = InChannel[1].mid - CH_work[2];
            
            percent2 = (unsigned char)((CH_work[2] * 100)/(InChannel[1].mid - InChannel[1].min));
        }
        else if (CH_work[2] == InChannel[1].mid)
        {
            //sector
            sector = sector +10;
            
            percent2 = 0;
        }
    }
    else if (StickNo == 2)
    {
        //to big?
        if (CH_work[3] >= InChannel[CH3].max) CH_work[3] = InChannel[CH3].max;
        
        if (CH_work[3] <= InChannel[CH3].min) CH_work[3] = InChannel[CH3].min;
        
        //to big?
        if (CH_work[4] >= InChannel[CH4].max) CH_work[4] = InChannel[CH4].max;
        
        if (CH_work[4] <= InChannel[CH4].min) CH_work[4] = InChannel[CH4].min;
        
        //Channel 3
        if (CH_work[3] > InChannel[CH3].mid)
        {
            //sector
            sector++;
            
            //percent of channel
            CH_work[3] = CH_work[3] - InChannel[CH3].mid;
            
            percent1 = (unsigned char)((CH_work[3] * 100)/(InChannel[CH3].max - InChannel[CH3].mid));
        }
        else if (CH_work[3] < InChannel[CH3].mid)
        {
            //percent of channel
            CH_work[3] = InChannel[CH3].mid - CH_work[3];
            
            percent1 = (unsigned char)((CH_work[3] * 100)/(InChannel[CH3].mid - InChannel[CH3].min));
        }
        else if (CH_work[3] == InChannel[CH3].mid)
        {
            //sector
            sector++;
            
            percent1 = 0;
        }
        
        //Channel 4
        
        if (CH_work[4] > InChannel[CH4].mid)
        {
            //sector
            sector = sector +10;
            
            //percent of channel
            CH_work[4] = CH_work[4] - InChannel[CH4].mid;
            
            percent2 = (unsigned char)((CH_work[4] * 100)/(InChannel[CH4].max - InChannel[CH4].mid));
        }
        else if (CH_work[4] < InChannel[CH4].mid)
        {
            //percent of channel
            CH_work[4] = InChannel[CH4].mid - CH_work[4];
            
            percent2 = (unsigned char)((CH_work[4] * 100)/(InChannel[CH4].mid - InChannel[CH4].min));
        }
        else if (CH_work[4] == InChannel[CH4].mid)
        {
            //sector
            sector = sector +10;
            
            percent2 = 0;
        }
    }
    
    //get data out of table
    if (percent1>100) percent1 = 100;
    if (percent2>100) percent2 = 100;
    
    podVelocity = (uint8_t) pgm_read_word (&aVelocity[percent1][percent2]);
    podDirection = (uint8_t) pgm_read_word (&aDirection[percent1][percent2]);
    
    //calculate aMaxHyp
    if (POD_DriveConig.Sticktype == 0)
    {
        maxHyp = (uint8_t) pgm_read_word (&aMaxHyp[podDirection]);
        
        // calculation of absolute speed
        podVelocity = (podVelocity * 100) / maxHyp;
    }

    //for safety
    if (podVelocity >= 100) podVelocity = 100;
    
    //limiting configured speed
    podVelocity = podVelocity * (POD_DriveConig.MaxSpeed / 100.0);
    
    //calculation of absolute direction
    switch (sector)
    {
        case 0:
            //sector 3 (180-270°)
            podDirection = 270 - podDirection;
            break;
        case 1:
            //sector 4 (279-360°)
            podDirection = 270 + podDirection;
            break;
        case 10:
            //sector 2 (90-180°)
            podDirection = 90 + podDirection;
            break;
        case 11:
            //sector 1 (0-90°)
            podDirection = 90 - podDirection;
            break;
        default:
            break;
    }
    
    // there is no 360° only 0°
    if (podDirection == 360) podDirection = 0;
    
    // setting direction of = pos to 0°
    if ((percent1 <= 3) && (percent2 <= 3)) podDirection = 0;
    
    //turnradius
    if (POD_DriveConig.TurningRange > 0)
    {
        if (podVelocity <= POD_DriveConig.TurningRange)
        {
            podVelocity = 0;
        }
        else
        {
            trange = 100.0 / (100 - POD_DriveConig.TurningRange);
            
            podVelocity = (podVelocity - POD_DriveConig.TurningRange) * trange;
        }
    }
    
    
    //expo
}

void POD_INIT(void)
{
    char sValue2[4];
    
    lcd_command(LCD_CLEAR);
    
    lcd_printlc_P(1,1, PSTR("POD INIT"));
    
    //cli
    cli();
    
    //read data form eeprom
    
    //read Drive Config
    POD_DriveConig.PODs_No = eeprom_read_byte(&eePOD_Config.PODs_No);
    POD_DriveConig.PODs_Linked = eeprom_read_byte(&eePOD_Config.PODs_Linked);
    POD_DriveConig.MaxSpeed = eeprom_read_byte(&eePOD_Config.MaxSpeed);
    POD_DriveConig.RotSpeed = eeprom_read_word(&eePOD_Config.RotSpeed);
    POD_DriveConig.Sticktype = eeprom_read_byte(&eePOD_Config.Sticktype);
    POD_DriveConig.TurningRange = eeprom_read_byte(&eePOD_Config.TurningRange);
    POD_DriveConig.RotSpeed = eeprom_read_word(&eePOD_Config.RotSpeed);
    POD_DriveConig.DO_Channel = eeprom_read_byte(&eePOD_Config.DO_Channel);
    
    //to be deleted
    POD_DriveConig.ControlMode = 0;
    
    itoa(POD_DriveConig.PODs_No, sValue2, 10);
    lcd_printlc(2, 1, sValue2);
    
    itoa(eeprom_read_byte(&eeDummy), sValue2, 10);
    lcd_printlc(2, 15, sValue2);
    
    //read channel config
    for (uint8_t tmp = 0; tmp <= 7; tmp++)
    {
        InChannel[tmp].min = eeprom_read_dword(&eePPM_IN[tmp].min);
        InChannel[tmp].mid = eeprom_read_dword(&eePPM_IN[tmp].mid);
        InChannel[tmp].max = eeprom_read_dword(&eePPM_IN[tmp].max);
    }
    
    //read stepper config
    for (uint8_t tmp = 0; tmp <= (POD_DriveConig.PODs_No - 1); tmp++)
    {
        PODStepper[tmp].StepsTo0 = eeprom_read_word(&eePODStepper[tmp].StepsTo0);
        PODStepper[tmp].StepsFullCircle = eeprom_read_word(&eePODStepper[tmp].StepsFullCircle);
        PODStepper[tmp].StepAngle = eeprom_read_float(&eePODStepper[tmp].StepAngle);
    }
    
    itoa(PODStepper[0].StepsFullCircle, sValue2, 10);
    lcd_printlc(3, 1, sValue2);
    itoa(PODStepper[0].StepsTo0, sValue2, 10);
    lcd_printlc(3, 10, sValue2);
    itoa(PODStepper[1].StepsFullCircle, sValue2, 10);
    lcd_printlc(4, 1, sValue2);
    itoa(PODStepper[1].StepsTo0, sValue2, 10);
    lcd_printlc(4, 10, sValue2);

    //sei
    sei();
    
    //set Servo outputs
    servodrivecmd = 0;
    ServoDriveOUT[0] = 0;
    ServoDriveOUT[1] = 0;
    ServoDriveOUT[2] = 0;
    ServoDriveOUT[3] = 0;
    ServoDriveOUT[4] = 0;
    ServoDriveOUT[5] = 0;
    ServoDriveOUT[6] = 0;
    ServoDriveOUT[7] = 0;
    ServoDriveOUT[8] = 0;
    ServoDriveOUT[9] = 0;
    ServoDriveOUT[10] = 0;
    ServoDriveOUT[11] = 0;
    
    i2c_start_wait(SERVODRIVE+I2C_WRITE);
    i2c_write(0);
    i2c_write(servodrivecmd);
    for (uint8_t tmp = 0; tmp <= 11; tmp++)
    {
        i2c_write(LOW_BYTE(ServoDriveOUT[tmp]));
        i2c_write(HIGH_BYTE(ServoDriveOUT[tmp]));
    }
    i2c_stop();
    
    _delay_ms(100);
    
    servodrivecmd = 111;
    
    i2c_start_wait(SERVODRIVE+I2C_WRITE);
    i2c_write(0);
    i2c_write(servodrivecmd);
    i2c_stop();
    
    _delay_ms(100);

    // init stepper
    for (unsigned char temp = POD_DriveConig.PODs_No; temp>=1; temp--)
    {
        POD_StepperInit(temp);
    }
    
    InitTimer3();
    StartTimer3();
    
    lcd_command(LCD_CLEAR);
}


//Initialize timer1 (PPM)
void InitTimer1(void)
{
    //Set Initial Timer value
    TCNT1=0;
    //First capture on rising edge
    //TCCR1B|=(1<<ICES1);
    //First capture on falling edge
    TCCR1B &= ~(1<<ICES1);
    //Enable input capture and overflow interrupts
    TIMSK1|=(1<<ICIE1)|(1<<TOIE1);
}

void StartTimer1(void)
{
    //Start timer without prescalle
    TCCR1B|=(1<<CS10);
}
//Initialize timer3 (PWM)
void InitTimer3(void)
{
    //Set Initial Timer value
    TCCR3A |= 1<<WGM31;
    TCCR3B |= 1<<WGM32 | 1<<WGM33;
    TIMSK3  |= 1<<OCIE3A | 1<<TOIE3;

}

void StartTimer3(void)
{
    TCCR3B |= 1<<CS31;
    ICR3 = POD_DriveConig.RotSpeed;
    OCR3A = 200;
}

ISR(TIMER3_OVF_vect)
{
    PORTB |= (1<<PB6);
    
    //reset all stepper
    POD_STEP_PORT |= (1 << 4);
    POD_STEP_PORT |= (1 << 5);
    POD_STEP_PORT |= (1 << 6);
    POD_STEP_PORT |= (1 << 7);
}

//PWM compare ISR
ISR(TIMER3_COMPA_vect)
{
    
    //test
    PORTB &= ~(1<<PB6);
    
    if (calcDrives == 0)
    {
        POD_Dirve_Stepper_PWM();
    }
    
}

//PPM capture ISR
ISR(TIMER1_CAPT_vect)
{
    //get a edge
    switch (Flag)
    {
        case 0:
            //save captured timestamp
            Capt1 = ICR1;
            //reset overflows
            T1Overflow = 0;
            //increment Flag
            Flag = 1;
            break;
            
        case 1:
            //save captured timestamp
            Capt2 = ICR1;
            //reset Flag
            Flag = 2;
            //State for analysis
            State = 1;
            break;
            
        case 2:
            //save old timestamp
            Capt1 = Capt2;
            //save captured timestamp
            Capt2 = ICR1;
            //State for analysis
            State = 1;
            break;
            
        default:
            break;
    }

    if (State == 1)
    {
        //time calculation
        tFrame = (Capt2-Capt1+(T1Overflow*0x10000));
        T1Overflow = 0;
        PPM_Frame++;
        State = 0;
    }
    
    switch (PPM_Frame)
    {
        //syncframe
        case 1:
            
            if ((tFrame > 60000) && (PPM_Ch == 0))
            {
                if (PPM_Read == 0) PPM_Channel[PPM_Ch] = tFrame;
                PPM_Ch++;
            }
            else
            {
                PPM_Ch = 0;
                PPM_Frame = 0;
                PPM_Signal = 0;
            }
            break;
            
        //channel 1 frame
        case 2:
            
            if ((tFrame > 16000) && (tFrame < 44000) && (PPM_Ch == 1))
            {
                if (PPM_Read == 0) PPM_Channel[PPM_Ch] = tFrame;
                PPM_Ch++;
            }
            else
            {
                PPM_Ch = 0;
                PPM_Frame = 0;
                PPM_Signal = 0;
            }
            break;
            
        //channel 2 frame
        case 3:
            
            if ((tFrame > 16000) && (tFrame < 440000) && (PPM_Ch == 2))
            {
                if (PPM_Read == 0) PPM_Channel[PPM_Ch] = tFrame;
                PPM_Ch++;
            }
            else
            {
                PPM_Ch = 0;
                PPM_Frame = 0;
                PPM_Signal = 0;
            }
            break;
            
        //channel 3 frame
        case 4:
            
            if ((tFrame > 16000) && (tFrame < 44000) && (PPM_Ch == 3))
            {
                if (PPM_Read == 0) PPM_Channel[PPM_Ch] = tFrame;
                PPM_Ch++;
            }
            else
            {
                PPM_Ch = 0;
                PPM_Frame = 0;
                PPM_Signal = 0;
            }
            break;
        
        //channel 4 frame
        case 5:
            
            if ((tFrame > 16000) && (tFrame < 44000) && (PPM_Ch == 4))
            {
                if (PPM_Read == 0) PPM_Channel[PPM_Ch] = tFrame;
                PPM_Ch++;
            }
            else
            {
                PPM_Ch = 0;
                PPM_Frame = 0;
                PPM_Signal = 0;
            }
            break;
            
        //channel 5 frame
        case 6:
            
            if ((tFrame > 16000) && (tFrame < 44000) && (PPM_Ch == 5))
            {
                if (PPM_Read == 0) PPM_Channel[PPM_Ch] = tFrame;
                PPM_Ch++;
            }
            else
            {
                PPM_Ch = 0;
                PPM_Frame = 0;
                PPM_Signal = 0;
            }
            break;
            
        //channel 6 frame
        case 7:
            
            if ((tFrame > 16000) && (tFrame < 44000) && (PPM_Ch == 6))
            {
                if (PPM_Read == 0) PPM_Channel[PPM_Ch] = tFrame;
                PPM_Ch++;
            }
            else
            {
                PPM_Ch = 0;
                PPM_Frame = 0;
                PPM_Signal = 0;
            }
            break;
            
        //channel 7 frame
        case 8:
            
            if ((tFrame > 16000) && (tFrame < 44000) && (PPM_Ch == 7))
            {
                if (PPM_Read == 0) PPM_Channel[PPM_Ch] = tFrame;
                PPM_Ch++;
            }
            else
            {
                PPM_Ch = 0;
                PPM_Frame = 0;
                PPM_Signal = 0;
            }
            break;
            
        //channel 8 frame
        case 9:
            
            if ((tFrame > 16000) && (tFrame < 44000) && (PPM_Ch == 8))
            {
                if (PPM_Read == 0) PPM_Channel[PPM_Ch] = tFrame;
                PPM_Ch = 0;
                PPM_Frame = 0;
                PPM_Signal = 1;
            }
            else
            {
                PPM_Ch = 0;
                PPM_Frame = 0;
                PPM_Signal = 0;
            }
            break;
            
        default:
            break;
    }
}

//PPM: Overflow ISR
ISR(TIMER1_OVF_vect)
{
    //increment overflow counter
    T1Overflow++;
}



//init debouncing
void InitDebounce(void)
{
    // Configure debouncing routines
    KEY_DDR &= ~ALL_KEYS;                // configure key port for input
    KEY_PORT |= ALL_KEYS;                // and turn on pull up resistors
    
    TCCR2B = (1<<CS02)|(1<<CS00);         // divide by 1024
    TCNT2 = (uint8_t)(int16_t)-(F_CPU / 1024 * 10e-3 + 0.5);  // preload for 10ms
    TIMSK2 |= 1<<TOIE2;                   // enable timer interrupt
}

ISR( TIMER2_OVF_vect )                            // every 10ms
{
    static uint8_t ct0, ct1, rpt;
    uint8_t i;
    
    TCNT2 = (uint8_t)(int16_t)-(F_CPU / 1024 * 10e-3 + 0.5);  // preload for 10ms
    
    i = key_state ^ ~KEY_PIN;                       // key changed ?
    ct0 = ~( ct0 & i );                             // reset or count ct0
    ct1 = ct0 ^ (ct1 & i);                          // reset or count ct1
    i &= ct0 & ct1;                                 // count until roll over ?
    key_state ^= i;                                 // then toggle debounced state
    key_press |= key_state & i;                     // 0->1: key press detect
    
    if( (key_state & REPEAT_MASK) == 0 )            // check repeat function
        rpt = REPEAT_START;                          // start delay
    if( --rpt == 0 ){
        rpt = REPEAT_NEXT;                            // repeat delay
        key_rpt |= key_state & REPEAT_MASK;
    }
}

///////////////////////////////////////////////////////////////////
//
// check if a key has been pressed. Each pressed key is reported
// only once
//
uint8_t get_key_press( uint8_t key_mask )
{
    cli();                                          // read and clear atomic !
    key_mask &= key_press;                          // read key(s)
    key_press ^= key_mask;                          // clear key(s)
    sei();
    return key_mask;
}

///////////////////////////////////////////////////////////////////
//
// check if a key has been pressed long enough such that the
// key repeat functionality kicks in. After a small setup delay
// the key is reported being pressed in subsequent calls
// to this function. This simulates the user repeatedly
// pressing and releasing the key.
//
uint8_t get_key_rpt( uint8_t key_mask )
{
    cli();                                          // read and clear atomic !
    key_mask &= key_rpt;                            // read key(s)
    key_rpt ^= key_mask;                            // clear key(s)
    sei();
    return key_mask;
}

///////////////////////////////////////////////////////////////////
//
// check if a key is pressed right now
//
uint8_t get_key_state( uint8_t key_mask )

{
    key_mask &= key_state;
    return key_mask;
}

///////////////////////////////////////////////////////////////////
//
uint8_t get_key_short( uint8_t key_mask )
{
    cli();                                          // read key state and key press atomic !
    return get_key_press( ~key_state & key_mask );
}

///////////////////////////////////////////////////////////////////
//
uint8_t get_key_long( uint8_t key_mask )
{
    return get_key_press( get_key_rpt( key_mask ));
}


// ***********************************************************************
// Show menu function
// ***********************************************************************
void show_menu(void)
{	unsigned char line_cnt;					// Count up lines on LCD
    unsigned char from;
    unsigned char till = 0;
    
    unsigned char temp;						// Save "from" temporarily for always visible header and cursor position
    
    // Get beginning and end of current (sub)menu
    while (till <= selected)
    {	till += menu[till].num_menupoints;
    }
    from = till - menu[selected].num_menupoints;
    till--;
    
    temp = from;							// Save from for later use
    
    
    //--------------------------------------------------------------------
    // Always visible Header
    //--------------------------------------------------------------------
#if         defined USE_ALWAYS_VISIBLE_HEADER
    
    line_cnt = 2;							// Set line counter to one since first line is reserved for header
    
    // Write header
    //LCDWriteFromROM(0,0,menu[temp].text);
    lcd_printlc(1, 1, (char*)menu[temp].text);
    
    
    // Output for two row display becomes fairly easy
#if defined USE_TWO_ROW_DISPLAY
    
//    LCDWriteFromROM(0,UPPER_SPACE,menu[selected].text);
//    gotoxy(0,UPPER_SPACE);
//    LCDWriteChar(SELECTION_CHAR);
    
    lcd_printlc(UPPER_SPACE, 1, menu[selected].text);
    lcd_gotolc(UPPER_SPACE, 1);
    lcd_putchar(SELECTION_CHAR);
    
#endif	// USE_TWO_ROW_DISPLAY
    
    
    // Output for four row display
#if defined USE_FOUR_ROW_DISPLAY	||	defined		USE_THREE_ROW_DISPLAY
    
    // Output formatting for selection somewhere in between (sub)menu top and bottom
    if ( (selected >= (from + UPPER_SPACE)) && (selected <= (till - LOWER_SPACE)) )
    {	from = selected - (UPPER_SPACE-1);//-1
        till = from + (DISPLAY_ROWS - 1);//-2
        
        for (; from<=till; from++)
        {
            lcd_printlc(line_cnt, 2, (char*)menu[from].text);
            line_cnt++;
        }
        lcd_gotolc((UPPER_SPACE), 1);
        lcd_putchar(SPACE_CHAR);
        lcd_gotolc((UPPER_SPACE+1), 1);
        lcd_putchar(SELECTION_CHAR);
        lcd_gotolc((DISPLAY_ROWS), 1);
        lcd_putchar(SPACE_CHAR);
    }
    
    // Output formatting for selection close to (sub)menu top and bottom
    // (distance between selection and top/bottom defined as UPPER- and LOWER_SPACE)
    else
    {	// Top of (sub)menu
        if (selected < (from + UPPER_SPACE))
        {	from = selected;
            till = from + (DISPLAY_ROWS - 1);//-2
            
            for (; from<=till; from++)
            {
                lcd_printlc(line_cnt, 2, (char*)menu[from].text);
                line_cnt++;
            }
            
            lcd_gotolc((UPPER_SPACE), 1);//-1
            lcd_putchar(SELECTION_CHAR);
            lcd_gotolc((UPPER_SPACE+1), 1);
            lcd_putchar(SPACE_CHAR);
            lcd_gotolc((DISPLAY_ROWS), 1);
            lcd_putchar(SPACE_CHAR);
        }
        
        // Bottom of (sub)menu
        if (selected == till)
        {	from = till - (DISPLAY_ROWS - 2);//-2
            
            for (; from<=till; from++)
            {
                lcd_printlc(line_cnt, 2, (char*)menu[from].text);
                line_cnt++;
            }
            
            lcd_gotolc((UPPER_SPACE), 1);//-1
            lcd_putchar(SPACE_CHAR);
            lcd_gotolc((UPPER_SPACE+1), 1);
            lcd_putchar(SPACE_CHAR);
            lcd_gotolc((DISPLAY_ROWS), 1);//-1
            lcd_putchar(SELECTION_CHAR);
        }
    }
    
#endif	// USE_FOUR_ROW_DISPLAY
    
    
    //--------------------------------------------------------------------
    // Header not always visible
    //--------------------------------------------------------------------
#else	// !defined USE_ALWAYS_VISIBLE_HEADER
    
    line_cnt = 0;							// Set line counter to zero since all rows will be written
    
    // Output formatting for selection somewhere in between (sub)menu top and bottom
    if ( (selected >= (from + UPPER_SPACE)) && (selected <= (till - LOWER_SPACE)) )
    {	from = selected - UPPER_SPACE;
        till = from + (DISPLAY_ROWS);
        
        // 		for (from; from<=till; from++)
        // 		{	LCDWriteFromROM(0,line_cnt,menu[from].text);
        // 	 	 	line_cnt++;
        //		}
        

        for (from; from<=till; from++)
        {	lcd_printlc(line_cnt, 1, menu[from].text);
            line_cnt++;
        }
        
        
        //		gotoxy(0, UPPER_SPACE);
        //		LCDWriteChar(SELECTION_CHAR);
        
        lcd_gotolc(UPPER_SPACE, 1);
        lcd_putchar(SELECTION_CHAR);
        
    }
    
    // Output formatting for selection close to (sub)menu top and bottom
    // (distance between selection and top/bottom defined as UPPER- and LOWER_SPACE)
    else
    {	// Top of (sub)menu
        if (selected < (from + UPPER_SPACE))
        {	till = from + (DISPLAY_ROWS);
            
            //			for (from; from<=till; from++)
            // 			{	LCDWriteFromROM(0,line_cnt,menu[from].text);
            // 	 	 		line_cnt++;
            //			}
            
            for (from; from<=till; from++)
            {   lcd_printlc(line_cnt, 1, menu[from].text);
                line_cnt++;
            }
            
            //			gotoxy(0, (selected-temp));
            //			LCDWriteChar(SELECTION_CHAR);
            lcd_gotolc((selected-temp), 1);
            lcd_putchar(SELECTION_CHAR);
        }
        
        // Bottom of (sub)menu
        if (selected == till)
        {	from = till - (DISPLAY_ROWS);
            
            //			for (from; from<=till; from++)
            // 			{	LCDWriteFromROM(0,line_cnt,menu[from].text);
            // 	 	 		line_cnt++;
            //			}
            
            for (from; from<=till; from++)
            {
                lcd_printlc(line_cnt, 1, menu[from].text);
                line_cnt++;
            }
            
            //			gotoxy(0, (DISPLAY_ROWS - 1));
            //			LCDWriteChar(SELECTION_CHAR);
            lcd_gotolc((DISPLAY_ROWS), 1);
            lcd_putchar(SELECTION_CHAR);
            
            
        }
    }
    
#endif	// !defined USE_ALWAYS_VISIBLE_HEADER
    
}

// ***********************************************************************
// Browse menu function
// ***********************************************************************
void browse_menu(void)
{
    PODConfig = 1;
    
    do
    {	show_menu();
        
        if (LEFT_KEY)
        {selected = menu[selected].up;
        }
        
        if (RIGHT_KEY)
        {selected = menu[selected].down;
        }
        
        if (ENTER_KEY)
        {
            if (menu[selected].fp != 0)
                menu[selected].fp();
            
            selected = menu[selected].enter;
        }
        
        _delay_ms(25);
        
    }while(PODConfig);
}

void ENDConfig (void)
{
    eeprom_write_byte(&eeDummy, 77);
    eeprom_read_byte(&eeDummy);
    PODConfig = 0;
}

void DriveConfigPODNo (void)
{
    
    unsigned char tmp = 0;
    char sValue[3];
        
    lcd_command(LCD_CLEAR);
    
    lcd_printlc_P(1,1, PSTR("How many PODs are"));
    lcd_printlc_P(2,1, PSTR("connected?"));
    
    cli();
    
    tmp = eeprom_read_byte(&eePOD_Config.PODs_No);
    
    if ((tmp == 0) || tmp > 4)
    {
        tmp = 1;
    }
    
    lcd_printlc_P(3,1, PSTR("PODs No: "));
    
    while (!get_key_press(1<<KEY_Enter))
    {
        if (get_key_press(1<<KEY_Left)) tmp--;
        if (get_key_press(1<<KEY_Right)) tmp++;
        
        if (tmp > 4) tmp = 4;
        if (tmp < 1) tmp = 1;
        
        itoa(tmp, sValue, 10);
        lcd_printlc(3, 10, sValue);
        _delay_ms(100);
        
    }

    eeprom_write_byte(&eePOD_Config.PODs_No, tmp);

    tmp = eeprom_read_byte(&eePOD_Config.PODs_No);
    
    sei();
    
    itoa(tmp, sValue, 10);
    
    lcd_printlc(4, 10, sValue);
    
    lcd_printlc_P(4,2, PSTR("set to: "));
    
    _delay_ms(1000);
    
    for (; tmp>=1; tmp--)
    {
        PODStepperConfig (tmp);
    }
    
}

void DriveConfigPODlinked (void)
{
    cli();
    
    unsigned char tmp = 0;
    
    lcd_command(LCD_CLEAR);
    
    lcd_printlc_P(1,1, PSTR("PODs static linked?"));
    
    tmp = eeprom_read_byte(&eePOD_Config.PODs_Linked);
    
    if (tmp >= 2) tmp = 0;
    
    lcd_printlc_P(3,1, PSTR("linked: "));
    
    while (!get_key_press(1<<KEY_Enter))
    {
        if (get_key_press(1<<KEY_Left)) tmp--;
        if (get_key_press(1<<KEY_Right)) tmp++;
        
        if (tmp >= 2) tmp = 2;
        if (tmp <= 0) tmp = 0;
        
        if (tmp == 0) lcd_printlc_P(3,9, PSTR("NO      "));
        if (tmp == 1) lcd_printlc_P(3,9, PSTR("YES     "));
        if (tmp == 2) lcd_printlc_P(3,9, PSTR("SWITCHED"));
        _delay_ms(100);
        
    }
    
    eeprom_write_byte(&eePOD_Config.PODs_Linked, tmp);
    
    tmp = eeprom_read_byte(&eePOD_Config.PODs_Linked);
    
    lcd_printlc_P(4,1, PSTR("set to: "));
    
    if (tmp == 0) lcd_printlc_P(4,9, PSTR("NO      "));
    if (tmp == 1) lcd_printlc_P(4,9, PSTR("YES     "));
    if (tmp == 2) lcd_printlc_P(4,9, PSTR("SWITCHED"));
    
    sei();
    
    while (!get_key_press(1<<KEY_Enter));
    
}

void DriveConfigTurningRange (void)
{
    char sValue[3];
    
    unsigned char tmp = 0;
    
    cli();
    
    lcd_command(LCD_CLEAR);
    
    lcd_printlc_P(1,1, PSTR("Turning Range"));
    
    tmp = eeprom_read_byte(&eePOD_Config.TurningRange);
    
    lcd_printlc_P(3,2, PSTR("range: "));
    
    while (!get_key_press(1<<KEY_Enter))
    {
        if (get_key_press(1<<KEY_Left)) tmp = tmp - 1;
        if (get_key_press(1<<KEY_Right)) tmp = tmp + 1;
        
        if (tmp >= 30) tmp = 30;
        if (tmp <= 0) tmp = 0;
        
        itoa(tmp, sValue, 10);
        lcd_printlc(3, 10, sValue);
        lcd_print_P(PSTR(" %   "));

        _delay_ms(100);
    }
    
    eeprom_write_byte(&eePOD_Config.TurningRange, tmp);
    
    tmp = eeprom_read_byte(&eePOD_Config.TurningRange);
    
    lcd_printlc_P(4,1, PSTR("set to: "));
    
    itoa(tmp, sValue, 10);
    
    lcd_printlc(4, 10, sValue);
    
    lcd_print_P(PSTR(" %"));
    
    sei();
    
    while (!get_key_press(1<<KEY_Enter));
    
}

void DriveConfigEXPO (void)
{
    cli();
    
    unsigned char tmp = 0;
    char sValue[3];
    
    lcd_command(LCD_CLEAR);
    
    lcd_printlc_P(1,1, PSTR("Which EXPO?"));
    
    tmp = eeprom_read_byte(&eePOD_Config.Expo);
    
    if ((tmp == 0) || tmp > 4)
    {
        tmp = 1;
    }
    
    lcd_printlc_P(3,4, PSTR("EXPO: "));
    
    while (!get_key_press(1<<KEY_Enter))
    {
        if (get_key_press(1<<KEY_Left)) tmp--;
        if (get_key_press(1<<KEY_Right)) tmp++;
        
        if (tmp > 4) tmp = 4;
        if (tmp < 1) tmp = 1;
        
        itoa(tmp, sValue, 10);
        lcd_printlc(3, 10, sValue);
        _delay_ms(100);
        
    }
    
    eeprom_write_byte(&eePOD_Config.Expo, tmp);
    
    tmp = eeprom_read_byte(&eePOD_Config.Expo);
    
    itoa(tmp, sValue, 10);
    
    lcd_printlc(4, 10, sValue);
    
    lcd_printlc_P(4,2, PSTR("set to: "));
    
    sei();
    
    while (!get_key_press(1<<KEY_Enter));
    
}

void DriveConfigMaxSpeed (void)
{
    cli();
    
    unsigned char tmp = 0;
    char sValue[3];
    
    lcd_command(LCD_CLEAR);
    
    lcd_printlc_P(1,1, PSTR("Maximum Speed?"));
    
    tmp = eeprom_read_byte(&eePOD_Config.MaxSpeed);
    
    if ((tmp == 0) || tmp > 4)
    {
        tmp = 1;
    }
    
    lcd_printlc_P(3,1, PSTR("max speed: "));
    
    while (!get_key_press(1<<KEY_Enter))
    {
        if (get_key_press(1<<KEY_Left)) tmp = tmp - 5;
        if (get_key_press(1<<KEY_Right)) tmp = tmp + 5;
        
        if (tmp >= 100) tmp = 100;
        if (tmp <= 10) tmp = 10;
        
        itoa(tmp, sValue, 10);
        lcd_printlc(3, 12, sValue);
        lcd_print_P(PSTR(" %"));
        _delay_ms(100);
        
    }
    
    eeprom_write_byte(&eePOD_Config.MaxSpeed, tmp);
    
    tmp = eeprom_read_byte(&eePOD_Config.MaxSpeed);
    
    lcd_printlc_P(4,4, PSTR("set to: "));
    
    itoa(tmp, sValue, 10);
    
    lcd_printlc(4, 12, sValue);
    lcd_print_P(PSTR(" %"));
    
    sei();
    
    while (!get_key_press(1<<KEY_Enter));
    
}

void DriveConfigCtrlMode (void)
{
    cli();
    
    unsigned char tmp = 0;
    char sValue[3];
    
    lcd_command(LCD_CLEAR);
    
    lcd_printlc_P(1,1, PSTR("Control Mode"));
    
    tmp = eeprom_read_byte(&eePOD_Config.ControlMode);
    
    if ((tmp < 3) || tmp > 8)
    {
        tmp = 0;
    }
    
    lcd_printlc_P(3,1, PSTR("PPM Channel: "));
    
    while (!get_key_press(1<<KEY_Enter))
    {
        if (get_key_press(1<<KEY_Left)) tmp--;
        if (get_key_press(1<<KEY_Right)) tmp++;
        
        if (tmp > 8) tmp = 8;
        if (tmp <= 2) tmp = 2;
        
        if (tmp == 2)
        {
            lcd_printlc_P(3,14, PSTR("off"));
        }
        else
        {
            itoa(tmp, sValue, 10);
            lcd_printlc_P(3,15, PSTR("  "));
            lcd_printlc(3, 14, sValue);
        }
        _delay_ms(100);
        
    }
    
    if (tmp == 2)
    {
        eeprom_write_byte(&eePOD_Config.ControlMode, 0);
    }
    else
    {
        eeprom_write_byte(&eePOD_Config.ControlMode, tmp);
    }
    
    tmp = eeprom_read_byte(&eePOD_Config.ControlMode);
    
    lcd_printlc_P(4,6, PSTR("set to: "));
    
    if (tmp == 0)
    {
        lcd_printlc_P(4,14, PSTR("off"));
    }
    else
    {
        itoa(tmp, sValue, 10);
        lcd_printlc_P(4,15, PSTR("  "));
        lcd_printlc(4, 14, sValue);
    }
    
    sei();
    
    while (!get_key_press(1<<KEY_Enter));
    
}

void DriveConfigRotSpeed (void)
{
    cli();
    
    unsigned int tmp = 0;
    char sValue[4];
    
    lcd_command(LCD_CLEAR);
    
    lcd_printlc_P(1,1, PSTR("POD rotation speed?"));
    
    tmp = eeprom_read_word(&eePOD_Config.RotSpeed);
    
    if ((tmp < 2500) || tmp > 10000)
    {
        tmp = 2500;
    }
    
    lcd_printlc_P(3,1, PSTR("rotation speed: "));
    
    while (!get_key_press(1<<KEY_Enter))
    {
        if (get_key_press(1<<KEY_Left)) tmp = tmp - 3750;
        if (get_key_press(1<<KEY_Right)) tmp = tmp + 3750;
        
        if (tmp > 10000) tmp = 10000;
        if (tmp < 2500) tmp = 2500;
        
        if (tmp == 2500) lcd_printlc_P(3,17, PSTR("fast"));
        if (tmp == 6250) lcd_printlc_P(3,17, PSTR("medium"));
        if (tmp == 10000) lcd_printlc_P(3,17, PSTR("slow"));

        _delay_ms(100);
        
    }
    
    eeprom_write_word(&eePOD_Config.RotSpeed, tmp);
    
    tmp = eeprom_read_word(&eePOD_Config.RotSpeed);
    
    lcd_printlc_P(4,9, PSTR("set to: "));
    
    utoa(tmp, sValue, 10);
    
    lcd_printlc(4, 17, sValue);
    
    sei();
    
    while (!get_key_press(1<<KEY_Enter));
    
}

void DO_Channel (void)
{
    cli();
    
    unsigned char tmp = 0;
    char sValue[3];
    
    lcd_command(LCD_CLEAR);
    
    lcd_printlc_P(1,1, PSTR("CH to switch DO"));
    
    tmp = eeprom_read_byte(&eePOD_Config.DO_Channel);
    
    if ((tmp < 3) || tmp > 8)
    {
        tmp = 2;
    }
    
    lcd_printlc_P(3,1, PSTR("PWM Channel: "));
    
    while (!get_key_press(1<<KEY_Enter))
    {
        if (get_key_press(1<<KEY_Left)) tmp--;
        if (get_key_press(1<<KEY_Right)) tmp++;
        
        if (tmp > 8) tmp = 8;
        if (tmp <= 2) tmp = 2;
        
        if (tmp == 2)
        {
            lcd_printlc_P(3,14, PSTR("off"));
        }
        else
        {
            itoa(tmp, sValue, 10);
            lcd_printlc_P(3,15, PSTR("  "));
            lcd_printlc(3, 14, sValue);
        }
        _delay_ms(100);
        
    }
    
    if (tmp == 2)
    {
        eeprom_write_byte(&eePOD_Config.DO_Channel, 0);
    }
    else
    {
        eeprom_write_byte(&eePOD_Config.DO_Channel, tmp);
    }
    
    tmp = eeprom_read_byte(&eePOD_Config.DO_Channel);
    
    lcd_printlc_P(4,6, PSTR("set to: "));
    
    if (tmp == 0)
    {
        lcd_printlc_P(4,14, PSTR("off"));
    }
    else
    {
        itoa(tmp, sValue, 10);
        lcd_printlc_P(4,15, PSTR("  "));
        lcd_printlc(4, 14, sValue);
    }
    
    sei();
    
    while (!get_key_press(1<<KEY_Enter));
    
}

void DO_Status (void)
{
//    cli();
//    
//    unsigned char tmp = 0;
//    char sValue[3];
//    
//    lcd_command(LCD_CLEAR);
//    
//    lcd_printlc_P(1,1, PSTR("DO 9-16"));
//    
//    tmp = eeprom_read_byte(&eePOD_Config.DO_Byte2);
//    
//    if ((tmp < 3) || tmp > 8)
//    {
//        tmp = 0;
//    }
//    
//    lcd_printlc_P(3,1, PSTR("PWM Channel: "));
//    
//    while (!get_key_press(1<<KEY_Enter))
//    {
//        if (get_key_press(1<<KEY_Left)) tmp--;
//        if (get_key_press(1<<KEY_Right)) tmp++;
//        
//        if (tmp > 8) tmp = 8;
//        if (tmp <= 2) tmp = 2;
//        
//        if (tmp == 2)
//        {
//            lcd_printlc_P(3,14, PSTR("off"));
//        }
//        else
//        {
//            itoa(tmp, sValue, 10);
//            lcd_printlc_P(3,15, PSTR("  "));
//            lcd_printlc(3, 14, sValue);
//        }
//        _delay_ms(100);
//        
//    }
//    
//    if (tmp == 2)
//    {
//        eeprom_write_byte(&eePOD_Config.DO_Byte2, 0);
//    }
//    else
//    {
//        eeprom_write_byte(&eePOD_Config.DO_Byte2, tmp);
//    }
//    
//    tmp = eeprom_read_byte(&eePOD_Config.DO_Byte2);
//    
//    lcd_printlc_P(4,6, PSTR("set to: "));
//    
//    if (tmp == 0)
//    {
//        lcd_printlc_P(4,14, PSTR("off"));
//    }
//    else
//    {
//        itoa(tmp, sValue, 10);
//        lcd_printlc_P(4,15, PSTR("  "));
//        lcd_printlc(4, 14, sValue);
//    }
//    
//    sei();
    
    while (!get_key_press(1<<KEY_Enter));
    
}

void DOutStatusLED1 (void)
{
    cli();
    
    unsigned char tmp = 0;
    char sValue[3];
    
    lcd_command(LCD_CLEAR);
    
    lcd_printlc_P(1,1, PSTR("Status of LED1"));
    lcd_printlc_P(2,1, PSTR("on external Output?"));
    
    
    tmp = eeprom_read_byte(&eePOD_Config.StatusLED1);
    
    if ((tmp == 0) || tmp > 16)
    {
        tmp = 1;
    }
    
    lcd_printlc_P(3,2, PSTR("DO No: "));
    
    while (!get_key_press(1<<KEY_Enter))
    {
        if (get_key_press(1<<KEY_Left)) tmp--;
        if (get_key_press(1<<KEY_Right)) tmp++;
        
        if (tmp > 16) tmp = 16;
        if (tmp < 1) tmp = 1;
        
        itoa(tmp, sValue, 10);
        lcd_printlc(3, 9, sValue);
        lcd_print_P(PSTR(" "));
        _delay_ms(100);
        
    }
    
    eeprom_write_byte(&eePOD_Config.StatusLED1, tmp);
    
    tmp = eeprom_read_byte(&eePOD_Config.StatusLED1);
    
    itoa(tmp, sValue, 10);
    
    lcd_printlc(4, 9, sValue);
    
    lcd_printlc_P(4,1, PSTR("set to: "));
    
    sei();
    
    while (!get_key_press(1<<KEY_Enter));
    
}

void DOutStatusLED2 (void)
{
    cli();
    
    unsigned char tmp = 0;
    char sValue[3];
    
    lcd_command(LCD_CLEAR);
    
    lcd_printlc_P(1,1, PSTR("Status of LED2"));
    lcd_printlc_P(2,1, PSTR("on external Output?"));
    
    
    tmp = eeprom_read_byte(&eePOD_Config.StatusLED2);
    
    if ((tmp == 0) || tmp > 16)
    {
        tmp = 1;
    }
    
    lcd_printlc_P(3,2, PSTR("DO No: "));
    
    while (!get_key_press(1<<KEY_Enter))
    {
        if (get_key_press(1<<KEY_Left)) tmp--;
        if (get_key_press(1<<KEY_Right)) tmp++;
        
        if (tmp > 16) tmp = 16;
        if (tmp < 1) tmp = 1;
        
        itoa(tmp, sValue, 10);
        lcd_printlc(3, 9, sValue);
        lcd_print_P(PSTR(" "));
        _delay_ms(100);
        
    }
    
    eeprom_write_byte(&eePOD_Config.StatusLED2, tmp);
    
    tmp = eeprom_read_byte(&eePOD_Config.StatusLED2);
    
    itoa(tmp, sValue, 10);
    
    lcd_printlc(4, 9, sValue);
    
    lcd_printlc_P(4,1, PSTR("set to: "));
    
    sei();
    
    while (!get_key_press(1<<KEY_Enter));
    
}

void RadioConfSticktype (void)
{
    cli();
    
    unsigned char tmp = 0;
    
    lcd_command(LCD_CLEAR);
    
    lcd_printlc_P(1,1, PSTR("Sticktype"));
    
    tmp = eeprom_read_byte(&eePOD_Config.Sticktype);
    
    if (tmp >= 2) tmp = 0;
    
    lcd_printlc_P(3,1, PSTR("with corners? "));
    
    while (!get_key_press(1<<KEY_Enter))
    {
        if (get_key_press(1<<KEY_Left)) tmp--;
        if (get_key_press(1<<KEY_Right)) tmp++;
        
        if (tmp >= 1) tmp = 1;
        if (tmp <= 0) tmp = 0;
        
        if (tmp == 0) lcd_printlc_P(3,15, PSTR("YES"));
        if (tmp == 1) lcd_printlc_P(3,15, PSTR("NO "));
        _delay_ms(100);
        
    }
    
    eeprom_write_byte(&eePOD_Config.Sticktype, tmp);
    
    tmp = eeprom_read_byte(&eePOD_Config.Sticktype);
    
    lcd_printlc_P(4,4, PSTR("set to: "));
    
    if (tmp == 0) lcd_printlc_P(4,15, PSTR("YES"));
    if (tmp == 1) lcd_printlc_P(4,15, PSTR("NO "));
    
    sei();
    
    while (!get_key_press(1<<KEY_Enter));
}

void PPMINConf (void)
{
    unsigned char tmp = 1;
    char sValue[3];

    
    lcd_command(LCD_CLEAR);
    
    lcd_printlc_P(1,1, PSTR("PPM In Conf"));
    lcd_printlc_P(1,1, PSTR("choose channel"));
    lcd_printlc_P(3,1, PSTR("CH: "));
    
    while (!get_key_press(1<<KEY_Enter))
    {
        if (get_key_press(1<<KEY_Left)) tmp--;
        if (get_key_press(1<<KEY_Right)) tmp++;
        
        if (tmp <= 1) tmp = 1;
        if (tmp >= 9) tmp = 9;
        
        if (tmp == 1)
        {
            lcd_printlc_P(3,5, PSTR("none"));
        }
        else
        {
            itoa((tmp-1), sValue, 10);
            lcd_printlc(3, 5, sValue);
            lcd_printlc_P(3,6, PSTR("   "));
        }
        
        _delay_ms(100);
        
    }
    
    tmp--;
    
    if (tmp > 0)
    {
        PPMINConfChannel (tmp);
        
        lcd_command(LCD_CLEAR);
        
        lcd_printlc_P(4,1, PSTR("Test End"));
    }
    else
    {
        return;
    }

    while (!get_key_press(1<<KEY_Enter));
}

void PPMINConfChannel (unsigned char CHno)
{
    char sValue[3];
    char sValue2[6];
    unsigned char tmp = 1;
    uint32_t calcCH = 0;
    
    lcd_command(LCD_CLEAR);
    
    lcd_printlc_P(1,1, PSTR("PPM In Conf CH"));
    itoa(CHno, sValue, 10);
    lcd_printlc(1, 15, sValue);
    
    lcd_printlc_P(2,1, PSTR("Type: "));
    
    while (!get_key_press(1<<KEY_Enter))
    {
        if (get_key_press(1<<KEY_Left)) tmp--;
        if (get_key_press(1<<KEY_Right)) tmp++;
        
        if (tmp <= 1) tmp = 1;
        if (tmp >= 2) tmp = 2;
        
        if (tmp == 1)
        {
            lcd_printlc_P(2,7, PSTR("normal     "));
        }
        else
        {
            lcd_printlc_P(2,7, PSTR("2way Switch"));
        }
        
        _delay_ms(100);
    }
    
    lcd_printlc_P(2,1, PSTR("                    "));
    
    if (tmp == 1)
    {
        lcd_printlc_P(2,1, PSTR("set CH to     pos"));
        lcd_printlc_P(2,11, PSTR("MIN"));
        lcd_printlc_P(3,1, PSTR("then press ENTER"));
        
        while (!get_key_press(1<<KEY_Enter));
        
        lcd_printlc_P(2,1, PSTR("receiving...        "));
        lcd_printlc_P(3,1, PSTR("    [          ]    "));
        
        
        for (int i=0; i<=9; i++)
        {
            PPM_Read = 1;
            
            if (i == 0)
            {
                InChannel[(CHno-1)].min = PPM_Channel[CHno];
            }
            else
            {
                if (PPM_Channel[CHno] < InChannel[(CHno-1)].min) InChannel[(CHno-1)].min = PPM_Channel[CHno];
            }
            
            PPM_Read = 0;
            
            lcd_printlc_P(3,(i+6), PSTR("#"));
            
            _delay_ms(500);
        }
        
        lcd_printlc_P(4,1, PSTR("value: "));
        ltoa((InChannel[(CHno-1)].min/20), sValue2, 10);
        lcd_printlc(4, 8, sValue2);
        
        _delay_ms(1000);
        
        lcd_printlc_P(2,1, PSTR("                    "));
        lcd_printlc_P(3,1, PSTR("                    "));
        lcd_printlc_P(4,1, PSTR("                    "));
        lcd_printlc_P(2,1, PSTR("set CH to     pos"));
        lcd_printlc_P(2,11, PSTR("MID"));
        lcd_printlc_P(3,1, PSTR("then press ENTER"));
        
        while (!get_key_press(1<<KEY_Enter));
        
        lcd_printlc_P(2,1, PSTR("receiving...        "));
        lcd_printlc_P(3,1, PSTR("    [          ]    "));
        
        
        for (int i=0; i<=9; i++)
        {
            PPM_Read = 1;
            
            calcCH = calcCH + PPM_Channel[CHno];
            
            PPM_Read = 0;
            
            lcd_printlc_P(3,(i+6), PSTR("#"));
            
            _delay_ms(500);
        }
        
        InChannel[(CHno-1)].mid = calcCH / 10;
        
        lcd_printlc_P(4,1, PSTR("value: "));
        ltoa((InChannel[(CHno-1)].mid/20), sValue2, 10);
        lcd_printlc(4, 8, sValue2);
        
        _delay_ms(1000);
        lcd_printlc_P(2,1, PSTR("                    "));
        lcd_printlc_P(3,1, PSTR("                    "));
        lcd_printlc_P(4,1, PSTR("                    "));
        lcd_printlc_P(2,1, PSTR("set CH to     pos"));
        lcd_printlc_P(2,11, PSTR("MAX"));
        lcd_printlc_P(3,1, PSTR("then press ENTER"));
        
        while (!get_key_press(1<<KEY_Enter));
        
        lcd_printlc_P(2,1, PSTR("receiving...        "));
        lcd_printlc_P(3,1, PSTR("    [          ]    "));
        
        
        for (int i=0; i<=9; i++)
        {
            PPM_Read = 1;
            
            if (i == 0)
            {
                InChannel[(CHno-1)].max = PPM_Channel[CHno];
            }
            else
            {
                if (PPM_Channel[CHno] > InChannel[(CHno-1)].max) InChannel[(CHno-1)].max = PPM_Channel[CHno];
            }
            
            PPM_Read = 0;
            
            lcd_printlc_P(3,(i+6), PSTR("#"));
            
            _delay_ms(500);
        }
        
        lcd_printlc_P(4,1, PSTR("value: "));
        ltoa((InChannel[(CHno-1)].max/20), sValue2, 10);
        lcd_printlc(4, 8, sValue2);
        
        cli();
        
        eeprom_write_dword(&eePPM_IN[CHno-1].min, InChannel[(CHno-1)].min);
        eeprom_write_dword(&eePPM_IN[CHno-1].mid, InChannel[(CHno-1)].mid);
        eeprom_write_dword(&eePPM_IN[CHno-1].max, InChannel[(CHno-1)].max);
        
        sei();
        
    }
    
    if (tmp == 2)
    {
        ;
    }
    
    while (!get_key_press(1<<KEY_Enter));
}

void ServoRangeConf (void)
{
    unsigned char tmp = 1;
    char sValue[3];
    
    
    lcd_command(LCD_CLEAR);
    
    lcd_printlc_P(1,1, PSTR("Servo Range Conf"));
    lcd_printlc_P(1,1, PSTR("choose channel"));
    lcd_printlc_P(3,1, PSTR("CH: "));
    
    while (!get_key_press(1<<KEY_Enter))
    {
        if (get_key_press(1<<KEY_Left)) tmp--;
        if (get_key_press(1<<KEY_Right)) tmp++;
        
        if (tmp <= 1) tmp = 1;
        if (tmp >= 9) tmp = 9;
        
        if (tmp == 1)
        {
            lcd_printlc_P(3,5, PSTR("none"));
        }
        else
        {
            itoa((tmp-1), sValue, 10);
            lcd_printlc(3, 5, sValue);
            lcd_printlc_P(3,6, PSTR("   "));
        }
        
        _delay_ms(100);
        
    }
    
    tmp--;
    
    if (tmp > 0)
    {
        PPMINConfChannel (tmp);
        
        cli();
        
        // write to eeprom
        
        sei();
        
        lcd_command(LCD_CLEAR);
        
        lcd_printlc_P(4,1, PSTR("Test End"));
    }
    else
    {
        return;
    }
    
    while (!get_key_press(1<<KEY_Enter));
}

void ServoRangeConfChannel (unsigned char CHno)
{
    char sValue[3];
/*    char sValue2[6];
    unsigned char tmp = 1;
    uint32_t calcCH = 0; */
    
    lcd_command(LCD_CLEAR);
    
    lcd_printlc_P(1,1, PSTR("Servo Range xxx"));
    itoa(CHno, sValue, 10);
    lcd_printlc(1, 15, sValue);
    
    lcd_printlc_P(2,1, PSTR("Type: "));
}

void ESCProgMode (void)
{
    unsigned char tmp = 1;
    char sValue[3];
    
    
    lcd_command(LCD_CLEAR);
    
    lcd_printlc_P(1,1, PSTR("ESC porgramming"));
    lcd_printlc_P(1,1, PSTR("choose channel"));
    lcd_printlc_P(3,1, PSTR("CH: "));
    
    while (!get_key_press(1<<KEY_Enter))
    {
        if (get_key_press(1<<KEY_Left)) tmp--;
        if (get_key_press(1<<KEY_Right)) tmp++;
        
        if (tmp <= 1) tmp = 1;
        if (tmp >= 9) tmp = 9;
        
        if (tmp == 1)
        {
            lcd_printlc_P(3,5, PSTR("none"));
        }
        else
        {
            itoa((tmp-1), sValue, 10);
            lcd_printlc(3, 5, sValue);
            lcd_printlc_P(3,6, PSTR("   "));
        }
        
        _delay_ms(100);
        
    }
    
    if (tmp != 0)
    {
        tmp--;
        
        ESCProgModeChannel(tmp);
    }
    
    lcd_command(LCD_CLEAR);
    
    lcd_printlc_P(4,1, PSTR("Test End"));
    
    while (!get_key_press(1<<KEY_Enter));
}

void ESCProgModeChannel (unsigned char CHno)
{
    char sValue[6];
    float help;
    
    //set Servo outputs
    ServoDriveOUT[0] = 0;
    ServoDriveOUT[1] = 0;
    ServoDriveOUT[2] = 0;
    ServoDriveOUT[3] = 0;
    ServoDriveOUT[4] = 0;
    ServoDriveOUT[5] = 0;
    ServoDriveOUT[6] = 0;
    ServoDriveOUT[7] = 0;
    ServoDriveOUT[8] = 0;
    ServoDriveOUT[9] = 0;
    ServoDriveOUT[10] = 0;
    ServoDriveOUT[11] = 0;
    
    //ToDo
    servodrivecmd = 111;
    
    POD_set_ServoDrive();
    
    _delay_ms(100);
    
    lcd_command(LCD_CLEAR);
    
    lcd_printlc_P(1,1, PSTR("ESC Programming"));
    itoa(CHno, sValue, 10);
    lcd_printlc(2, 10, sValue);
    
    lcd_printlc_P(2,1, PSTR("Channel: "));
    
    cli();
    
    InChannel[0].min = eeprom_read_dword(&eePPM_IN[0].min);
    InChannel[0].max = eeprom_read_dword(&eePPM_IN[0].max);
    
    sei();
    
    while (!get_key_press(1<<KEY_Enter))
    {
        PPM_Read = 1;
        CH_in[1] = PPM_Channel[1];
        PPM_Read = 0;
        
        if (CH_in[1] < InChannel[0].min) CH_in[1] = InChannel[0].min;
        if (CH_in[1] > InChannel[0].max) CH_in[1] = InChannel[0].max;
        
        if (CH_in[1] > InChannel[0].min)
        {
            help = (((float)CH_in[1] - (float)InChannel[0].min) / ((float)InChannel[0].max - (float)InChannel[0].min)) * 100.0;
        }
        else
        {
            help = 0.0;
        }
        
        ServoDriveOUT[(CHno-1)] = 2500 + (help * 25);
        utoa(ServoDriveOUT[(CHno-1)], sValue, 10);
        //dtostrf(help, 2, 2, sValue);
        lcd_printlc(4, 10, sValue);
        
        utoa(CH_in[1], sValue, 10);
        lcd_printlc(4, 1, sValue);
        
        utoa(InChannel[0].max, sValue, 10);
        lcd_printlc(3, 10, sValue);
        
        utoa(InChannel[0].min, sValue, 10);
        lcd_printlc(3, 1, sValue);

        POD_set_ServoDrive();
        
        _delay_ms(10);
    }
    
    servodrivecmd = 0;
    POD_set_ServoDrive();

    //send data
}

//Stepper initialzation
void POD_StepperInit(unsigned char PODno)
{
    PODno--;
    
    if ( !(POD_HALLSENS_PIN & (1<<(PODno+4))) )
    {
        //Hallsensor high
        while ( !(POD_HALLSENS_PIN & (1<<(PODno+4))) )
        {
            POD_Step(PODno, CCW);
        }
        
        //back to 0°
        for (unsigned int step_tmp = PODStepper[PODno].StepsTo0; step_tmp != 0; step_tmp--)
        {
            POD_Step(PODno, CW);
        }
        
    }
    //Hallsensor low
    else
    {
        //Hallsensor low
        while ( (POD_HALLSENS_PIN & (1<<(PODno+4))) )
        {
            POD_Step(PODno, CCW);
        }
            
        //Hallsensor high
        while ( !(POD_HALLSENS_PIN & (1<<(PODno+4))) )
        {
            POD_Step(PODno, CCW);
        }
        
        //back to 0°
        for (unsigned int step_tmp = PODStepper[PODno].StepsTo0; step_tmp != 0; step_tmp--)
        {
            POD_Step(PODno, CW);
        }
    }
    
    PODStepper[PODno].Steps_Is = 0;
}

void PODStepperConfig (unsigned char PODno)
{
    char sValue[3];
    char sValue2[4];

    lcd_command(LCD_CLEAR);
    
    lcd_printlc_P(1,1, PSTR("POD alignement"));
    lcd_printlc_P(2,1, PSTR("POD No: "));
    itoa(PODno, sValue, 10);
    lcd_printlc(2, 9, sValue);
    lcd_printlc_P(3,1, PSTR("calibrating "));
        
    PODno--;
    
    PODStepper[PODno].StepsTo0 = 0;
    PODStepper[PODno].StepsFullCircle = 0;
    PODStepper[PODno].StepAngle = 0;
    
    //steps for calibration
    
    //Hallsensor high
    if ( !(POD_HALLSENS_PIN & (1<<(PODno+4))) )
    {
        //Hallsensor high
        while ( !(POD_HALLSENS_PIN & (1<<(PODno+4))) )
        {
            POD_Step(PODno, CCW);
            //variable1 hochzählen
            PODStepper[PODno].StepsTo0++;
        }
        
        itoa(PODStepper[PODno].StepsTo0, sValue2, 10);
        lcd_printlc(4, 1, sValue2);
        //_delay_ms(1000);
        
        //Hallsensor low
        while ( (POD_HALLSENS_PIN & (1<<(PODno+4))) )
        {
            POD_Step(PODno, CCW);
            //variable2 hochzählen
            PODStepper[PODno].StepsFullCircle++;
        }
        
        itoa(PODStepper[PODno].StepsFullCircle, sValue2, 10);
        lcd_printlc(4, 12, sValue2);
        //_delay_ms(1000);
        
        //Hallsensor high
        while ( !(POD_HALLSENS_PIN & (1<<(PODno+4))) )
        {
            POD_Step(PODno, CCW);
            //variable2 hochzählen
            PODStepper[PODno].StepsFullCircle++;
        }
        
        itoa(PODStepper[PODno].StepsFullCircle, sValue2, 10);
        lcd_printlc(4, 12, sValue2);
        //_delay_ms(1000);
        
        //back to 0°
        for (unsigned int step_tmp = PODStepper[PODno].StepsTo0; step_tmp != 0; step_tmp--)
        {
            POD_Step(PODno, CW);
            itoa(step_tmp, sValue2, 10);
        }

        lcd_printlc(4, 6, sValue2);
    }
    //Hallsensor low
    else
    {
        //Hallsensor low
        while ( (POD_HALLSENS_PIN & (1<<(PODno+4))) )
        {
            POD_Step(PODno, CCW);
            //variable1 hochzählen
            PODStepper[PODno].StepsTo0++;
        }
        
        itoa(PODStepper[PODno].StepsTo0, sValue2, 10);
        lcd_printlc(4, 1, sValue2);
        //_delay_ms(1000);
        
        //Hallsensor high
        while ( !(POD_HALLSENS_PIN & (1<<(PODno+4))) )
        {
            POD_Step(PODno, CCW);
            //variable1 hochzählen
            PODStepper[PODno].StepsTo0++;
        }
        
        itoa(PODStepper[PODno].StepsTo0, sValue2, 10);
        lcd_printlc(4, 1, sValue2);
        //_delay_ms(1000);
        
        //Hallsensor low
        while ( (POD_HALLSENS_PIN & (1<<(PODno+4))) )
        {
            POD_Step(PODno, CCW);
            //variable2 hochzählen
            PODStepper[PODno].StepsFullCircle++;
        }
        
        itoa(PODStepper[PODno].StepsFullCircle, sValue2, 10);
        lcd_printlc(4, 12, sValue2);
        //_delay_ms(1000);
        
        //Hallsensor high
        while ( !(POD_HALLSENS_PIN & (1<<(PODno+4))) )
        {
            POD_Step(PODno, CCW);
            //variable2 hochzählen
            PODStepper[PODno].StepsFullCircle++;
        }
        
        itoa(PODStepper[PODno].StepsFullCircle, sValue2, 10);
        lcd_printlc(4, 12, sValue2);
        //_delay_ms(1000);
        
        //back to 0°
        for (unsigned int step_tmp = PODStepper[PODno].StepsTo0; step_tmp != 0; step_tmp--)
        {
            POD_Step(PODno, CW);
        }
    }
    
    //calculate StepAngle
    PODStepper[PODno].StepAngle = 360.0 / PODStepper[PODno].StepsFullCircle;
    
    dtostrf(PODStepper[PODno].StepAngle,2,3,sTestPPM);
    //ftoa(PODStepper[PODno].StepAngle,2,2,sTestPPM);
    lcd_printlc(1, 10, sTestPPM);
    lcd_print_P(PSTR(" "));
    
    _delay_ms(2000);
    
    lcd_printlc_P(4,1, PSTR("                    "));
    
    itoa(PODStepper[PODno].StepsTo0, sValue2, 10);
    lcd_printlc(4, 1, sValue2);
    
    itoa(PODStepper[PODno].StepsFullCircle, sValue2, 10);
    lcd_printlc(4, 12, sValue2);
 
    cli();
    
    lcd_command(LCD_CLEAR);
    
    eeprom_write_word(&eePODStepper[PODno].StepsTo0, (uint16_t)PODStepper[PODno].StepsTo0);
    eeprom_write_word(&eePODStepper[PODno].StepsFullCircle, PODStepper[PODno].StepsFullCircle);
    eeprom_write_float(&eePODStepper[PODno].StepAngle, PODStepper[PODno].StepAngle);
    
    lcd_printlc_P(3,1, PSTR("read eeprom"));
    
    itoa(eeprom_read_word(&eePODStepper[PODno].StepsTo0),sValue2, 10);
    lcd_printlc(4, 1, sValue2);
    
    itoa(eeprom_read_word(&eePODStepper[PODno].StepsFullCircle),sValue2, 10);
    lcd_printlc(4, 12, sValue2);
    
    _delay_ms(2500);
    
    sei();
    
    lcd_command(LCD_CLEAR);
}

void POD_Step(unsigned char PODno, char Direction)
{
    //set Direction
    if (Direction == CW)
    {
        POD_STEP_PORT |= (1 << PODno);
        
        PODStepper[PODno].Steps_Is++;
        
        if (PODStepper[PODno].Steps_Is == PODStepper[PODno].StepsFullCircle)
        {
            PODStepper[PODno].Steps_Is = 0;
        }
    }
    else
    {
        POD_STEP_PORT &= ~(1 << PODno);
        if (PODStepper[PODno].Steps_Is == 0)
        {
            PODStepper[PODno].Steps_Is = PODStepper[PODno].StepsFullCircle - 1;
        }
        else
        {
            PODStepper[PODno].Steps_Is--;
        }
    }
    
    //set high
    POD_STEP_PORT |= (1 << (PODno + 4));
    //wait
    _delay_us(VSTEP1);
    //set low
    POD_STEP_PORT &= ~(1 << (PODno + 4));
    //wait
    _delay_us(VSTEP2);
    //_podDelay_us(POD_DriveConig.RotSpeed);
    
}
void _podDelay_us (uint16_t time)
{
    while (time >= 0)
    {
        _delay_us(1);
        time--;
    }
}



