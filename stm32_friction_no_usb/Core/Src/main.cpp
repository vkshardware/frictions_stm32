/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention       Frictions GAZ-71 DC motors controller. Firmware v5.0
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "FlashPROM.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


#define PARAMETERS 45     //See also BUFFSIZE 45+1 in FlashPROM.h
#define PAGES_COUNT 6
#define STR_LENGTH 15*2 // 2 bytes per symbol

#define bool uint16_t
#define false 0
#define true 1
#define PWM_8BIT 64  // 8-bit PWM 
#define I2C_SLAVE_ADDRESS 0x67
#define I2C_TIMEOUT 10
#define I2C_BUFFER 14 


//status byte 0
#define STATUS_BIT0_L_ON 0
#define STATUS_BIT0_R_ON 1
#define STATUS_BIT0_L_D  2
#define STATUS_BIT0_L_U  3
#define STATUS_BIT0_R_D  4
#define STATUS_BIT0_R_U  5
#define STATUS_BIT0_L_BUT 6
#define STATUS_BIT0_R_BUT 7

//status byte 1
#define STATUS_BIT1_L_EN   0
#define STATUS_BIT1_R_EN   1
#define STATUS_BIT1_L_ON_U 2
#define STATUS_BIT1_L_ON_D 3
#define STATUS_BIT1_R_ON_U 4
#define STATUS_BIT1_R_ON_D 5
#define STATUS_BIT1_L_TRIP_STAGE1 6   // L TRIP current protection jamming
#define STATUS_BIT1_R_TRIP_STAGE1 7   // R TRIP current protection jamming

//status byte 2
#define STATUS_BIT2_L_TRIP_STAGE2 0  // L TRIP over current protection short circuit
#define STATUS_BIT2_R_TRIP_STAGE2 1  // R TRIP over current protection short circuit
#define STATUS_BIT2_WATERMODE 2		 // Water Mode
#define STATUS_BIT2_SQUEEZE 3        // Release both sides
#define STATUS_BIT2_HOLD_ON_LOAD 4   // hold drive under load



#define RESERVE1 (1 << PB0)  //Reserve 1 Output
#define RESERVE2 (1 << PB1)  //Reserve 2 Input

#define ON_R1 (1 << PB2)  // Right Forward Output
#define ON_R2 (1 << PB3)  //Right Backward Output

#define SW_R (1 << PD0) // Release Right
#define RIGHT_DOWN (1 << PD1) // Right Down limit switch
#define RIGHT_UP (1 << PD2) // Right Up limit switch
#define SQUEEZE (1 << PD3) // Release and stay both sides
#define WATER_MODE (1 << PD4) // Water Mode - slowly going back
#define LEFT_DOWN (1 << PD5) // Left Down limit switch
#define LEFT_UP (1 << PD6) // Left Up limit switch
#define SW_L (1 << PD7) // Release Left

#define ON_L1 (1 << PA6)  //Left Forward Output
#define ON_L2 (1 << PA7)  //Left Backward Output


#define CURRENT_OFF_DELAY 20   //current off delay 20 ms
#define CURRENT_COEFFICIENT 25

#define WAIT_10kHZ 0x37
#define WAIT_100HZ 0xF63B
#define WAIT_5Hz 20
#define WAIT_1Hz 100


#define TIMER1_MAXCOUNT 10

#define PROTECTIONS_COUNT 6

#define SQUELCH 5 // CURRENT OFF protection: reject noise if PWM on
#define SQUELCH_MAIN_MODE 3

extern void erase_flash(void);
extern uint32_t flash_search_adress(uint32_t address, uint16_t cnt);
extern void write_to_flash(myBuf_t *buff);
extern void read_last_data_in_flash(myBuf_t *buff);
bool sq_off_left, sq_off_right = false;

struct AnalogFilter{
	uint64_t in_value;
	uint16_t out_value;
	uint16_t time_rate;
	uint16_t analog_limit;
	uint16_t time_count;
	bool trip;
};


struct SwitchFilter{
	bool outstate;
	bool lock;
	bool on_trigger;
	bool off_trigger;
	bool reset;
	bool off;
	bool protection_reset;
	uint16_t scanrate;
	GPIO_TypeDef * port;
	uint16_t pin;
	uint16_t curr_scan;
};

struct JoystickFilter{
	uint16_t in_value;
	uint8_t step_scan[10];
	uint8_t scanrate;
	uint8_t steps[10];
	uint8_t joystick_step;
	uint8_t mirror_joy_step;
	bool enabled;
};

struct Timer_v_in{
	bool timer_v_in_trigger;
  uint16_t timer_v_in_on;
  uint16_t timer_v_in_off;
	uint16_t timer_v_off_pause;
};

struct pwm{
		uint8_t dc_fill;
		bool enable;
	  uint8_t duty_cycle;
		GPIO_TypeDef * port;
	  uint16_t pin;
	  uint8_t curr_tick;
};	

struct double_pwm{
		uint8_t dc_fill;
		bool enable;
	  bool trigger;
	  uint16_t duty_cycle;
		GPIO_TypeDef * port_hi;
	  uint16_t pin_hi;
		GPIO_TypeDef * port_lo;
	  uint16_t pin_lo;
	  uint8_t curr_tick;
};	

struct i2c_data_{
  uint8_t statusbyte0;
  uint8_t statusbyte1;
  uint8_t statusbyte2;
  uint8_t current_page;
  uint8_t paramfromdisp;
  uint8_t paramfromdisp_addr;
};

struct _ProcessChannel{
  uint8_t gear_mode;
	bool button_pressed;
	struct double_pwm * pwm; 
	uint16_t  duty_cycle; 
	bool squeeze;
	bool force;
	uint8_t * led;
	uint8_t * strength;
	uint8_t T_release;
	uint8_t T_on;
	bool fault;
	uint16_t timer0;
	uint16_t timer1;
};



struct MainDataSet{
  bool Water_mode;
  bool Squeeze_mode;
  bool Left_button;
  bool Right_button;
	uint8_t SW_L_state;     
  uint8_t SW_R_state;     
  uint8_t Left_fault_stage1;
	uint8_t Left_fault_stage2;
  uint8_t Right_fault_stage1;
	uint8_t Right_fault_stage2;
	uint8_t Left_On;
	uint8_t Right_On;
	uint8_t Left_Current;
	uint8_t Right_Current;
	uint8_t DO_mode1;
	uint8_t DO_error;
	uint8_t DI_mode1;
	uint8_t DI_mode2;
};

struct MenuRow{
  uint8_t id;
	char desc_cust[STR_LENGTH];
	char desc_en[STR_LENGTH];
  uint8_t def;
  uint8_t min;
  uint8_t max;
  uint8_t writable; // 1- writable, 0 - readonly
  int8_t dp; //decimal point position
  uint8_t symb; // 0 - null, 1 - c, 2 - %, 3- A, 4 - s, 5 - mc, 6 - ms, 7 - Celsius, 8 - km/h
};


const MenuRow menus[] = {
  {1,"01 ??. ??????","01 Sq. time",9,3,100,1,1,1},
  {2,"02 ??. ?????","02 Return time",20,3,100,1,1,1},
  {3,"03 ??. ??. ??.","03 Moveup time",10,0,100,1,1,1},
  {4,"04 ??. ???.","04 Joys. L",0,0,0,0,0,2},
  {5,"05 ???. ???.","05 CMD L",0,0,0,0,1,0},
  {6,"06 X_L ?????.","06 X_L conn.",0,0,0,0,0,0},
  {7,"07 ??. ??.","07 Joys. R",0,0,0,0,0,2},
  {8,"08 ???. ??.","08 CMD R",0,0,0,0,1,0},
  {9,"09 X_R ?????.","09 X_R conn.,",0,0,0,0,0,0},
  {10,"10 X_L ???.","10 X_L low",10,0,99,1,0,2},
  {11,"11 X_L 4 ??.","11 X_L 4 st.",25,0,99,1,0,2},
  {12,"12 X_L 3 ??.","12 X_L 3 st.",28,0,99,1,0,2},
  {13,"13 X_L 2 ??.","13 X_L 2 st.",31,0,99,1,0,2},
  {14,"14 X_L 1 ??.","14 X_L 1 st.",34,0,99,1,0,2},
  {15,"15 X_L 1 ???.","15 X_L 1 res.",45,0,99,1,0,2},
  {16,"16 X_L 2 ???.","16 X_L 2 res.",48,0,99,1,0,2},
  {17,"17 X_L 3 ???.","17 X_L 3 res.",53,0,99,1,0,2},
  {18,"18 X_L 4 ???.","18 X_L 4 res.",56,0,99,1,0,2},
  {19,"19 X_L ????.","19 X_L high",90,0,99,1,0,2},
  {20,"20 X_R ???.","20 X_R low",10,0,99,1,0,2},
  {21,"21 X_R 4 ??.","21 X_R 4 st.",25,0,99,1,0,2},
  {22,"22 X_R 3 ??.","22 X_R 3 st.",28,0,99,1,0,2},
  {23,"23 X_R 2 ??.","23 X_R 2 st.",31,0,99,1,0,2},
  {24,"24 X_R 1 ??.","24 X_R 1 st.",34,0,99,1,0,2},
  {25,"25 X_R 1 ???.","25 X_R 1 res.",45,0,99,1,0,2},
  {26,"26 X_R 2 ???.","26 X_R 2 res.",48,0,99,1,0,2},
  {27,"27 X_R 3 ???.","27 X_R 3 res.",53,0,99,1,0,2},
  {28,"28 X_R 4 ???.","28 X_R 4 res.",56,0,99,1,0,2},
  {29,"29 X_R ????.","29 X_R high",90,0,99,1,0,2},
  {30,"30 I????. ???.","30 Ijam. L",10,1,50,1,0,3},
  {31,"31 T????. ???.","31 Tjam. L",10,1,99,1,-1,5},
  {32,"32 I????. ??.","32 Ijam. R",10,1,50,1,0,3},
  {33,"33 T????. ??.","33 Tjam. R",10,1,99,1,-1,5},
  {34,"34 I??. ???.","34 Iovercur. L",45,0,80,1,0,3},
  {35,"35 I??. ??.","35 Iovercur. R",45,0,80,1,0,3},
  {36,"36 ?????. ???.","36 Holdon L",0,0,10,1,-1,2},
  {37,"37 ?????. ??.","37 Holdon R",0,0,10,1,-1,2},
  {38,"38 ?????. ????.","38 Ret. norm",10,1,10,1,-1,2},
  {39,"39 ?????. ????","39 Ret. water",4,1,10,1,-1,2},
  {40,"40 ????. ???.","40 HAN logic",0,0,1,1,0,0},
  {41,"41 CAN ????.","41 CAN speed",6,0,8,1,0,0}, //6 (125kbps), 3 (250kbps), 2 (375kbps), 1 (750kbps)
  {42,"42 CAN ???.","42 CAN address",0x11,0x01,0xFE,1,0,0}, 
  {43,"43 DO ?????.","43 DO func.",0,0,6,1,0,0}, 
  {44,"44 ????????", "44 Firmware", 50,0,100,0,1,0},
  {45,"45 ????", "45 Language",1,0,1,1,0,0},
};

class ChannelControl{	
	private:
	   
	public:
    bool switch_up;
    bool switch_down;
	  bool control_source; //0 - joystick, 1 - button
	  bool water_mode;  
	  bool long_release;
	  bool forward_end_trigger;

    uint8_t timer_pwm_backward;
	  uint8_t timer_pwm_hold_on;
	  uint8_t water_dc;
	  uint8_t hold_on_dc;
	  uint8_t normal_mode_dc;
	  
	  uint16_t timer_count;
	  uint8_t goal_step;
	  uint8_t curr_step;
	  uint16_t prev_timer_count_position;
	  float timer_count_position;

	  bool timer_count_en;
	  bool moving_up_stop_trip;
	  bool moving_fwd;
	  bool moving_bwd;
	  bool jog_direction;
	  bool lock; 
	  bool prev_lock_state;

	  GPIO_TypeDef * port1;
	  uint16_t pin1;
	  
	  GPIO_TypeDef * port2;
	  uint16_t pin2;
	  	  
	  void InitChannel();
	  void Forward();
	  void ForwardHoldOn();
	  void Backward();
	  void VoltageOff();
	  void Stop();
	  int8_t Process();
};



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

uint16_t params_buf[PARAMETERS+1]; // zero byte - FLASH not erased info
uint32_t res_addr = 0;
uint32_t tick = 0;
uint16_t speed_tick = 0;
uint16_t adc_curr_L, adc_curr_R = 0; 
uint32_t temp01, temp02 = 0;
ChannelControl Left, Right;
struct AnalogFilter pa[7]; // analog pin filter pins PA0-PA5

struct SwitchFilter in_left, in_right, sw_leftdown, sw_leftup, sw_rightdown, sw_rightup, sw_water, sw_squeeze, sw_mode1, sw_mode2; 
//Button_L  PB10 
//Button_R  PA10
//Mode 2    PB11
//Mode 1    PA15
//Water     PA9
//Squeeze   PA8
//SW_L_Down PB15
//SW_L_Up   PB14
//SW_R_Down PB13
//SW_R_Up   PB12
//Mode 1    PA15
//Mode 2    PB11

uint8_t i2c_buf[I2C_BUFFER];
uint8_t i2c_receive[I2C_BUFFER];
uint8_t regAddress = 0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

struct MainDataSet main_data;
struct i2c_data_ i2c_data;
uint32_t timers[8];
uint32_t tick100Hz = 0;
uint16_t  temp0, temp2 = 0;

ADC_ChannelConfTypeDef sConfig = {0};
ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};

AnalogFilter valLn, valLp, valRn, valRp; //jamming protection
AnalogFilter oc_valLn, oc_valLp, oc_valRn, oc_valRp;  //over current protection (short circuit)
JoystickFilter an_R, an_L;

float _err_measure = 0.8; 
float _q = 0.1;   

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void Init_Channels()
{
  //Base PCB (add USB)
	//PB1 NP_ML+
  //PB0 NP_ML-
  //PA7 NP_MR+
  //PA6 NP_MR-
	
	//First USB (no USB)
	//PB10 NP_ML+
	//PB1 NP_ML-
	//PA7 NP_MR+
	//PA6 NP_MR-
		
	Left.port1 = GPIOB;
	Left.pin1 = GPIO_PIN_10;
	Left.port2 = GPIOB;
	Left.pin2 = GPIO_PIN_1;
	Left.InitChannel();
	Left.timer_pwm_backward = 4;
	Left.timer_pwm_hold_on = 10;

	Right.port1 = GPIOA;
	Right.pin1 = GPIO_PIN_7;
	Right.port2 = GPIOA;
	Right.pin2 = GPIO_PIN_6;
	Right.InitChannel();
	Right.timer_pwm_backward = 4; // Making time interval between Left and Right Channels
	Right.timer_pwm_hold_on = 10;
}


void InitMainScreenSet()
{

  main_data.Left_button = false;
  main_data.Right_button = false;
  main_data.Squeeze_mode = false;
  main_data.Water_mode = false;
	main_data.Left_On = 0;
	main_data.Right_On = 0;
	main_data.Left_fault_stage1 = 0;
	main_data.Left_fault_stage2 = 0;
	main_data.Right_fault_stage1 = 0;
	main_data.Right_fault_stage2 = 0;
	main_data.DI_mode1 = 0;
	main_data.DI_mode2 = 0;
	main_data.DO_error = 0;
	main_data.DO_error = 0;
  main_data.SW_L_state = 0;
	main_data.SW_R_state = 0;
	main_data.Left_Current = 0;
	main_data.Right_Current = 0;
}

void InitAnalogPins(void)
{
	//CS_MR-
	pa[0].trip = false;
	pa[0].in_value = 0;
	pa[0].time_rate = 3;
	pa[0].time_count = 0;
	pa[0].out_value = 0;
	pa[0].analog_limit = 0; 

	//CS_MR+
  pa[1].trip = false;
	pa[1].in_value = 0;
	pa[1].time_rate = 3;
	pa[1].time_count = 0;
	pa[1].out_value = 0;
	pa[1].analog_limit = 0; 
	
	//CS_ML-
  pa[2].trip = false;
	pa[2].in_value = 0;
	pa[2].time_rate = 3; 
	pa[2].time_count = 0;
	pa[2].out_value = 0;
	pa[2].analog_limit = 0; 
	
	//CS_ML+
  pa[3].trip = false;
	pa[3].in_value = 0;
	pa[3].time_rate = 3;  
	pa[3].time_count = 0;
	pa[3].out_value = 0;
	pa[3].analog_limit = 0;	
	
	// Joystick L resistor initialization
	
	an_L.scanrate = 2;	
	an_L.joystick_step = 0;
	an_L.mirror_joy_step = 0;
	an_L.enabled = false;	
	
	// Joystick R resistor initialization

	an_R.scanrate = 2;	
	an_R.joystick_step = 0;
	an_R.mirror_joy_step = 0;
	an_R.enabled = false;
	
}

bool CurrentProtection(AnalogFilter * ana)
{
    if (ana->analog_limit == 0) return false;
 	
	if (ana->in_value > ana->analog_limit)
	{
		if (ana->time_count >= ana->time_rate)
		{
			ana->trip= true;
		} else
		  ana->time_count++;
	} else 
	  ana->time_count = 0;
	
	return ana->trip;
}


void _joystick_filter(JoystickFilter * source)
{
	int i = 0;
	
	if ((source->in_value < source->steps[0]) || (source->in_value > source->steps[9]))
	{
		source->joystick_step = 0;
		source->enabled = false;
		
	} 
	else
	
	{
	
		source->enabled = true;	
	
		for (i = 0; i <= 4; i++) // scan 0-50 base joystick bandwidth
		if ((source->in_value > source->steps[i]) && (source->in_value <= source->steps[i+1]))
		{
		
			if (source->step_scan[i] >= source->scanrate){
			
				source->joystick_step = 4-i;
					
			} else
					
			source->step_scan[i]++;
				
			if  (source->scanrate == 0xFF) source->step_scan[i] = 0;
		
		}
		else
		source->step_scan[i] = 0;
		
		
		for (i = 9; i >= 5; i--) // scan 50-100 reserve mirror joystick bandwidth
		if ((source->in_value < source->steps[i]) && (source->in_value >= source->steps[i-1]))
		{
		
			if (source->step_scan[i] >= source->scanrate){
			
				source->mirror_joy_step = i-5;
					
			} else
					
			source->step_scan[i]++;
				
			if  (source->scanrate == 0xFF) source->step_scan[i] = 0;
		
		}
		else
		source->step_scan[i] = 0;
	
	}


}



bool _switch_filter(struct SwitchFilter * source)
{	
	
	source->outstate = false;
	
	if (!(HAL_GPIO_ReadPin(source->port,source->pin)))
	{
		if (source->curr_scan >= source->scanrate){	
		   
			if (source->off) 
			{
				
			  source->on_trigger = true;
			  source->protection_reset = true;
			}
			
		    
			source->off = false;
			
			source->outstate = true;
			
		} else
	
		source->curr_scan++;
		
		
		if  (source->scanrate == 0xFFFF) source->curr_scan = 0;
		
	} else
	{ 
		  if (!source->off) source->off_trigger = true;
		
	    source->curr_scan = 0;	
  		source->off = true;
			
		
		  if (source->off_trigger) 
			{ 
				source->lock = false;
				source->off_trigger = false;
			}
	}
	
	return source->outstate;
}



void Init_Protections()
{
	// Jamming protections
	
	valLn.trip = false;
	valLn.in_value = 0;
	valLn.time_rate = params_buf[31];
	valLn.time_count = 0;
	valLn.analog_limit = params_buf[30]*CURRENT_COEFFICIENT; 

	valLp.trip = false;
	valLp.in_value = 0;
	valLp.time_rate = params_buf[31];
	valLp.time_count = 0;
	valLp.analog_limit = params_buf[30]*CURRENT_COEFFICIENT; 	
	
	valRn.trip = false;
	valRn.in_value = 0;
	valRn.time_rate = params_buf[33];
	valRn.time_count = 0;
	valRn.analog_limit = params_buf[32]*CURRENT_COEFFICIENT; 
	
	valRp.trip = false;
	valRp.in_value = 0;
	valRp.time_rate = params_buf[33];
	valRp.time_count = 0;
	valRp.analog_limit = params_buf[32]*CURRENT_COEFFICIENT; 

    // Short circuit overcurrent protection
 
  oc_valLn.trip = false;
	oc_valLn.in_value = 0;
	oc_valLn.analog_limit = params_buf[34]*CURRENT_COEFFICIENT;
	oc_valLn.time_count=0;

	oc_valLp.trip = false;
	oc_valLp.in_value = 0;
	oc_valLp.analog_limit = params_buf[34]*CURRENT_COEFFICIENT;
	oc_valLp.time_count=0;
	
	oc_valRn.trip = false;
	oc_valRn.in_value = 0;
	oc_valRn.analog_limit = params_buf[35]*CURRENT_COEFFICIENT;
	oc_valRn.time_count=0;
		
	oc_valRp.trip = false;
	oc_valRp.in_value = 0;
	oc_valRp.analog_limit = params_buf[35]*CURRENT_COEFFICIENT;
	oc_valRp.time_count=0;
}



bool AnalogPinTrip(struct AnalogFilter * ana)
{
  if (ana->analog_limit == 0) return false;
 	
	if (ana->in_value > ana->analog_limit)
	{
		if (ana->time_count >= ana->time_rate)
		{
			ana->trip=true;
		} else
		  ana->time_count++;
	} else
  {	
	  ana->time_count = 0;
		ana->trip=false;
	}
	return ana->trip;
}

void AnalogAverage(struct AnalogFilter * ana)
{
	if (ana->time_count == 0) return;
	
  ana->out_value = (uint16_t) sqrt(ana->in_value/ana->time_count); // RMS average
			
	ana->time_count = 0;
	ana->in_value = 0;
}

void AnalogAddValue(struct AnalogFilter * ana, uint16_t value)
{
	ana->in_value += value*value; // sum of squares
	ana->time_count++;
}

void Init_i2c_data()
{
  i2c_data.statusbyte0 = 0;
  i2c_data.statusbyte1 = 0;
  i2c_data.statusbyte2 = 0;
  i2c_data.paramfromdisp = 0;
  i2c_data.paramfromdisp_addr = 0;
  i2c_data.current_page = 0;
}


void Init_Switches()
{
    //Button_L  First PCB (no USB) -> PA11, Base PCB (add USB) -> PB10 
		in_left.outstate = false;
	  in_left.lock = false;
	  in_left.off_trigger = false;
	  in_left.on_trigger = false;
	  in_left.protection_reset = false;
	  in_left.port = GPIOA;
	  in_left.pin = GPIO_PIN_11;
	  in_left.scanrate = 4;
	
	  //Button_R  PA10
		in_right.outstate = false;
	  in_right.lock = false;
	  in_right.off_trigger = false;
	  in_right.on_trigger = false;
	  in_right.protection_reset = false;
	  in_right.port = GPIOA;
	  in_right.pin = GPIO_PIN_10;
	  in_right.scanrate = 4;
	
		//SW_L_Down PB15
		sw_leftdown.outstate = false;
	  sw_leftdown.lock = false;
	  sw_leftdown.off_trigger = false;
		sw_leftdown.on_trigger = false;
	  sw_leftdown.port = GPIOB;
	  sw_leftdown.pin = GPIO_PIN_15;
	  sw_leftdown.scanrate = 4;
    
		//SW_L_Up   PB14
		sw_leftup.outstate = false;
		sw_leftup.lock = false;
		sw_leftup.off_trigger = false;
		sw_leftup.on_trigger = false;
	  sw_leftup.port = GPIOB;
	  sw_leftup.pin = GPIO_PIN_14;
	  sw_leftup.scanrate = 4;
		
		//SW_R_Down PB13
		sw_rightdown.outstate = false;
		sw_rightdown.lock = false;
		sw_rightdown.off_trigger = false;
		sw_rightdown.on_trigger = false;
	  sw_rightdown.port = GPIOB;
	  sw_rightdown.pin = GPIO_PIN_13;
	  sw_rightdown.scanrate = 4;		
		
		//SW_R_Up   PB12	
    sw_rightup.outstate = false;
		sw_rightup.lock = false;
		sw_rightup.off_trigger = false;
		sw_rightup.on_trigger = false;
	  sw_rightup.port = GPIOB;
	  sw_rightup.pin = GPIO_PIN_12;
	  sw_rightup.scanrate = 4;		

    //Water     PA9
		sw_water.outstate = false;
		sw_water.lock = false;
		sw_water.off_trigger = false;
		sw_water.on_trigger = false;
	  sw_water.port = GPIOA;
	  sw_water.pin = GPIO_PIN_9;
	  sw_water.scanrate = 4;		
		
		//Squeeze   PA8
    sw_squeeze.outstate = false;
		sw_squeeze.lock = false;
		sw_squeeze.off_trigger = false;
		sw_squeeze.on_trigger = false;
	  sw_squeeze.port = GPIOA;
	  sw_squeeze.pin = GPIO_PIN_8;
	  sw_squeeze.scanrate = 4;	

    //Mode 1    PA15
    sw_mode1.outstate = false;
		sw_mode1.lock = false;
		sw_mode1.off_trigger = false;
		sw_mode1.on_trigger = false;
	  sw_mode1.port = GPIOA;
	  sw_mode1.pin = GPIO_PIN_15;
	  sw_mode1.scanrate = 4;	
		
		//Mode 2    First PCB (no USB) -> PA12, Base PCB (add USB) -> PB11 
    sw_mode2.outstate = false;
		sw_mode2.lock = false;
		sw_mode2.off_trigger = false;
		sw_mode2.on_trigger = false;
	  sw_mode2.port = GPIOA;
	  sw_mode2.pin = GPIO_PIN_12;
	  sw_mode2.scanrate = 4;	
}

void ChannelControl::InitChannel()
{
	lock = false;
	prev_lock_state = false;
	prev_timer_count_position = 0;
	timer_count_position = 0;
	moving_up_stop_trip = false;
	switch_up = false;
	control_source = false;
	switch_down = false;
	timer_count = 0;
	moving_fwd = false;
	moving_bwd = false;
	timer_count_en = false;
	long_release = false;
	
	timer_pwm_backward = 0;
	timer_pwm_hold_on = 0;
	
	water_dc = 0;
	hold_on_dc = 0;
	normal_mode_dc = 0;
	
	water_mode = false;
	curr_step = 0;
	goal_step = 0;
}

void ChannelControl::Backward()
{
	//320Hz PWM 
	
	if (((!water_mode) && (timer_pwm_backward <= normal_mode_dc)) || ((water_mode) && (timer_pwm_backward <= water_dc)))
	{
	   // Moving backward
		HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_RESET);
		
	} 
	else 
	
	VoltageOff();
	
	
}

void ChannelControl::VoltageOff()
{
		HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_RESET);
}

void ChannelControl::ForwardHoldOn()
//  160 Hz PWM
{
    if (hold_on_dc > 0)
    {
		 moving_fwd = true;
		 moving_bwd = false;
		  
		if (timer_pwm_hold_on < hold_on_dc)
		{
			HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_SET);

		  

	   } else VoltageOff();
	} 
	
	else Stop();

}



void ChannelControl::Forward()
{
		 HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_SET);
	   moving_fwd = true;
	   moving_bwd = false;

}

void ChannelControl::Stop()
{
  VoltageOff();
	moving_fwd = false;
	moving_bwd = false;
}

void ProcessPWM(struct pwm * channel) //one transistor PWM
{
	
	 if (channel->dc_fill == 0){
		 HAL_GPIO_WritePin(channel->port, channel->pin, GPIO_PIN_RESET);
		 return;
	 }
	 
	 channel->curr_tick++;
	
	if (channel->curr_tick >= PWM_8BIT)
	{
		 channel->curr_tick = 0;
	}
  
	if (channel->enable && (channel->dc_fill >= channel->curr_tick))
	{
		HAL_GPIO_WritePin(channel->port, channel->pin, GPIO_PIN_SET);
	} else
	  HAL_GPIO_WritePin(channel->port, channel->pin, GPIO_PIN_RESET);
}

int8_t ChannelControl::Process()
{
		uint16_t relative_step = 0;
	  uint16_t condition = 0;
		uint16_t condition2 = 0;
		
		uint16_t coefficient_fwd, coefficient_bwd = 10;
		
		if ((goal_step > curr_step) || moving_fwd)  // Forward
		{
			relative_step = goal_step - curr_step;
			
			if (moving_fwd && (relative_step == 0)) relative_step = 1;
			
			
			
			if (!jog_direction) {
				Stop();
				HAL_Delay(CURRENT_OFF_DELAY);
				timer_count_en = true;
				moving_fwd = true;
				moving_bwd = false;
				timer_count = 0;
				
				if (control_source) curr_step = 0;
			}
			
			jog_direction = true;
			
			
			if (!control_source) coefficient_fwd = 12;   // If it is Joystick control, increase forward time
			  
		    else
			    coefficient_fwd = 10;
				
			if (long_release) coefficient_fwd = 12; // Increase release time if SQUEEZE input active
			
			condition2 = params_buf[3]*coefficient_fwd; //moving_up_stop
			condition = params_buf[1]*coefficient_fwd/4*relative_step; //wait_sec_up

			
	        if (timer_count != prev_timer_count_position)
	 	    {
                 timer_count_position++;                    // Increase real actuator position timer
			      prev_timer_count_position = timer_count;
			} 
			
			moving_up_stop_trip = (timer_count_position >= (float) condition2) && (params_buf[3] >= params_buf[1]); //moving_up_stop >= wait_sec_up
			
			  	
			
			
			if ((timer_count <= condition) && (curr_step < 4) && (!switch_up) && (!moving_up_stop_trip))
			{
				Forward(); //Channel turn Forward
				
				timer_count_en = true;
				timer_pwm_hold_on = 0;
				forward_end_trigger = false;
				
			
			} else
			{

				//if (!switch_up) curr_step = 4;
			
				if ((timer_count_en) && (curr_step < 4)) curr_step +=relative_step;

				if ((curr_step < 4) || (!control_source))
				{
					
				  timer_count_en = false;
				  timer_count = 0;
				
				}
				
				if ((!forward_end_trigger) && (curr_step == 4) && (hold_on_dc > 0))
				{
					VoltageOff();
					HAL_Delay(CURRENT_OFF_DELAY);
					
					forward_end_trigger = true;
				}
				 		
				if ((!switch_up) && (curr_step == 4) && (hold_on_dc > 0)) {
					
					timer_count_en = false;					  
					ForwardHoldOn(); // Hold on pulsing on released actuator under load
				} else
				
				Stop();
			}
		}
		
		if ((goal_step < curr_step) || moving_bwd)   // Backward
		{
			relative_step = curr_step - goal_step;
			
			
			if (moving_bwd && (relative_step == 0)) relative_step = 1;
			
			if (jog_direction) {
				Stop();
				HAL_Delay(CURRENT_OFF_DELAY); // current off delay
				timer_count_en = true;
				moving_bwd = true;
				moving_fwd = false;
				timer_count = 0;
			}
			
			jog_direction = false;
			
			
			if (water_mode) 
				coefficient_bwd = 100/water_dc;
			else
			   coefficient_bwd = 10;
			
			
			if (goal_step == 0) {
				
				condition = params_buf[2]*coefficient_bwd;  //wait_sec_down; big backward time guaranteed to return to actuator starting position
			} 
			else
			condition = params_buf[1]*coefficient_bwd/4*relative_step;
						
			
	        if (timer_count !=  prev_timer_count_position)
	        {
		        if (timer_count_position > 0) timer_count_position = timer_count_position - (10 / (float) coefficient_bwd);  //Decrease real actuator position timer
		        
				if (timer_count_position < 0) timer_count_position = 0;
				prev_timer_count_position = timer_count;
	        }
			
			if ((timer_count <= condition) && (!switch_down))
			{
				Backward();  //Channel turn Backward
				timer_count_en = true;
				
			} else  
			{
				
				
				if ((timer_count_en) && (goal_step > 0)) curr_step -=relative_step; //
				
				if ((timer_count_en) && (goal_step == 0)) curr_step = 0; //fuse condition
				
				timer_count_en = false;
				
				timer_count = 0;
				
				Stop();			
			}
			
		} 
		
		if (switch_down)
		{
			timer_count_position = 0;
		}
		
	
	return curr_step;
}


bool CheckParamsLimits(uint16_t * params)
{
	bool result = true;
	
	for (int i=1; i<= PARAMETERS; i++)
	{
	 if ((uint16_t) params[i] < (uint16_t) menus[i-1].min) result = false;
	 if ((uint16_t) params[i] > (uint16_t) menus[i-1].max) result = false;
	}
	
	return result;
}


void WriteDefsToFlash()
{
	params_buf[0] = 0;
	
	
	for (int i = 1; i <= PARAMETERS; i++){
		   params_buf[i] = menus[i-1].def;
	}
	
	write_to_flash(&params_buf[0]);

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) // 100Hz Timer
{ 
	if(htim->Instance == TIM3)
  {
		if (Left.timer_count_en) Left.timer_count++;
    if (Left.timer_count == 0xFFFF) Left.timer_count = 0;

    if (Right.timer_count_en) Right.timer_count++;
    if (Right.timer_count == 0xFFFF) Right.timer_count = 0;

	
		tick100Hz++;
		
		//in_left, in_right, sw_leftdown, sw_leftup, sw_rightdown, sw_rightup, sw_water, sw_squeeze, sw_mode1, sw_mode2; 
		
		_switch_filter(&in_left);  
		_switch_filter(&in_right);  
		_switch_filter(&sw_leftdown); 
		_switch_filter(&sw_leftup);  
		_switch_filter(&sw_rightdown);  
		_switch_filter(&sw_rightup); 
		_switch_filter(&sw_water);  
		_switch_filter(&sw_squeeze); 
		_switch_filter(&sw_mode1);  
		_switch_filter(&sw_mode2); 		
		
		
		CurrentProtection(&valLn);
		CurrentProtection(&valLp);
		
		if (valLn.trip || valLp.trip)
	  {
			// Stop Left Channel
			Left.Stop();
		}

	
		CurrentProtection(&valRn);
		CurrentProtection(&valRp);
		
		if (valRn.trip || valRp.trip)
		{
			// Stop Right Channel
			Right.Stop();
		} 
		

		
	
		Left.switch_down = sw_leftdown.outstate;
		Left.switch_up = sw_leftup.outstate;
	
		Right.switch_down = sw_rightdown.outstate;
		Right.switch_up = sw_rightup.outstate;


	}
	
	if (htim->Instance == TIM4)
	{
    	if (Left.timer_pwm_backward < 10) Left.timer_pwm_backward++; else Left.timer_pwm_backward = 0;
	    if (Right.timer_pwm_backward < 10) Right.timer_pwm_backward++; else Right.timer_pwm_backward = 0;

	    if (Left.timer_pwm_hold_on < 20) Left.timer_pwm_hold_on++; else Left.timer_pwm_hold_on = 0;
	    if (Right.timer_pwm_hold_on < 20) Right.timer_pwm_hold_on++; else Right.timer_pwm_hold_on = 0;	
	}
}

void ProtectionProcess(uint32_t channel, uint32_t value)
{
	
	 switch (channel)
	 {
		 case ADC_CHANNEL_0: //MR-
			  if (Right.water_mode || (Right.hold_on_dc > 0))
				{
					oc_valRn.time_rate = SQUELCH; 
				}	
				else
					oc_valRn.time_rate = SQUELCH_MAIN_MODE;  
				
				oc_valRn.in_value = value;
				
				CurrentProtection(&oc_valRn); //Fast short circuit
				if (oc_valRn.trip)
				{
					// Stop Right Channel
				  Right.Stop();
				}
		 break;
		 
		 case ADC_CHANNEL_1: //MR+
			  if (Right.water_mode || (Right.hold_on_dc > 0))
				{
					oc_valRp.time_rate = SQUELCH; 
				}	
				else
					oc_valRp.time_rate = SQUELCH_MAIN_MODE;  
				
				oc_valRp.in_value = value;
				CurrentProtection(&oc_valRp); //Fast short circuit
				if (oc_valRp.trip)
				{
					// Stop Right Channel
				  Right.Stop();
				}
		 break;		 
		 
		 case ADC_CHANNEL_2: //ML-
			  if (Left.water_mode || (Left.hold_on_dc > 0))
				{
					oc_valLn.time_rate = SQUELCH; 
				}	
				else
					oc_valLn.time_rate = SQUELCH_MAIN_MODE;  
				
				oc_valLn.in_value = value;
				CurrentProtection(&oc_valLn); //Fast short circuit
				if (oc_valLn.trip)
				{
					// Stop Left Channel
				  Left.Stop();
					
				} 
		 break;		
		 
		 case ADC_CHANNEL_3: //ML+
			 
		    if (Left.water_mode || (Left.hold_on_dc > 0))
				{
					oc_valLp.time_rate = SQUELCH; 
				}	
				else
					oc_valLp.time_rate = SQUELCH_MAIN_MODE;  
				
				oc_valLp.in_value = value;
				CurrentProtection(&oc_valLp); //Fast short circuit
				if (oc_valLp.trip)
				{
					// Stop Left Channel
				  Left.Stop();

				} 
		 break;		
	 }
	 
	
}


uint32_t GetADC1_Value(uint32_t channel)
{

		uint32_t g_ADCValue = 0;
		sConfig.Channel = channel;

		sConfig.Rank = 1;

		sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5; //or any other value available.

		//add to channel select
    
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
				return -1;
		}
    HAL_ADC_Start(&hadc1);
		

		if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
			
		g_ADCValue = HAL_ADC_GetValue(&hadc1);

		//remove from channel select

		HAL_ADC_Stop(&hadc1);
		
		sConfig.Rank = 0; //ADC_RANK_NONE;

		
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			return -1;
		}
		

		return (g_ADCValue);
}

void Init_Joystick()
{
	for (int i = 0; i <=9; i++)
	{
		an_L.steps[i] = params_buf[i+10];
		an_R.steps[i] = params_buf[i+20];
	}
	
}


void WriteParamsToFlash()
{
	params_buf[0] = 0;
	
	for (int i = 1; i <= PARAMETERS;i++)
	{
		if ((params_buf[i] < (uint16_t) menus[i-1].min) || (params_buf[i] > (uint16_t) menus[i-1].max)) params_buf[i] = menus[i-1].def;
	}
	
	Init_Protections();
	Init_Joystick();
	
	write_to_flash(&params_buf[0]);
}

void UpdateStatus()
{
	bool write_params = false;
	uint16_t temp_curr = 0;
	
	main_data.Left_On = Left.moving_bwd | Left.moving_fwd; 
	main_data.Right_On = Right.moving_bwd | Right.moving_fwd;
     	

	i2c_data.statusbyte0 = ((main_data.Left_On) << STATUS_BIT0_L_ON) | 
												 ((main_data.Right_On)  << STATUS_BIT0_R_ON) | 
		                      (sw_leftdown.outstate << STATUS_BIT0_L_D) | 
	                        (sw_leftup.outstate << STATUS_BIT0_L_U) | 
		                      (sw_rightdown.outstate << STATUS_BIT0_R_D) | 
	                        (sw_rightup.outstate << STATUS_BIT0_R_U) | 
	                        (in_left.outstate << STATUS_BIT0_L_BUT) | 
	                        (in_right.outstate << STATUS_BIT0_R_BUT);
        
	i2c_data.statusbyte1 = (1 << STATUS_BIT1_L_EN) | (1 << STATUS_BIT1_R_EN) | (Left.moving_fwd << STATUS_BIT1_L_ON_U) | (Left.moving_bwd << STATUS_BIT1_L_ON_D) | 
																		 (Right.moving_fwd << STATUS_BIT1_R_ON_U) | (Right.moving_bwd << STATUS_BIT1_R_ON_D) |
																		 ((valLn.trip | valLp.trip) << STATUS_BIT1_L_TRIP_STAGE1) |
																		 ((valRn.trip | valRp.trip) << STATUS_BIT1_R_TRIP_STAGE1)  ;
		
	i2c_data.statusbyte2 = ((oc_valLn.trip | oc_valLp.trip) << STATUS_BIT2_L_TRIP_STAGE2) | ((oc_valRn.trip | oc_valRp.trip) << STATUS_BIT2_R_TRIP_STAGE2) | 
		              (sw_squeeze.outstate << STATUS_BIT2_SQUEEZE) | (sw_water.outstate << STATUS_BIT2_WATERMODE);		
	
	

	i2c_buf[1] = i2c_data.statusbyte0;
	i2c_buf[2] = i2c_data.statusbyte1;
	i2c_buf[3] = i2c_data.statusbyte2;
	i2c_buf[4] = main_data.Left_Current;
	i2c_buf[5] = main_data.Right_Current;
	i2c_buf[6] = i2c_data.current_page;
	
	
	// current parameters at page						 
	for (int i=0; i<5; i++)
	{
		if ((i2c_data.current_page > 0) && (i2c_data.current_page <= 9))
		{      
		  int index = i+1+(i2c_data.current_page-1)*5;
			
			if (index <= PARAMETERS)
			    i2c_buf[7+i] = params_buf[index];
			else
			    i2c_buf[7+i] = 0;
		}
		else
			 i2c_buf[7+i] = 0;
	}
	
	if ((i2c_data.paramfromdisp_addr > 0) && (i2c_data.paramfromdisp_addr <= PARAMETERS))
	{
		if (i2c_data.paramfromdisp >= menus[i2c_data.paramfromdisp_addr-1].min)
		if (i2c_data.paramfromdisp <= menus[i2c_data.paramfromdisp_addr-1].max)
		{
			write_params = (params_buf[i2c_data.paramfromdisp_addr] != (uint16_t) i2c_data.paramfromdisp);
				
			params_buf[i2c_data.paramfromdisp_addr] = (uint16_t) i2c_data.paramfromdisp;
			
			if (write_params)	
			{
				WriteParamsToFlash();  			
			}
			
		}
	}
	params_buf[4] = an_L.in_value;
	params_buf[5] = Left.goal_step*10+Left.curr_step;
	params_buf[6] = an_L.enabled;
	
	params_buf[7] = an_R.in_value;
	params_buf[8] = Right.goal_step*10+Right.curr_step;
	params_buf[9] = an_R.enabled;	
	
	temp_curr = (pa[2].out_value+pa[3].out_value)*10/CURRENT_COEFFICIENT;
	if (temp_curr > 0xFF) temp_curr = 0xFF;
	
	main_data.Left_Current = temp_curr;
	
	temp_curr = (pa[0].out_value+pa[1].out_value)*10/CURRENT_COEFFICIENT;
	if (temp_curr > 0xFF) temp_curr = 0xFF;
	
	main_data.Right_Current = temp_curr;

}

void Process_Channels()
{
  uint8_t step = 0;
		
    if ((!valLn.trip) && (!valLp.trip) && (!oc_valLn.trip) && (!oc_valLp.trip))
		{
			if (sw_squeeze.off_trigger) //SQEEZE switch input
			{
				sq_off_left = true; //return drives to start position on negative edge SQUEEZE input
				sq_off_right = true;
				sw_squeeze.off_trigger = false;
			}
			
			if (in_left.outstate && !(params_buf[40] && in_right.outstate))  // Button L process
			{
				Left.control_source = true;
				Left.goal_step = 4;			
			
        			
			}	
			else   //Joystick Process
			{
				
				if (Left.control_source)
				{
				    Left.goal_step = 0;
				    
					if ((Left.curr_step == 0) && (!Left.moving_bwd)) Left.control_source = false;
					 
				} else
				{
					if (an_L.enabled)
					{
						step = an_L.joystick_step;
					} else
						step = an_R.mirror_joy_step;
						
						
					if (an_L.enabled || an_R.enabled)
					if ((step > 0) || (Left.curr_step > 0) || Left.moving_bwd || Left.moving_fwd )
					{
							Left.goal_step = step;
							
					}
						
				}		
				
			}
			

			if (in_left.on_trigger)
			{
				Left.lock = false;
				in_left.on_trigger = false;
			}
			
			if ((!Left.prev_lock_state) && Left.lock && (!in_right.outstate)) Left.lock = false; 
			
			Left.prev_lock_state = Left.lock;
					
			Left.water_mode = sw_water.outstate;
			Left.long_release = sw_squeeze.outstate;
			
			
			
			
			if (sw_squeeze.outstate) {
				
				Left.control_source = true;
				Left.goal_step = 4;

			}
			
			
      if (sq_off_left){
				Left.goal_step = 0;
				sq_off_left = false;
			}
			
			
			Left.hold_on_dc = params_buf[36];
			Left.normal_mode_dc = params_buf[38];
			Left.water_dc = params_buf[39];
			
			Left.Process(); // Process L channel
			
		} 


      if ((!valRn.trip) && (!valRp.trip) && (!oc_valRn.trip) && (!oc_valRp.trip))
      {
			
			if (in_right.outstate && !(params_buf[40] && in_left.outstate))  // Button R process
			{
				Right.control_source = true;
				Right.goal_step = 4;				
			}	
			else   //Joystick Process
			{
				if (Right.control_source)
				{
				    Right.goal_step = 0;
				    
					if ((Right.curr_step == 0) && (!Right.moving_bwd)) Right.control_source = false;
				} else
				{
					if (an_R.enabled)
					{
						step = an_R.joystick_step;
					} else
						step = an_L.mirror_joy_step;
						
						
					if (an_L.enabled || an_R.enabled)
					if ((step > 0) || (Right.curr_step > 0) || Right.moving_bwd || Right.moving_fwd)
					{
							Right.goal_step = step;		
					}
						
				  
				}			
			}

			if (in_right.on_trigger)
			{
				Right.lock = false;
				in_right.on_trigger = false;
			}
			
			if ((!Right.prev_lock_state) && Right.lock && (!in_left.outstate)) Right.lock = false;
			
			Right.prev_lock_state = Right.lock;
			
			Right.water_mode = sw_water.outstate;
			Right.long_release = sw_squeeze.outstate; 

			if (sw_squeeze.outstate) {
				Right.control_source = true;
				Right.goal_step = 4; 
			}
			if (sq_off_right)
			{
				Right.goal_step = 0;
				sq_off_right = false;
			}
			
			
			Right.hold_on_dc = params_buf[37];
			Right.normal_mode_dc = params_buf[38];
			Right.water_dc = params_buf[39];
			
			Right.Process(); // Process R channel		
			
		}
}


void CANsendmsg()
{
	CAN_TxHeaderTypeDef msgHeader;
  uint8_t msgData[8];

	if (params_buf[41] == 0) return;
	
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 0)
  {

    msgHeader.StdId = params_buf[42]; //CAN bus node address
    msgHeader.DLC = 8;
    msgHeader.TransmitGlobalTime = DISABLE;
    msgHeader.RTR = CAN_RTR_DATA;
    msgHeader.IDE = CAN_ID_STD;
    
    uint32_t mailBoxNum = 0;
		
		msgData[0] = i2c_data.statusbyte0;
		msgData[1] = i2c_data.statusbyte1;
		msgData[2] = i2c_data.statusbyte2;
		msgData[3] = main_data.Left_Current;
		msgData[4] = main_data.Right_Current;

    
    HAL_CAN_AddTxMessage(&hcan, &msgHeader, &msgData[0], &mailBoxNum);
  }
}


void ProcessDIevents()
{
	
//Base PCB (with USB)
//Button_L  PB10 
//Button_R  PA10
//SW_L_Down PB15
//SW_L_Up   PB14
//SW_R_Down PB13
//SW_R_Up   PB12	
//Water     PA9		
//Squeeze   PA8
//Mode 1    PA15	
//Mode 2    PB11
	
	if (in_left.outstate) //Button L
	{
		main_data.Left_button = 1;
		
	} else
	{
		main_data.Left_button = 0;
    
	}

	if (in_right.outstate) //Button R
	{
		main_data.Right_button = 1;
	} else
	{
		main_data.Right_button = 0;
    
	}	
	
	//in_left, in_right, sw_leftdown, sw_leftup, sw_rightdown, sw_rightup, sw_water, sw_squeeze, sw_mode1, sw_mode2; 
	
	if (sw_mode1.outstate) //DI Mode 1
	{
    main_data.DI_mode1 = 1;  
	}
	else
	{
		main_data.DI_mode1 = 0;
	}

	if (sw_mode2.outstate) //DI Mode 2
	{
    main_data.DI_mode2 = 1;  
	}
	else
	{
		main_data.DI_mode2 = 0;
	}
	
	if (sw_mode2.outstate) //Squeeze
	{
		 main_data.Squeeze_mode = 1;				
	} 
	else
	{
		 main_data.Squeeze_mode = 0;	
	}
	
	
	if (sw_water.outstate) //Water
	{
		main_data.Water_mode = 1;
	} else
	{
		main_data.Water_mode  = 0;
	}


	if (sw_leftdown.outstate) //SW_L_Down
	{
		main_data.SW_L_state = 1;
	} else
	if (sw_leftup.outstate) //SW_L_Up
	{
		main_data.SW_L_state = 2;
	}	
	else
	  main_data.SW_L_state = 0;
	
	if (sw_rightdown.outstate) //SW_R_Down
	{
		main_data.SW_R_state = 1;
	} else
	if (sw_rightup.outstate) //SW_R_Up
	{
		main_data.SW_R_state = 2;
	}	
	else
	  main_data.SW_R_state = 0;
	
	
}

void DO_Error(bool state)
{
  if (state) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
	else
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
}

void DO_Mode1(bool state)
{
  if (state) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	else
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}

float expRunningAverageAdaptive(float newVal) {
  static float filVal = 0;
  float k;

  if (abs(newVal - filVal) > 1.5) k = 0.9;
  else k = 0.03;
  
  filVal += (newVal - filVal) * k;
  return filVal;
}


float simpleKalman(float newVal) {
  float _kalman_gain, _current_estimate;
  static float _err_estimate = _err_measure;
  static float _last_estimate;
  _kalman_gain = (float)_err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + (float)_kalman_gain * (newVal - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;
  return _current_estimate;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t value = 0;
	bool tmp = false;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
	MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */


	res_addr = flash_search_adress(STARTADDR, PARAMETERS * DATAWIDTH);
	read_last_data_in_flash(&params_buf[0]); 
	 
	if (params_buf[0] == 0xFFFF)
	{
		erase_flash();
		HAL_Delay(5);
		WriteDefsToFlash();
	}
	
	
	if (!CheckParamsLimits(&params_buf[0]))
	{
		HAL_Delay(5);
		WriteDefsToFlash();	
	}	 
	
  MX_CAN_Init();
  MX_CRC_Init();

	for (int j=0; j<8; j++)
	     timers[j] = 0;	
	
	InitMainScreenSet();
	//InitPWM();
	Init_i2c_data();
	InitAnalogPins();
  Init_Channels();
  //UpdateChannelsData();
  Init_Protections();
	Init_Switches();
	Init_Joystick();
	
  CAN_FilterTypeDef canFilterConfig;
  canFilterConfig.FilterBank = 0;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterIdHigh = 0x0000;
  canFilterConfig.FilterIdLow = 0x0000;
  canFilterConfig.FilterMaskIdHigh = 0x0000;
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canFilterConfig.FilterActivation = ENABLE;
  canFilterConfig.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan, &canFilterConfig);
  
	
	if (params_buf[18] > 0)
	{
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  }

	HAL_TIM_Base_Start_IT(&htim3);		
	HAL_TIM_Base_Start_IT(&htim4);	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		HAL_IWDG_Refresh(&hiwdg);
		
		if (in_left.protection_reset && (valLp.trip || oc_valLp.trip || valLn.trip || oc_valLn.trip)) {
			valLp.trip = false;
			oc_valLp.trip = false;
			valLn.trip = false;
			oc_valLn.trip = false;
			Left.InitChannel();
			in_left.protection_reset = false;
		} 
		
		// DO Processing
		if (params_buf[43] == 0) DO_Error(valLp.trip || oc_valLp.trip || valLn.trip || oc_valLn.trip ||
			                                valRp.trip || oc_valRp.trip || valRn.trip || oc_valRn.trip);
		if (params_buf[43] == 1) DO_Error(true);
		if (params_buf[43] == 2) DO_Mode1(true); else DO_Mode1(false);
		


		if (in_right.protection_reset && (valRp.trip || oc_valRp.trip || valRn.trip || oc_valRn.trip)) {
			valRp.trip = false;
			oc_valRp.trip = false;
			valRn.trip = false;
			oc_valRn.trip = false;
			Right.InitChannel();
			in_right.protection_reset = false;
		}
		
		value = GetADC1_Value(ADC_CHANNEL_0);
		// Fast overcurrent protection
		ProtectionProcess(ADC_CHANNEL_0,value);
		//Average RMS of current value for jamming protection
		AnalogAddValue(&pa[0], value);  //MR-
		
		value = GetADC1_Value(ADC_CHANNEL_1);
		ProtectionProcess(ADC_CHANNEL_1, value);
		AnalogAddValue(&pa[1], value);  //MR+
		
		value = GetADC1_Value(ADC_CHANNEL_2);
		ProtectionProcess(ADC_CHANNEL_2, value);
		AnalogAddValue(&pa[2], value);  //ML-
		
		value = GetADC1_Value(ADC_CHANNEL_3);
		ProtectionProcess(ADC_CHANNEL_3, value);
		AnalogAddValue(&pa[3], value); //ML+
		
		
		Process_Channels();
		

		
		if ((HAL_GetTick() - timers[0]) > 100) //0.1 sec
		{ 
			UpdateStatus();
			ProcessDIevents();
			
			timers[0] = HAL_GetTick();	
		}
		
		

		
		if ((HAL_GetTick() - timers[1]) > 150) //0.15 sec
		{	
			
			HAL_I2C_Master_Transmit(&hi2c1, (I2C_SLAVE_ADDRESS << 1), &i2c_buf[0], I2C_BUFFER,  I2C_TIMEOUT);
			
			CANsendmsg();
			timers[1] = HAL_GetTick();	
		}
		
		
		if ((HAL_GetTick() - timers[2]) > 10) //every 10 ms
		{			
			AnalogAverage(&pa[0]);
			valRn.in_value = pa[0].out_value;  //MR-
			AnalogAverage(&pa[1]);
			valRp.in_value = pa[1].out_value;  //MR+
			AnalogAverage(&pa[2]);
			valLn.in_value = pa[2].out_value;  //ML-
			AnalogAverage(&pa[3]);
			valLp.in_value = pa[3].out_value;  //ML+
			

			
			
			timers[2] = HAL_GetTick();
		}

		
		if ((HAL_GetTick() - timers[3]) > 50) //50 msec
		{
			an_L.in_value = simpleKalman(GetADC1_Value(ADC_CHANNEL_4)/41);     //X_L  /4096*100%
      an_R.in_value = simpleKalman(GetADC1_Value(ADC_CHANNEL_5)/41); 	   //X_R  /4096*100%		

	    _joystick_filter(&an_L);
	    _joystick_filter(&an_R);
			 
			timers[3] = HAL_GetTick();
		}
		
		
		if ((HAL_GetTick() - timers[4]) > 500) //0.50 sec
		{
					
			HAL_I2C_Master_Receive_IT(&hi2c1, (I2C_SLAVE_ADDRESS << 1), &i2c_receive[0], I2C_BUFFER);
			i2c_data.current_page 			= i2c_receive[1];
			i2c_data.paramfromdisp 			= i2c_receive[2];
			i2c_data.paramfromdisp_addr = i2c_receive[3];
				
			
			
			timers[4] = HAL_GetTick();
			
		} 

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */
	
	if (params_buf[41] > 0)
	  hcan.Init.Prescaler = params_buf[41]; //params_buf[18] = 6 (125kbps), 3 (250kbps), 2 (375kbps), 1 (750kbps)
	else 
		hcan.Init.Prescaler = 6;

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_7TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Reload = 325;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 240;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 100;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 75;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}



/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB10 PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}



/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
