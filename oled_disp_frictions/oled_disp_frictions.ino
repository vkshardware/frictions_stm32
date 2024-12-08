#define F_CPU 8000000UL

#include <GyverOLED.h>
#include "GyverOLEDMenu_mod.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include "util.h"
#include "icons.h"


#define TWI_SLAVEADDR	0x67  //I2C Led Display

#define KEY1 1 << PC3    // KEY1 up button, arduino pin 17
#define KEY2 1 << PC2    // KEY2 enter button, arduino pin 16
#define KEY3 1 << PC1    // KEY3 down button, arduino pin 15

#define LED1 1 << PD0    // LED1 left, arduino pin 0
#define LED2 1 << PD1    // LED2 right, arduino pin 1

#define UPDATE_DISPLAY 5
#define KEY_PUSH_DELAY 3
#define KEY_LONGPUSH_DELAY 25
#define BLINK_BUS_DELAY 12

#define WAIT_100HZ 0xB1  // Timer2 100Hz

#define PARAMETERS 42



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
#define STATUS_BIT1_L_TRIP_STAGE1 6   // L TRIP current protection
#define STATUS_BIT1_R_TRIP_STAGE1 7   // R TRIP current protection

//status byte 2

#define STATUS_BIT2_L_TRIP_STAGE2 0  // L TRIP over current protection short circuit
#define STATUS_BIT2_R_TRIP_STAGE2 1  // R TRIP over current protection short circuit
#define STATUS_BIT2_WATERMODE 2		 // Water Mode
#define STATUS_BIT2_SQUEEZE 3        // Release both sides
#define STATUS_BIT2_HOLD_ON_LOAD 4   // hold drive under load

#define KEY_PUSH_DELAY 3
#define KEY_LONGPUSH_DELAY 25

#define STR_LENGTH 15*2 // 2 bytes per symbol
#define I2C_REG_SIZE 9
#define MENUS_AT_PAGE 6

struct SwitchFilter{
	bool outstate;
	bool long_outstate;
	bool prev;
	unsigned char scanrate;
	unsigned char long_scanrate;
	volatile unsigned char *port;
	unsigned char pin;
	unsigned char curr_scan;
  unsigned char lock;
};

SwitchFilter Key1, Key2, Key3;


struct MainScreenSet{
  bool Water_mode;
  bool Squeeze_mode;
  bool Hold_on;
  bool Left_button;
  bool Right_button;
  uint8_t Right_Switches; // Empty, DOWN, UP, DOWN+UP
  uint8_t Left_Switches; // Empty, DOWN, UP, DOWN+UP
  uint8_t Right_Movement; // 0 - stop, 1 - move down, 2 - move up
  uint8_t Left_Movement; // 0 - stop, 1 - move down, 2 - move up
  uint8_t Right_fault; // 0 - normal, 1 - F1, 2 - F2
  uint8_t Left_fault; // 0 - normal, 1 - F1, 2 - F2
};

struct MenuRow{
  uint8_t id;
  char desc[STR_LENGTH];
  uint8_t def;
  uint8_t min;
  uint8_t max;
  uint8_t writable; // 1- writable, 0 - readonly
  int8_t dp; //decimal point position
  uint8_t symb; // 0 - null, 1 - seconds, 2 - %, 3- A
};

const MenuRow menus[] PROGMEM = {
  {1,"01 Вр. выжима",9,3,100,1,1,1},
  {2,"02 Вр. возвр",20,3,100,1,1,1},
  {3,"03 Вр. дв. вв.",10,0,100,1,1,1},
  {4,"04 Дж. лев.",0,0,0,0,0,2},
  {5,"05 Кмд. лев.",0,0,0,0,0,0},
  {6,"06 X_L подкл.",0,0,0,0,1,0},
  {7,"07 Дж. пр.",0,0,0,0,0,2},
  {8,"08 Кмд. пр.",0,0,0,0,1,0},
  {9,"09 X_R подкл.",0,0,0,0,0,0},
  {10,"10 X_L ниж.",10,0,99,1,0,2},
  {11,"11 X_L 4 ст.",25,0,99,1,0,2},
  {12,"12 X_L 3 ст.",30,0,99,1,0,2},
  {13,"13 X_L 2 ст.",35,0,99,1,0,2},
  {14,"14 X_L 1 ст.",40,0,99,1,0,2},
  {15,"15 X_L 1 рез.",61,0,99,1,0,2},
  {16,"16 X_L 2 рез.",66,0,99,1,0,2},
  {17,"17 X_L 3 рез.",71,0,99,1,0,2},
  {18,"18 X_L 4 рез.",76,0,99,1,0,2},
  {19,"19 X_L верх.",90,0,99,1,0,2},
  {20,"20 X_R ниж.",10,0,99,1,0,2},
  {21,"21 X_R 4 ст.",25,0,99,1,0,2},
  {22,"22 X_R 3 ст.",30,0,99,1,0,2},
  {23,"23 X_R 2 ст.",35,0,99,1,0,2},
  {24,"24 X_R 1 ст.",40,0,99,1,0,2},
  {25,"25 X_R 1 рез.",61,0,99,1,0,2},
  {26,"26 X_R 2 рез.",66,0,99,1,0,2},
  {27,"27 X_R 3 рез.",71,0,99,1,0,2},
  {28,"28 X_R 4 рез.",76,0,99,1,0,2},
  {29,"29 X_R верх.",90,0,99,1,0,2},
  {30,"30 Iзакл. лев.",10,1,50,1,0,3},
  {31,"31 Tзакл. лев.",10,1,99,1,-1,1},
  {32,"32 Iзакл. пр.",10,1,50,1,0,3},
  {33,"33 Tзакл. пр.",10,1,99,1,-1,1},
  {34,"34 Iкз. лев.",45,0,80,1,0,3},
  {35,"35 Iкз. пр.",45,0,80,1,0,3},
  {36,"36 Удерж. лев.",0,0,10,1,-1,2},
  {37,"37 Удерж. пр.",0,0,10,1,-1,2},
  {38,"38 Возвр. норм.",10,1,10,1,-1,2},
  {39,"39 Возвр. вода",4,1,10,1,-1,2},
  {40,"40 Спец. лог.",0,0,1,1,0,0},
  {41,"41 CAN скор.",4,0,6,1,0,0},
  {42,"42 CAN адр.",0x11,0x01,0xFE,1,0,0}, 
};

MainScreenSet main_oled;

struct i2c_data_{
  uint8_t statusbyte0;
  uint8_t statusbyte1;
  uint8_t statusbyte2;
  uint8_t paramtodisp;
  uint8_t paramtodisp_addr;
  uint8_t paramfromdisp;
  uint8_t paramfromdisp_addr;
};

i2c_data_ i2c_data;
char i2c_buf[I2C_REG_SIZE];

GyverOLED<SSH1106_128x64, OLED_BUFFER, OLED_SPI, 10, 8, 9> oled;

OledMenu<6, GyverOLED<SSH1106_128x64, OLED_BUFFER, OLED_SPI, 10, 8, 9>> menu(&oled);

char param_text[MENUS_AT_PAGE][STR_LENGTH+2*4];
uint8_t params_buf[MENUS_AT_PAGE];
uint8_t params_min[MENUS_AT_PAGE];
uint8_t params_max[MENUS_AT_PAGE];
int8_t params_dp[MENUS_AT_PAGE];
uint8_t params_symb[MENUS_AT_PAGE];

int tmp = 0;

bool next_state = false;
uint8_t timer2_count = 0;
bool menu_visible = false;
uint8_t old_param_value = 0;

int menu_index, selected_menu = 0;
int menu_page = 1;
int val = 0;

bool Setup_param = false;
uint8_t requested_menu = 1;
bool menu_received_ack = 0;



void Init_i2c_data()
{
  i2c_data.statusbyte0 = 0;
  i2c_data.statusbyte1 = 0;
  i2c_data.statusbyte2 = 0;
  i2c_data.paramfromdisp = 0;
  i2c_data.paramfromdisp_addr = 0;
  i2c_data.paramtodisp_addr = 0;
  i2c_data.paramtodisp = 0;
}

void InitMainScreenSet()
{
  uint8_t i = 0;

  main_oled.Left_button = false;
  main_oled.Left_Switches = 0;

  main_oled.Right_button = false;
  main_oled.Right_Switches = 0;

  main_oled.Squeeze_mode = false;
  main_oled.Water_mode = false;
  main_oled.Left_Movement = 0;
  main_oled.Right_Movement = 0;
  main_oled.Left_fault = 0;
  main_oled.Right_fault = 0;
  main_oled.Hold_on = 0;

}

void print_friction(uint8_t side, uint8_t state)
{
  uint8_t x, y = 0;
 
  y = 14;

  if (side == 0) x = 50; else
  if (side == 1) x = 101;


  switch(state)
  {
    case 0: break;
    case 1: oled.drawBitmap(x,y,friction_full,16,26);
            break;
    case 2: break;
    case 3: oled.drawBitmap(x,y,friction_updown,16,26);
            break;          
  }
}

void print_arrow(uint8_t side, uint8_t state)
{
  uint8_t x, y = 0;
 
  y = 22;

  if (side == 0) x = 39; else
  if (side == 1) x = 123;


  switch(state)
  {
    case 0: break;
    case 1: oled.drawBitmap(x,y,up_arrow,5,14);
            break;
    case 2: oled.drawBitmap(x,y,down_arrow,5,14);
            break;          
  }
}

bool _switch_filter(SwitchFilter * source)
{
	bool result;
	
	result = false;
	
	
	if (!((*source->port) & source->pin))
	{
		if (source->curr_scan >= source->scanrate){
			source->prev = true;
		} 
		
    	if (source->curr_scan >= source->long_scanrate){
			source->prev = false;	
			source->long_outstate = true;					
       } else
       source->curr_scan++;
		
		
		if  (source->scanrate == 0xFF) source->curr_scan = 0;
		
	} else
	{
		if (source->prev) result = true;
		
		source->prev = false;		
		source->long_outstate = false;
		source->curr_scan = 0;
	}
	
	
	return result;
}


ISR (TIMER2_OVF_vect) // 100 Hz
{	
  Key1.outstate = _switch_filter(&Key1);
  Key2.outstate = _switch_filter(&Key2);
  Key3.outstate = _switch_filter(&Key3);

  timer2_count  +=1;


	TCNT2 = WAIT_100HZ;

}

void drawIcon8x8(byte index) {
  size_t s = sizeof icons_8x8[index];  //можна так, а можна просто 8 
  for(unsigned int i = 0; i < s; i++) {
    oled.drawByte(pgm_read_byte(&(icons_8x8[index][i])));
  }
}



void MainScreen(MainScreenSet * screen)
{
  oled.clear();
  oled.drawBitmap(0,0, gaz71, 128, 64);

  oled.setCursorXY(70, 56); 
  oled.setScale(1);
  oled.print("GAZ-71");

   // left clutch state
  
  switch(screen->Left_Switches)
  {
     case 0:  break;
     case 1:  oled.setCursorXY(43, 56); 
              oled.print("Down");
              break;
     case 2:  oled.setCursorXY(50, 56); 
              oled.print("Up");
              break;
     case 3:  oled.setCursorXY(45, 56);
              oled.print("D+U"); 
              break;    
  }
  
  oled.setCursorXY(53, 22); 

  switch(screen->Left_fault)
  {
     case 0:  print_friction(0, screen->Left_Switches);
              break;
     case 1:  oled.print("F1");
              break;
     case 2:  oled.print("F2");
              break;
  }

  print_arrow(0,screen->Left_Movement);


   // right clutch state

  switch(screen->Right_Switches)
  {
    case 0:  break;
    case 1:   oled.setCursorXY(105, 56);
              oled.print("Down");
              break;
     case 2:  oled.setCursorXY(108, 56);
              oled.print("Up");
              break;
     case 3:  oled.setCursorXY(105, 56);
              oled.print("D+U");
              break;    
  }
  
  oled.setCursorXY(104, 22); 

  switch(screen->Right_fault)
  {
     case 0:  print_friction(1, screen->Right_Switches);
              break;
     case 1:  oled.print("F1");
              break;
     case 2 ... 3:  oled.print("F2");
              break;
  }
  
  
  print_arrow(1, screen->Right_Movement);
  
  if (screen->Left_button)  // left letter (button)
  {
    oled.setCursorXY(7, 5); 
    oled.print("L");
    oled.circle(8,8,8,OLED_STROKE);
  }

  if (screen->Right_button)
  {
    oled.setCursorXY(29, 5); // rigth letter (button)
    oled.print("R"); 
    oled.circle(30,8,8,OLED_STROKE);
  }

  if (screen->Left_fault || screen->Right_fault)  //Global fault
  {
    oled.setCursorXY(0, 28);
    oled.print("FAULT");
  }


  if (screen->Water_mode)  //Water mode
  {
    oled.setCursorXY(0, 42);
    oled.print("WATER");
  }

  if (screen->Squeeze_mode)  //Squeeze mode
  {
    oled.setCursorXY(0, 56);
    oled.print("SQUEEZE");
  }

  oled.update();
}

void receiveEvent(int howMany) {
  
  uint8_t i = 0;

  while (0 < Wire.available()) { 
    i2c_buf[i] = Wire.read();
    if (i < (I2C_REG_SIZE-1)) i++;   
    
  }
    
  i2c_data.statusbyte0 = i2c_buf[1];
  i2c_data.statusbyte1 = i2c_buf[2];
  i2c_data.statusbyte2 = i2c_buf[3];
  i2c_data.paramtodisp = i2c_buf[4];
  i2c_data.paramtodisp_addr = i2c_buf[5];

}

void requestEvent() {

  if (menu_visible) 
  { 
    i2c_buf[0] = 0;
    i2c_buf[1] = requested_menu+(menu_page-1)*5;
    i2c_buf[2] = i2c_data.paramfromdisp;
    i2c_buf[3] = i2c_data.paramfromdisp_addr;
    requested_menu++;  
  } else
  {
    requested_menu = 0;
    i2c_buf[1] = 0;
    i2c_buf[2] = 0;
    i2c_buf[3] = 0;
  }



  Wire.write(&i2c_buf[0],I2C_REG_SIZE);



  if (requested_menu > 5) 
  {
    requested_menu = 1;
  } 

}

uint8_t BuffAddrFromDataAddr(uint8_t data_addr)
{ 
  if (data_addr == 0) return 0;

  int8_t result = data_addr % 5;
  if (result == 0) result = 5; 

  return result-1;
} 

void UpdateParams()
{
  int i,j = 0;

  if ((requested_menu > 0) && (requested_menu < 6))
      params_buf[BuffAddrFromDataAddr(i2c_data.paramtodisp_addr)] = i2c_data.paramtodisp; 

  for (i = 0; i < 5; i++)
  {
    if (menu_page > 0)
    j=i+((menu_page-1)*5); else
    j = 0;
    params_min[i] = pgm_read_byte(&menus[j].min);
    params_max[i] = pgm_read_byte(&menus[j].max);
    params_dp[i] =  (int8_t) pgm_read_byte(&menus[j].dp);
    params_symb[i] = pgm_read_byte(&menus[j].symb);

    //params_buf[i]=pgm_read_byte(&menus[j].def);
  }
      
}



void UpdateTextBuf()
{
  uint8_t i, j, id = 0;

  for (i = 0; i < 5; i++)
  {
    if (menu_page > 0)
       j=i+((menu_page-1)*5); else
       j = 0;

    //id = pgm_read_byte(&menus[j].id); 

    if (j > PARAMETERS) strcpy(&param_text[i][0], "");
    else

    strlcpy_P(&param_text[i][0], menus[j].desc,STR_LENGTH);


  }
}


void BuildMenu(void)
{ 
  int tmp,i,j = 0;
 
  UpdateTextBuf();
  UpdateParams();
  
  
  menu.addItem("<--  ВЫХОД"); 
  for (i = 0; i < 5; i++)
  {
    //params_buf[i]=pgm_read_byte(&menus[i].def);
    menu.addItem(&param_text[i][0], 1, &params_buf[i],&params_min[i], &params_max[i], &params_dp[i], &params_symb[i]); 
  }

}

void Update_status()
{
  bool mov_l, mov_r = false;

  main_oled.Left_button = i2c_data.statusbyte0 & (1 << STATUS_BIT0_L_BUT);
  main_oled.Right_button = i2c_data.statusbyte0 & (1 << STATUS_BIT0_R_BUT);
  main_oled.Water_mode = i2c_data.statusbyte2 & (1 << STATUS_BIT2_WATERMODE);
 
  if (i2c_data.statusbyte0 & (1 << STATUS_BIT0_L_ON))
  PORTD |= LED1;
  else
  PORTD &= ~(LED1);
 

  if (i2c_data.statusbyte0 & (1 << STATUS_BIT0_R_ON))
  PORTD |= LED2;
  else
  PORTD &= ~(LED2); 

  main_oled.Left_Movement = 0;
  if (i2c_data.statusbyte1 & (1 << STATUS_BIT1_L_ON_U)) main_oled.Left_Movement+=1;
  if (i2c_data.statusbyte1 & (1 << STATUS_BIT1_L_ON_D)) main_oled.Left_Movement+=2;  

  main_oled.Right_Movement = 0;
  if (i2c_data.statusbyte1 & (1 << STATUS_BIT1_R_ON_U)) main_oled.Right_Movement+=1;
  if (i2c_data.statusbyte1 & (1 << STATUS_BIT1_R_ON_D)) main_oled.Right_Movement+=2;  

  main_oled.Left_Switches=0;
  if (i2c_data.statusbyte0 & (1 << STATUS_BIT0_L_D)) main_oled.Left_Switches+=1;
  if (i2c_data.statusbyte0 & (1 << STATUS_BIT0_L_U)) main_oled.Left_Switches+=2;

  main_oled.Right_Switches=0;
  if (i2c_data.statusbyte0 & (1 << STATUS_BIT0_R_D)) main_oled.Right_Switches+=1;
  if (i2c_data.statusbyte0 & (1 << STATUS_BIT0_R_U)) main_oled.Right_Switches+=2;

  main_oled.Left_fault = 0;
  if (i2c_data.statusbyte1 & (1 << STATUS_BIT1_L_TRIP_STAGE1)) main_oled.Left_fault+=1;
  if (i2c_data.statusbyte2 & (1 << STATUS_BIT2_L_TRIP_STAGE2)) main_oled.Left_fault+=2;  

  main_oled.Right_fault = 0;
  if (i2c_data.statusbyte1 & (1 << STATUS_BIT1_R_TRIP_STAGE1)) main_oled.Left_fault+=1;
  if (i2c_data.statusbyte2 & (1 << STATUS_BIT2_R_TRIP_STAGE2)) main_oled.Left_fault+=2;  
        
  main_oled.Squeeze_mode=i2c_data.statusbyte2 & (1 << STATUS_BIT2_SQUEEZE);  
  main_oled.Hold_on = i2c_data.statusbyte2 & (1 << STATUS_BIT2_HOLD_ON_LOAD); 

}

void setup() {

  TCCR2A = 0; 
  TCCR2B = (1 << CS22)|(1 << CS21)|(1 << CS20); //1024 divider
		
	TCNT2 = WAIT_100HZ;
		
	TIFR2 = (1<<TOV2);
	TIMSK2 = (1<<TOIE2);

  DDRD = LED1 | LED2;
  
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = (1<<WDE) | (1<<WDP2) | (1<<WDP1);

  init_switches();
  sei();

  Wire.setClock(100000);
  Wire.begin(TWI_SLAVEADDR); 
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);


  oled.init();
  oled.clear(); 
  oled.update();

  Init_i2c_data();

  InitMainScreenSet();

  MainScreen(&main_oled);
  
  menu.onChange(onItemChange, true);
  menu.onPrintOverride(onItemPrintOverride); 

  BuildMenu();
  
  menu_visible = false;
  menu.showMenu(menu_visible); 


}

void onItemChange(const int index, const void* val, const byte valType) {
  
  if (valType == VAL_ACTION) {
    if (index == 0) {
      menu_visible = false;
      menu.showMenu(menu_visible);
      MainScreen(&main_oled);
    }
  }


}

boolean onItemPrintOverride(const int index, const void* val, const byte valType) {

  if (index > 0)
  if (!((index >=3) && (menu_page == 9)))
      return false;
      

  return true;
} 

void init_switches(){
	
	Key1.outstate=false;
	Key1.long_outstate = false;
	Key1.curr_scan=0;
	Key1.scanrate = KEY_PUSH_DELAY;
	Key1.long_scanrate = KEY_LONGPUSH_DELAY;
	Key1.prev = false;
	Key1.port = &PINC;
	Key1.pin = KEY1;
  Key1.lock = 0;
	
	Key2.outstate=false;
	Key2.long_outstate = false;
	Key2.curr_scan=0;
	Key2.scanrate=KEY_PUSH_DELAY;
  Key2.long_scanrate = KEY_LONGPUSH_DELAY;
	Key2.prev = false;
	Key2.port = &PINC;
	Key2.pin = KEY2;
  Key2.lock = 0;

  Key3.outstate=false;
	Key3.long_outstate = false;
	Key3.curr_scan=0;
	Key3.scanrate=KEY_PUSH_DELAY;
  Key3.long_scanrate = KEY_LONGPUSH_DELAY;
	Key3.prev = false;
	Key3.port = &PINC;
	Key3.pin = KEY3;
  Key3.lock = 0;
}



void menu_page_select(int index)
{
  switch (index) {
    
    case 0 ... 5:   menu_page = 1;
                    break;
    case 6 ... 11:  menu_page = 2;
                    break;
    case 12 ... 17: menu_page = 3;
                    break;
    case 18 ... 23: menu_page = 4;
                    break;
    case 24 ... 29: menu_page = 5;
                    break;       
    case 30 ... 35: menu_page = 6;
                    break;
    case 36 ... 41: menu_page = 7;
                    break; 
    case 42 ... 47: menu_page = 8;
                    break;             
    case 48 ... 51: menu_page = 9;
                    break;  
  }

}

void DownButton()
{
  if (menu_visible)
  {
  
        if (menu_index < PARAMETERS+8) //PARAMETERS+9-1. 9 EXIT titles
        {
          if (!Setup_param)
          {
            menu_index++;          
          }
        }
            
        else
        {
          if (!Setup_param)
               menu_index = 0;
        }

        menu_page_select(menu_index);
      
      
        UpdateTextBuf();
        //UpdateParams();
        menu.refresh();

       
        if (menu_index == 0)
        {
          if (!Setup_param)
               menu.gotoIndex_(0); 
          else
               menu.selectPrev(0);



        } else  
        if (!Setup_param)
            menu.selectNext(0);
        else 
            menu.selectPrev(0);
 
  }
}

void UpButton()
{
  if (menu_visible)
  {    
        if (menu_index == 0) 
        {
           menu_index = PARAMETERS+8;    
        }
        else
        if (menu_index > 0) 
        {
          if (!Setup_param)
          {
            menu_index--;
          }
        }

        menu_page_select(menu_index);    
        UpdateTextBuf();
        //UpdateParams();
        menu.refresh();

        if (menu_index == PARAMETERS+8)
        {
          if (!Setup_param)
          {
            menu.gotoIndex_(2); 
          } else
           menu.selectNext(0);


        } else  
        {
          if (!Setup_param)
            menu.selectPrev(0);
          else
            menu.selectNext(0);
        }

        
  }

}

uint8_t getSelectedMenu()
{
  uint8_t selected = 0;
  
  if (menu_page > 0)
  selected = menu.getSelectedItemIndex()+(menu_page-1)*5-1; 
  if ((selected < 0) || ( selected >= PARAMETERS)) selected = 0;
  
  return selected;
}
  

void loop() {

  int i,j = 0;
  int8_t addr = 0;


  asm("wdr"); 

    if (menu_index < 0) menu_index = 0;

		if (Key3.outstate && (!Key3.lock)) { // down button
			Key3.lock = 1;

      DownButton();      

    } else

    if (Key3.long_outstate) { 
      DownButton();
    }

    if (Key1.outstate && (!Key1.lock)) { // up button
			Key1.lock = 1;
      UpButton();

    } else
    
    if (Key1.long_outstate) {
       UpButton();
    }

    if (Key2.outstate && (!Key2.lock)) {   // enter button
			Key2.lock = 1;

      

      if (!menu_visible) {
          oled.setScale(1);
          menu_visible = true;
          //menu_index = 0;
          menu.showMenu(menu_visible);
      } else 
      {

       
        if (menu.getSelectedItemIndex() == 0) menu.toggleChangeSelected();
        else
        {
          selected_menu = getSelectedMenu();

          if (pgm_read_byte(&menus[selected_menu].writable)) 
              menu.toggleChangeSelected();
        }

        

  
        Setup_param = menu.oledMenuItems[menu.getSelectedItemIndex()].isChange;
        if (!Setup_param)
        { 
          addr = menu.getSelectedItemIndex()-1;
          if (addr < 0 ) addr = 0;


          if (old_param_value != params_buf[addr]) {
            i2c_data.paramfromdisp = params_buf[addr];
            i2c_data.paramfromdisp_addr = getSelectedMenu()+1;

          } 
          else
          {
            old_param_value = params_buf[addr];
            i2c_data.paramfromdisp = 0;
            i2c_data.paramfromdisp_addr = 0;            
          }
        }

        
      }
      
      


      
    } else
    
    if (Key2.long_outstate) {
       // do nothing
    }

    if (!Key1.outstate) Key1.lock = 0;
    if (!Key2.outstate) Key2.lock = 0; 
    if (!Key3.outstate) Key3.lock = 0;

      
    EVERY_N_MILLISECONDS(300){

      if (!menu_visible)  
      {      
        MainScreen(&main_oled);
      } else
      if (!Setup_param) {
        UpdateParams();
        menu.refresh();
      } else
      {

      }
    }
      
    EVERY_N_MILLISECONDS(200){
      Update_status();
    }

}
