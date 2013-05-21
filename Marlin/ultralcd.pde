#include "language.h"
#include "temperature.h"
#include "ultralcd.h"

#ifdef ULTRA_LCD  // this wraps the entire file.

#include "Marlin.h"
#include "language.h"
#include "temperature.h"
#include "EEPROMwrite.h"
#ifndef MCP23017_LCD
#ifdef PCF8574T_LCD
#include <LiquidCrystal_I2C.h>
#else
#include <LiquidCrystal.h>
#endif
#endif

//===========================================================================
//=============================imported variables============================
//===========================================================================

extern volatile int feedmultiply;
extern volatile bool feedmultiplychanged;

extern volatile int extrudemultiply;

extern long position[4];
#ifdef SDSUPPORT
#include "cardreader.h"
extern CardReader card;
#endif

//===========================================================================
//=============================public variables============================
//===========================================================================
volatile char buttons=0;  //the last checked buttons in a bit array.
long encoderpos=0;
short lastenc=0;

//===========================================================================
//=============================private  variables============================
//===========================================================================
static char messagetext[LCD_WIDTH+1]="";

#ifdef MCP23017_LCD
LiquidTWI2 lcd(0);
#else

#ifdef PCF8574T_LCD

// SainSmart I2C LCD2004 adapter for HD44780 LCD screens
// PCF8574T
#define I2C_ADDR    0x3F // 27  // Define I2C Address where the PCF8574A is
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7
#define LCD_BACKLIGHT   0xFF

LiquidCrystal_I2C lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

#else
LiquidCrystal lcd(LCD_PINS_RS, LCD_PINS_ENABLE, LCD_PINS_D4, LCD_PINS_D5,LCD_PINS_D6,LCD_PINS_D7);  //RS,Enable,D4,D5,D6,D7
#endif
#endif
static unsigned long previous_millis_lcd=0;
//static long previous_millis_buttons=0;

#ifdef NEWPANEL
static long blocking=0;
#else
static long blocking[8]= {0,0,0,0,0,0,0,0};
#endif

static MainMenu menu;

int last_message_priority=0;

void lcdProgMemprint(const char *str)
{
	char ch=pgm_read_byte(str);
	while(ch)
	{
		lcd.print(ch);
		ch=pgm_read_byte(++str);
	}
}
#define lcdprintPGM(x) lcdProgMemprint(MYPGM(x))

//===========================================================================
//=============================functions         ============================
//===========================================================================

int intround(const float &x)
{
	return int(0.5+x);
}

void lcd_status(const char* message, int priority)
{
	if (last_message_priority > priority) return;
	//	CRITICAL_SECTION_START;
	last_message_priority = priority;
	strncpy(messagetext,message,LCD_WIDTH);
	messagetext[strlen(message)]=0;
	//	CRITICAL_SECTION_END;
}

void lcd_status(const String message,int priority)
{
	if (last_message_priority > priority) return;
	last_message_priority = priority;
	message.toCharArray (messagetext,LCD_WIDTH);
}

void lcd_statuspgm(const char* message,int priority)
{
#include "wiring.h"
	if (last_message_priority > priority) return;
	//	CRITICAL_SECTION_START;
	last_message_priority = priority;
	char ch=pgm_read_byte(message);
	char *target=messagetext;
	uint8_t cnt=0;
	while(ch &&cnt<LCD_WIDTH)
	{
		*target=ch;
		target++;
		cnt++;
		ch=pgm_read_byte(++message);
	}
	*target=0;
	//	CRITICAL_SECTION_END;
}

void lcd_alertstatuspgm(const char* message)
{
	lcd_statuspgm(message,	ALERT_PRIORITY);
	menu.showStatus();
}

void lcd_clear_message( int priority/*=-1*/ )
{
	if (last_message_priority <= priority || priority <0)
	{
		last_message_priority = 0;
		strncpy(messagetext,"                                 ",LCD_WIDTH);
	}
}

FORCE_INLINE void clear()
{
	lcd.clear();
}

void lcd_init()
{
	//beep();
#ifdef ULTIPANEL
	buttons_init();
#endif

	byte Degree[8] =
	{
		B01100,
		B10010,
		B10010,
		B01100,
		B00000,
		B00000,
		B00000,
		B00000
	};
	byte Thermometer[8] =
	{
		B00100,
		B01010,
		B01010,
		B01010,
		B01010,
		B10001,
		B10001,
		B01110
	};
	byte uplevel[8]=
	{
		B00100,
		B01110,
		B11111,
		B00100,
		B11100,
		B00000,
		B00000,
		B00000
	}; //thanks joris
	byte refresh[8]=
	{
		B00000,
		B00110,
		B11001,
		B11000,
		B00011,
		B10011,
		B01100,
		B00000,
	}; //thanks joris
	byte folder [8]=
	{
		B00000,
		B11100,
		B11111,
		B10001,
		B10001,
		B11111,
		B00000,
		B00000
	}; //thanks joris
	byte sdcard [8]=
	{
		B00000,
		B11100,
		B10010,
		B10001,
		B10001,
		B10001,
		B11111,
		B00000
	};
	byte serial [8]=
	{
		B01000,
		B11110,
		B01000,
		B00000,
		B00010,
		B11111,
		B00010,
		B00000
	};
	byte panel [8]=
	{
		B00000,
		B01110,
		B10001,
		B10101,
		B10001,
		B01110,
		B00000,
		B00000
	};

#ifdef MCP23017_LCD
	lcd.setMCPType(LTI_TYPE_MCP23017);
#endif
	lcd.begin(LCD_WIDTH, LCD_HEIGHT);
	lcd.createChar(1,Degree);
	lcd.createChar(2,Thermometer);
	lcd.createChar(3,uplevel);
	lcd.createChar(4,refresh);
	lcd.createChar(5,folder);
	lcd.createChar(6,sdcard);
	lcd.createChar(7,serial);
	lcd.createChar(8,panel);
	LCD_MESSAGEPGM(WELCOME_MSG);
#ifdef PANUCATT_VIKI
	lcd.setBacklight(0);
#endif
#ifdef PCF8574T_LCD
	lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE); // init the backlight
	lcd.setBacklight (HIGH);
#endif
}

void beep()
{
	//return;
#ifdef ULTIPANEL
#if (BEEPER > -1)
	{
		pinMode(BEEPER,OUTPUT);
		for(int8_t i=0; i<20; i++)
		{
			WRITE(BEEPER,HIGH);
			delay(5);
			WRITE(BEEPER,LOW);
			delay(5);
		}
	}
#endif
#endif
}

void beepshort()
{
	//return;
#ifdef ULTIPANEL
#if (BEEPER > -1)
	{
		pinMode(BEEPER,OUTPUT);
		for(int8_t i=0; i<10; i++)
		{
			WRITE(BEEPER,HIGH);
			delay(3);
			WRITE(BEEPER,LOW);
			delay(3);
		}
	}
#endif
#endif
}

void lcd_status()
{
#ifdef MCP23017_LCD
	if (lcd.readButtons() & BUTTON_SELECT)
		buttons |= EN_C;
#endif
#ifdef PANUCATT_VIKI
#define LED_A 0x04
#define LED_B 0x02
#define LED_C 0x01
	static uint8_t ledsprev = 0;
	uint8_t leds = 0;

	if (degTargetBed()) leds |= LED_A;
	if (degTargetHotend0()) leds |= LED_B;
	if (FanSpeed) leds |= LED_C;
#if EXTRUDERS > 1
	if (degTargetHotend1()) leds |= LED_C;
#endif

	if (leds != ledsprev)
	{
		lcd.setBacklight(leds);
		ledsprev = leds;
	}
#endif
#ifdef ULTIPANEL
	static uint8_t oldbuttons=0;
	//static long previous_millis_buttons=0;
	//static long previous_lcdinit=0;
	//  buttons_check(); // Done in temperature interrupt
	//previous_millis_buttons=millis();

	for(int8_t i=0; i<8; i++)
	{
#ifndef NEWPANEL
		if((blocking[i]>ms))
			buttons &= ~(1<<i);
#else
		if(blocking>now)
			buttons &= ~(1<<i);
#endif
	}

	// no change to buttons, not time to update LCD panel
	if((buttons==oldbuttons) &&  (now - previous_millis_lcd) < LCD_UPDATE_INTERVAL)
		return;

	oldbuttons=buttons;
#else		// ULTIPLANEL

	if(((millis() - previous_millis_lcd) < LCD_UPDATE_INTERVAL)   )
		return;
#endif
	if (last_message_priority>0)
	{
		last_message_priority--;
		if (last_message_priority==0)
			strncpy(messagetext,"                                 ",LCD_WIDTH);
	}
	else if (last_message_priority<0) last_message_priority = 1;
	previous_millis_lcd=now;
	menu.update();
}

#ifdef ULTIPANEL

void buttons_init()
{
#ifdef NEWPANEL
	pinMode(BTN_EN1,INPUT);
	pinMode(BTN_EN2,INPUT);
#ifdef BTN_ENC
	pinMode(BTN_ENC,INPUT);
#endif
	pinMode(SDCARDDETECT,INPUT);
	WRITE(BTN_EN1,HIGH);
	WRITE(BTN_EN2,HIGH);
#ifdef BTN_ENC
	WRITE(BTN_ENC,HIGH);
#endif
#if (SDCARDDETECT > -1)
	{
		WRITE(SDCARDDETECT,HIGH);
	}
#endif
#else
	pinMode(SHIFT_CLK,OUTPUT);
	pinMode(SHIFT_LD,OUTPUT);
	pinMode(SHIFT_EN,OUTPUT);
	pinMode(SHIFT_OUT,INPUT);
	WRITE(SHIFT_OUT,HIGH);
	WRITE(SHIFT_LD,HIGH);
	WRITE(SHIFT_EN,LOW);
#endif
}

void buttons_check()
{
#ifdef NEWPANEL
	uint8_t newbutton=0;
	if(READ(BTN_EN1)==0)  newbutton|=EN_A;
	if(READ(BTN_EN2)==0)  newbutton|=EN_B;
#ifdef BTN_ENC
	if((blocking<millis()) &&(READ(BTN_ENC)==0))
		newbutton|=EN_C;
#endif
	buttons=newbutton;
#else   //read it from the shift register
	uint8_t newbutton=0;
	WRITE(SHIFT_LD,LOW);
	WRITE(SHIFT_LD,HIGH);
	unsigned char tmp_buttons=0;
	for(int8_t i=0; i<8; i++)
	{
		newbutton = newbutton>>1;
		if(READ(SHIFT_OUT))
			newbutton|=(1<<7);
		WRITE(SHIFT_CLK,HIGH);
		WRITE(SHIFT_CLK,LOW);
	}
	buttons=~newbutton; //invert it, because a pressed switch produces a logical 0
#endif

	//manage encoder rotation
	char enc=0;
	if(buttons&EN_A)
		enc|=(1<<0);
	if(buttons&EN_B)
		enc|=(1<<1);
	if(enc!=lastenc)
	{
		switch(enc)
		{
		case encrot0:
			if(lastenc==encrot3)
				encoderpos++;
			else if(lastenc==encrot1)
				encoderpos--;
			break;
		case encrot1:
			if(lastenc==encrot0)
				encoderpos++;
			else if(lastenc==encrot2)
				encoderpos--;
			break;
		case encrot2:
			if(lastenc==encrot1)
				encoderpos++;
			else if(lastenc==encrot3)
				encoderpos--;
			break;
		case encrot3:
			if(lastenc==encrot2)
				encoderpos++;
			else if(lastenc==encrot0)
				encoderpos--;
			break;
		default:
			;
		}
	}
	lastenc=enc;
}

#endif

MainMenu::MainMenu()
{
	status=Main_Status;
	displayStartingRow=0;
	activeline=0;
	force_lcd_update=true;
	linechanging=false;
	tune=false;
}

void MainMenu::displayTemps()
{
	static int olddegHotEnd0=-1;
	static int oldtargetHotEnd0=-1;

	if(force_lcd_update)  //initial display of content
	{
		encoderpos=feedmultiply;
		clear();
		lcd.setCursor(0,0);
		lcdprintPGM("\002---/OFF\001 ");
#if defined BED_USES_THERMISTOR || defined BED_USES_AD595
		lcd.setCursor(10,0);
		lcdprintPGM("B:---/OFF\001 ");
#elif EXTRUDERS > 1
		lcd.setCursor(10,0);
		lcdprintPGM("\002---/OFF\001 ");
#endif
	}

	int tHotEnd0=intround(degHotend0());
	if((tHotEnd0!=olddegHotEnd0)||force_lcd_update)
	{
		lcd.setCursor(1,0);
		lcd.print(ftostr3(tHotEnd0));
		olddegHotEnd0=tHotEnd0;
	}
	int ttHotEnd0=intround(degTargetHotend0());
	if((ttHotEnd0!=oldtargetHotEnd0)||force_lcd_update)
	{
		lcd.setCursor(5,0);
		if (ttHotEnd0>10) lcd.print(ftostr3(ttHotEnd0));
		else lcdprintPGM("OFF");
		oldtargetHotEnd0=ttHotEnd0;
	}
#if defined BED_USES_THERMISTOR || defined BED_USES_AD595
	static int oldtBed=-1;
	static int oldtargetBed=-1;
	int tBed=intround(degBed());
	if((tBed!=oldtBed)||force_lcd_update)
	{
		lcd.setCursor(12,0);
		lcd.print(ftostr3(tBed));
		oldtBed=tBed;
	}
	int targetBed=intround(degTargetBed());
	if((targetBed!=oldtargetBed)||force_lcd_update)
	{
		lcd.setCursor(16,0);
		if (targetBed>10) lcd.print(ftostr3(targetBed));
		else lcdprintPGM("OFF");
		oldtargetBed=targetBed;
	}
#elif EXTRUDERS > 1
	static int olddegHotEnd1=-1;
	static int oldtargetHotEnd1=-1;
	int tHotEnd1=intround(degHotend1());
	if((tHotEnd1!=olddegHotEnd1)||force_lcd_update)
	{
		lcd.setCursor(12,0);
		lcd.print(ftostr3(tHotEnd1));
		olddegHotEnd1=tHotEnd1;
	}
	int ttHotEnd1=intround(degTargetHotend1());
	if((ttHotEnd1!=oldtargetHotEnd1)||force_lcd_update)
	{
		lcd.setCursor(16,0);
		if (ttHotEnd1>10) lcd.print(ftostr3(ttHotEnd1));
		else lcdprintPGM("OFF");
		oldtargetHotEnd1=ttHotEnd1;
	}
#endif
}

//do a periodic force pdate, just in case something screwed up
const int force_update_interval=30;
String last_drawn_progress_string[3] ;

void MainMenu::showStatus()
{
	bool reentracy_blocker = false;
	if (reentracy_blocker) return; reentracy_blocker = true;
#if LCD_HEIGHT==4
	static int force_update_ticker=0;
	static STATES prev_state = MAX_STATES;
	//	if (force_update_ticker++ % force_update_interval ==0) force_lcd_update=true;
	if (state!=prev_state) force_lcd_update=true;
	prev_state = state;
	//force_lcd_update=true;

	displayTemps();

	static int oldfeedmultiply=0;
	int curfeedmultiply=feedmultiply;

	if(feedmultiplychanged == true)
	{
		feedmultiplychanged = false;
		encoderpos = curfeedmultiply;
	}
	// TODO move this to where the speed rate is changed
	// so this feed check only happens if you have an LCD?
/*
	if(encoderpos!=curfeedmultiply/ *||force_lcd_update* /)
	{
		curfeedmultiply=encoderpos;
		if(curfeedmultiply<10)
			curfeedmultiply=10;
		if(curfeedmultiply>999)
			curfeedmultiply=999;
		feedmultiply=curfeedmultiply;
		encoderpos=curfeedmultiply;
	}*/
	lcd.setCursor(0,1);
	if (state.Controller()==SDCARD) lcd.print ("\006");
	else if (state.Controller()==SERIAL_HOST) lcd.print ("\007");
	else lcd.print ("\008");		// panel
	if (force_lcd_update) lcd.print (STATE_STRINGS[state]);
	if (progress_string[0]!=last_drawn_progress_string[0] || force_lcd_update)
	{
		lcd.setCursor(0,2);
		lcd.print(progress_string[0]);
		last_drawn_progress_string[0]=progress_string[0];
	}
	if (progress_string[1]!=last_drawn_progress_string[1]|| force_lcd_update)
	{
		lcd.setCursor(11,1);
		lcd.print(progress_string[1]);
		last_drawn_progress_string[1]=progress_string[1];
	}
	if (progress_string[2]!=last_drawn_progress_string[2]|| force_lcd_update)
	{
		lcd.setCursor(11,2);
		lcd.print(progress_string[2]);
		last_drawn_progress_string[2]=progress_string[2];
	}

	if(messagetext[0]!='\0')
	{
		lcd.setCursor(0,LCD_HEIGHT-1);
		lcd.print(messagetext);
		uint8_t n=strlen(messagetext);
		for(int8_t i=0; i<LCD_WIDTH-n; i++)
			lcd.print(" ");
		messagetext[0]='\0';
	}

#else //smaller LCDS----------------------------------
	static int olddegHotEnd0=-1;
	static int oldtargetHotEnd0=-1;
	if(force_lcd_update)  //initial display of content
	{
		encoderpos=feedmultiply;
		lcd.setCursor(0,0);
		lcdprintPGM("\002---/---\001 ");
	}

	int tHotEnd0=intround(degHotend0());
	int ttHotEnd0=intround(degTargetHotend0());

	if((abs(tHotEnd0-olddegHotEnd0)>1)||force_lcd_update)
	{
		lcd.setCursor(1,0);
		lcd.print(ftostr3(tHotEnd0));
		olddegHotEnd0=tHotEnd0;
	}
	if((ttHotEnd0!=oldtargetHotEnd0)||force_lcd_update)
	{
		lcd.setCursor(5,0);
		lcd.print(ftostr3(ttHotEnd0));
		oldtargetHotEnd0=ttHotEnd0;
	}

	if(messagetext[0]!='\0')
	{
		lcd.setCursor(0,LCD_HEIGHT-1);
		lcd.print(messagetext);
		uint8_t n=strlen(messagetext);
		for(int8_t i=0; i<LCD_WIDTH-n; i++)
			lcd.print(" ");
		messagetext[0]='\0';
	}

#endif
	force_lcd_update=false;
	reentracy_blocker = false;
}

// ---------------------------------------------------------------------------------------------------------------------------------------------------

enum {
	ItemP_exit, 
	ItemP_autostart,
	ItemP_disstep,
	ItemP_home, 
	ItemP_origin, 
	ItemP_preheat_pla, 
	ItemP_preheat_abs, 
	ItemP_cooldown,
	/*ItemP_extrude,*/
	ItemP_move,
	ItemP_resetMetrics
};

//any action must not contain a ',' character anywhere, or this breaks:
#define MENUITEM(repaint_action, click_action) \
{\
	if(force_lcd_update)  { lcd.setCursor(0,line);  repaint_action; } \
	if((activeline==line) && CLICKED) {click_action} \
}

void MainMenu::showPrepare()
{
#ifdef ULTIPANEL
	uint8_t line=0;
	clearIfNecessary();
	for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
	{
		//Serial.println((int)(line-lineoffset));
		switch(i)
		{
		case ItemP_exit:
			MENUITEM(  lcdprintPGM(MSG_MAIN)  ,  BLOCK; status=Main_Menu; beepshort(); ) ;
			break;
		case ItemP_autostart:
			MENUITEM(  lcdprintPGM(MSG_AUTOSTART)  ,  BLOCK;
#ifdef SDSUPPORT
			card.lastnr=0; card.setroot(); card.checkautostart(true);
#endif
			beepshort(); ) ;
			break;
		case ItemP_disstep:
			MENUITEM(  lcdprintPGM(MSG_DISABLE_STEPPERS)  ,  BLOCK; enquecommand("M84"); beepshort(); ) ;
			break;
		case ItemP_home:
			MENUITEM(  lcdprintPGM(MSG_AUTO_HOME)  ,  BLOCK; enquecommand("G28"); beepshort(); ) ;
			break;
		case ItemP_origin:
			MENUITEM(  lcdprintPGM(MSG_SET_ORIGIN)  ,  BLOCK; enquecommand("G92 X0 Y0 Z0"); beepshort(); ) ;
			break;
		case ItemP_preheat_pla:
			MENUITEM(  lcdprintPGM(MSG_PREHEAT_PLA)  ,  BLOCK; setTargetHotend0(plaPreheatHotendTemp); setTargetBed(plaPreheatHPBTemp);
#if FAN_PIN > -1
			FanSpeed = plaPreheatFanSpeed;
			analogWrite(FAN_PIN,  FanSpeed);
#endif
			beepshort(); );
			break;
		case ItemP_preheat_abs:
			MENUITEM(  lcdprintPGM(MSG_PREHEAT_ABS)  ,  BLOCK; setTargetHotend0(absPreheatHotendTemp); setTargetBed(absPreheatHPBTemp);
#if FAN_PIN > -1
			FanSpeed = absPreheatFanSpeed;
			analogWrite(FAN_PIN,  FanSpeed);
#endif
			beepshort(); );
			break;
		case ItemP_cooldown:
			MENUITEM(  lcdprintPGM(MSG_COOLDOWN)  ,  BLOCK; setTargetHotend0(0); setTargetHotend1(0); setTargetHotend2(0); setTargetBed(0); beepshort(); ) ;
			break;
			//    case ItemP_extrude:
			//    MENUITEM(  lcdprintPGM(" Extrude")  ,  BLOCK;enquecommand("G92 E0");enquecommand("G1 F700 E50");beepshort(); ) ;
			//  break;
		case ItemP_move:
			MENUITEM(  lcdprintPGM(MSG_MOVE_AXIS) , BLOCK; status=Sub_PrepareMove; beepshort(); );
			break;
		case ItemP_resetMetrics:
			MENUITEM(  lcdprintPGM(MSG_METRICS) , BLOCK; ResetMetrics(); beepshort(); );
			break;

		default:
			break;
		}
		line++;
	}
	updateActiveLines(ItemP_resetMetrics,encoderpos);
#endif
}


// ---------------------------------------------------------------------------------------------------------------------------------------------------
enum
{
	ItemAM_exit,
	ItemAM_X, 
	ItemAM_Y, 
	ItemAM_Z, 
	ItemAM_E, 
	ItemAM_ERetract
};

void MainMenu::showAxisMove()
{
	uint8_t line=0;
	int oldencoderpos=0;
	clearIfNecessary();
	for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
	{
		switch(i)
		{
		case ItemAM_exit:
			MENUITEM(  lcdprintPGM(MSG_PREPARE_ALT)  ,  BLOCK; status=Main_Prepare; beepshort(); ) ;
			break;
		case ItemAM_X:
			{
				//oldencoderpos=0;
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(" X:");
					lcd.setCursor(11,line);
					lcd.print(ftostr52(current_head_position[X_AXIS]));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						enquecommand("G91");
					}
					else
					{
						enquecommand("G90");
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if (encoderpos >0)
					{
						enquecommand("G1 F700 X0.1");
						oldencoderpos=encoderpos;
						encoderpos=0;
					}

					else if (encoderpos < 0)
					{
						enquecommand("G1 F700 X-0.1");
						oldencoderpos=encoderpos;
						encoderpos=0;
					}
					lcd.setCursor(11,line);
					lcd.print(ftostr52(current_head_position[X_AXIS]));
				}
			}
			break;
		case ItemAM_Y:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(" Y:");
					lcd.setCursor(11,line);
					lcd.print(ftostr52(current_head_position[Y_AXIS]));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						enquecommand("G91");
					}
					else
					{
						enquecommand("G90");
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if (encoderpos >0)
					{
						enquecommand("G1 F700 Y0.1");
						oldencoderpos=encoderpos;
						encoderpos=0;
					}

					else if (encoderpos < 0)
					{
						enquecommand("G1 F700 Y-0.1");
						oldencoderpos=encoderpos;
						encoderpos=0;
					}
					lcd.setCursor(11,line);
					lcd.print(ftostr52(current_head_position[Y_AXIS]));
				}
			}
			break;
		case ItemAM_Z:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(" Z:");
					lcd.setCursor(11,line);
					lcd.print(ftostr52(current_head_position[Z_AXIS]));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						enquecommand("G91");
					}
					else
					{
						enquecommand("G90");
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if (encoderpos >0)
					{
						enquecommand("G1 F70 Z0.1");
						oldencoderpos=encoderpos;
						encoderpos=0;
					}

					else if (encoderpos < 0)
					{
						enquecommand("G1 F70 Z-0.1");
						oldencoderpos=encoderpos;
						encoderpos=0;
					}
					lcd.setCursor(11,line);
					lcd.print(ftostr52(current_head_position[Z_AXIS]));
				}
			}
			break;
		case ItemAM_E:
			// ErikDB: TODO: this length should be changed for volumetric.
			MENUITEM(  lcdprintPGM(MSG_EXTRUDE)  ,  BLOCK; enquecommand("G92 E0"); enquecommand("G1 F70 E1"); beepshort(); ) ;
			break;
		case ItemAM_ERetract:
			// ErikDB: TODO: this length should be changed for volumetric.
			MENUITEM(  lcdprintPGM(MSG_RETRACT)  ,  BLOCK; enquecommand("G92 E0"); enquecommand("G1 F700 E-1"); beepshort(); ) ;
			break;
		default:
			break;
		}
		line++;
	}
	updateActiveLines(ItemAM_ERetract,encoderpos);
}

enum {ItemT_exit,ItemT_speed,ItemT_flow,ItemT_nozzle,
#if (HEATER_BED_PIN > -1)
	ItemT_bed,
#endif
	ItemT_fan
};

void MainMenu::showTune()
{
	uint8_t line=0;
	clearIfNecessary();
	for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
	{
		//Serial.println((int)(line-lineoffset));
		switch(i)
		{
		case ItemT_exit:
			MENUITEM(  lcdprintPGM(MSG_MAIN)  ,  BLOCK; status=Main_Menu; beepshort(); ) ;
			break;
		case ItemT_speed:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_SPEED);
					lcd.setCursor(13,line);
					lcd.print(itostr3((int) feedmultiply));
				}

				if((activeline!=line) )
					break;

				if(CLICKED) //AnalogWrite(FAN_PIN,  fanpwm);
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=feedmultiply;
					}
					else
					{
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if(encoderpos<1) encoderpos=1;
					if(encoderpos>400) encoderpos=400;
					feedmultiply = encoderpos;
					feedmultiplychanged=true;
					lcd.setCursor(13,line);
					lcd.print(itostr3(encoderpos));
				}
			}
			break;
		case ItemT_nozzle:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_NOZZLE);
					lcd.setCursor(13,line);
					lcd.print(ftostr3(intround(degTargetHotend0())));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=intround(degTargetHotend0());
					}
					else
					{
						setTargetHotend0(encoderpos);
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>HEATER_0_MAXTEMP) encoderpos=HEATER_0_MAXTEMP;
					lcd.setCursor(13,line);
					lcd.print(itostr3(encoderpos));
				}
			}
			break;
#if (HEATER_BED_PIN > -1)
		case ItemT_bed:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_BED);
					lcd.setCursor(13,line);
					lcd.print(ftostr3(intround(degTargetBed())));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=intround(degTargetBed());
					}
					else
					{
						setTargetBed(encoderpos);
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>BED_MAXTEMP) encoderpos=BED_MAXTEMP;
					lcd.setCursor(13,line);
					lcd.print(itostr3(encoderpos));
				}
			}
			break;
#endif

		case ItemT_fan:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_FAN_SPEED);
					lcd.setCursor(13,line);
					lcd.print(ftostr3(FanSpeed));
				}

				if((activeline!=line) )
					break;

				if(CLICKED) //nalogWrite(FAN_PIN,  fanpwm);
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=FanSpeed;
					}
					else
					{
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>255) encoderpos=255;
					FanSpeed=encoderpos;
					analogWrite(FAN_PIN,  FanSpeed);
					lcd.setCursor(13,line);
					lcd.print(itostr3(encoderpos));
				}
			}
			break;
		case ItemT_flow://axis_steps_per_unit[i] = code_value();
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_FLOW);
					lcd.setCursor(13,line);
					lcd.print(itostr3((int) extrudemultiply));
				}

				if((activeline!=line) )
					break;

				if(CLICKED) //AnalogWrite(FAN_PIN,  fanpwm);
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=extrudemultiply;
					}
					else
					{
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}

				if(linechanging)
				{
					if(encoderpos<5) encoderpos=5;
					if(encoderpos>250) encoderpos=250;
					lcd.setCursor(13,line);
					lcd.print(itostr3(encoderpos));
				}
			}
			break;
		default:
			break;
		}
		line++;
	}
	updateActiveLines(ItemT_fan,encoderpos);
}

//does not work
// #define MENUCHANGEITEM(repaint_action,  enter_action, accept_action,  change_action) \
//   {\
//     if(force_lcd_update)  { lcd.setCursor(0,line);  repaint_action; } \
//     if(activeline==line)  \
//     { \
//       if(CLICKED) \
//       { \
//         linechanging=!linechanging; \
//         if(linechanging)  {enter_action;} \
//         else {accept_action;} \
//       }  \
//       else \
//       if(linechanging) {change_action};}\
//   }
//

enum
{
	ItemCT_exit,ItemCT_nozzle0,
#ifdef AUTOTEMP
	ItemCT_autotempactive,
	ItemCT_autotempmin,ItemCT_autotempmax,ItemCT_autotempfact,
#endif
#if EXTRUDERS > 1
	ItemCT_nozzle1,
#endif
#if EXTRUDERS > 2
	ItemCT_nozzle2,
#endif
#if defined BED_USES_THERMISTOR || defined BED_USES_AD595
	ItemCT_bed,
#endif
	ItemCT_fan,
	ItemCT_PID_P,ItemCT_PID_I,ItemCT_PID_D,ItemCT_PID_C,
	ItemCT_PLA_PreHeat_Setting,
	ItemCT_ABS_PreHeat_Setting,
};

void MainMenu::showControlTemp()
{
	uint8_t line=0;
	clearIfNecessary();
	for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
	{
		switch(i)
		{
		case ItemCT_exit:
			MENUITEM(  lcdprintPGM(MSG_CONTROL)  ,  BLOCK; status=Main_Control; beepshort(); ) ;
			break;
		case ItemCT_nozzle0:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_NOZZLE);
					lcd.setCursor(13,line);
					lcd.print(ftostr3(intround(degTargetHotend0())));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=intround(degTargetHotend0());
					}
					else
					{
						setTargetHotend0(encoderpos);
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>260) encoderpos=260;
					lcd.setCursor(13,line);
					lcd.print(itostr3(encoderpos));
				}
			}
			break;
#if EXTRUDERS > 1
		case ItemCT_nozzle1:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_NOZZLE1);
					lcd.setCursor(13,line);
					lcd.print(ftostr3(intround(degTargetHotend1())));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=intround(degTargetHotend1());
					}
					else
					{
						setTargetHotend1(encoderpos);
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>260) encoderpos=260;
					lcd.setCursor(13,line);
					lcd.print(itostr3(encoderpos));
				}
			}
			break;
#endif
#if EXTRUDERS > 2
		case ItemCT_nozzle2:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_NOZZLE2);
					lcd.setCursor(13,line);
					lcd.print(ftostr3(intround(degTargetHotend2())));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=intround(degTargetHotend2());
					}
					else
					{
						setTargetHotend1(encoderpos);
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>260) encoderpos=260;
					lcd.setCursor(13,line);
					lcd.print(itostr3(encoderpos));
				}
			}
			break;
#endif
#ifdef AUTOTEMP
		case ItemCT_autotempmin:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_MIN);
					lcd.setCursor(13,line);
					lcd.print(ftostr3(autotemp_min));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=intround(autotemp_min);
					}
					else
					{
						autotemp_min=encoderpos;
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>260) encoderpos=260;
					lcd.setCursor(13,line);
					lcd.print(itostr3(encoderpos));
				}
			}
			break;
		case ItemCT_autotempmax:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_MAX);
					lcd.setCursor(13,line);
					lcd.print(ftostr3(autotemp_max));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=intround(autotemp_max);
					}
					else
					{
						autotemp_max=encoderpos;
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>260) encoderpos=260;
					lcd.setCursor(13,line);
					lcd.print(itostr3(encoderpos));
				}
			}
			break;
		case ItemCT_autotempfact:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_FACTOR);
					lcd.setCursor(13,line);
					lcd.print(ftostr32(autotemp_factor));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=intround(autotemp_factor*100);
					}
					else
					{
						autotemp_max=encoderpos;
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>99) encoderpos=99;
					lcd.setCursor(13,line);
					lcd.print(ftostr32(encoderpos/100.));
				}
			}
			break;
		case ItemCT_autotempactive:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_AUTOTEMP);
					lcd.setCursor(13,line);
					if(autotemp_enabled)
						lcdprintPGM(MSG_ON);
					else
						lcdprintPGM(MSG_OFF);
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					autotemp_enabled=!autotemp_enabled;
					lcd.setCursor(13,line);
					if(autotemp_enabled)
						lcdprintPGM(MSG_ON);
					else
						lcdprintPGM(MSG_OFF);
					BLOCK;
				}
			}
			break;
#endif //autotemp
#if defined BED_USES_THERMISTOR || defined BED_USES_AD595
		case ItemCT_bed:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_BED);
					lcd.setCursor(13,line);
					lcd.print(ftostr3(intround(degTargetBed())));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=intround(degTargetBed());
					}
					else
					{
						setTargetBed(encoderpos);
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>260) encoderpos=260;
					lcd.setCursor(13,line);
					lcd.print(itostr3(encoderpos));
				}
			}
			break;
#endif
		case ItemCT_fan:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_FAN_SPEED);
					lcd.setCursor(13,line);
					lcd.print(ftostr3(FanSpeed));
				}

				if((activeline!=line) )
					break;

				if(CLICKED) //nalogWrite(FAN_PIN,  fanpwm);
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=FanSpeed;
					}
					else
					{
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>255) encoderpos=255;
					FanSpeed=encoderpos;
					analogWrite(FAN_PIN,  FanSpeed);
					lcd.setCursor(13,line);
					lcd.print(itostr3(encoderpos));
				}
			}
			break;
#ifdef PIDTEMP
		case ItemCT_PID_P:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(" PID-P: ");
					lcd.setCursor(13,line);
					lcd.print(itostr4(Kp));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=(long)Kp;
					}
					else
					{
						Kp= encoderpos;
						encoderpos=activeline*lcdslow;
					}
					BLOCK;
					beepshort();
				}
				if(linechanging)
				{
					if(encoderpos<1) encoderpos=1;
					if(encoderpos>9990) encoderpos=9990;
					lcd.setCursor(13,line);
					lcd.print(itostr4(encoderpos));
				}
			}
			break;
		case ItemCT_PID_I:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_PID_I);
					lcd.setCursor(13,line);
					lcd.print(ftostr51(Ki/PID_dT));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=(long)(Ki*10/PID_dT);
					}
					else
					{
						Ki= encoderpos/10.*PID_dT;
						encoderpos=activeline*lcdslow;
					}
					BLOCK;
					beepshort();
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>9990) encoderpos=9990;
					lcd.setCursor(13,line);
					lcd.print(ftostr51(encoderpos/10.));
				}
			}
			break;
		case ItemCT_PID_D:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_PID_D);
					lcd.setCursor(13,line);
					lcd.print(itostr4(Kd*PID_dT));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=(long)(Kd/5./PID_dT);
					}
					else
					{
						Kd= encoderpos;
						encoderpos=activeline*lcdslow;
					}
					BLOCK;
					beepshort();
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>9990) encoderpos=9990;
					lcd.setCursor(13,line);
					lcd.print(itostr4(encoderpos));
				}
			}
			break;
		case ItemCT_PID_C:
#ifdef PID_ADD_EXTRUSION_RATE
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_PID_C);
					lcd.setCursor(13,line);
					lcd.print(itostr3(Kc));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=(long)Kc;
					}
					else
					{
						Kc= encoderpos;
						encoderpos=activeline*lcdslow;
					}
					BLOCK;
					beepshort();
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>990) encoderpos=990;
					lcd.setCursor(13,line);
					lcd.print(itostr3(encoderpos));
				}
			}
#endif
#endif
			break;
		case ItemCT_PLA_PreHeat_Setting:
			MENUITEM(  lcdprintPGM(MSG_PREHEAT_PLA_SETTINGS)  ,  BLOCK; status=Sub_PreheatPLASettings; beepshort(); ) ;
			break;
		case ItemCT_ABS_PreHeat_Setting:
			MENUITEM(  lcdprintPGM(MSG_PREHEAT_ABS_SETTINGS)  ,  BLOCK; status=Sub_PreheatABSSettings; beepshort(); ) ;
			break;
		default:
			break;
		}
		line++;
	}

	updateActiveLines(ItemCT_ABS_PreHeat_Setting,encoderpos);
}

enum
{
	ItemCM_exit,
	ItemCM_acc, 
	ItemCM_xyjerk,
	ItemCM_vmaxx, 
	ItemCM_vmaxy, 
	ItemCM_vmaxz, 
	ItemCM_vmaxe,
	ItemCM_vtravmin,
	ItemCM_vmin,
	ItemCM_amaxx, 
	ItemCM_amaxy, 
	ItemCM_amaxz, 
	ItemCM_amaxe,
	ItemCM_aret,
	ItemCM_xsteps,
	ItemCM_ysteps, 
	ItemCM_zsteps, 
	ItemCM_esteps
};

float stepsize (float value)
{
	if (value > 10000) return 100;
	if (value > 1000) return 10;
	if (value > 100) return 1;
	if (value > 10) return 0.1;
	if (value > 1) return 0.01;
	if (value > 0.1) return 0.001;
	return 0.001;
}

int stepsize (int value)
{
	if (value > 1000) return 100;
	if (value > 100) return 10;
	return 1;
}


void MainMenu::showControlMotion()
{
	uint8_t line=0;
	clearIfNecessary();
	for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
	{
		switch(i)
		{
		case ItemCM_exit:
			MENUITEM(  lcdprintPGM(MSG_CONTROL)  ,  BLOCK; status=Main_Control; beepshort(); ) ;
			break;
		case ItemCM_acc:
			HandleAdjustment (line,MSG_ACC, acceleration,0.1,9999.9f);
			break;
		case ItemCM_xyjerk: //max_xy_jerk
			HandleAdjustment (line,MSG_VXY_JERK, max_xy_jerk,0.1,99.9f,3,2);
			break;
		case ItemCM_vmaxx:
			HandleAdjustment (line,MSG_VMAX MSG_X, max_feedrate[X_AXIS],0.1,9999.9f);
			break;
		case ItemCM_vmaxy:
			HandleAdjustment (line,MSG_VMAX MSG_Y, max_feedrate[Y_AXIS],0.1,9999.9f);
			break;
		case ItemCM_vmaxz:
			HandleAdjustment (line,MSG_VMAX MSG_Z, max_feedrate[Z_AXIS],0.1,9999.9f);
			break;
		case ItemCM_vmaxe:
			HandleAdjustment (line,MSG_VMAX MSG_E, max_feedrate[E_AXIS],0.1,9999.9f);
			break;
		case ItemCM_vmin:
			HandleAdjustment (line, MSG_VMIN,minimumfeedrate,0.0f,99.9f);
			break;
		case ItemCM_vtravmin:
			HandleAdjustment (line, MSG_VTRAV_MIN,mintravelfeedrate,0.0f,99.9f);
			break;

		case ItemCM_amaxx:
			{
				float t =  max_acceleration_units_per_sq_second[X_AXIS];
				HandleAdjustment (line," Amax " MSG_X,t,1.01f,9999.9f,6,0);
				max_acceleration_units_per_sq_second[X_AXIS] = t;
			}
			break;
		case ItemCM_amaxy:
			{
				float t =  max_acceleration_units_per_sq_second[Y_AXIS];
				HandleAdjustment (line," Amax " MSG_Y,t,1.01f,9999.9f,6,0);
				max_acceleration_units_per_sq_second[Y_AXIS] = t;
			}
			break;
		case ItemCM_amaxz:
			{
				float t =  max_acceleration_units_per_sq_second[Z_AXIS];
				HandleAdjustment (line," Amax " MSG_Z,t,1.01f,9999.9f,6,0);
				max_acceleration_units_per_sq_second[Z_AXIS] = t;
			}
			break;
		case ItemCM_amaxe:
			{
				float t =  max_acceleration_units_per_sq_second[E_AXIS];
				HandleAdjustment (line," Amax " MSG_E,t,1.01f,9999.9f,6,0);
				max_acceleration_units_per_sq_second[E_AXIS] = t;
			}
			break;

		case ItemCM_aret://float retract_acceleration = 7000;
			HandleAdjustment (line, MSG_A_RETRACT,retract_acceleration,0.1,9999.9f,5,1);
			break;
		case ItemCM_xsteps://axis_steps_per_unit[i] = code_value();
			HandleAdjustment (line, MSG_XSTEPS,axis_steps_per_unit[X_AXIS],0.1,99999.9f,5,2);
		 
			break;
		case ItemCM_ysteps://axis_steps_per_unit[i] = code_value();
			HandleAdjustment (line, MSG_YSTEPS,axis_steps_per_unit[Y_AXIS],0.1,99999.9f,5,2);

			break;
		case ItemCM_zsteps://axis_steps_per_unit[i] = code_value();
			HandleAdjustment (line, MSG_ZSTEPS,axis_steps_per_unit[Z_AXIS],0.1,99999.9f,5,2);
			break;
		case ItemCM_esteps://axis_steps_per_unit[i] = code_value();
			HandleAdjustment (line, MSG_ESTEPS,axis_steps_per_unit[E_AXIS],0.1,99999.9f,5,2);
			break;

		default:
			break;
		}
		line++;
	}
	updateActiveLines(ItemCM_esteps,encoderpos);
}

enum
{
	ItemR_exit,
	ItemR_autoretract,
	ItemR_retract_length,ItemR_retract_feedrate,ItemR_retract_zlift,
	ItemR_unretract_length,ItemR_unretract_feedrate,
};

void MainMenu::showControlRetract()
{
#ifdef FWRETRACT
	uint8_t line=0;
	clearIfNecessary();
	for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
	{
		switch(i)
		{
		case ItemR_exit:
			MENUITEM(  lcdprintPGM(MSG_CONTROL)  ,  BLOCK; status=Main_Control; beepshort(); ) ;
			break;

			//float retract_length=2, retract_feedrate=1200, retract_zlift=0.4;
			//float retract_recover_length=0, retract_recover_feedrate=500;
		case ItemR_autoretract:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_AUTORETRACT);
					lcd.setCursor(13,line);
					if(autoretract_enabled)
						lcdprintPGM(MSG_ON);
					else
						lcdprintPGM(MSG_OFF);
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					autoretract_enabled=!autoretract_enabled;
					lcd.setCursor(13,line);
					if(autoretract_enabled)
						lcdprintPGM(MSG_ON);
					else
						lcdprintPGM(MSG_OFF);
					BLOCK;
				}
			}
			break;

		case ItemR_retract_length:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_CONTROL_RETRACT);
					lcd.setCursor(13,line);
					lcd.print(ftostr52(retract_length));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=(long)(retract_length*100);
					}
					else
					{
						retract_length= encoderpos/100.;
						encoderpos=activeline*lcdslow;
					}
					BLOCK;
					beepshort();
				}
				if(linechanging)
				{
					if(encoderpos<1) encoderpos=1;
					if(encoderpos>990) encoderpos=990;
					lcd.setCursor(13,line);
					lcd.print(ftostr52(encoderpos/100.));
				}
			}
			break;
		case ItemR_retract_feedrate:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_CONTROL_RETRACTF);
					lcd.setCursor(13,line);
					lcd.print(itostr4(retract_feedrate));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=(long)(retract_feedrate/5);
					}
					else
					{
						retract_feedrate= encoderpos*5.;
						encoderpos=activeline*lcdslow;
					}
					BLOCK;
					beepshort();
				}
				if(linechanging)
				{
					if(encoderpos<1) encoderpos=1;
					if(encoderpos>990) encoderpos=990;
					lcd.setCursor(13,line);
					lcd.print(itostr4(encoderpos*5));
				}
			}
			break;
		case ItemR_retract_zlift://float retract_acceleration = 7000;
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_CONTROL_RETRACT_ZLIFT);
					lcd.setCursor(13,line);
					lcd.print(ftostr52(retract_zlift));;
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=(long)(retract_zlift*10);
					}
					else
					{
						retract_zlift= encoderpos/10.;
						encoderpos=activeline*lcdslow;
					}
					BLOCK;
					beepshort();
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>990) encoderpos=990;
					lcd.setCursor(13,line);
					lcd.print(ftostr52(encoderpos/10.));
				}
			}
			break;
		case ItemR_unretract_length:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_CONTROL_RETRACT_RECOVER);
					lcd.setCursor(13,line);
					lcd.print(ftostr52(retract_recover_length));;
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=(long)(retract_recover_length*100);
					}
					else
					{
						retract_recover_length= encoderpos/100.;
						encoderpos=activeline*lcdslow;
					}
					BLOCK;
					beepshort();
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>990) encoderpos=990;
					lcd.setCursor(13,line);
					lcd.print(ftostr52(encoderpos/100.));
				}
			}
			break;

		case ItemR_unretract_feedrate:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_CONTROL_RETRACT_RECOVERF);
					lcd.setCursor(13,line);
					lcd.print(itostr4(retract_recover_feedrate));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=(long)retract_recover_feedrate/5;
					}
					else
					{
						retract_recover_feedrate= encoderpos*5.;
						encoderpos=activeline*lcdslow;
					}
					BLOCK;
					beepshort();
				}
				if(linechanging)
				{
					if(encoderpos<1) encoderpos=1;
					if(encoderpos>990) encoderpos=990;
					lcd.setCursor(13,line);
					lcd.print(itostr4(encoderpos*5));
				}
			}
			break;

		default:
			break;
		}
		line++;
	}
	updateActiveLines(ItemR_unretract_feedrate,encoderpos);
#endif
}

enum
{
	ItemC_exit,ItemC_temp,ItemC_move,
#ifdef FWRETRACT
	ItemC_rectract,
#endif
	ItemC_store, ItemC_load,ItemC_failsafe
};

void MainMenu::showControl()
{
	uint8_t line=0;
	clearIfNecessary();
	for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
	{
		switch(i)
		{
		case ItemC_exit:
			MENUITEM(  lcdprintPGM(MSG_MAIN_WIDE)  ,  BLOCK; status=Main_Menu; beepshort(); ) ;
			break;
		case ItemC_temp:
			MENUITEM(  lcdprintPGM(MSG_TEMPERATURE_WIDE)  ,  BLOCK; status=Sub_TempControl; beepshort(); ) ;
			break;
		case ItemC_move:
			MENUITEM(  lcdprintPGM(MSG_MOTION_WIDE)  ,  BLOCK; status=Sub_MotionControl; beepshort(); ) ;
			break;
#ifdef FWRETRACT
		case ItemC_rectract:
			MENUITEM(  lcdprintPGM(MSG_RECTRACT_WIDE)  ,  BLOCK; status=Sub_RetractControl; beepshort(); ) ;
			break;
#endif
		case ItemC_store:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_STORE_EPROM);
				}
				if((activeline==line) && CLICKED)
				{
					//enquecommand("M84");
					beepshort();
					BLOCK;
					EEPROM_StoreSettings();
				}
			}
			break;
		case ItemC_load:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_LOAD_EPROM);
				}
				if((activeline==line) && CLICKED)
				{
					//enquecommand("M84");
					beepshort();
					BLOCK;
					EEPROM_RetrieveSettings();
				}
			}
			break;
		case ItemC_failsafe:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_RESTORE_FAILSAFE);
				}
				if((activeline==line) && CLICKED)
				{
					//enquecommand("M84");
					beepshort();
					BLOCK;
					EEPROM_RetrieveSettings(true);
				}
			}
			break;
		default:
			break;
		}
		line++;
	}
	updateActiveLines(ItemC_failsafe,encoderpos);
}

void MainMenu::showSD()
{
#ifdef SDSUPPORT
	uint8_t line=0;

	clearIfNecessary();
	static uint8_t nrfiles=0;
	if(force_lcd_update)
	{
		if(card.cardOK)
		{
			nrfiles=card.getnrfilenames();
		}
		else
		{
			nrfiles=0;
			lineoffset=0;
		}
	}
	bool enforceupdate=false;
	for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
	{
		switch(i)
		{
		case 0:
			MENUITEM(  lcdprintPGM(MSG_MAIN)  ,  BLOCK; status=Main_Menu; beepshort(); ) ;
			break;
			//     case 1:
			//       {
			//         if(force_lcd_update)
			//         {
			//           lcd.setCursor(0,line);
			//            #ifdef CARDINSERTED
			//           if(CARDINSERTED)
			//           #else
			//           if(true)
			//           #endif
			//           {
			//             lcdprintPGM(" \004Refresh");
			//           }
			//           else
			//           {
			//             lcdprintPGM(" \004Insert Card");
			//           }
			//
			//         }
			//         if((activeline==line) && CLICKED)
			//         {
			//           BLOCK;
			//           beepshort();
			//           card.initsd();
			//           force_lcd_update=true;
			//            nrfiles=card.getnrfilenames();
			//         }
			//       }break;
		case 1:
			MENUITEM(  lcd.print(" "); card.getWorkDirName();
			if(card.filename[0]=='/') lcdprintPGM(MSG_REFRESH);
			else
			{
				lcd.print("\005");
				lcd.print(card.filename);
				lcd.print("/..");
			}  ,
				BLOCK;
			if(SDCARDDETECT == -1) card.initsd();
			card.updir();
			enforceupdate=true;
			lineoffset=0;
			beepshort(); ) ;

			break;
		default:
			{
#define FIRSTITEM 2
				if(i-FIRSTITEM<nrfiles)
				{
					if(force_lcd_update)
					{
						card.getfilename(i-FIRSTITEM);
						//Serial.print("Filenr:");Serial.println(i-2);
						lcd.setCursor(0,line);
						lcdprintPGM(" ");
						if(card.filenameIsDir) lcd.print("\005");
						if (card.longFilename[0])
						{
							card.longFilename[LCD_WIDTH-1] = '\0';
							lcd.print(card.longFilename);
						}
						else
						{
							lcd.print(card.filename);
						}
					}
					if((activeline==line) && CLICKED)
					{
						BLOCK
							card.getfilename(i-FIRSTITEM);
						if(card.filenameIsDir)
						{
							for(int8_t i=0; i<strlen(card.filename); i++)
								card.filename[i]=tolower(card.filename[i]);
							card.chdir(card.filename);
							lineoffset=0;
							enforceupdate=true;
						}
						else
						{
							char cmd[30];
							for(int8_t i=0; i<strlen(card.filename); i++)
								card.filename[i]=tolower(card.filename[i]);
							sprintf(cmd,"M23 %s",card.filename);
							//sprintf(cmd,"M115");
							enquecommand(cmd);
							enquecommand("M24");
							beep();
							status=Main_Status;
							if (card.longFilename[0])
							{
								card.longFilename[LCD_WIDTH-1] = '\0';
								lcd_status(card.longFilename);
							}
							else
							{
								lcd_status(card.filename);
							}
						}
					}
				}
			}
			break;
		}
		line++;
	}
	updateActiveLines(FIRSTITEM+nrfiles-1,encoderpos);
	if(enforceupdate)
	{
		force_lcd_update=true;
		enforceupdate=false;
	}
#endif
}

enum {ItemM_watch, ItemM_prepare, ItemM_control, ItemM_file, ItemM_pause,ItemM_allOff,ItemM_lastjob};
void MainMenu::showMainMenu()
{
#ifndef ULTIPANEL
	force_lcd_update=false;
#endif
	if(tune)
	{
		if(!(movesplanned() || IS_SD_PRINTING))
		{
			force_lcd_update=true;
			tune=false;
		}
	}
	else
	{
		if(movesplanned() || IS_SD_PRINTING)
		{
			force_lcd_update=true;
			tune=true;
		}
	}
	clearIfNecessary();
	uint8_t line=0;
	for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
	{
		switch(i)
		{
		case ItemM_watch:
			MENUITEM(  lcdprintPGM(MSG_WATCH)  ,  BLOCK; status=Main_Status; beepshort(); ) ;
			break;
		case ItemM_prepare:
			MENUITEM(  if(!tune) lcdprintPGM(MSG_PREPARE); else  lcdprintPGM(MSG_TUNE); ,  BLOCK; status=Main_Prepare; beepshort(); ) ;
			break;

		case ItemM_control:
			MENUITEM(  lcdprintPGM(MSG_CONTROL_ARROW)  ,  BLOCK; status=Main_Control; beepshort(); ) ;
			break;
#ifdef SDSUPPORT
		case ItemM_file:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
#ifdef CARDINSERTED
					if(CARDINSERTED)
#else
					if(true)
#endif
					{
						if(card.sdprinting)
							lcdprintPGM(MSG_STOP_PRINT);
						else
							lcdprintPGM(MSG_CARD_MENU);
					}
					else
					{
						lcdprintPGM(MSG_NO_CARD);
					}
				}
#ifdef CARDINSERTED
				if(CARDINSERTED)
#endif
					if((activeline==line)&&CLICKED)
					{
						card.printingHasFinished();
						BLOCK;
						status=Main_SD;
						beepshort();
					}
			}
			break;
		case ItemM_pause:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
#ifdef CARDINSERTED
					if(CARDINSERTED)
#else
					if(true)
#endif
					{
						if(card.sdprinting)
							lcdprintPGM(MSG_PAUSE_PRINT);
						else
							lcdprintPGM(MSG_RESUME_PRINT);
					}
					else
					{
						//lcdprintPGM(MSG_NO_CARD);
					}
				}
#ifdef CARDINSERTED
				if(CARDINSERTED)
#endif
					if((activeline==line) && CLICKED)
					{
						if(card.sdprinting)
						{
							card.pauseSDPrint();
							beepshort();
							status = Main_Status;
						}
						else
						{
							card.startFileprint();
							job.Start(true);
							beepshort();
							status = Main_Status;
						}
					}
			}
			break;
#else
		case ItemM_file:
			break;
		case ItemM_pause:
			break;
#endif

		case ItemM_allOff:
			MENUITEM(  lcdprintPGM(" All Off")  ,  BLOCK; FanSpeed=0; setTargetHotend0(0); setTargetHotend1(0); setTargetHotend2(0); setTargetBed(0); enquecommand("M84"); beepshort(); ) ;
			break;

		case ItemM_lastjob:
			MENUITEM(  lcdprintPGM(" Last Job Info")  ,  BLOCK; if (job.Count() >0) state=DONE;  beepshort(); ) ;
			status=Main_Status;
			break;

		default:
			SERIAL_ERROR_START;
			SERIAL_ERRORLNPGM(MSG_SERIAL_ERROR_MENU_STRUCTURE);
			break;
		}
		line++;
	}

	uint8_t numberOfLines = 5;
#ifdef SDSUPPORT
	numberOfLines = 5;
#else
	numberOfLines = 4;
#endif
	if (job.Count() >0) numberOfLines++;
	updateActiveLines(numberOfLines,encoderpos);
}

void MainMenu::update()
{
	static MainStatus oldstatus=Main_Menu;  //init automatically causes foce_lcd_update=true
	static long timeoutToStatus=0;
	static bool oldcardstatus=false;
#ifdef CARDINSERTED
	if((CARDINSERTED != oldcardstatus))
	{
		force_lcd_update=true;
		oldcardstatus=CARDINSERTED;
		lcd_init(); // to maybe revive the lcd if static electricty killed it.
		//Serial.println("echo: SD CHANGE");
		if(CARDINSERTED)
		{
			card.initsd();
			LCD_MESSAGEPGM(MSG_SD_INSERTED);
		}
		else
		{
			card.release();
			LCD_MESSAGEPGM(MSG_SD_REMOVED);
		}
	}
#endif

	if(status!=oldstatus)
	{
		force_lcd_update=true;
		encoderpos=0;
		lineoffset=0;

		oldstatus=status;
	}
	if( (encoderpos!=lastencoderpos) || CLICKED)
		timeoutToStatus=millis()+STATUSTIMEOUT;

	switch(status)
	{
	case Main_Status:
		{
			showStatus();
			if(CLICKED)
			{
				linechanging=false;
				BLOCK
					status=Main_Menu;
				timeoutToStatus=millis()+STATUSTIMEOUT;
			}
		}
		break;
	case Main_Menu:
		{
			showMainMenu();
			linechanging=false;
		}
		break;
	case Main_Prepare:
		{
			if(tune)
			{
				showTune();
			}
			else
			{
				showPrepare();
			}
		}
		break;
	case Sub_PrepareMove:
		{
			showAxisMove();
		}
		break;
	case Main_Control:
		{
			showControl();
		}
		break;
	case Sub_MotionControl:
		{
			showControlMotion();
		}
		break;
	case Sub_RetractControl:
		{
			showControlRetract();
		}
		break;
	case Sub_TempControl:
		{
			showControlTemp();
		}
		break;
	case Main_SD:
		{
			showSD();
		}
		break;
	case Sub_PreheatPLASettings:
		{
			showPLAsettings();
		}
		break;
	case Sub_PreheatABSSettings:
		{
			showABSsettings();
		}
		break;
	}

	if(timeoutToStatus<millis())
		status=Main_Status;
	//force_lcd_update=false;
	lastencoderpos=encoderpos;
}

enum
{
	ItemPLAPreHeat_Exit,
	ItemPLAPreHeat_set_PLA_FanSpeed,
	ItemPLAPreHeat_set_nozzle,
	ItemPLAPreHeat_set_HPB,
	ItemPLAPreHeat_Store_Eprom
};

void MainMenu::showPLAsettings()
{
#ifdef ULTIPANEL
	uint8_t line=0;
	clearIfNecessary();
	for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
	{
		switch(i)
		{
		case ItemPLAPreHeat_Exit:
			MENUITEM(  lcdprintPGM(MSG_TEMPERATURE_RTN)  ,  BLOCK; status=Sub_TempControl; beepshort(); ) ;
			break;

		case ItemPLAPreHeat_set_PLA_FanSpeed:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_FAN_SPEED);
					lcd.setCursor(13,line);
					lcd.print(ftostr3(plaPreheatFanSpeed));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=plaPreheatFanSpeed;
					}
					else
					{
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>255) encoderpos=255;
					plaPreheatFanSpeed=encoderpos;
					lcd.setCursor(13,line);
					lcd.print(itostr3(encoderpos));
				}
			}
			break;

		case ItemPLAPreHeat_set_nozzle:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_NOZZLE);
					lcd.setCursor(13,line);
					lcd.print(ftostr3(plaPreheatHotendTemp));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=plaPreheatHotendTemp;
					}
					else
					{
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>260) encoderpos=260;
					plaPreheatHotendTemp = encoderpos;
					lcd.setCursor(13,line);
					lcd.print(itostr3(encoderpos));
				}
			}
			break;

		case ItemPLAPreHeat_set_HPB:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_BED);
					lcd.setCursor(13,line);
					lcd.print(ftostr3(plaPreheatHPBTemp));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=plaPreheatHPBTemp;
					}
					else
					{
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>250) encoderpos=150;
					plaPreheatHPBTemp = encoderpos;
					lcd.setCursor(13,line);
					lcd.print(itostr3(encoderpos));
				}
			}
			break;
		case ItemPLAPreHeat_Store_Eprom:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_STORE_EPROM);
				}
				if((activeline==line) && CLICKED)
				{
					//enquecommand("M84");
					beepshort();
					BLOCK;
					EEPROM_StoreSettings();
				}
			}
			break;
		default:
			break;
		}
		line++;
	}
	updateActiveLines(ItemPLAPreHeat_Store_Eprom,encoderpos);
#endif
}

enum
{
	ItemABSPreHeat_Exit,
	ItemABSPreHeat_set_FanSpeed,
	ItemABSPreHeat_set_nozzle,
	ItemABSPreHeat_set_HPB,
	ItemABSPreHeat_Store_Eprom
};

void MainMenu::showABSsettings()
{
#ifdef ULTIPANEL
	uint8_t line=0;
	clearIfNecessary();
	for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
	{
		switch(i)
		{
		case ItemABSPreHeat_Exit:
			MENUITEM(  lcdprintPGM(MSG_TEMPERATURE_RTN)  ,  BLOCK; status=Sub_TempControl; beepshort(); ) ;
			break;

		case ItemABSPreHeat_set_FanSpeed:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_FAN_SPEED);
					lcd.setCursor(13,line);
					lcd.print(ftostr3(absPreheatFanSpeed));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=absPreheatFanSpeed;
					}
					else
					{
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>255) encoderpos=255;
					absPreheatFanSpeed=encoderpos;
					lcd.setCursor(13,line);
					lcd.print(itostr3(encoderpos));
				}
			}
			break;

		case ItemABSPreHeat_set_nozzle:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_NOZZLE);
					lcd.setCursor(13,line);
					lcd.print(ftostr3(absPreheatHotendTemp));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=absPreheatHotendTemp;
					}
					else
					{
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>260) encoderpos=260;
					absPreheatHotendTemp = encoderpos;
					lcd.setCursor(13,line);
					lcd.print(itostr3(encoderpos));
				}
			}
			break;

		case ItemABSPreHeat_set_HPB:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_BED);
					lcd.setCursor(13,line);
					lcd.print(ftostr3(absPreheatHPBTemp));
				}

				if((activeline!=line) )
					break;

				if(CLICKED)
				{
					linechanging=!linechanging;
					if(linechanging)
					{
						encoderpos=absPreheatHPBTemp;
					}
					else
					{
						encoderpos=activeline*lcdslow;
						beepshort();
					}
					BLOCK;
				}
				if(linechanging)
				{
					if(encoderpos<0) encoderpos=0;
					if(encoderpos>250) encoderpos=150;
					absPreheatHPBTemp = encoderpos;
					lcd.setCursor(13,line);
					lcd.print(itostr3(encoderpos));
				}
			}
			break;
		case ItemABSPreHeat_Store_Eprom:
			{
				if(force_lcd_update)
				{
					lcd.setCursor(0,line);
					lcdprintPGM(MSG_STORE_EPROM);
				}
				if((activeline==line) && CLICKED)
				{
					//enquecommand("M84");
					beepshort();
					BLOCK;
					EEPROM_StoreSettings();
				}
			}
			break;
		default:
			break;
		}
		line++;
	}
	updateActiveLines(ItemABSPreHeat_Store_Eprom,encoderpos);
#endif
}

void MainMenu::updateActiveLines( const uint8_t &maxlines,volatile long &encoderpos )
{
	if(linechanging) return; // an item is changint its value, do not switch lines hence
	lastlineoffset=lineoffset;
	long curencoderpos=encoderpos;
	force_lcd_update=false;
	if(  (abs(curencoderpos-lastencoderpos)<lcdslow) )
	{
		lcd.setCursor(0,activeline);lcd.print((activeline+lineoffset)?' ':' ');
		if(curencoderpos<0)
		{
			lineoffset--;
			if(lineoffset<0) lineoffset=0;
			curencoderpos=lcdslow-1;
		}
		if(curencoderpos>(LCD_HEIGHT-1+1)*lcdslow)
		{
			lineoffset++;
			curencoderpos=(LCD_HEIGHT-1)*lcdslow;
			if(lineoffset>(maxlines+1-LCD_HEIGHT))
				lineoffset=maxlines+1-LCD_HEIGHT;
			if(curencoderpos>maxlines*lcdslow)
				curencoderpos=maxlines*lcdslow;
		}
		lastencoderpos=encoderpos=curencoderpos;
		activeline=curencoderpos/lcdslow;
		if(activeline<0) activeline=0;
		if(activeline>LCD_HEIGHT-1) activeline=LCD_HEIGHT-1;
		if(activeline>maxlines)
		{
			activeline=maxlines;
			curencoderpos=maxlines*lcdslow;
		}
		if(lastlineoffset!=lineoffset)
			force_lcd_update=true;
		lcd.setCursor(0,activeline);lcd.print((activeline+lineoffset)?'>':'\003');
	}
}

void MainMenu::clearIfNecessary()
{
	if(lastlineoffset!=lineoffset ||force_lcd_update)
	{
		force_lcd_update=true;
		lcd.clear();
	}
}

void MainMenu::updateFloatDisplay( int line, float value, int a, int b )
{
	int pos = 12;
	pos-=max(5-(a+b),0);
	lcd.setCursor(pos,line);
	lcd.print(ftostr (value,a,b,false));
}


/*
void MainMenu::updateIntDisplay( int line, int value )
{
	lcd.setCursor(13,line);
	lcd.print(itostr5 (value));
}*/

void MainMenu::HandleAdjustment( uint8_t line, const char * str, float & t , int min/*=1*/, int max/*=990*/,int a/*=4*/, int b /*=1 */ )
{
	if(force_lcd_update)
	{
		lcd.setCursor(0,line);
		lcd.print(str);
		updateFloatDisplay(line, t,a,b);	
	}

	if((activeline!=line) )
		return;
	static float encoder_step = 1;
	if(CLICKED)
	{
		linechanging=!linechanging;
		// starting to change value -- load the current value into encoderpos
		if(linechanging)
		{
			encoder_step = stepsize(t);
			encoderpos = (long) t / encoder_step;
		}
		else
		{
			t=constrain (encoderpos*encoder_step,min,max);
			encoderpos=activeline*lcdslow;
		}
		BLOCK;
		beepshort();
	}
	if(linechanging)
	{
		float t2= constrain (encoderpos*encoder_step,min,max);
		encoderpos = (long) (t2 / encoder_step);
		if (stepsize (t2)!=encoder_step)
		{
			encoder_step = stepsize(t2);
			encoderpos = (long) t2 / encoder_step;
		}
		updateFloatDisplay(line, t2,a,b);	
	}	
}


#endif //ULTRA_LCD