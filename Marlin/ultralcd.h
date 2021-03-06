#ifndef ULTRALCD_H
#define ULTRALCD_H
#include "Marlin.h"
#include "Utility.h"
#include  "Watchdog.h"


#define LCD_UPDATE_INTERVAL 350

// priority number determines what gets seen and for how long.  time = priority * LCD_UPDATE_INTERVAL (in ms)
#define ALERT_PRIORITY 100
#define USER_MESSAGE_PRIORITY 40
#define WARNING_MESSAGE_PRIORITY 40
#define ERROR_MESSAGE_PRIORITY 200
#define DEFAULT_MESSAGE_PRIORITY 20

#ifdef ULTRA_LCD




#ifdef PCF8574T_LCD
#include <LiquidCrystal_I2C.h>
#endif
#ifndef PCF8574T_LCD
#include <LiquidCrystal.h>
#endif

void lcd_status();
void lcd_init();
void lcd_status(const char* message, int priority=DEFAULT_MESSAGE_PRIORITY);
void lcd_status(const String message, int priority=DEFAULT_MESSAGE_PRIORITY);
void lcd_clear_message(int priority=-1);
void beep();
void beepshort();
void buttons_init();
void buttons_check();

#define STATUSTIMEOUT 15000


#ifdef GFX_128x64_OLED
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// extern Adafruit_SSD1306  lcd;
#endif


#ifdef MCP23017_LCD
extern LiquidTWI2 lcd;
#else
#ifdef PCF8574T_LCD
extern LiquidCrystal_I2C lcd;
#else
#ifndef GFX_128x64_OLED
extern LiquidCrystal lcd;
#endif
#endif

#endif

extern volatile char buttons;  //the last checked buttons in a bit array.

#ifdef NEWPANEL
#define EN_C (1<<BLEN_C)
#define EN_B (1<<BLEN_B)
#define EN_A (1<<BLEN_A)

#define CLICKED (buttons&EN_C)
#define BLOCK {blocking=millis()+blocktime;}
#if (SDCARDDETECT > -1)
#ifdef SDCARDDETECTINVERTED
#define CARDINSERTED (READ(SDCARDDETECT)!=0)
#else
#define CARDINSERTED (READ(SDCARDDETECT)==0)
#endif
#endif  //SDCARDTETECTINVERTED

#else

//atomatic, do not change
#define B_LE (1<<BL_LE)
#define B_UP (1<<BL_UP)
#define B_MI (1<<BL_MI)
#define B_DW (1<<BL_DW)
#define B_RI (1<<BL_RI)
#define B_ST (1<<BL_ST)
#define EN_B (1<<BLEN_B)
#define EN_A (1<<BLEN_A)

#define CLICKED ((buttons&B_MI)||(buttons&B_ST))
#define BLOCK {blocking[BL_MI]=millis()+blocktime;blocking[BL_ST]=millis()+blocktime;}

#endif

// blocking time for recognizing a new keypress of one key, ms
#define blocktime 500
#define lcdslow 5

enum MainStatus
{
	Main_Status,
	Main_Menu, 
	Main_Prepare,
	Sub_PrepareMove, 
	Main_Control, 
	Main_SD,
	Sub_TempControl,
	Sub_MotionControl,
	Sub_RetractControl, 
	Sub_PreheatPLASettings, 
	Sub_PreheatABSSettings
};

class MainMenu
{
public:
	MainMenu();
	void update();
	int8_t activeline;
	MainStatus status;
	uint8_t displayStartingRow;

	void displayTemps();
	void showStatus();
	void showMainMenu();
	void showPrepare();
	void showTune();
	void showControl();
	void showControlMotion();

	void HandleAdjustment ( uint8_t line, const char * str, float & t , int min=1, int max=990,int a=4, int b =1 );
	//float HandleAdjustment ( uint8_t line, const char * str, long & t , int min=1, int max=990);

	void updateFloatDisplay( int line, float value , int a=4, int b =1);


	void showControlTemp();
	void showControlRetract();
	void showAxisMove();
	void showSD();
	void showPLAsettings();
	void showABSsettings();
	bool force_lcd_update;
	long lastencoderpos;
	int8_t lineoffset;
	int8_t lastlineoffset;

	bool linechanging;

	bool tune;

private:
	void updateActiveLines(const uint8_t &maxlines,volatile long &encoderpos);

	void clearIfNecessary();
	void updateIntDisplay( int line, int value );
};

/*
//conversion routines, could need some overworking
char *ftostr51(const float &x);
char *ftostr52(const float &x);
char *ftostr31(const float &x);
char *ftostr3(const float &x);*/

#define LCD_INIT lcd_init();
#define LCD_MESSAGE(x) lcd_status(x);
#define LCD_MESSAGEPRI(x,y) lcd_status(x,y);
#define LCD_MESSAGEPGMPRI(x,y) lcd_statuspgm(MYPGM(x),y);
#define LCD_MESSAGEPGM(x) lcd_statuspgm(MYPGM(x));
#define LCD_ALERTMESSAGEPGM(x) lcd_alertstatuspgm(MYPGM(x),-1);
#define LCD_STATUS lcd_status()
#define LCD_MESSAGE_CLEAR lcd_clear_message()
#define LCD_MESSAGE_CLEARPRI(x) lcd_clear_message(x);
#else //no lcd
#define LCD_INIT
#define LCD_STATUS
#define LCD_MESSAGE(x)
#define LCD_MESSAGEPGM(x)
#define LCD_ALERTMESSAGEPGM(x)
#define LCD_MESSAGE_CLEAR
#define LCD_MESSAGEPRI(x,y) 
#define LCD_MESSAGEPGMPRI(x,y)
#define LCD_MESSAGE_CLEARPRI(x) 

FORCE_INLINE void lcd_status() {};

#define CLICKED false
#define BLOCK ;
	//void beep() {};

#endif

void lcd_statuspgm(const char* message, int priority = DEFAULT_MESSAGE_PRIORITY);
void lcd_alertstatuspgm(const char* message);

#endif //ULTRALCD
