#include "Marlin.h"
#include "ultralcd.h"
#include "BlinkM_funcs.h"


//return for string conversion routines
static char conv[10];

//**********************************************************************************************************
//  convert float to string with 123 format
char *ftostr3(const float &x)
{
    return ftostr (x,3,0,false);

}

// -------------------------------------------------------------------------------------------------------------------------------------
char *itostr2(const uint8_t &x)
{
    //sprintf(conv,"%5.1f",x);
    int xx=x;
    conv[0]=(xx/10)%10+'0';
    conv[1]=(xx)%10+'0';
    conv[2]=0;
    return conv;
}

// -------------------------------------------------------------------------------------------------------------------------------------
char *ftostr(float x,int pre,int post, bool sign)
{
    static int powers[7]=
    {
        1,
        10,
        100,
        1000,
        10000,
        100000,
        1000000
    };
    long xx=x*powers[post];
    int pos =0 ;
    if (sign)
        {
            conv[pos++]=(xx>=0)?'+':'-';
        }
    xx=abs(xx);
    for (; pre >0; pre--)
        conv[pos++]=(xx / powers[pre+post-1]) % 10+'0';
    if (post>0)
        {
            conv[pos++]='.';
            for (; post >0; post--)
                conv[pos++]=(xx / powers[pre+post-1]) % 10+'0';
        }
    conv[pos++]=0;
    return conv;
}

//  convert float to string with +123.4 format
char *ftostr31(const float &x)
{
    return ftostr (x, 3,1,true);


}

// no sign 12.345
char *ftostr23a(const float &x)
{
    return ftostr (x, 2,3,false);

}

// no sign 123.45
char *ftostr32a(const float &x)
{
    return ftostr (x, 3,2,false);

}

char *ftostr32(const float &x)
{
    return ftostr (x, 3,2,true);

}

char *itostr31(const int &xx)
{
    conv[0]=(xx>=0)?'+':'-';
    conv[1]=(xx/1000)%10+'0';
    conv[2]=(xx/100)%10+'0';
    conv[3]=(xx/10)%10+'0';
    conv[4]='.';
    conv[5]=(xx)%10+'0';
    conv[6]=0;
    return conv;
}

char *itostr3(const int &xx)
{
    conv[0]=(xx/100)%10+'0';
    conv[1]=(xx/10)%10+'0';
    conv[2]=(xx)%10+'0';
    conv[3]=0;
    return conv;
}

char *itostr4(const int &xx)
{
    conv[0]=(xx/1000)%10+'0';
    conv[1]=(xx/100)%10+'0';
    conv[2]=(xx/10)%10+'0';
    conv[3]=(xx)%10+'0';
    conv[4]=0;
    return conv;
}

char *itostr5(const int &xx)
{
    conv[0]=(xx/10000)%10+'0';
    conv[1]=(xx/1000)%10+'0';
    conv[2]=(xx/100)%10+'0';
    conv[3]=(xx/10)%10+'0';
    conv[4]=(xx)%10+'0';
    conv[5]=0;
    return conv;
}

//  convert float to string with +1234.5 format
char *ftostr51(const float &x)
{
    return ftostr (x, 5,1,true);

}

//  convert float to string with +123.45 format
char *ftostr52(const float &x)
{
    return ftostr (x, 5,2,true);

}

//  convert float to string with 12345.6 format
char *ftostr52a(const float &x)
{
    return ftostr (x, 5,2,false);

}

//  convert float to string with 12345.6 format
char *ftostr61(const float &x)
{
    return ftostr (x, 6,1,false);

}

//-------------------------------------------------------------------------------------------------------------------
//************************************
// Method:    EchoTimeSpan
// FullName:  EchoTimeSpan
// Access:    public
// Returns:   char* predfined buffer
// Qualifier:
// Parameter: long t  time in seconds
// Parameter: bool shortform if true, use concise HH:MM:SS or DD;HH:MM format
//************************************
char* EchoTimeSpan (long t, bool shortform)
{
    static char timespan_buffer[26];
    int sec,min,hr;
    hr = t/3600;
    t-=hr*3600;
    //if (hr>99) hr=99;
    min=t/60;
    t-=min*60;
    min%=60;

    sec=t%60;
    if (hr <= 99)
        {
            if (shortform)
                sprintf(timespan_buffer,"%02d:%02d:%02d",(hr),(min),(sec));
            else
                sprintf(timespan_buffer,"%i hrs, %i min, %i sec",hr,min,sec);
        }
    else
        {
            int days =hr / 24;
            hr -= days * (24);
            if (shortform)
                sprintf(timespan_buffer,"%sdays",ftostr((float) t / (3600*24.0),3,1));
            else
                sprintf(timespan_buffer,"%i days %i hrs, %i min, %i sec",days, hr,min,sec);
        }

    return timespan_buffer;
};




//-------------------------------------------------------------------------------------------------------------------
// uses I2C RGB LED since we have only one PWM pin available
// like the BlinkM
void SetStatusLEDColor (byte r, byte g, byte b, bool fade)
{

#ifdef BLINK_M

    const byte sclPin = 7;  // digital pin 7 wired to 'c' on BlinkM
    const byte sdaPin = 6;  // digital pin 6 wired to 'd' on BlinkM
    const byte pwrPin = 5;  // digital pin 5 wired to '+' on BlinkM
    const byte gndPin = 4;  // digital pin 4 wired to '-' on BlinkM
    BlinkM_stopScript(BLINK_M_ADDR);
    if (fade)
        BlinkM_fadeToRGB(BLINK_M_ADDR,r,g,b);
    else
        BlinkM_setRGB(BLINK_M_ADDR,r,g,b);
#endif
    /*
    	SERIAL_ECHO_START;
    	SERIAL_ECHO(" R=");
    	SERIAL_ECHO (r);
    	SERIAL_ECHO ("  G=");
    	SERIAL_ECHO(g);
    	SERIAL_ECHO(" B=");
    	SERIAL_ECHOLN(b);
    */
}


// -------------------------------------------------------------------------------------------------------------------------------------
unsigned long CalculateRemainingTime( float percent_complete,unsigned long elapsed_time )
{
    unsigned long total_time = elapsed_time / ( percent_complete / 100.0);
    return total_time - elapsed_time;
}

//-------------------------------------------------------------------------------------------------------------------
extern "C" {
    extern unsigned int __bss_end;
    extern unsigned int __heap_start;
    extern void *__brkval;

    // -------------------------------------------------------------------------------------------------------------------------------------
    int freeMemory()
    {
        int free_memory;

        if((int)__brkval == 0)
            free_memory = ((int)&free_memory) - ((int)&__bss_end);
        else
            free_memory = ((int)&free_memory) - ((int)__brkval);

        return free_memory;
    }
}

// -------------------------------------------------------------------------------------------------------------------------------------
void MyTone( int pin, int freq, int dur )
{
    int a;
    pinMode(pin, OUTPUT);
    for (a=0; a<1000; a++)
        {
            digitalWrite(pin, 255);
            //		analogWrite(pin, 255);
            delay(3);
            digitalWrite(pin, 0);
            //		analogWrite(pin, 0);
            delay(3);
            LCD_STATUS;
        }
}

// -------------------------------------------------------------------------------------------------------------------------------------
void Error( char * message )
{
    LCD_MESSAGE_CLEAR;
    //SetLEDColor(ERROR_COLOR);
    state = ERROR;
    error_beep();
    SERIAL_ECHOLN(message);
    LCD_MESSAGEPRI (message,ERROR_MESSAGE_PRIORITY);
}



// -------------------------------------------------------------------------------------------------------------------------------------
void Warning( char * message )
{
    SetStatusLEDColor(WARNING_COLOR);
    warning_beep();
    SERIAL_ECHOLN(message);
    LCD_MESSAGEPRI (message, WARNING_MESSAGE_PRIORITY);
}



// -------------------------------------------------------------------------------------------------------------------------------------
void error_beep()
{
    ;

#ifdef ULTRA_LCD
    beep();
#endif
}

void warning_beep()
{
    ;
#ifdef ULTRA_LCD
    beepshort();
#endif
}