// control struct
// tallennus eprom
// mapedit
// adaptation
// map upload ctrl seq !m8x8010203040506...!

// 2011-06-28 Servo is disabled because they do not last long, use N75 or similiar

#include <avr/pgmspace.h>
#include <EEPROM.h>
//#include <Servo.h>
/*
// oma vanha setuppi
#define PIN_BUTTON A5
#define PIN_HEARTBEAT 13
#define PIN_MAP A1
#define PIN_TPS A0
#define PIN_RPM_TRIGGER 2
#define PIN_VNT_N75 9    
#define PIN_LDA_N75 8
#define PIN_TEMP1 A2
#define PIN_TEMP2 A3
#define PIN_OUTPUT1 7
#define PIN_OUTPUT2 10

#define PIN_LCD_RS 11 // white  // PINK +5V // GRAY gnd
#define PIN_LCD_ENABLE 12  // red
#define PIN_LCD_D4 3  // BLUE
#define PIN_LCD_D5 4  // YELLOW
#define PIN_LCD_D6 5  // GREEN
#define PIN_LCD_D7 6  // BROWN
*/
  // new defaults
#define PIN_BUTTON A5
#define PIN_HEARTBEAT 13
#define PIN_MAP A1
#define PIN_TPS A0
#define PIN_RPM_TRIGGER 2
#define PIN_VNT_N75 9    
#define PIN_LDA_N75 8
#define PIN_TEMP1 A2
#define PIN_TEMP2 A3
#define PIN_OUTPUT1 7
#define PIN_OUTPUT2 10

#define PIN_LCD_RS 11 // white  // PINK +5V // GRAY gnd
#define PIN_LCD_ENABLE 12  // red
#define PIN_LCD_D4 3  // BLUE
#define PIN_LCD_D5 4  // YELLOW
#define PIN_LCD_D6 5  // GREEN
#define PIN_LCD_D7 6  // BROWN

#include <LiquidCrystal.h>

LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_ENABLE, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

// Calculate avarage values 
#define AVG_MAX 15 

#define STATUS_IDLE 1
#define STATUS_CRUISING 2

//Servo servoVnt;
//Servo servoLda;

#define MAP_AXIS_RPM 0xDE
#define MAP_AXIS_TPS 0xAD
#define MAP_AXIS_CBAR 0xDD
#define MAP_AXIS_CELSIUS 0xAA
#define MAP_AXIS_VOLTAGE 0xAB
#define MAP_AXIS_DUTY_CYCLE 0xAC
#define MAP_AXIS_RAW 0x0

unsigned char vntActuatorPressureRequestMap[] = {
    'M','2','D',
    0x8,0x8,0xDE,0xAD,0xDD,     // 01 - new version 
    85,85,85,85,85,85,85,85,
    85,85,85,85,85,85,85,85,
    85,85,85,85,85,85,85,85,
    85,85,85,85,85,85,85,85,
    85,85,85,85,85,85,85,85,
    85,85,85,85,85,85,85,85,
    85,85,85,85,85,85,85,85,
    85,85,85,85,85,85,85,85,
    00,00,00,                  // lastX,lastY,lastRet
};

unsigned char vntActuatorPositionLimitMap[] = {
    'M','2','D',
    0x8,0x8,0xDE,0xAD,0xAC,
    255,255,255,255,  255,255,255,255,
    255,255,255,255,  255,255,255,255,
    255,255,255,255,  255,255,255,255,
    255,255,255,255,  255,255,255,255,
    255,255,255,255,  255,255,255,255,
    255,255,255,255,  255,255,255,255,
    255,255,255,255,  255,255,255,255,
    255,255,255,255,  255,255,255,255,
    00,00,00,                  // lastX,lastY,lastRet
};

unsigned char ldaPositionMap[] = {
    'M','2','D',
    0x8,0x8,0xDE,0xDD,0xAC,
    0,0,0,0, 0,0,0,0,
    32,32,32,32, 32,32,32,32,
    64,64,64,64, 64,64,64,64,
    128,128,128,128, 128,128,128,128,
    192,192,192,192, 192,192,192,192,
    255,255,255,255, 255,255,255,255,
    255,255,255,255, 255,255,255,255,
    255,255,255,255, 255,255,255,255,
    00,00,00,                  // lastX,lastY,lastRet
};

// R1 = 2400ohm, R2 = old style Bosch temp sensor
unsigned char tempMap[] = {
    'M','1','D',
    0x8,0x1,0xAB,0x00,0xAA,
    // values at -64,-32,0,32, 64,96,128,160 °C
    175+64,120+64,90+64,62+64,44+64,30+64,6+64,-55+64,
//    255,227,179,109,51,19,9,0,  Calculated from bosch reference, does not seems to be accurate?
    00,00,00,                  // lastX,lastY,lastRet
};

// Use slighly logaritmic output positions instead of plain linear
/*unsigned char vntActuatorMovementRemap[] = {
 'M','1','D',
 0x8,0x0,0xDE,0xAD,0x00,
 0,2,8,20,50,90,180,255
 };
 */
unsigned char vntActuatorMovementRemap[] = {
    'M','1','D',
    0x8,0x1,0x0,0x00,0x00,
    0,0,10,20,90,140,190,255,
    00,00,00,  
    //  0x8,0x0,0xDE,0xAD,0x00,
    //0,10,50,110, 130,140,176,255 // Slight S-shape
    //  0,0,4,13,32,110,130,255
    //0,0,0,0,30,90,170,255
};


// Also used in NVRAM data store magic header
prog_uchar versionString[] PROGMEM  = "DmnDslCtrl v1.4."; 
prog_uchar statusString1[] PROGMEM  = " Active view: ";

#define MODE_VANESOPENIDLE 1
#define MODE_DUTYCYCLEMAP 2
#define MODE_VNTOUTPUTINVERTED 4
#define MODE_LDAOUTPUTINVERTED 8

#define MAIN_LOOP_DELAY (1000/20) // ms
#define TEMP_HYSTERESIS 3

// used for RPM counting
volatile unsigned long rpmLastTeethSeenTime = 0;
volatile unsigned long rpmNow = 0;

// contains configurable data. Can be stored in eeprom
struct settingsStruct {
    int tpsMin;
    int tpsMax;
    int mapMin;
    int mapMax;
    int rpmMax; 	
    int rpmTeethsPerRotation;
    unsigned char vntMinDC;
    unsigned char vntMaxDC;
    unsigned char ldaMinDC;
    unsigned char ldaMaxDC;  
    unsigned char mode;
    unsigned char inDamp;
    unsigned char outDamp;
    unsigned char servoStabilizerThreshold;
    unsigned char servoStabilizerSmoothValue;
    int output1EnableTemp;
    int output2EnableTemp;
};

settingsStruct settings;

//  contains calculated output data. calculated each run of mainloop
struct controlsStruct {
    // inputs
    volatile int tpsInput;
    unsigned char tpsCorrected;
    volatile int mapInput;
    unsigned char mapCorrected;
    
    // outputs
    unsigned char ldaPosition;
    unsigned char ldaPositionRemapped;
    
    unsigned char vntTargetPressure;
    unsigned char vntPosition;
    
    unsigned char vntPositionRemapped;  
    unsigned char vntPositionTarget;
    // calculated value
    volatile int rpmActual;
    volatile unsigned char rpmCorrected;
    unsigned char statusBits;
    //bool idling;
    int temp1;
    int temp2;
    char output1Enabled;
    char output2Enabled;
    unsigned char clipPos;
};

controlsStruct controls;

struct avgStruct {
    unsigned char pos;
    unsigned char size;
    volatile unsigned int avgData[AVG_MAX]; 
};

avgStruct tpsAvg;
avgStruct mapAvg;


char buffer[100]; // general purpose buffer, mainly used for string storage when printing from flash
unsigned long lastPacketTime;
prog_uchar mapVisualitionHelp[] PROGMEM  = "Top Left is 0,0 (press: L - toggle live mode)";

unsigned char page=0;
char *pages[] = {
    "About","Adaptation","Actuator Fine-tune","Edit: vntActuatorPressureRequestMap","Edit: vntActuatorPositionLimitMap","Edit: ldaPositionMap","Edit: temperatureMap","Edit: vntActuatorMovementRemap","Export data","Output Tests"};
unsigned char *editorMaps[]={vntActuatorPressureRequestMap,vntActuatorPositionLimitMap,ldaPositionMap,tempMap,vntActuatorMovementRemap};


unsigned char clearScreen[] =  {
    27,'[','2','J',27,'[','H'};

prog_uchar ANSIclearEolAndLf[] PROGMEM = {
    27,'[','K','\r','\n',0};
prog_uchar ANSIgoHome[] PROGMEM = {
    27,'[','1',';','1','H',0};
prog_uchar ANSIclearEos[] PROGMEM = {
    27,'[','J',0};

void setPwmFrequency(int pin, int divisor) {
    byte mode;
    if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
        switch(divisor) {
            case 1: mode = 0x01; break;
            case 8: mode = 0x02; break;
            case 64: mode = 0x03; break;
            case 256: mode = 0x04; break;
            case 1024: mode = 0x05; break;
            default: return;
        }
        if(pin == 5 || pin == 6) {
            TCCR0B = TCCR0B & 0b11111000 | mode;
        } else {
            TCCR1B = TCCR1B & 0b11111000 | mode;
        }
    } else if(pin == 3 || pin == 11) {
        switch(divisor) {
            case 1: mode = 0x01; break;
            case 8: mode = 0x02; break;
            case 32: mode = 0x03; break;
            case 64: mode = 0x04; break;
            case 128: mode = 0x05; break;
            case 256: mode = 0x06; break;
            case 1024: mode = 0x7; break;
            default: return;
        }
        TCCR2B = TCCR2B & 0b11111000 | mode;
    }
}

void setup() {
    digitalWrite(PIN_HEARTBEAT,HIGH);
    lcd.begin(16, 2);
    // Print a message to the LCD.
    strcpy_P(buffer, (PGM_P)&versionString);   
    lcd.print(buffer);  
    lcd.setCursor(0, 1);
    
    Serial.begin(115200);
    //	Serial.begin(19200);
    Serial.print("Boot:");
    
    pinMode(PIN_HEARTBEAT,OUTPUT); // DEBUG led
    pinMode(PIN_BUTTON,INPUT); // Reset switch
    digitalWrite(PIN_BUTTON, HIGH);  // activate pull up resistor
    digitalWrite(PIN_RPM_TRIGGER,HIGH); // pullup for honeywell
    attachInterrupt(0, rpmTrigger, RISING); // or falling!
    setPwmFrequency(9, 64); // was 1024
 //   setPwmFrequency(9, 32); // was 1024
    pinMode(PIN_OUTPUT1,OUTPUT);
    pinMode(PIN_OUTPUT2,OUTPUT);
    
    // servoLda.attach(PIN_LDA_SERVO);
    // servoVnt.attach(PIN_VNT_SERVO);
    digitalWrite(PIN_LDA_N75,LOW);
    digitalWrite(PIN_VNT_N75,LOW);
    
    lcd.print("Load:");
    Serial.print("OK, Load:");
    if (loadFromEEPROM(false) == false) {
        Serial.print("skipped.");
        lcd.print("skipped.");
        loadDefaults();
        delay(2000);
    } 
    else {
        Serial.println("OK");
        lcd.print("OK.");

         // sweep at startup using fixed settings (temp enable 2.9.2011 to figure out stuck n75)
         controls.ldaPositionRemapped = 255;
         controls.vntPositionRemapped = 255;
         updateOutputValues(true);
         delay(300);
         controls.ldaPositionRemapped = 0;
         controls.vntPositionRemapped = 0;
         updateOutputValues(true);
         delay(300);  
         
        
    }
    Serial.println("\r\n");
    Serial.write(clearScreen,sizeof(clearScreen));
    
    tpsAvg.size=AVG_MAX;
    mapAvg.size=AVG_MAX;
    
    digitalWrite(PIN_HEARTBEAT,LOW);  

    pageAbout(1); // force output
}

void loadDefaults() {
    memset(&settings,0,sizeof(settingsStruct));
    settings.tpsMax = 1023;
    settings.mapMax = 1023;
    settings.rpmTeethsPerRotation = 6;
    settings.rpmMax = 6000;
    settings.vntMinDC = 100;
    settings.vntMaxDC = 110;
    settings.ldaMinDC = 100;
    settings.ldaMaxDC = 110;
    settings.mode = 0;  
    settings.inDamp = 20;
    settings.outDamp = 20;
    settings.servoStabilizerThreshold=70;
    settings.servoStabilizerSmoothValue=14;
    settings.output1EnableTemp = 85;
    settings.output2EnableTemp = 85; 
}

unsigned char mapValues(int raw,int mapMin,int mapMax) {
    if (raw < mapMin)
        return 0;
    if (raw >= mapMax)
        return 0xff;
    
    return map(raw,mapMin,mapMax,0,255);
}

unsigned char mapValuesSqueeze(int raw,int mapMin,int mapMax) {
    return map(raw,0,255,mapMin,mapMax);
}

unsigned char mapInterpolate(unsigned char p1,unsigned char p2, unsigned char pos) {
    return (p1*(100-pos)+p2*pos)/100;
}

unsigned char mapLookUp(unsigned char *mapData,unsigned char x,unsigned char y) {
    unsigned char isInterpolated = *(mapData+2);
    unsigned char tableSizeX = *(mapData+3);
    unsigned char tableSizeY = *(mapData+4);
    unsigned char yPos;
    *(mapData+8+tableSizeX*tableSizeY) = x;
    *(mapData+8+tableSizeX*tableSizeY+1) = y;

    if (tableSizeY) {
        yPos = y / (256/(tableSizeY-1));
    } 
    else {
        yPos = 0;
    }
    unsigned char xPos = (x / (256/(tableSizeX-1)));
    int ofs = 8; // skip headers
    
    unsigned char p1 = *(mapData+ofs+(yPos*tableSizeX)+xPos);
    unsigned char p2 = *(mapData+ofs+(yPos*tableSizeX)+(((xPos+1)>=tableSizeX)?xPos:xPos+1));
    ;
    unsigned char p3 = *(mapData+ofs+((((yPos+1)>=tableSizeX)?yPos:yPos+1)*tableSizeX)+xPos);
    unsigned char p4 = *(mapData+ofs+((((yPos+1)>=tableSizeX)?yPos:yPos+1)*tableSizeX)+(((xPos+1)>=tableSizeX)?xPos:xPos+1));
    
    unsigned char ret;
    if (isInterpolated == 'D') {
        int amountX = (x % (256/(tableSizeX-1)))*(10000/(256/(tableSizeX-1)));
        if (tableSizeY) {
            // 2D
            int amountY = (y % (256/(tableSizeY-1)))*(10000/(256/(tableSizeY-1)));
            char y1 = mapInterpolate(p1,p2,amountX /100);
            char y2 = mapInterpolate(p3,p4,amountX /100);
            ret = mapInterpolate(y1,y2,amountY /100);
        } 
        else {
            // 1D
            ret = mapInterpolate(p1,p2,amountX /100);
        }
    } 
    else {
        ret = p1;
    }
    *(mapData+8+tableSizeX*tableSizeY+2) = ret;
    return ret;
}


char mapDebugCharValue(unsigned char c) {
    if (c<5) {
        return ' ';
    } 
    else if (c<20) {
        return '.';
    } 
    else if (c<60) {
        return ':';
    } 
    else if (c<128) {
        return '!';
    } 
    else if (c<180) {
        return 'o';
    } 
    else if (c<220) {
        return 'O';
    } 
    else  {
        return '@';
    }
}


// Fetches and print string from flash to preserve some ram!
void printFromFlash(prog_uchar *str) {
    strcpy_P(buffer, (PGM_P)str);   
    Serial.print(buffer);
}

int EEPROMwriteData(int offset, byte *ptr,int size) {
    int i;
    for (i = 0; i < size; i++)
        EEPROM.write(offset++, *(ptr++));
    return i;
}

int EEPROMreadData(int offset, byte *ptr,int size) {
    int i;
    for (i = 0; i < size; i++)
        *(ptr++) = EEPROM.read(offset++);
    return i;
}

void saveToEEPROM() {
    int ofs=0;
    // write magic header
    strcpy_P(buffer, (PGM_P)&versionString);   
    ofs += EEPROMwriteData(0,(byte*)&buffer,strlen(buffer));
    // write control struct
    ofs += EEPROMwriteData(ofs,(byte*)&settings,sizeof(settingsStruct));
    ofs += EEPROMwriteData(ofs,(byte*)&vntActuatorPressureRequestMap,sizeof(vntActuatorPressureRequestMap));
    ofs += EEPROMwriteData(ofs,(byte*)&vntActuatorPositionLimitMap,sizeof(vntActuatorPositionLimitMap));
    ofs += EEPROMwriteData(ofs,(byte*)&vntActuatorMovementRemap,sizeof(vntActuatorMovementRemap));
    ofs += EEPROMwriteData(ofs,(byte*)&tempMap,sizeof(tempMap));
    ofs += EEPROMwriteData(ofs,(byte*)&ldaPositionMap,sizeof(ldaPositionMap));
    Serial.print(ofs,DEC);
    Serial.print("SAVED");
    delay(1000); 
}

bool loadFromEEPROM(bool force) {
    int ofs;
    // if reset pin is active, no not load anything from eeprom
    if (digitalRead(PIN_BUTTON) == 0) {
        Serial.print("PIN_BUTTON active..");
        delay(2000);
        return false;
    }
    // Check magic header to prevent data corruption of blank board or wrong version save file
    if (!force) {
        strcpy_P(buffer, (PGM_P)&versionString);   
        for (ofs=0;ofs<strlen(buffer);ofs++) {
            if (EEPROM.read(ofs) != buffer[ofs])
                return false;
        }
    }
    ofs = strlen(buffer);
    ofs += EEPROMreadData(ofs,(byte*)&settings,sizeof(settingsStruct));
    ofs += EEPROMreadData(ofs,(byte*)&vntActuatorPressureRequestMap,sizeof(vntActuatorPressureRequestMap));
    ofs += EEPROMreadData(ofs,(byte*)&vntActuatorPositionLimitMap,sizeof(vntActuatorPositionLimitMap));
    ofs += EEPROMreadData(ofs,(byte*)&vntActuatorMovementRemap,sizeof(vntActuatorMovementRemap));
    ofs += EEPROMreadData(ofs,(byte*)&tempMap,sizeof(tempMap));
    ofs += EEPROMreadData(ofs,(byte*)&ldaPositionMap,sizeof(ldaPositionMap));
    
    return true;
}

int toBars(int raw) {
    return raw*1.18; // defaults for 3bar map
}

int toTemperature( int rawValue) {
    int ret = mapLookUp(tempMap,rawValue,0);
    return ret-64;
}

int toVoltage(int raw) {
    // mVolt
    return int(raw*19.608);
}

int toRpm(int raw) {
    return round(((float)settings.rpmMax/255)*(float)raw);
}

int toTps(int raw) {
    // percent
    return int(raw/2.55);
}

void printIntWithPadding(int val,unsigned char width,char padChar) {
    // print enough leading zeroes!
    memset(buffer,padChar,30);
    // append string presentation of number to end
    itoa(val,buffer+30,10);
    // print string with given width
    Serial.print(buffer+30+strlen(buffer+30)-width);
}

void printStringWithPadding(prog_uchar *str,unsigned char width,char padChar) {
    // print enough leading zeroes!
    memset(buffer,padChar,30);
    // append string presentation of number to end
    strcpy_P(buffer+30, (PGM_P)str);   
    
    // print string with given width
    Serial.print(buffer+30+strlen(buffer+30)-width);
}

void printPads(unsigned char n, char padChar) {
    memset(buffer,padChar,n);
    buffer[n] = 0;
    Serial.print(buffer);
}

// User interface functions
void pageHeader() {
    //Serial.write(clearScreen,sizeof(clearScreen));    
    printFromFlash(ANSIgoHome);
    printFromFlash(versionString);   
    printFromFlash(statusString1);   
    Serial.print(pages[page]);
    printFromFlash(ANSIclearEolAndLf);
    printFromFlash(ANSIclearEolAndLf);
}

// Stored in the 32kB FLASH
prog_uchar aboutString1[] PROGMEM  = "(c) 2011 Juho Pesonen. Visit http://dmn.kuulalaakeri.org/vnt-lda/";
prog_uchar aboutString2[] PROGMEM  = "Press: <space> to jump next view, or press ...";
prog_uchar aboutString3[] PROGMEM  = "Questions? Or feedback? Send mail to dmn@qla.fi";
prog_uchar aboutString4[] PROGMEM = "To upload new maps, visit the site and use the editor to create table.";
prog_uchar aboutString5[] PROGMEM  = "Then paste table data to this terminal window.";

void pageAbout(char key) {
    
    if (key) {
        // update only if key pressed
        pageHeader();
        printFromFlash(aboutString1);   
        printFromFlash(ANSIclearEolAndLf);
        printFromFlash(ANSIclearEolAndLf);
        
        printFromFlash(aboutString2);   
        printFromFlash(ANSIclearEolAndLf);	
        for (char i=0;i<10;i++) {
            printPads(11,' ');
            Serial.print("<");
            Serial.print(i,DEC);
            Serial.print("> ");
            Serial.print(pages[i]);			
            printFromFlash(ANSIclearEolAndLf);
        }
        printFromFlash(ANSIclearEolAndLf);
        
        printFromFlash(aboutString3);   
        printFromFlash(ANSIclearEolAndLf);
        printFromFlash(ANSIclearEolAndLf);
        
        printFromFlash(aboutString4); 
        printFromFlash(ANSIclearEolAndLf);    
        printFromFlash(aboutString5);      
        
        printFromFlash(ANSIclearEos);
    }
} 

// TPS input: 200 Corrected: 0 (low:200, high:788);
prog_uchar statusRPM[] PROGMEM  = "RPM actual:";
prog_uchar statusCorrected[] PROGMEM  = " Corrected:";
prog_uchar statusTPSinput[] PROGMEM  = "TPS input:";
prog_uchar statusLow[] PROGMEM  = ":";
prog_uchar statusMAPinput[] PROGMEM  = "MAP input:";
prog_uchar statusVNTactOutput[] PROGMEM  = "VNT actuator output:";
prog_uchar statusLDAactOutput[] PROGMEM  = "LDA actuator output:";
prog_uchar statusHeader[] PROGMEM  = "Sensor values and adaptation map limits (l=live, y=save, p/P=load, R=reset)";
//                                          0123456789012345678901234567890123456789012345678901234567890123456789
prog_uchar statusTableHeader[] PROGMEM  = "                    Raw val.  Corr.val. Map min.  Map max.  Mode";
prog_uchar statusRowTPS[] PROGMEM = "TPS";
prog_uchar statusRowMAP[] PROGMEM = "MAP";
prog_uchar statusRowRPM[] PROGMEM = "RPM";
prog_uchar statusVNTtargetDutyCycle[] PROGMEM = "VNT target dc.";
prog_uchar statusVNTactualDutyCycle[] PROGMEM = "VNT duty cycle";
prog_uchar statusVNTactualTargetPress[] PROGMEM = "VNT target press.";
prog_uchar statusLDAdutyCycle[] PROGMEM = "LDA duty cycle";
prog_uchar statusVNTtableStyle[] PROGMEM =  "Vnt Table mode";
prog_uchar statusBits[] PROGMEM = "statusBits";
//                                          0123456789012345678901234567890123456789012345678901234567890123456789
prog_uchar statusVNTtableStyleDC[] PROGMEM =  "Duty cycle";
prog_uchar statusVNTtableStyleMAP[] PROGMEM = "Target press.";
prog_uchar statusOpenAtIdle[] PROGMEM = "VNT open@idle";
prog_uchar statusNone[] PROGMEM = "-";

prog_uchar statusSelected[] PROGMEM = "[X]";
prog_uchar statusUnSelected[] PROGMEM = "[ ]";
prog_uchar statusLDAOutputInverted[] PROGMEM = "LDA Output Inverted K ";
prog_uchar statusVNTOutputInverted[] PROGMEM = "VNT Output Inverted J   ";

prog_uchar statusTemp1[] PROGMEM = "Temp#1: ";
prog_uchar statusTemp2[] PROGMEM = "Temp#2: ";
prog_uchar statusC[] PROGMEM = "°C";
prog_uchar statusOn[] PROGMEM = " (on)  ";
prog_uchar statusOff[] PROGMEM = " (off) ";
prog_uchar statusFooter[] PROGMEM = "To change adaptation value, press the letter after value.";
prog_uchar statusFooter2[] PROGMEM = "For example: q = decrease 'Map Low' for TPS / Q increase 'Map Low' for TPS";

char oldKey;
bool isLive = true;

void pageStatusAndAdaption(char key) {
    int x = 1;
    
    if (oldKey == key)
        x = 10;
    
    switch (key) {
        case 'l':
        case 'L': 
            isLive=!isLive; 
            break;
        case 'i': 
            if (settings.inDamp-x>0) settings.inDamp -= x; 
            break;
        case 'I': 
            if (settings.inDamp+x<AVG_MAX) settings.inDamp += x; 
            break;
        case 'o': 
            if (settings.outDamp-x>0) settings.outDamp -= x; 
            break;
        case 'O': 
            if (settings.outDamp+x<AVG_MAX) settings.outDamp += x; 
            break;
        case 'q': 
            if (settings.tpsMin-x>0) settings.tpsMin -= x; 
            break;
        case 'Q': 
            if (settings.tpsMin+x<settings.tpsMax) settings.tpsMin += x; 
            break;
        case 'w': 
            if (settings.tpsMax-x>settings.tpsMin) settings.tpsMax -= x; 
            break;
        case 'W': 
            if (settings.tpsMax+x<1024) settings.tpsMax += x; 
            break;
        case 'a': 
            if (settings.mapMin-x>0) settings.mapMin -= x; 
            break;
        case 'A': 
            if (settings.mapMin+x<settings.mapMax) settings.mapMin += x; 
            break;
        case 's': 
            if (settings.mapMax-x>settings.mapMin) settings.mapMax -= x; 
            break;
        case 'S': 
            if (settings.mapMax+x<1024) settings.mapMax += x; 
            break;
        case 'f': 
            if (settings.rpmTeethsPerRotation>1) settings.rpmTeethsPerRotation -= 1; 
            break;				
        case 'F': 
            if (settings.rpmTeethsPerRotation<99) settings.rpmTeethsPerRotation += 1; 
            break;				
        case 'd': 
            if (settings.rpmMax-100>1000) settings.rpmMax -= 100; 
            break;		
        case 'D': 
            if (settings.rpmMax+100<9999) settings.rpmMax += 100; 
            break;		
        case 'z': 
            if (settings.vntMinDC-x>=0) settings.vntMinDC -= x; 
            break;
        case 'Z': 
            if (settings.vntMinDC+x<settings.vntMaxDC) settings.vntMinDC += x; 
            break;
        case 'x': 
            if (settings.vntMaxDC-x>settings.vntMinDC) settings.vntMaxDC -= x; 
            break;
        case 'X': 
            if (settings.vntMaxDC+x<256) settings.vntMaxDC += x; 
            break;		
        case 'g': 
            if (settings.ldaMinDC-x>=0) settings.ldaMinDC -= x; 
            break;
        case 'G': 
            if (settings.ldaMinDC+x<settings.ldaMaxDC) settings.ldaMinDC += x; 
            break;
        case 'h': 
            if (settings.ldaMaxDC-x>settings.ldaMinDC) settings.ldaMaxDC -= x; 
            break;
        case 'H': 
            if (settings.ldaMaxDC+x<256) settings.ldaMaxDC += x; 
            break;		
        case 'c':
        case 'C': 
            settings.mode = settings.mode ^ MODE_VANESOPENIDLE; 
            break;		
        case 'm':
        case 'M': 
            settings.mode = settings.mode ^ MODE_DUTYCYCLEMAP; 
            break;		
        case 'k':
        case 'K': 
            settings.mode = settings.mode ^ MODE_LDAOUTPUTINVERTED; 
            break;		
        case 'j':
        case 'J': 
            settings.mode = settings.mode ^ MODE_VNTOUTPUTINVERTED; 
            break;				
        case 'y': 
            saveToEEPROM(); 
            break;
        case 'p': 
            loadFromEEPROM(0); 
            break;
        case 'P': 
            loadFromEEPROM(1); 
            break;
        case 'R': 
            loadDefaults(); 
            break;		
    }
    if (key) {
        lastPacketTime = millis();
    }
    oldKey = key;	
    
    if (!key && !isLive)
        return;
    
    // update always if live
    pageHeader(); 
    
    printFromFlash(statusHeader);
    printFromFlash(ANSIclearEolAndLf);
    printFromFlash(ANSIclearEolAndLf);
    
    printFromFlash(statusTableHeader);
    printFromFlash(ANSIclearEolAndLf);
    
    printStringWithPadding(statusRowTPS,19,' ');
    printPads(1,' ');
    printIntWithPadding(controls.tpsInput,4,'0'); // RAW
    printPads(6,' ');
    printIntWithPadding(controls.tpsCorrected,3,'0'); // Corr.
    printPads(7,' ');	
    printIntWithPadding(settings.tpsMin,4,'0'); // Map low
    printPads(1,' ');	
    Serial.print("Q");
    printPads(4,' ');	
    printIntWithPadding(settings.tpsMax,4,'0'); // Map high
    Serial.print(" W");
    printFromFlash(ANSIclearEolAndLf);
    
    printStringWithPadding(statusRowMAP,19,' ');
    printPads(1,' ');
    printIntWithPadding(controls.mapInput,4,'0'); // RAW
    printPads(6,' ');
    printIntWithPadding(controls.mapCorrected,3,'0'); // Corr.
    printPads(7,' ');	
    printIntWithPadding(settings.mapMin,4,'0'); // Map low
    printPads(1,' ');	
    Serial.print("A");
    printPads(4,' ');	
    printIntWithPadding(settings.mapMax,4,'0'); // Map high
    Serial.print(" S");
    printFromFlash(ANSIclearEolAndLf);
    
    printStringWithPadding(statusRowRPM,19,' ');
    printPads(1,' ');
    printIntWithPadding(controls.rpmActual,4,'0'); // RAW
    printPads(6,' ');
    printIntWithPadding(controls.rpmCorrected,3,'0'); // Corrected
    printPads(17,' ');
    printIntWithPadding(settings.rpmMax,4,'0'); 
    Serial.print(" D");
    printPads(4,' ');	
    Serial.print("No.teeths=");
    printIntWithPadding(settings.rpmTeethsPerRotation,2,'0'); 
    Serial.print(" F");
    
    printFromFlash(ANSIclearEolAndLf);
    
    if (settings.mode & MODE_DUTYCYCLEMAP) {
        printStringWithPadding(statusVNTactualDutyCycle,19,' ');
    } 
    else {
        printStringWithPadding(statusVNTactualTargetPress,19,' ');
    }
    printPads(1,' ');
    printIntWithPadding(controls.vntTargetPressure,3,'0'); // target
    printPads(7,' ');
    printIntWithPadding(controls.vntPosition,3,'0'); // actual
    Serial.print("/");
    printIntWithPadding(controls.vntPositionRemapped,3,'0'); 
    printPads(3,' ');	
    printIntWithPadding(settings.vntMinDC,3,'0'); 
    Serial.print(" Z");
    printPads(5,' ');
    printIntWithPadding(settings.vntMaxDC,3,'0'); 
    Serial.print(" X");
    printPads(5,' ');
    if (settings.mode & MODE_VANESOPENIDLE) {
        printFromFlash(statusOpenAtIdle);
    } 
    else {
        printFromFlash(statusNone);
    }
    Serial.print(" C");
    
    
    printFromFlash(ANSIclearEolAndLf);
    
    printStringWithPadding(statusLDAdutyCycle,19,' ');
    printPads(1,' ');
    printIntWithPadding(controls.ldaPosition,3,'0'); // RAW
    printPads(10,' ');
    Serial.print("/");
    printIntWithPadding(controls.ldaPositionRemapped,3,'0'); 
    printPads(3,' ');	
    printIntWithPadding(settings.ldaMinDC,3,'0'); 
    Serial.print(" G");
    printPads(5,' ');
    printIntWithPadding(settings.ldaMaxDC,3,'0'); 
    Serial.print(" H");
    printFromFlash(ANSIclearEolAndLf);
    
    printStringWithPadding(statusVNTtableStyle,19,' ');
    printPads(41,' ');
    /*printIntWithPadding(settings.vntOpeningPressureThreshold,3,'0'); 
     Serial.print(" B");
     
     // printPads(15,' ');
     Serial.print(" Id");
     printPads(1,' ');
     printIntWithPadding(settings.inDamp,3,'0'); 
     Serial.print(" Od");
     printPads(1,' ');
     printIntWithPadding(settings.outDamp,3,'0'); 
     */
    if (settings.mode & MODE_DUTYCYCLEMAP) {
        printFromFlash(statusVNTtableStyleDC);
    } 
    else {
        printFromFlash(statusVNTtableStyleMAP);
    }
    Serial.print(" M");
    printFromFlash(ANSIclearEolAndLf);
    
    printStringWithPadding(statusBits,19,' ');
    printPads(3,' ');
    printIntWithPadding(controls.statusBits,1,'0');
    
    printFromFlash(ANSIclearEolAndLf);
    printFromFlash(ANSIclearEolAndLf);
    
    printPads(20,' ');
    if (settings.mode & MODE_VNTOUTPUTINVERTED) {
        printFromFlash(statusSelected);
        
    } 
    else {
        printFromFlash(statusUnSelected);
    }
    printFromFlash(statusVNTOutputInverted);
    
    if (settings.mode & MODE_LDAOUTPUTINVERTED) {
        printFromFlash(statusSelected);
        
    } 
    else {
        printFromFlash(statusUnSelected);
    }
    printFromFlash(statusLDAOutputInverted);
    
    printFromFlash(ANSIclearEolAndLf);
    
    printPads(20,' ');
    printFromFlash(statusTemp1);
    printIntWithPadding(controls.temp1,4,' ');
    printFromFlash(statusC);
    if (controls.output1Enabled) {
        printFromFlash(statusOn);
    } else {
        printFromFlash(statusOff);
    }
    
    printFromFlash(statusTemp2);
    printIntWithPadding(controls.temp2,4,' ');
    printFromFlash(statusC);
    if (controls.output2Enabled) {
        printFromFlash(statusOn);
    } else {
        printFromFlash(statusOff);
    }
    
    printFromFlash(ANSIclearEolAndLf);
    printFromFlash(ANSIclearEolAndLf);
    
    printFromFlash(statusFooter);
    printFromFlash(ANSIclearEolAndLf);
    
    printFromFlash(statusFooter2);
    
    printFromFlash(ANSIclearEos);
}

void pagePreferences(char key) {
    
}

void pageLdaMapVisual(char key) {
    switch (key) {
        case 'l':
        case 'L': 
            isLive=!isLive; 
            break;
    }
    if (!key && !isLive)
        return;
    pageHeader();
    printFromFlash(mapVisualitionHelp);
    printFromFlash(ANSIclearEolAndLf);	
    unsigned char tableSizeX = *(vntActuatorPressureRequestMap+3);
    unsigned char tableSizeY = *(vntActuatorPressureRequestMap+4);
    Serial.print(tableSizeX,DEC);
    Serial.print("x");
    Serial.print(tableSizeY,DEC);
    printFromFlash(ANSIclearEolAndLf);
    
    int x,y;
    for (y=0;y<256;y=y+16)  {
        for (x=0;x<256;x=x+4)  {
            if (controls.mapCorrected  >= y && controls.mapCorrected  < y+16 && controls.rpmCorrected >=x && controls.rpmCorrected <x+4) {
                Serial.write('O');
            } 
            else {
                Serial.write(mapDebugCharValue(mapLookUp(ldaPositionMap,x,y)));
            }
        }
        if (controls.mapCorrected >= y && controls.mapCorrected < y+16)
            Serial.print("<< ");
        
        printFromFlash(ANSIclearEolAndLf);
    }
    for (x=0;x<256;x=x+4)  {
        if (controls.rpmCorrected>=x && controls.rpmCorrected<x+4) {
            Serial.print("^");
            continue;
        } 
        else {
            Serial.print(" ");
        }	
    }
    printFromFlash(ANSIclearEolAndLf);
}

void pageVntMapVisual(char key) {
    unsigned char yVal,yPos;
    switch (key) {
        case 'l':
        case 'L': 
            isLive=!isLive; 
            break;
    }
    
    if (!key && !isLive)
        return;
    pageHeader();
    printFromFlash(mapVisualitionHelp);
    printFromFlash(ANSIclearEolAndLf);		
    
    unsigned char tableSizeX = *(vntActuatorPressureRequestMap+3);
    unsigned char tableSizeY = *(vntActuatorPressureRequestMap+4);
    Serial.print(tableSizeX,DEC);
    Serial.print("x");
    Serial.print(tableSizeY,DEC);
    Serial.print(" -- ");
    
    if (settings.mode & MODE_DUTYCYCLEMAP) {
        printFromFlash(statusVNTtableStyleDC);
    } 
    else {
        printFromFlash(statusVNTtableStyleMAP);
    }
    printFromFlash(ANSIclearEolAndLf);		
    
    
    int x,y;
    for (y=0;y<256;y=y+16)  {
        for (x=0;x<256;x=x+4)  {
            if (controls.tpsCorrected  >= y && controls.tpsCorrected  < y+16 && controls.rpmCorrected >=x && controls.rpmCorrected <x+4) {
                Serial.write('O');
            } 
            else {
                Serial.write(mapDebugCharValue(mapLookUp(vntActuatorPressureRequestMap,x,y)));
            }
        }
        if (controls.tpsCorrected >= y && controls.tpsCorrected < y+16)
            Serial.print("<< ");
        
        printFromFlash(ANSIclearEolAndLf);
    }
    for (x=0;x<256;x=x+4)  {
        if (controls.rpmCorrected>=x && controls.rpmCorrected<x+4) {
            Serial.print("^");
            continue;
        } 
        else {
            Serial.print(" ");
        }	
    }
    printFromFlash(ANSIclearEos);	
}

prog_uchar statusOutput1[] PROGMEM = "Output tests:";
prog_uchar statusOutput2[] PROGMEM = "<Q> Set output VNT output to Map min. for 2 seconds, value=";
prog_uchar statusOutput3[] PROGMEM = "<W> Set output VNT output to Map max. for 2 seconds, value=";
prog_uchar statusOutput4[] PROGMEM = "<E> Sweep output VNT output between min & max";

prog_uchar statusOutput5[] PROGMEM = "<A> Set output LDA output to Map min. for 2 seconds, value=";
prog_uchar statusOutput6[] PROGMEM = "<S> Set output LDA output to Map max. for 2 seconds, value=";
prog_uchar statusOutput7[] PROGMEM = "<D> Sweep output LDA output between min & max";

prog_uchar statusOutput8[] PROGMEM = "<Z> Enable OUTPUT1 for 2 seconds";
prog_uchar statusOutput9[] PROGMEM = "<X> Enable OUTPUT2 for 2 seconds";

void pageOutputTests(char key) {
    
    if (key) {
        switch(key) {
            case 'q':
            case 'Q':
                controls.vntPositionRemapped = settings.vntMinDC;
                updateOutputValues(true);
                delay(2000);				
                break;
            case 'w':
            case 'W':
                controls.vntPositionRemapped = settings.vntMaxDC;
                updateOutputValues(true);				
                delay(2000);							
                break;
            case 'e':
            case 'E':
                for (controls.vntPositionRemapped = settings.vntMinDC;
                     controls.vntPositionRemapped<settings.vntMaxDC;
                     controls.vntPositionRemapped++) {
                    updateOutputValues(true);
                    delay(20);
                }
                for (controls.vntPositionRemapped = settings.vntMaxDC;
                     controls.vntPositionRemapped>settings.vntMinDC;
                     controls.vntPositionRemapped--) {
                    updateOutputValues(true);
                    delay(20);
                }				
                break;
            case 'a':
            case 'A':
                controls.ldaPositionRemapped = settings.ldaMinDC;
                updateOutputValues(true);
                delay(2000);				
                break;
            case 's':
            case 'S':
                controls.ldaPositionRemapped = settings.ldaMaxDC;
                updateOutputValues(true);				
                delay(2000);							
                break;
            case 'd':
            case 'D':
                for (controls.ldaPositionRemapped = settings.ldaMinDC;
                     controls.ldaPositionRemapped<settings.ldaMaxDC;
                     controls.ldaPositionRemapped++) {
                    updateOutputValues(true);
                    delay(20);
                }
                for (controls.ldaPositionRemapped = settings.ldaMaxDC;
                     controls.ldaPositionRemapped>settings.ldaMinDC;
                     controls.ldaPositionRemapped--) {
                    updateOutputValues(true);
                    delay(20);
                }				
                break;
            case 'z':			
            case 'Z':
                controls.output1Enabled = true;
                updateOutputValues(true);				
                delay(2000);							
                break;
            case 'x':			
            case 'X':
                controls.output2Enabled = true;
                updateOutputValues(true);				
                delay(2000);							
                break;
        }
        pageHeader();
        
        printFromFlash(statusOutput1); 		
        printFromFlash(ANSIclearEolAndLf);
        printFromFlash(ANSIclearEolAndLf);
        printFromFlash(statusOutput2); 
        Serial.print(settings.vntMinDC,DEC); 
        printFromFlash(ANSIclearEolAndLf);
        printFromFlash(statusOutput3); 
        Serial.print(settings.vntMaxDC,DEC); 
        printFromFlash(ANSIclearEolAndLf);
        printFromFlash(statusOutput4); 
        printFromFlash(ANSIclearEolAndLf);
        printFromFlash(ANSIclearEolAndLf);
        printFromFlash(statusOutput5); 
        Serial.print(settings.ldaMinDC,DEC);
        printFromFlash(ANSIclearEolAndLf);
        printFromFlash(statusOutput6); 
        Serial.print(settings.ldaMaxDC,DEC);
        printFromFlash(ANSIclearEolAndLf);
        printFromFlash(statusOutput7); 
        printFromFlash(ANSIclearEolAndLf);
        printFromFlash(ANSIclearEolAndLf);
        printFromFlash(statusOutput8); 
        printFromFlash(ANSIclearEolAndLf);
        printFromFlash(statusOutput9); 
        printFromFlash(ANSIclearEolAndLf);
        printFromFlash(ANSIclearEos);
    }
}

prog_uchar exportConf[] PROGMEM = "Configuration dump:";
prog_uchar exportVntMap[] PROGMEM = "VNT Map dump:";
prog_uchar exportLdaMap[] PROGMEM = "LDA Map dump:";

void pageExport(char key) {
    if (key) {
        pageHeader();
        printFromFlash(exportConf);
        printFromFlash(ANSIclearEolAndLf);
        Serial.print("!AA");
        for (int i=0;i<sizeof(settingsStruct);i++) {
            if (i%32 == 31) 
                printFromFlash(ANSIclearEolAndLf);
            unsigned char v = (unsigned char)*(i+((unsigned char*)&settings));
            if (v<16)
                Serial.print("0");
            Serial.print(v,HEX);
        }
        Serial.print("!");
        printFromFlash(ANSIclearEolAndLf);
        printFromFlash(ANSIclearEolAndLf);
        
        printFromFlash(exportVntMap);
        printFromFlash(ANSIclearEolAndLf);
        Serial.print("!AB");
        for (int i=0;i<sizeof(vntActuatorPressureRequestMap);i++) {
            if (i && i%16 == 0) 
                printFromFlash(ANSIclearEolAndLf);
            unsigned char v = (unsigned char)*(i+((unsigned char*)&vntActuatorPressureRequestMap));
            if (v<16)
                Serial.print("0");
            Serial.print(v,HEX);
        }
        Serial.print("!");
        printFromFlash(ANSIclearEolAndLf);
        printFromFlash(ANSIclearEolAndLf);
        
        printFromFlash(exportLdaMap);
        printFromFlash(ANSIclearEolAndLf);
        Serial.print("!AC");
        for (int i=0;i<sizeof(ldaPositionMap);i++) {
            if (i && i%16 == 0) 
                printFromFlash(ANSIclearEolAndLf);
            unsigned char v = (unsigned char)*(i+((unsigned char*)&ldaPositionMap));
            if (v<16)
                Serial.print("0");
            Serial.print(v,HEX);
        }
        Serial.print("!");
        
        printFromFlash(ANSIclearEolAndLf);
        
        printFromFlash(ANSIclearEos);	
    }
}

void gotoXY(char x,char y) {
    Serial.print(byte(27));
    Serial.print("[");
    Serial.print(y,DEC);
    Serial.print(";");
    Serial.print(x,DEC);
    Serial.print("H");
}

void printMapAxis(unsigned char axisType,unsigned char idx,bool verbose) {
    switch (axisType) {
        case MAP_AXIS_RPM:
            Serial.print(toRpm(idx),DEC);
            if (verbose) Serial.print(" RPM");
            break;
        case MAP_AXIS_TPS:
            Serial.print(toTps(idx),DEC);
            if (verbose) Serial.print("% TPS");
            break;
        case MAP_AXIS_CBAR:
            Serial.print(toBars(idx),DEC);
            if (verbose) Serial.print(" cBar");
            break;
        case MAP_AXIS_VOLTAGE:
            Serial.print(toVoltage(idx),DEC);
            if (verbose) Serial.print(" mV");
            break;
        case MAP_AXIS_CELSIUS:
            Serial.print(idx-64,DEC);
            if (verbose) Serial.print(" °C");
            break;
        default:
            Serial.print(idx,DEC);
            if (verbose) Serial.print(" Raw");
    }
}
struct mapEditorDataStruct {
    char cursorX;
    char cursorY;
    unsigned char lastX;
    unsigned char lastY;
    char currentMap;
    unsigned char clipboard;
} mapEditorData;

/*char *editorMapsHeader[] = {
    "VNT Actuator target press (cBar, X=RPM, Y=TPS)","VNT Actuator max pos. (duty cycle, X=RPM, Y=TPS)","LDA Actuator pos. (duty cycle, X=RPM, Y=cBar)","Temp. sensor correction  (C, X=input voltage)","Actuator position remap (Duty Cycle, X=requested duty cycle)"};
*/
prog_uchar mapCurrentOutput[] PROGMEM = "Current output:";
prog_uchar mapEditorHelp[] PROGMEM = "Press: h/j/k/l to move, - / + dec/inc, c/v copy/paste cell, y save";

void pageMapEditor(unsigned char mapIdx,int key,boolean showCurrent=false) {    
    unsigned char *mapData = editorMaps[mapIdx];
    unsigned char tableSizeX = *(mapData+3);
    unsigned char tableSizeY = *(mapData+4);
    unsigned char axisTypeX = *(mapData+5);
    unsigned char axisTypeY = *(mapData+6);
    unsigned char axisTypeResult = *(mapData+7);
    unsigned char lastXpos = *(mapData+8+tableSizeX*tableSizeY);
    unsigned char lastYpos = *(mapData+8+tableSizeX*tableSizeY+1);
    unsigned char lastValue = *(mapData+8+tableSizeX*tableSizeY+2);
    
    const char xPad = 5;
    const char xSpace = 7;
    const char yPad = 5;
    const char ySpace = 2;
    
    switch (key) {
        case -2:
            gotoXY(xPad+xSpace+mapEditorData.cursorX*xSpace+1,yPad+ySpace+mapEditorData.cursorY*ySpace);
            printPads(xSpace-2,' ');
            gotoXY(xPad+xSpace+mapEditorData.cursorX*xSpace+1,yPad+ySpace+mapEditorData.cursorY*ySpace);
            printMapAxis(axisTypeResult,*(mapData+8+mapEditorData.cursorX+mapEditorData.cursorY*tableSizeX),0);

            return;
            break;
        case -1:
            // erase cursor
            gotoXY(xPad+xSpace+mapEditorData.cursorX*xSpace,yPad+ySpace+mapEditorData.cursorY*ySpace);
            Serial.print(" ");
            gotoXY(xPad+xSpace+mapEditorData.cursorX*xSpace+xSpace-1,yPad+ySpace+mapEditorData.cursorY*ySpace);
            Serial.print(" ");
            return;
            break;
        case 'c':
            mapEditorData.clipboard = *(mapData+8+mapEditorData.cursorX+mapEditorData.cursorY*tableSizeX);
            return;
            break;
        case 'v':
            (*(mapData+8+mapEditorData.cursorX+mapEditorData.cursorY*tableSizeX)) = mapEditorData.clipboard;
            pageMapEditor(mapIdx,-2);
            return;
            break;
        case 'h':
            pageMapEditor(mapIdx,-1);
            if (mapEditorData.cursorX>0)
                mapEditorData.cursorX--;
            pageMapEditor(mapIdx,0);
            return;
            break;
        case 'l':
            pageMapEditor(mapIdx,-1);
            if (mapEditorData.cursorX<tableSizeX-1)
                mapEditorData.cursorX++;
            pageMapEditor(mapIdx,0);
            return;
            break;
        case 'k':
            pageMapEditor(mapIdx,-1);
            if (mapEditorData.cursorY>0)
                mapEditorData.cursorY--;
            pageMapEditor(mapIdx,0);
            return;
            break;
        case 'j':
            pageMapEditor(mapIdx,-1);
            if (mapEditorData.cursorY<tableSizeY-1)
                mapEditorData.cursorY++;
            pageMapEditor(mapIdx,0);
            return;
            break;
        case '+':
            if (*(mapData+8+mapEditorData.cursorX+mapEditorData.cursorY*tableSizeX)<0xff)
                (*(mapData+8+mapEditorData.cursorX+mapEditorData.cursorY*tableSizeX))++;
            pageMapEditor(mapIdx,-2);
            return;
            break;
        case '-':
            if (*(mapData+8+mapEditorData.cursorX+mapEditorData.cursorY*tableSizeX)>0)
                (*(mapData+8+mapEditorData.cursorX+mapEditorData.cursorY*tableSizeX))--;
            pageMapEditor(mapIdx,-2);
            return;
            break;
        case 'y':
            saveToEEPROM();
        case 0:
            if (showCurrent) {
                // Current interpreted value
                gotoXY(xPad+xSpace,yPad+tableSizeY*ySpace+2);
                printFromFlash(mapCurrentOutput);
                printMapAxis(axisTypeResult,lastValue,1);
                printFromFlash(ANSIclearEolAndLf);
            }
                
            // update cursors only:
            gotoXY(2,yPad+ySpace+round((float)mapEditorData.lastY*(float)((float)(tableSizeY-1)*(float)ySpace/255)));
            Serial.print("  ");
            gotoXY(xPad+xSpace+round((float)mapEditorData.lastX*(float)((float)tableSizeX*(float)xSpace/255)),4);
            Serial.print(" ");
            
            mapEditorData.lastY = lastYpos;
            mapEditorData.lastX = lastXpos;
            
            gotoXY(2,yPad+ySpace+round((float)lastYpos*(float)((float)(tableSizeY-1)*(float)ySpace/255)));
            Serial.print(">>");
            gotoXY(xPad+xSpace+round((float)lastXpos*(float)((float)tableSizeX*(float)xSpace/255)),4);
            Serial.print("v"); 
            
            gotoXY(xPad+xSpace+mapEditorData.cursorX*xSpace,yPad+ySpace+mapEditorData.cursorY*ySpace);
            Serial.print(">");
            gotoXY(xPad+xSpace+mapEditorData.cursorX*xSpace+xSpace-1,yPad+ySpace+mapEditorData.cursorY*ySpace);
            Serial.print("<");

            return;
            break;
        default:
            if (mapEditorData.currentMap!=mapIdx) {
                mapEditorData.cursorX=0;
                mapEditorData.cursorY=0;
                mapEditorData.currentMap = mapIdx;
            }
            pageHeader();
            printFromFlash(ANSIclearEos);
            gotoXY(0,3);
            printFromFlash(mapEditorHelp);
    }
    // Table X header
    
    for (int x=0;x<tableSizeX;x++) {
        gotoXY(xPad+(1+x)*xSpace,yPad);
        int idx = round((float)((255/(float)(tableSizeX-1)))*(float)x);
        printPads(1,' ');
        printMapAxis(axisTypeX,idx, ((x==0||x==(tableSizeX-1))?true:false));
    }
    gotoXY(xPad+xSpace,yPad+1);
    printPads(tableSizeX*xSpace,'-');

    // Table Y header
    
    for (int y=0;y<tableSizeY;y++) {
        gotoXY(xPad-1,yPad+(1+y)*ySpace);
        int idx = round((float)((255/(float)(tableSizeY-1)))*(float)y);
        
        printMapAxis(axisTypeY,idx,true);
        gotoXY(xPad+xSpace-1,yPad+(1+y)*ySpace);
        Serial.print("|");
        if (y<tableSizeY-1) {
            gotoXY(xPad+xSpace-1,yPad+(1+y)*ySpace+1); // works for ySpace=2
            Serial.print("|");
        }

    }
    for (int y=0;y<tableSizeY;y++) {
        for (int x=0;x<tableSizeX;x++) {
             gotoXY(xPad+(1+x)*xSpace,yPad+(1+y)*ySpace);
             printPads(1,' ');
             //Serial.print(*(mapData+8+x*y),DEC);
             printMapAxis(axisTypeResult,*(mapData+8+x+y*tableSizeX),0);
          }
    }

}

prog_uchar debugHeader[] PROGMEM = "Target pres.   Actual press.  Actuator pos.  RPM  TPS";
// 0123456789012345678901234567890123456789012345678901234567890123456789


void pageHelp(char key) {
    //pageHeader();
    //Serial.print("help!");
    //printFromFlash(ANSIclearEos);	
    if (key) {
        printFromFlash(debugHeader);
        Serial.print("\r\n");
        printIntWithPadding(toBars(controls.vntTargetPressure),3,'0');
        printPads(12,' ');
        printIntWithPadding(toBars(controls.mapCorrected),3,'0');
        printPads(12,' ');
        printIntWithPadding(controls.vntPosition,3,'0');
        printPads(12,' ');
        printIntWithPadding(controls.rpmActual,4,'0');
        printPads(1,' ');
        printIntWithPadding(controls.tpsCorrected,4,'0');
        Serial.print("\r\n");
    }
}
unsigned char i;

void pageDataLogger(char key) {
    i++;
    if (i % 3 == 0) {
        Serial.write(2); // stx
        Serial.print(controls.mapCorrected,DEC);
        Serial.print(",");
        Serial.print(controls.vntTargetPressure,DEC);
        Serial.print(",");
        Serial.print(controls.vntPosition,DEC);
        Serial.print(",");
        Serial.print(controls.vntPositionTarget,DEC);
        Serial.print(",");
        Serial.print(controls.tpsCorrected,DEC);
        Serial.print(",");
        Serial.print(controls.ldaPosition,DEC);
        Serial.print(",");
        Serial.print(controls.rpmActual,DEC);
        Serial.print(",");
        Serial.print(controls.statusBits,DEC);
        Serial.print(",");
        Serial.print(millis()/1000,DEC); 
        Serial.write(3);
    }
    /* test mode
     Serial.write(2); // stx
     Serial.print((controls.mapCorrected+i) % 256,DEC);
     Serial.print(",");
     Serial.print((controls.vntTargetPressure+i) % 32,DEC);
     Serial.print(",");
     Serial.print((controls.vntPosition+i) % 44,DEC);
     Serial.print(",");
     Serial.print((controls.vntPositionTarget+i) % 55,DEC);
     Serial.print(",");
     Serial.print((controls.tpsCorrected+i) % 200,DEC);
     Serial.print(",");
     Serial.print((controls.rpmActual),DEC);
     Serial.print(",");  
     Serial.write(3); // etx */
}

prog_uchar ServoFineTuneChargePressureRequest[] PROGMEM = "Charge pressure, request:";
prog_uchar ServoFineTuneChargePressureActual[] PROGMEM = "Charge pressure, actual:";
prog_uchar ServoFineTuneInDamp[] PROGMEM = "In damp.:";
prog_uchar ServoFineTuneOutDamp[] PROGMEM = "Out damp.:";
prog_uchar ServoStabilizerThreshold[] PROGMEM = "Stabilizer threshold:";
prog_uchar ServoStabilizerSmoothValue[] PROGMEM = "Stabilizer smoothing:";
prog_uchar ServoStabilizerActive[] PROGMEM = " (active)";

void pageServoFineTune(char key) {
    pageHeader();
    
    switch (key) {
            
        case 'i':
            if (settings.inDamp>1) settings.inDamp--;
            break;
        case 'I':
            if (settings.inDamp<255) settings.inDamp++;
            break;
        case 'o':
            if (settings.outDamp>1) settings.outDamp--;
            break;
        case 'O':
            if (settings.outDamp<255) settings.outDamp++;
            break;
        case 't':
            if (settings.servoStabilizerThreshold>1) settings.servoStabilizerThreshold--;
            break;
        case 'T':
            if (settings.servoStabilizerThreshold<255) settings.servoStabilizerThreshold++;
            break;
            
        case 's':
            if (settings.servoStabilizerSmoothValue>1) settings.servoStabilizerSmoothValue--;
            break;
        case 'S':
            if (settings.servoStabilizerSmoothValue<255) settings.servoStabilizerSmoothValue++;
            break;
        case 'y': 
            saveToEEPROM(); 
            break;
    }
    
    Serial.print("Actual:"); 
    printPads(controls.vntPosition /4,'*');
    //printPads(64-(controls.vntPosition /4),'.');
    printFromFlash(ANSIclearEolAndLf);
    Serial.print("Target:");
    printPads(controls.vntPositionTarget /4,'*');
    //printPads(64-(controls.vntPositionTarget /4),'.');
    printFromFlash(ANSIclearEolAndLf);    
    gotoXY(7+controls.clipPos/4,4);
    Serial.print("C");

    gotoXY(1,5);
    
    printStringWithPadding(ServoFineTuneChargePressureRequest,25,' ');
    printPads(1,' ');
    printIntWithPadding(toBars(controls.vntTargetPressure),3,'0');
    printFromFlash(ANSIclearEolAndLf);
    
    printStringWithPadding(ServoFineTuneChargePressureActual,25,' ');
    printPads(1,' ');
    printIntWithPadding(toBars(controls.mapCorrected),3,'0');
    printFromFlash(ANSIclearEolAndLf);
    
    printStringWithPadding(ServoFineTuneInDamp,25,' ');
    printPads(1,' ');
    printIntWithPadding(settings.inDamp,3,'0');
    Serial.print(" I");
    printFromFlash(ANSIclearEolAndLf);
    
    
    printStringWithPadding(ServoFineTuneOutDamp,25,' ');
    printPads(1,' ');
    printIntWithPadding(settings.outDamp,3,'0');
    Serial.print(" O");
    printFromFlash(ANSIclearEolAndLf);
    
    printStringWithPadding(ServoStabilizerThreshold,25,' ');
    printPads(1,' ');
    printIntWithPadding(settings.servoStabilizerThreshold,3,'0');
    Serial.print(" T");
    if (controls.statusBits & STATUS_CRUISING)
        printFromFlash(ServoStabilizerActive);
    printFromFlash(ANSIclearEolAndLf);
    
    printStringWithPadding(ServoStabilizerSmoothValue,25,' ');
    printPads(1,' ');
    printIntWithPadding(settings.servoStabilizerSmoothValue,3,'0');
    Serial.print(" S");
    printFromFlash(ANSIclearEolAndLf);
    
    printFromFlash(ANSIclearEos);	
    //Serial.print("\r\n");
    //printIntWithPadding(toBars(controls.vntTargetPressure),3,'0');
}

void readSysExCommand() {
    // sysex are in format !<byteid><data>!
    // timeouts in a few seconds if no end-of-data delimited found
    // all non 0-F characters are skipped in data part
    char hexChars[] = "0123456789ABCDEF0123456789abcdef";
    
    char b;
    
    printFromFlash(ANSIclearEolAndLf);
    Serial.print("SYSEX");
    memset(buffer,0,sizeof(buffer));
    
    lastPacketTime = millis();
    int offset = 0;
    unsigned char *ptr = (unsigned char*)&buffer; 
    int bufferSize = sizeof(buffer);
    while (millis()-lastPacketTime < 45000) {
        if (Serial.available()) {
            b = Serial.read();
            lastPacketTime = millis();
            if (b != '!') {
                char interpretedValue = -1;
                
                for (int i=0;i<32;i++) 
                    if (hexChars[i] == b) {
                        interpretedValue = i % 16;
                    }
                
                if (interpretedValue != -1) {
                    //buffer[(offset>>1) % sizeof(buffer)] += (offset%2==0?interpretedValue<<4:interpretedValue);
                    *(ptr+(offset>>1) % bufferSize) = (offset%2==0?interpretedValue<<4:*(ptr+(offset>>1) % bufferSize)+interpretedValue);
                    offset++;
                }
                if (offset==2 && ptr==(unsigned char*)&buffer) {
                    // first byte is read, set destion ptr
                    if (*(ptr) == 0xAA) {
                        ptr = (unsigned char*)&settings;
                        bufferSize = sizeof(settingsStruct);
                        offset=0;
                    }
                    if (*(ptr) == 0xAB) {
                        ptr = (unsigned char*)&vntActuatorPressureRequestMap;
                        bufferSize = sizeof(vntActuatorPressureRequestMap);						
                        offset=0;
                    }
                    if (*(ptr) == 0xAC) {
                        ptr = (unsigned char*)&ldaPositionMap;
                        bufferSize = sizeof(ldaPositionMap);													
                        offset=0;							
                    }						
                }
                // process data;
            } 
            else {
                break;
            }
        }
    }
    if (b=='!') {
        // got full buffer of data
        // TODO
    } 
    else {
        Serial.print("..ABORTED");
        delay(2000);
        
    }
    
}

// Filter lowest 7% and highest 7% off and calculate avarage -- filtering currently disabled

int getFilteredAvarage(struct avgStruct *a) {
    int threshold = 64; // about 7% (in scale 0-1023)
    int minVal = 0;
    int maxVal = 255;
    long int avgAll = 0;
    
    for (int i=0; i < a->size;i++) {
        if (a->avgData[i] < minVal) {
            minVal = a->avgData[i];
        } 
        if (a->avgData[i] > maxVal) {
            maxVal = a->avgData[i];
        } 
        avgAll += a->avgData[i];
    }
    avgAll = (int)(avgAll / a->size);
    return avgAll;
    
    /*
     int samples = 0;
     long int avg = 0;
     
     for (int i=0; i<a->size;i++) {
     if (a->avgData[i]>(minVal+threshold) && a->avgData[i]<(maxVal-threshold)) {
     samples++;
     avg += a->avgData[i];
     }
     }
     
     if (samples == 0) {
     return avgAll;
     } else {
     return avg/samples;
     }   
     */
}

/*
 unsigned char getAvarage(struct avgStruct *a,unsigned char newVal) {
 a->pos++;
 unsigned char size = (a->size?a->size:settings.inDamp);
 if (a->pos > size)
 a->pos=0;
 a->avgData[a->pos] = newVal;
 unsigned long ret = 0;
 for (int i=0;i<size;i++)
 ret += a->avgData[i];
 
 return ret / size;
 }g
 
 unsigned char getHighest(struct avgStruct *a,unsigned char newVal) {
 a->pos++;
 unsigned char size = (a->size?a->size:settings.inDamp);
 if (a->pos > size)
 a->pos=0;
 a->avgData[a->pos] = newVal;
 unsigned long ret = 0;
 
 for (int i=0;i<size;i++)
 if (a->avgData[i]>ret) ret= a->avgData[i];
 
 return ret;
 }
 
 */
boolean avarageThresholdExceeded(struct avgStruct *a,unsigned char threshold) {
    unsigned char minval = a->avgData[0];
    unsigned char maxval = a->avgData[0];
    unsigned char size = (a->size?a->size:settings.inDamp);
    for (int i=0;i<size;i++) {
        if (a->avgData[i] > maxval)
            maxval = a->avgData[i];
        if (a->avgData[i] < minval)
            minval = a->avgData[i];
    }
    
    return (maxval-minval)>threshold?true:false;
}

unsigned int lastOut = 0;
unsigned char lastPos = 0;

/*
 
 unsigned char simulatePressureActuator(int maxPressure,int currentPressure) {
 
 unsigned char cruisingMap = getHighest(&mapAvg,currentPressure);
 
 if (controls.statusBits & STATUS_CRUISING) {
 currentPressure = cruisingMap; // override
 } 
 
 // limit max movement on litte excepted gas flows (low rpm & tps range -- bottom left of the map)
 unsigned int sweetSpot = 40+((controls.tpsCorrected * controls.rpmCorrected) / 74);
 if (sweetSpot > 255)
 sweetSpot = 255;
 
 int idx = ((currentPressure*100/maxPressure)*255)/114;
 if (idx>255)
 idx = 255;
 
 int i = mapLookUp(vntActuatorMovementRemap,idx,0);
 
 controls.vntPositionTarget = i;
 
 // limit max by sweetSpot
 i = (i*sweetSpot)/255;
 
 if (lastPos < controls.vntPositionTarget) {
 // out
 int diff = 1+((controls.vntPositionTarget - lastPos)/2);
 if (diff>settings.outDamp) 
 diff = settings.outDamp;
 lastPos += diff;
 }
 if (lastPos > controls.vntPositionTarget) {
 // in
 int diff = 1+((lastPos - controls.vntPositionTarget)/2);
 if (diff>settings.inDamp) 
 diff = settings.inDamp;
 lastPos -= diff;
 }
 return lastPos;
 }
 */

unsigned char simulatePressureActuator(int maxPressure,int currentPressure) {
    
    unsigned int i,ret;
    
    // linear movemen
    // i = ((currentPressure*100/maxPressure)*256)/100;
    
    if (maxPressure == 0)
        return 255;
    
    // slighty logaritmic movement based on map
    long int idx = (((long int)currentPressure*100/(long int)maxPressure)*255)/100;
    if (idx<0) {
        i = 0;
    } else if (idx>255) {
        i = 255;
        /*
         if (i>255+60) {
         // too much boost decrease immediately (suppress indamp/outdamp);
         controls.vntPosition = i;
         */
    } else {
        i = mapLookUp(vntActuatorMovementRemap,idx,0); 
    }  
    
    controls.vntPositionTarget = i;
    return i;
    /*
     
     ret = getAvarage(&actuatorDamping,i);
     
     if (controls.statusBits & STATUS_CRUISING) {
     ret = getAvarage(&actuatorDamping,i);
     } 
     else {
     // Speed up while in not cruising mode
     for (unsigned char x=0;x<1+settings.inDamp/10;x++)
     ret = getAvarage(&actuatorDamping,i);
     }  
     
     return ret;*/
}

unsigned char integrateActuatorPos(int target,int current) {
    char hyst = 5;
    int i = controls.vntPositionTarget;
    int amount;
    if (target>current) {
        // underboost
        if (target-current>=hyst) {
            // recalculate pos.
            amount = (target-current)>3?(target-current)/3:1;
            if (i-amount>0) {
                i -= amount;
            } 
            else {
                i=0;
            }
            
        } 
    } 
    else {
        // overboost
        if (current-target>=hyst) {
            unsigned char amount = (current-target)>6?(current-target):6;
            if (i+amount<255) {
                i += amount;
            } 
            else {
                i=255;
            }
        }
    }
    controls.vntPositionTarget=i;
    return i;
}

void readValues() {
    //unsigned char mapVal = mapValues(controls.mapInput,settings.mapMin,settings.mapMax);
    // controls.mapCorrected = getFilteredAvarage(&mapAvg);
    //unsigned char tpsVal = mapValues(controls.tpsInput,settings.tpsMin,settings.tpsMax);
    
    controls.temp1 = toTemperature(analogRead(PIN_TEMP1)/4);
    controls.temp2 = toTemperature(analogRead(PIN_TEMP2)/4); 

    /*
    mapAvg.pos++;
    if (mapAvg.pos>=mapAvg.size)
        mapAvg.pos=0;
    mapAvg.avgData[mapAvg.pos] = analogRead(PIN_MAP);*/
    
    tpsAvg.pos++;
    if (tpsAvg.pos>=tpsAvg.size)
        tpsAvg.pos=0;
    tpsAvg.avgData[tpsAvg.pos] = analogRead(PIN_TPS);   
}

unsigned char accelVal = 0;

void processValues() {
    if (micros() - rpmLastTeethSeenTime > 200000) {
        // engine not running
        controls.rpmActual = 0;  
    }
    if (!controls.output1Enabled) {
        if (controls.temp1 >= settings.output1EnableTemp) {
            controls.output1Enabled = true;
        }
    } else {
        if (controls.temp1 <= settings.output1EnableTemp-TEMP_HYSTERESIS) {
            controls.output1Enabled = false;
        }
    }
    
    if (!controls.output2Enabled) {
        if (controls.temp2 >= settings.output2EnableTemp) {
            controls.output2Enabled = true;
        }
    } else {
        if (controls.temp2 <= settings.output2EnableTemp-TEMP_HYSTERESIS) {
            controls.output2Enabled = false;
        }
    }
    
    controls.rpmCorrected = mapValues(controls.rpmActual,0,settings.rpmMax);
    controls.mapInput = getFilteredAvarage(&mapAvg);
    controls.mapCorrected = mapValues(controls.mapInput,settings.mapMin,settings.mapMax);
    controls.tpsInput = getFilteredAvarage(&tpsAvg);
    controls.tpsCorrected = mapValues(controls.tpsInput,settings.tpsMin,settings.tpsMax);
    
    //getAvarage(&tpsAvg, controls.tpsCorrected);
    
    // interpolate curret output value
    //int dampFactor = (255-settings.vntOpeningPressureThreshold);
    
    if ( controls.tpsCorrected == 0 /*&& controls.rpmActual < 1200*/) { // TODO add hysteresis
        controls.statusBits |= STATUS_IDLE;
    } 
    else {
     /*   if (controls.rpmActual > 1250)*/
            controls.statusBits &= ~STATUS_IDLE;
    }
    
    
    if (settings.mode & MODE_DUTYCYCLEMAP) {
        if (!avarageThresholdExceeded(&tpsAvg,50)) {
            controls.statusBits |= STATUS_CRUISING;
        } else {
            // not cruising, set accel val
            if (controls.statusBits & STATUS_CRUISING)
                accelVal=10;
            controls.statusBits &= ~STATUS_CRUISING;
        }
        // Duty cycle map mode for VNT control
        // Uses RPM & TPS information.
        if (accelVal>0)
            accelVal--;
        controls.vntPositionTarget=accelVal;
        controls.vntTargetPressure = mapLookUp(vntActuatorPressureRequestMap,controls.rpmCorrected,controls.tpsCorrected);
        if (controls.vntTargetPressure<controls.vntPosition) {
            if ((int)controls.vntPosition-settings.inDamp<controls.vntTargetPressure) {
                controls.vntPosition = controls.vntTargetPressure;
            } 
            else {
                controls.vntPosition -= settings.inDamp;		
            }
        }
        
        if (controls.vntTargetPressure>controls.vntPosition) {
            if ((int)controls.vntPosition+settings.outDamp>controls.vntTargetPressure) {
                controls.vntPosition = controls.vntTargetPressure;
            } 
            else {
                controls.vntPosition += settings.outDamp;		
            }
        }
        if (controls.vntPosition>accelVal) {
            controls.vntPosition -= accelVal;
        } else {
            controls.vntPosition = 0;
        }
        // override vnt actuator position
        if ((settings.mode & MODE_VANESOPENIDLE) && (controls.statusBits & STATUS_IDLE)) {
            controls.vntPosition = 255;
        }		
        if (settings.mode & MODE_VNTOUTPUTINVERTED) {
            controls.vntPositionRemapped = mapValuesSqueeze(255-controls.vntPosition,settings.vntMinDC,settings.vntMaxDC);
        } 
        else {
            controls.vntPositionRemapped = mapValuesSqueeze(controls.vntPosition,settings.vntMinDC,settings.vntMaxDC);	
        }	
    } 
    else {
        // For adaptive target pressure mode -- TODO add hysteresis if needed
        
        controls.vntTargetPressure = mapLookUp(vntActuatorPressureRequestMap,controls.rpmCorrected,controls.tpsCorrected);
        controls.clipPos = mapLookUp(vntActuatorPositionLimitMap,controls.rpmCorrected,controls.tpsCorrected);

        controls.vntPositionTarget = simulatePressureActuator(controls.vntTargetPressure,controls.mapCorrected);
        if (controls.vntPositionTarget>controls.clipPos)
          controls.vntPositionTarget=controls.clipPos;
          
        //controls.vntPositionTarget = simulatePressureActuator(controls.vntTargetPressure,controls.mapCorrected);
        //controls.vntPosition = controls.vntPositionTarget;
        
        // TODO: replace with map!
        /*
        int diff = abs(controls.mapCorrected-controls.vntTargetPressure);
        int inDamp = settings.inDamp;
        int outDamp = settings.outDamp;
        if (diff < 10) {
            inDamp = inDamp /6;
        } else {
        } if (diff < 30) {
            inDamp = inDamp /5;
        } else if (diff < 30) {
            inDamp = inDamp /4;
        } else if (diff < 40) {
            inDamp = inDamp /2;
        }
        if (inDamp<1) inDamp = 1;
        if (outDamp<1) outDamp = 1;
        */
        
        unsigned char newPos;
        //  add smoothing if calculated new value is in range of -15/+15 ...
        if (abs((int)controls.vntPositionTarget - (int)controls.vntPosition) < settings.servoStabilizerThreshold) {
            if (settings.servoStabilizerSmoothValue>1) {
                newPos = (int)((controls.vntPosition*(settings.servoStabilizerThreshold-1)+controls.vntPositionTarget)/settings.servoStabilizerThreshold);
                controls.statusBits |= STATUS_CRUISING;
            } else {
                newPos = controls.vntPositionTarget;
                controls.statusBits &= ~STATUS_CRUISING;
            }
        } else {
            newPos = controls.vntPositionTarget; 
            controls.statusBits &= ~STATUS_CRUISING;
        }
        
        if (newPos<controls.vntPosition) {
            if (controls.vntPosition - newPos > settings.inDamp) {
                controls.vntPosition -= settings.inDamp;
            } else {
                controls.vntPosition = newPos;
            }
        } 
        if (newPos>controls.vntPosition) {
            if (newPos - controls.vntPosition > settings.outDamp) {
                controls.vntPosition += settings.outDamp;
            } else {
                controls.vntPosition = newPos;
            }
        }
        
        // Use limiter map to "help" keeping max charge pressure (do not open vanes too much)
        //if (controls.vntPosition>maxPos)
        //    controls.vntPosition = maxPos;
        // 22.8.2011 maxPos changed -> vnt map scaled according to max pos
        /*
         controls.vntPosition = simulatePressureActuator(controls.vntTargetPressure,
         controls.mapCorrected,
         settings.vntOpeningPressureThreshold); // max press, current press, preload press%.
         */
        
        unsigned char finalPos;
        // override vnt actuator position
        if ((settings.mode & MODE_VANESOPENIDLE) && (controls.statusBits & STATUS_IDLE)) {
            finalPos = 255;
        } else {
            finalPos = controls.vntPosition;
        }
        if (settings.mode & MODE_VNTOUTPUTINVERTED) {
            controls.vntPositionRemapped = mapValuesSqueeze(255-finalPos,settings.vntMinDC,settings.vntMaxDC);
        } 
        else {
            controls.vntPositionRemapped = mapValuesSqueeze(finalPos,settings.vntMinDC,settings.vntMaxDC);	
        }		
    }
    
    controls.ldaPosition = mapLookUp(ldaPositionMap,controls.rpmCorrected,controls.mapCorrected);
    
    if (settings.mode & MODE_LDAOUTPUTINVERTED) {
        controls.ldaPositionRemapped = mapValuesSqueeze(255-controls.ldaPosition,settings.ldaMinDC,settings.ldaMaxDC);
    } 
    else {
        controls.ldaPositionRemapped = mapValuesSqueeze(controls.ldaPosition,settings.ldaMinDC,settings.ldaMaxDC);
    }
}

void updateOutputValues(bool showDebug) {
    // PWM output pins
    analogWrite(PIN_VNT_N75,controls.vntPositionRemapped);
    analogWrite(PIN_LDA_N75,controls.ldaPositionRemapped);
    digitalWrite(PIN_OUTPUT1,controls.output1Enabled?HIGH:LOW);
    digitalWrite(PIN_OUTPUT2,controls.output2Enabled?HIGH:LOW);
    // Software controlled servos
    // servoLda.write(controls.ldaPositionRemapped);
    // servoVnt.write(controls.vntPositionRemapped);
    
    if (showDebug) {
        Serial.print("\033[1;70H");
        Serial.print(controls.vntPositionRemapped,DEC);
        Serial.print(" ");
        Serial.print(controls.ldaPositionRemapped,DEC);
        printFromFlash(ANSIclearEolAndLf);	
    }
}

bool s;
unsigned char teethNo = 0;
long int rpmMicros = 0;  

void rpmTrigger() {
    //__asm("cli");
    s = !s;
    unsigned long now = micros();
    // filter out implausible signals
    if (now - rpmLastTeethSeenTime > 400) { 
        teethNo++;
        if (teethNo == settings.rpmTeethsPerRotation) {
            teethNo = 0;
            controls.rpmActual = (unsigned int)(((1000000*60)/((now-rpmMicros))));
            rpmMicros = now;
        }
        //rpmNow = micros();
        //controls.rpmActual = ((settings.rpmTeethsPerRotation*100000000)/((now- rpmLastTeethSeenTime)*60));
        rpmLastTeethSeenTime = now; 
        digitalWrite(PIN_HEARTBEAT,(s?HIGH:LOW));
        
        //controls.mapInput = analogRead(PIN_TPS);
        //controls.tpsInput = analogRead(PIN_MAP);
        
        // Read TPS & MAP values
        //readValues();
        mapAvg.pos++;
        if (mapAvg.pos>=mapAvg.size)
            mapAvg.pos=0;
        mapAvg.avgData[mapAvg.pos] = analogRead(PIN_MAP);
    }
    
    //__asm("sei");
}

void updateLCD() {
    lcd.clear();
    lcd.setCursor(0,0);
    //           1234567890123456
    lcd.print("    cbar     rpm");
    lcd.setCursor(0,1);
    lcd.print("   /   C        ");
    
    lcd.setCursor(0,0);
    lcd.print(toBars(controls.mapCorrected));
    if (controls.rpmActual>=1000) {
        lcd.setCursor(10,0);
    } else {
        lcd.setCursor(10,0);
    }
    lcd.print(controls.rpmActual);
    lcd.setCursor(0,1);
    lcd.print(controls.temp1,DEC);
    lcd.setCursor(4,1);
    lcd.print(controls.temp2,DEC);
    
    lcd.setCursor(9,1);
    lcd.print(controls.tpsCorrected,DEC);
    lcd.setCursor(13,1);
    lcd.print(controls.vntPosition,DEC);
}

unsigned char status=0;
bool freezeModeEnabled=false;

unsigned char counter;

void loop() {
    // read sensor data (moved off from interrupt handler)
    readValues();
    
    counter++;
    unsigned char data = 0;
    unsigned long start = millis();
    
    // User interface for configuration and monitoring
    if (Serial.available()) {
        data = Serial.read();
        if (data >= '0' && data <= '9') {
            page = data - '0';
        } 
        else if (data == '!') {
            // Reads incoming map data etc.. R
            readSysExCommand();
        } 
        else if (data == ':') {
            freezeModeEnabled = !freezeModeEnabled;
        } 
        else if (data == '#') {
            page = 10;
        } else if (data == 27) {
            data = Serial.read();
            if (data == '[') {
                data = Serial.read();
                switch (data) {
                    case 'A':
                        data = 'k';
                        break;
                    case 'B':
                        data = 'j';
                        break;
                    case 'C':
                        data = 'l';
                        break;
                    case 'D':
                        data = 'h';
                        break;
                        
                    default:
                        data = 0;
                }
            }
        }
        
    }
    
    switch(page) {
        case 1:
            pageStatusAndAdaption(data);
            break;
        case 2:
            pageServoFineTune(data);
            break;    
        case 3:
            pageMapEditor(0,data);
            break;
        case 4:
            pageMapEditor(1,data,false);
            gotoXY(1,23);
            Serial.print("Target:");
            printPads(controls.vntPositionTarget /4,'*');
            printFromFlash(ANSIclearEolAndLf);    
            gotoXY(7+controls.clipPos/4,23);
            Serial.print("C");
            break; 
        case 5:
            pageMapEditor(2,data);
            break;
        case 6:
            pageMapEditor(3,data,false);
            break;			
        case 7:
            pageMapEditor(4,data);
            break;			
        case 8:
            pageExport(data);
            break;	
        case 9:
            pageOutputTests(data);
            break;
        case 10:
            pageDataLogger(data);
            break;
        case 0:
        default:
            pageAbout(data);
    }
    /*
    if (controls.rpmActual == 0) {
        // engine is not running, read values manually instead of interrupt based update
        readValues();
    }*/
    if (freezeModeEnabled) {
        // freeze settings (for debuggin)
        //printFromFlash(ANSIgoHome); 
        Serial.print("\rFREEZE ");
    } else {
        // update output values according to input
        processValues();
    }
    updateOutputValues(false);
    if (counter%8 == 0) 
        updateLCD();
    unsigned long diff = millis()-start;
    if ((int)MAIN_LOOP_DELAY-(int)diff>0 && diff < 3000) {
        delay(MAIN_LOOP_DELAY-diff);
    }
//    status=!status;
}

