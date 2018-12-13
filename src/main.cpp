#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_VL6180X.h"

#define touchPin1 D5
#define touchPin2 D6
#define beepPin D7 // Beep

#define WAIT_TIME 500
#define DETECT_RANGE 100
#define MAX_LAPS 100

#define SCREEN_WIDTH 128       // OLED display width, in pixels
#define SCREEN_HEIGHT 64       // OLED display height, in pixels
#define OLED_RESET LED_BUILTIN // Reset pin # (or -1 if sharing Arduino reset pin)

//----------------------------------------------------------------
TwoWire wire;
Adafruit_VL6180X vl;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &wire, OLED_RESET);

float range = 0;
int counter = 0;
bool lastTriggered = false;
bool wide = true;
unsigned long timeout = 0;
unsigned long start = 0;
unsigned long lasttime = 0;
static unsigned char Laps[MAX_LAPS];

//----------------------------------------------------------------
void Beep(int ms, int repeat)
{
    while(repeat-- > 0)
    {
        digitalWrite(beepPin, LOW);
        delay(ms);
        digitalWrite(beepPin, HIGH);

        if( repeat > 0)
            delay(ms);
    }
}

//----------------------------------------------------------------
void setup()
{
    pinMode(touchPin1, INPUT);
    pinMode(touchPin2, INPUT);
    pinMode(beepPin, OUTPUT);

    wire.begin(0, 2);

    Serial.begin(115200);
    // wait for serial port to open on native usb devices
    while (!Serial)
        delay(1);

    // scan for I2C device
    // Serial.println();
    // Serial.println("I2C scanner. Scanning ...");
    // byte count = 0;

    // for (byte i = 8; i < 120; i++)
    // {
    //   wire.beginTransmission(i);
    //   if (wire.endTransmission() == 0)
    //   {
    //     Serial.print("Found address: ");
    //     Serial.print(i, DEC);
    //     Serial.print(" (0x");
    //     Serial.print(i, HEX);
    //     Serial.println(")");
    //     count++;
    //     delay(1); // maybe unneeded?
    //   }           // end of good response
    // }             // end of for loop
    // Serial.println("Done.");
    // Serial.print("Found ");
    // Serial.print(count, DEC);
    // Serial.println(" device(s).");

    Beep(100, 2);

    //SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C) == false) // Address 0x3C for 128x64
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
        {} // Don't proceed, loop forever
    }

    display.clearDisplay();
    display.display();

    wire.setClock(400000);
    vl.begin(&wire);

    // Serial.println("Adafruit VL6180x test!");
    // if (vl.begin() == false)
    // {
    //     Serial.println("Failed to find sensor");
    //     while (1)
    //     {
    //     }
    // }
    start = millis();
}

//----------------------------------------------------------------
void renderTall()
{
    display.setRotation(3);
    display.clearDisplay();
    display.setTextColor(WHITE);

    // calculate total lap time.
    unsigned int total = 0;
    for(int i = 0; i < MAX_LAPS; ++i)
        total += Laps[i];

    int secs = min(255, (int)((millis() - start) / 1000));

    display.setTextSize(2);
    display.setCursor(0, 0);
    if(counter > 0)
        display.printf("%d\n\n%d", total, secs);
    else
        display.print("Ready");

    // lap counter
    display.setCursor(5, 60);
    display.print("Laps");
    display.setTextSize(4);
    display.setCursor(5, 90);
    display.print(counter);

    // distance bar
    display.drawFastVLine(62,0, (int)(range/2), WHITE);
    display.drawFastHLine(61,DETECT_RANGE/2,3, WHITE);

    display.display();
}
//----------------------------------------------------------------
void renderWide()
{
    display.setRotation(0);
    display.clearDisplay();
    display.setTextColor(WHITE);

    // calculate total lap time.
    unsigned int total = 0;
    for(int i = 0; i < MAX_LAPS; ++i)
        total += Laps[i];

    int secs = min(255, (int)((millis() - start) / 1000));

    display.setTextSize(2);
    display.setCursor(0, 0);
    if(counter > 0)
        display.printf("Total %d\n\n%d", total, secs);
    else
        display.print("Ready...");

    // lap counter
    display.setTextSize(4);
    display.setCursor(75, 25);
    display.print(counter);

    // distance bar
    display.drawFastHLine(0,62,(int)(range/2), WHITE);
    display.drawFastVLine(DETECT_RANGE/2,61,3, WHITE);

    display.display();
}

//----------------------------------------------------------------
void lidar()
{
    range = vl.readRange();
    uint8_t status = vl.readRangeStatus();

    bool triggered = (status == VL6180X_ERROR_NONE) && range < DETECT_RANGE;

    if (triggered != lastTriggered) // edge detect - need to debounce as well
    {
        lastTriggered = triggered;

        if (triggered)
        {
            timeout = millis();
            Beep(50,1);
        }
        else if ((millis() - timeout) > WAIT_TIME) // make sure we are triggered for more than WAIT_TIME before incrementing the counter
        {
            if( counter++ > 0)
            {
                lasttime = millis() - start;
                if( counter < MAX_LAPS)
                    Laps[counter - 1] = (unsigned char)(lasttime / 1000);
            }
            start = millis();
            Beep(150, 1);
        }
        else
        {
            Beep(50, 3);    // missed count
        }
    }
}



//----------------------------------------------------------------
void reset()
{
    Beep(50,1);

    counter = 0;
    lasttime = 0;
    start = millis();

    for(int i = 0; i < MAX_LAPS; ++i)
        Laps[i] = 0;

    wide = wide == false;
}

//----------------------------------------------------------------
void touch()
{
    static unsigned int debounce1 = 0;
    static unsigned int debounce2 = 0;

    if (digitalRead(touchPin1) == 1)
    {
        if(counter > 0)
            counter = counter - 1;

        if(counter == 0)
            reset();
    }

    if (digitalRead(touchPin2) == 1)
    {
        if( counter < MAX_LAPS - 1)
            counter = counter + 1;

        lasttime = 0;
        start = millis();
    }
}

//----------------------------------------------------------------
void loop()
{
    touch();  // check the touch panel
    lidar();  // check the range sensor

    if(wide)
        renderWide();
    else
        renderTall(); // display results

    delay(50);
}
