//ToDO: Add a screen timeout feature

#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_VL6180X.h"

#define VERSION "0.10"

#define touchPin1 D5
#define touchPin2 D6
#define beepPin D7 // Beep

#define DETECT_RANGE    100
#define MAX_LAPS        100
#define WAIT_TIME_MS    500
#define LONG_PRESS_MS   750
#define BUTTON_PRESS_MS 150
#define SCREEN_TIMEOUT_MS   60 * 1000

#define SCREEN_WIDTH 128       // OLED display width, in pixels
#define SCREEN_HEIGHT 64       // OLED display height, in pixels
#define OLED_RESET LED_BUILTIN // Reset pin # (or -1 if sharing Arduino reset pin)

//----------------------------------------------------------------
TwoWire wire;
Adafruit_VL6180X vl;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &wire, OLED_RESET);

//----------------------------------------------------------------
unsigned long screenTime = 0;
unsigned long updateTime = 0;
bool wide = true;   // wide or tall display format
unsigned long start = 0;
unsigned char Laps[MAX_LAPS];
int lapCounter = 0;
int lapTop = 0;

//----------------------------------------------------------------
void Beep(int ms, int repeat)
{
    while (repeat-- > 0)
    {
        digitalWrite(beepPin, LOW);
        delay(ms);
        digitalWrite(beepPin, HIGH);

        if (repeat > 0)
            delay(ms);
    }
}

//----------------------------------------------------------------
void TextAt(int x, int y, int size, const char *text)
{
    display.setCursor(x, y);
    display.setTextSize(size);
    display.print(text);
}
//----------------------------------------------------------------
void TextAt(int x, int y, int size, int val)
{
    display.setCursor(x, y);
    display.setTextSize(size);
    display.printf("%d", val);
}
//----------------------------------------------------------------
void setup()
{
    pinMode(touchPin1, INPUT);
    pinMode(touchPin2, INPUT);
    pinMode(beepPin, OUTPUT);

    wire.begin(5, 4);

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
        {
        } // Don't proceed, loop forever
    }

    wire.setClock(400000);
    vl.begin(&wire);

    display.clearDisplay();
    display.setTextColor(WHITE);
    display.print("\nLattice Lap Counter\n\nVersion: " VERSION);
    display.display();

    delay(2000);
}

//----------------------------------------------------------------
void renderTall(float range)
{
    display.setRotation(3);

    // calculate total lap time.
    unsigned int total = 0;
    for (int i = 0; i < MAX_LAPS; ++i)
        total += Laps[i];

    int secs = min(255, (int)((millis() - start) / 1000));

    display.setTextSize(2);
    display.setCursor(0, 0);
    if (lapCounter > 0)
    {
        TextAt(0, 0, 1, "Total Time ");
        TextAt(5, 14, 2, total);

        if( lapCounter == lapTop)
        {
            TextAt(0, 34, 1, "Current");
            TextAt(5, 48, 2, secs);
        }
        else
        {
            TextAt(0, 34, 1, "Lap Time");
            TextAt(5, 48, 2, Laps[lapCounter]);
        }
    }
    else
        display.print("Ready");

    // lap counter
    TextAt(0,75,1,"Lap");
    TextAt(5,92,4,lapCounter);

    // distance bar
    display.drawFastVLine(62, 0, (int)(range / 2), WHITE);
    display.drawFastHLine(61, DETECT_RANGE / 2, 3, WHITE);
}

//----------------------------------------------------------------
void renderWide(float range)
{
    display.setRotation(0);

    // calculate total lap time.
    unsigned int total = 0;
    for (int i = 0; i < MAX_LAPS; ++i)
        total += Laps[i];

    int secs = min(255, (int)((millis() - start) / 1000));

    display.setCursor(0, 0);
    if (lapCounter > 0)
    {
        TextAt(0, 3, 1, "Total Time ");
        TextAt(display.getCursorX(), 0, 2, total);

        if( lapCounter == lapTop)
        {
            TextAt(0, 21, 1, "Current");
            TextAt(0, 36, 2, secs);
        }
        else
        {
            TextAt(0, 21, 1, "Lap Time");
            TextAt(0, 36, 2, Laps[lapCounter]);
        }
    }
    else
    {
        display.setTextSize(2);
        display.print("Ready...");
    }
    display.drawFastHLine(0, 16, SCREEN_WIDTH - 1, WHITE);

    // lap counter
    TextAt(55, 36, 1, "Lap");
    TextAt(79, 25, 4, lapCounter);

    // distance bar
    display.drawFastHLine(0, 62, (int)(range / 2), WHITE);
    display.drawFastVLine(DETECT_RANGE / 2, 61, 3, WHITE);
}
//----------------------------------------------------------------
void renderScreen(float range)
{
    display.clearDisplay();
    display.setTextColor(WHITE);

    if((millis() - screenTime) < SCREEN_TIMEOUT_MS)
    {
        if (wide)
            renderWide(range);
        else
            renderTall(range); // display results
    }

    display.display();
}
//----------------------------------------------------------------
float lidar()
{
    static bool _lastTriggered = false;
    static unsigned long _startWait = 0;

    float range = vl.readRange();
    uint8_t status = vl.readRangeStatus();

    bool triggered = (status == VL6180X_ERROR_NONE) && range < DETECT_RANGE;

    if (triggered != _lastTriggered) // edge detect - need to debounce as well
    {
        _lastTriggered = triggered;

        if (triggered)
        {
            screenTime = millis();   // keep the screen awake
            _startWait = millis();
            Beep(50, 1);
        }
        else if ((millis() - _startWait) > WAIT_TIME_MS) // make sure we are triggered for more than WAIT_TIME_MS before incrementing the counter
        {
            // we have a valid trigger event
            if (lapCounter > 0)
            {
                unsigned long laptime = millis() - start;

                if (lapCounter < MAX_LAPS)
                    Laps[lapCounter] = (unsigned char)(laptime / 1000); // record the lap time
            }

            if(lapCounter == 0)
            {
                lapTop = 0;
                // clear the lap counts
                for (int i = 0; i < MAX_LAPS; ++i)
                    Laps[i] = 0;
            }

            start = millis();   // start the lap timer
            ++lapCounter;
            ++lapTop;

            Beep(150, 1);
        }
        else
        {
            Beep(30, 3); // missed count
        }
    }

    return range;
}

//----------------------------------------------------------------
void touch()
{
    static unsigned long buttonDown1 = 0;
    static unsigned long buttonDown1a = 0;
    static unsigned long buttonDown2 = 0;

    if (digitalRead(touchPin1) == 1)
    {
        screenTime = millis();   // keep the screen awake

        if( buttonDown1 == 0)
            buttonDown1 = millis();

        if((millis() - buttonDown1) >= BUTTON_PRESS_MS)    // check for short press
        {
            if (lapCounter > 0)
                lapCounter = lapCounter - 1;
            buttonDown1 = 0;
        }

        if(lapCounter == 0)
        {
            if( buttonDown1a == 0)
                buttonDown1a = millis();

            if((millis() - buttonDown1a) > LONG_PRESS_MS)    // check for long press
            {
                wide = ! wide;
                Beep(75,1);
                buttonDown1a = 0;
            }
        }
    }
    else
    {
        buttonDown1 = 0;
        buttonDown1a = 0;
    }

    // check the second touch panel
    if (digitalRead(touchPin2) == 1)
    {
        screenTime = millis();   // keep the screen awake

        if( buttonDown2 == 0)
            buttonDown2 = millis();

        if((millis() - buttonDown2) >= BUTTON_PRESS_MS)    // check for short press
        {
        if (lapCounter < lapTop)    // limit to the last lap index
            lapCounter = lapCounter + 1;
            buttonDown2 = 0;
        }
    }
    else
        buttonDown2 = 0;
}

//----------------------------------------------------------------
void loop()
{
    touch(); // check the touch panel

    if(( millis() - updateTime) > 75)   // update lidar and screen periodically
    {
        float range = lidar(); // check the range sensor

        renderScreen(range);

        updateTime = millis();
    }
}
