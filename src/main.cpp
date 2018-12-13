#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_VL6180X.h"

#define touchPin D5
#define beepPin D6 // Beep

#define WAIT_TIME 500
#define DETECT_RANGE 100

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
unsigned long timeout = 0;
unsigned long start = 0;
unsigned long lasttime = 0;

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
    pinMode(touchPin, INPUT);

    wire.begin(0, 2);

    Serial.begin(115200);
    // wait for serial port to open on native usb devices
    while (!Serial)
    {
        delay(1);
    }

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

    pinMode(beepPin, OUTPUT);

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
void render()
{
    display.clearDisplay();
    display.setTextColor(WHITE);

    if(counter > 0)
    {
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.printf("%d/%d", (int)(lasttime / 1000), (int)((millis() - start) / 1000));
    }

    display.setTextSize(4);
    display.setCursor(50, 25);
    display.print(counter);

    // display.setCursor(0, 54);
    // display.setTextSize(1);
    // display.print((int)range);


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
                lasttime = millis() - start;
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
void touch()
{
    if (digitalRead(touchPin) == 1)
    {
        if(counter > 0)
            counter = counter - 1;

        lasttime = 0;
        start = millis();
    }
}

//----------------------------------------------------------------
void loop()
{
    touch();  // check the touch panel
    lidar();  // check the range sensor
    render(); // display results

    delay(50);
}
