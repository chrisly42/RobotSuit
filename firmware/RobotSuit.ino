#include <FastLED.h>
#include <SPI.h>
#include <Adafruit_SPIFlash.h>
#include <seesaw_servo.h>
#include <SdFat.h>
#include <Adafruit_CircuitPlayground.h>
#include <Adafruit_Crickit.h>
#include <seesaw_neopixel.h>
#include <seesaw_motor.h>
#include <Adafruit_seesaw.h>
#include "AudioZeroDMA.h"

Adafruit_Crickit crickit;
seesaw_Servo servo1(&crickit);  // create servo object to control a servo
seesaw_Servo servo2(&crickit);  // create servo object to control a servo
seesaw_Motor motorLeft(&crickit);
seesaw_Motor motorRight(&crickit);

#define NUM_SOUNDFILES 16

const char *soundfiles[] = {
        "Fizzle.raw",
        "gost_1.raw",
        "gost_2.raw",
        "gost_3.raw",
        "gost_4.raw",
        "gost_5.raw",
        "gost_7.raw",
        "gost_11.raw",
        "gost_15.raw",
        "gost_16.raw",
//  "LaserBlasts.raw",
        "LightsaberTurnOff.raw",
        "LightsaberTurnOn.raw",
        "R2D2Again.raw",
        "RobotBlip.raw",
        "RobotBlip2.raw",
        //"SciFiRobot.raw",
        "ShortCircuitSound.raw",
        //"Smkn_invite.raw",
        // "Smkn_notice_highlite.raw"
};

boolean randomSoundEnabled = true;

#define CAPTOUCH_THRESH   900

#define NUM_STRIPE_LEDS 15
#define STRIPE_LEDS_PIN A1

Adafruit_FlashTransport_SPI flashTransport(SS, &SPI);
Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatFileSystem fatfs;

AudioZeroDMA audioPlayer;

uint16_t loudness = 0;
int16_t cylonePos = 0;
int8_t cyloneDir = 1;
uint16_t cyloneSpeed = 0;
uint16_t cyloneHue = 0;

CRGB stripeLeds[NUM_STRIPE_LEDS];

uint16_t antennaPos1;
uint16_t antennaPos2;
uint16_t antennaLedMask1 = 0x800;
uint16_t antennaLedPWMMask1 = 0xffff;
uint16_t antennaLedMask2 = 0x800;
uint16_t antennaLedPWMMask2 = 0xffff;
uint16_t motorLeftPos;
uint16_t motorRightPos;

uint16_t listenModeTime = 0;
float rotateEarLeftSpeed = 0.0;
float rotateEarRightSpeed = 0.0;
uint16_t rotateEarLeftTime = 0;
uint16_t rotateEarRightTime = 0;

boolean leftButtonPressed = false;
boolean rightButtonPressed = false;

float smoothAccelX = 0;
float smoothAccelY = 0;
float smoothAccelZ = 9.81;

boolean motorsEnabled = false;
boolean servosEnabled = false;

boolean motorsStopped = false;
boolean servosStopped = false;

#define NUM_RING_LEDS 10
#define RING_LEDS_PIN 8


CRGB ringLeds[NUM_RING_LEDS];

class Star
{
    uint8_t hue;
    uint16_t fadespeed;
    uint8_t brightness;
    uint8_t remain;
    uint8_t twinkle;
    uint8_t pureness;
    int16_t fadepos;
    boolean fadeout;

public:
    void init()
    {
        restart();
    }

    void restart()
    {
        hue = random8();
        fadespeed = (random8() << 1) + 64;
        brightness = random8(128, 255);
        remain = random8(32, 255);
        twinkle = random8() & 31;
        pureness = (random8() & 127) + 128;
        fadepos = 0;
        fadeout = false;
    }

    CHSV getValue()
    {
        return CHSV(hue + random8(twinkle), qadd8(pureness, random8(twinkle)), fadepos >> 6);
    }

    void animate()
    {
        if (fadeout) {
            fadepos -= fadespeed;
            if (fadepos <= 0) {
                restart();
            }
        } else {
            if ((fadepos >> 6) >= brightness) {
                fadeout = true;
            } else {
                fadepos += fadespeed;
            }
        }
    }
};

Star stars[NUM_RING_LEDS];

void setup()
{
    Serial.begin(115200);

    Serial.println("Hi!");

    CircuitPlayground.begin();
    CircuitPlayground.redLED(HIGH);
    if (!crickit.begin()) {
        Serial.println("ERROR!");
        while (1) {
            CircuitPlayground.redLED(HIGH);
            delay(100);
            CircuitPlayground.redLED(LOW);
            delay(100);
        }
    } else {
        Serial.println("Crickit started");
    }

    servo1.attach(CRICKIT_SERVO1, 450, 2300);  // attaches the servo to CRICKIT_SERVO1 pin
    servo2.attach(CRICKIT_SERVO2, 450, 2300);  // attaches the servo to CRICKIT_SERVO2 pin

    //attach motor a and b
    motorLeft.attach(CRICKIT_MOTOR_A1, CRICKIT_MOTOR_A2);
    motorRight.attach(CRICKIT_MOTOR_B1, CRICKIT_MOTOR_B2);

    // our default frequency is 1khz
    crickit.setPWMFreq(CRICKIT_DRIVE1, 1000);
    crickit.setPWMFreq(CRICKIT_DRIVE2, 1000);

    FastLED.addLeds<NEOPIXEL, STRIPE_LEDS_PIN>(stripeLeds, NUM_STRIPE_LEDS);
    FastLED.addLeds<NEOPIXEL, RING_LEDS_PIN>(ringLeds, NUM_RING_LEDS);
    ringLeds[0] = CRGB(255, 0, 0);
    stripeLeds[0] = CRGB(0, 255, 0);
    FastLED.show();

    for (uint8_t i = 0; i < NUM_RING_LEDS; i++) {
        random16_add_entropy(random());
        stars[i].init();
    }
    crickit.pinMode(CRICKIT_SIGNAL1, INPUT);
    crickit.pinMode(CRICKIT_SIGNAL2, INPUT);
    delay(100);
    CircuitPlayground.redLED(LOW);
    servosEnabled = motorsEnabled = CircuitPlayground.slideSwitch();

    while (!flash.begin()) {
        Serial.println("Error, failed to initialize flash chip!");
        delay(1000);
    }

    // First call begin to mount the filesystem.  Check that it returns true
    // to make sure the filesystem was mounted.
    while (!fatfs.begin(&flash)) {
        Serial.print("Flash chip JEDEC ID: 0x");
        Serial.println(flash.getJEDECID(), HEX);
        Serial.print("Failed to mount filesystem! ");
        Serial.println("Was CircuitPython loaded on the board first to create the filesystem?");
        delay(1000);
    }
    Serial.println("Mounted filesystem!");

    while (audioPlayer.begin() != 0) {
        Serial.println("AudioPlayer begin() failed");
        delay(1000);
    }
#if 1
    File file = fatfs.open("Smkn_invite.raw");
    if (file) {
        //Serial.println("Playing...");
        audioPlayer.play(22050, file);
    } else {
        //Serial.println("File not found!");
    }
#endif
}

void loop()
{
    audioPlayer.poll();
    random16_add_entropy(CircuitPlayground.soundSensor());

    float accelX = CircuitPlayground.motionX();
    float accelY = CircuitPlayground.motionY();
    float accelZ = CircuitPlayground.motionZ();

    for (uint8_t i = 0; i < NUM_RING_LEDS; i++) {
        stars[i].animate();
        ringLeds[i] = stars[i].getValue();
    }
    fadeToBlackBy(stripeLeds, NUM_STRIPE_LEDS, 16);
    uint8_t ledPos = cylonePos >> 8;
    uint8_t ledFract = cylonePos;
    cyloneHue += (abs(accelX) + abs(accelY)) * 8.0;

    stripeLeds[ledPos] += CHSV(cyloneHue >> 8, 255, dim8_raw(255 - ledFract));
    stripeLeds[ledPos + 1] += CHSV(cyloneHue >> 8, 255, dim8_raw(ledFract));
    cyloneSpeed += (abs(accelX) + abs(accelY) + abs(accelZ - 9.3));
    if (cyloneSpeed > 20) cyloneSpeed -= 5;
    if (cyloneSpeed > 400) cyloneSpeed = 400;
    if (cyloneDir > 0) {
        cylonePos += cyloneSpeed;
        if (cylonePos >= ((NUM_STRIPE_LEDS - 1) << 8)) {
            cylonePos = ((NUM_STRIPE_LEDS - 1) << 8);
            cyloneDir = -1;
        }
    } else {
        cylonePos -= cyloneSpeed;
        if (cylonePos <= 0) {
            cylonePos = 0;
            cyloneDir = 1;
        }
    }
    FastLED.show();

    smoothAccelX = smoothAccelX * 0.95 + accelX * 0.05;
    smoothAccelY = smoothAccelY * 0.95 + accelY * 0.05;
    smoothAccelZ = smoothAccelZ * 0.95 + accelZ * 0.05;

    /*Serial.print("X: "); Serial.print(smoothAccelX);
    Serial.print(" \tY: "); Serial.print(smoothAccelY);
    Serial.print(" \tZ: "); Serial.println(smoothAccelZ);*/
    //uint16_t val = ;   // read the touch input
    //Serial.println(crickit.touchRead(0));

    if (CircuitPlayground.leftButton() || (crickit.touchRead(0) > CAPTOUCH_THRESH)) {
        if (!leftButtonPressed) {
            leftButtonPressed = true;
            motorsEnabled = !motorsEnabled;
            if (!motorsEnabled) {
                servosEnabled = !servosEnabled;
            }
        }
    } else {
        leftButtonPressed = false;
    }
    if (CircuitPlayground.rightButton() || (crickit.touchRead(1) > CAPTOUCH_THRESH)) {
        if (!rightButtonPressed) {
            rightButtonPressed = true;
            randomSoundEnabled = !randomSoundEnabled;
            if (randomSoundEnabled) {
                File file = fatfs.open("Smkn_notice_highlite.raw");
                audioPlayer.play(22050, file);
            } else {
                File file = fatfs.open("Fizzle.raw");
                audioPlayer.play(22050, file);
            }
        }
    } else {
        rightButtonPressed = false;
    }

    motorLeftPos += 270;
    motorRightPos += 253;
    if (motorsEnabled) {
        if (listenModeTime == 0) {
            float leftSpeed = sin16(motorLeftPos) * (0.15 / 32768.0);
            leftSpeed += smoothAccelX * 0.1;
            if ((leftSpeed > -0.15) && (leftSpeed < 0.15)) leftSpeed = 0;
            motorLeft.throttle(leftSpeed);
            float rightSpeed = sin16(motorRightPos) * (0.15 / 32768.0);
            rightSpeed -= smoothAccelX * 0.1;
            if ((rightSpeed > -0.15) && (rightSpeed < 0.15)) rightSpeed = 0;
            motorRight.throttle(rightSpeed);
        } else {
            motorLeft.throttle(rotateEarLeftSpeed);
            if (rotateEarLeftTime) {
                rotateEarLeftTime--;
            } else {
                rotateEarLeftTime = random8(32);
                if (rotateEarLeftSpeed == 0.0) {
                    rotateEarLeftSpeed = random8(2) ? -1.0 : 1.0;
                } else {
                    rotateEarLeftSpeed = -rotateEarLeftSpeed;
                }
            }

            motorRight.throttle(rotateEarRightSpeed);
            if (rotateEarRightTime) {
                rotateEarRightTime--;
            } else {
                rotateEarRightTime = random8(32);
                if (rotateEarRightSpeed == 0.0) {
                    rotateEarRightSpeed = random8(2) ? -1.0 : 1.0;
                } else {
                    rotateEarRightSpeed = -rotateEarRightSpeed;
                }
            }

            listenModeTime--;
        }
        motorsStopped = false;
    } else if (!motorsStopped) {
        //Serial.println("Motors stopped!");
        motorLeft.throttle(0);
        motorRight.throttle(0);
        motorsStopped = true;
    }

    antennaPos1 += 37 + (loudness << 2);
    antennaPos2 += 63 + (loudness << 2);
    if (servosEnabled) {
        uint16_t pos =
                scale16(((sin16(antennaPos1) + 32768) >> 2) * 3 + ((cos16(motorLeftPos + antennaPos2) + 32768) >> 2),
                        150) + 15;
        servo1.write(pos);

        pos = scale16(((sin16(antennaPos2) + 32768) >> 2) * 3 + ((cos16(motorRightPos - antennaPos1) + 32768) >> 2),
                      150) + 15;
        servo2.write(pos);
        servosStopped = false;
    } else if (!servosStopped) {
        //Serial.println("Servos stopped!");
        servo1.write(90);
        servo2.write(90);
        servosStopped = true;
    }
    if (random8() > 253) {
        antennaLedMask1 ^= (1 << (random8(4) + 5));
    }
    if (random8() > 253) {
        antennaLedPWMMask1 ^= (1 << (random8(8) + 8));
    }
    if (random8() > 253) {
        antennaLedMask2 ^= (1 << (random8(4) + 5));
    }
    if (random8() > 253) {
        antennaLedPWMMask2 ^= (1 << (random8(8) + 8));
    }
    if (!(antennaPos1 & antennaLedMask1)) {
        crickit.analogWrite(CRICKIT_DRIVE1, (antennaPos1 | antennaLedPWMMask1));
    } else {
        crickit.analogWrite(CRICKIT_DRIVE1, CRICKIT_DUTY_CYCLE_OFF);
    }
    if (!(antennaPos2 & antennaLedMask2)) {
        crickit.analogWrite(CRICKIT_DRIVE2, (antennaPos2 | antennaLedPWMMask2));
    } else {
        crickit.analogWrite(CRICKIT_DRIVE2, CRICKIT_DUTY_CYCLE_OFF);
    }
    loudness = CircuitPlayground.mic.soundPressureLevel(10);
    if (loudness > 75) {
        listenModeTime = 50;
    }

    if (randomSoundEnabled && (random8() == 42)) {
        File file = fatfs.open(soundfiles[random8(NUM_SOUNDFILES)]);
        audioPlayer.play(22050 + (accelX * 200), file);
    }
}
