#include "mbed.h"
#include "arm_book_lib.h"

#define NUMBER_OF_AVG_SAMPLES                   100
#define OVER_TEMP_LEVEL                         30
#define TIME_INCREMENT_MS                       10
#define SERIAL_UPDATE_MS                        500
#define MQ2_THRESHOLD                           0.5  

DigitalOut alarmLed(LED1);
DigitalInOut sirenPin(PE_10);
UnbufferedSerial uartUsb(USBTX, USBRX, 115200);
AnalogIn lm35(A1);
AnalogIn potentiometer(A0);
AnalogIn mq2(A3);  

bool alarmState = OFF;
bool gasDetectorState = OFF;
bool overTempDetectorState = OFF;
float lm35ReadingsAverage = 0.0;
float lm35ReadingsSum = 0.0;
float lm35ReadingsArray[NUMBER_OF_AVG_SAMPLES];
float lm35TempC = 0.0;
float potentiometerReading = 0.0;
int accumulatedTimeAlarm = 0;

void inputsInit();
void outputsInit();
void updateSensorReadings();
void displaySensorData();
float analogReadingScaledWithTheLM35Formula(float analogReading);
void lm35ReadingsArrayInit();

int main()
{
    inputsInit();
    outputsInit();
    while (true) {
        updateSensorReadings();
        displaySensorData();
        delay(TIME_INCREMENT_MS);
    }
}

void inputsInit()
{
    lm35ReadingsArrayInit();
    sirenPin.mode(OpenDrain);
    sirenPin.input();
}

void outputsInit()
{
    alarmLed = OFF;
}

void updateSensorReadings()
{
    static int lm35SampleIndex = 0;
    int i = 0;

    lm35ReadingsArray[lm35SampleIndex] = lm35.read();
    lm35SampleIndex++;
    if (lm35SampleIndex >= NUMBER_OF_AVG_SAMPLES) {
        lm35SampleIndex = 0;
    }

    lm35ReadingsSum = 0.0;
    for (i = 0; i < NUMBER_OF_AVG_SAMPLES; i++) {
        lm35ReadingsSum += lm35ReadingsArray[i];
    }
    lm35ReadingsAverage = lm35ReadingsSum / NUMBER_OF_AVG_SAMPLES;
    lm35TempC = analogReadingScaledWithTheLM35Formula(lm35ReadingsAverage);

    potentiometerReading = potentiometer.read() * 100.0;

    if (lm35TempC > OVER_TEMP_LEVEL) {
        overTempDetectorState = ON;
        alarmState = ON;
    } else {
        overTempDetectorState = OFF;
    }

    if (mq2.read() > MQ2_THRESHOLD) {
        gasDetectorState = ON;
        alarmState = ON;
    } else {
        gasDetectorState = OFF;
    }

   
    if (!overTempDetectorState && !gasDetectorState) {
        alarmState = OFF;
    }

    if (alarmState) {
        accumulatedTimeAlarm += TIME_INCREMENT_MS;
        sirenPin.output();
        sirenPin = LOW;
        if (accumulatedTimeAlarm >= 500) {
            accumulatedTimeAlarm = 0;
            alarmLed = !alarmLed;
        }
    } else {
        alarmLed = OFF;
        sirenPin.input();
    }
}

void displaySensorData()
{
    static int serialUpdateTime = 0;
    char str[200];
    int stringLength;

    serialUpdateTime += TIME_INCREMENT_MS;
    if (serialUpdateTime >= SERIAL_UPDATE_MS) {
        serialUpdateTime = 0;

        sprintf(str, "Temperature: %.2f C | Potentiometer: %.2f%% | Gas: %s | Alarm: %s\r\n",
                lm35TempC,
                potentiometerReading,
                gasDetectorState ? "Detected" : "Not Detected",
                alarmState ? (overTempDetectorState && gasDetectorState ? "Temperature & Gas Alarm" :
                              overTempDetectorState ? "Temperature Alarm" : "Gas Alarm") : "No Alarm");
        stringLength = strlen(str);
        uartUsb.write(str, stringLength);
    }
}

float analogReadingScaledWithTheLM35Formula(float analogReading)
{
    return (analogReading * 3.3 / 0.01);
}

void lm35ReadingsArrayInit()
{
    int i;
    for (i = 0; i < NUMBER_OF_AVG_SAMPLES; i++) {
        lm35ReadingsArray[i] = 0;
    }
}