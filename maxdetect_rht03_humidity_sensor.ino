
// Code to read out the humidity and temperature values from a
// Maxdetect RHT03 humidity/temperature sensor.
// See https://www.sparkfun.com/products/10167 and http://www.humiditycn.com/fl1_1.html
// 
// Maintained at https://github.com/liyanage/maxdetect-rht03-humidity-sensor
//

#define PAYLOAD_BITS 40
#define PAYLOAD_BYTES (PAYLOAD_BITS/8)
#define PULSE_WIDTH_TRUE_THRESHOLD_MICROSECONDS 35
#define USE_SPARK_CORE 1

void sensorInterruptHandler();
    
int sensorPin = D0;

unsigned long listeningStartTime = 0;
volatile unsigned long lastInterruptTime = 0;
volatile int interruptCount = 0;
volatile unsigned char payload[PAYLOAD_BYTES];

double humidity = 0;
double temperature = 0;

enum {
    StateUnknown,
    StateIdle,
    StateListening
};
int state = StateIdle;

void setup() {
#if USE_SPARK_CORE
    Spark.variable("humidity", &humidity, DOUBLE);
    Spark.variable("temperature", &temperature, DOUBLE);
#endif
}

void loop() {
    if (state == StateIdle) {
        delay(1000);

        // Pull the pin low for 10ms to request a measurement from the sensor
        pinMode(sensorPin, OUTPUT);
        digitalWrite(sensorPin, HIGH);
        delay(10);
        digitalWrite(sensorPin, LOW);
        delay(10);
        digitalWrite(sensorPin, HIGH);

        // Configure the pin for input and set up the change interrupt handler
        pinMode(sensorPin, INPUT_PULLUP);
        listeningStartTime = micros();
        attachInterrupt(sensorPin, sensorInterruptHandler, CHANGE);

        state = StateListening;
    } else if (state == StateListening) {
        unsigned long listeningInterval = micros() - listeningStartTime;
        // We assume that the measurement is done after this interval
        if (listeningInterval > 4000000) {
            // Collect the data
            unsigned int rh = payload[0] << 8 | payload[1];
            unsigned int t = payload[2] << 8 | payload[3];
            humidity = (double)rh/10;
            temperature = (double)t/10;

            // Detach interrupt handler and reset all state for the next measurement
            detachInterrupt(sensorPin);
            lastInterruptTime = 0;
            listeningStartTime = 0;
            interruptCount = 0;
            for (int i = 0; i < PAYLOAD_BYTES; i++) {
                payload[i] = 0;
            }

            state = StateIdle;
        }
        delay(1000);
    }
}

void sensorInterruptHandler() {
    interruptCount++;
    
    unsigned long now = micros();
    unsigned long intervalSinceLastChange = 0;
    if (lastInterruptTime) {
        intervalSinceLastChange = now - lastInterruptTime;
    }
    lastInterruptTime = now;

    if (interruptCount < 4) {
        // The first three transitions are for the two leading 80us pulses, ignore those
        return;
    } else if (interruptCount % 2) {
        // Every even transition starting with the fourth is for the rising
        // edge that ends the 50us low period that comes before every bit.
        // We note the time of that even transition (see above) but ignore
        // it otherwise.
        // Every odd transition marks the end of a bit and the length
        // of the pulse will tell us if it was a 0 or 1 bit.
        int bitIndex = (interruptCount - 5) / 2;
        unsigned char value = intervalSinceLastChange > PULSE_WIDTH_TRUE_THRESHOLD_MICROSECONDS ? 1 : 0;
        // Store the bit into the appropriate bit position of the appropriate
        // payload byte.
        payload[bitIndex/8] |= (value << (7 - (bitIndex % 8)));
    }
}




