// This #include statement was automatically added by the Spark IDE.
#include "rht03-humidity-temperature-sensor.h"

double humidity = 0;
double temperature = 0;

int sensorPin = D0;
RHT03HumidityTemperatureSensor sensor(sensorPin);

void setup() {
    Serial.begin(9600);
    Spark.variable("humidity", &humidity, DOUBLE);
    Spark.variable("temperature", &temperature, DOUBLE);
}

void loop() {
    delay(2000);
    sensor.update();
    temperature = sensor.getTemperature();
    humidity = sensor.getHumidity();
    Serial.println("t/rh");
    Serial.println(temperature);
    Serial.println(humidity);
}


