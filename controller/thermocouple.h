/**
 * Copyright 2021 Minos Park <oven_sim@minospark.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef _THERMOCOUPLE_H_
#define _THERMOCOUPLE_H_

class Thermocouple
{
    public:
        Thermocouple( int pin, bool ifCelsius, float tunerVal ) {
            this->pin = pin;
            this->ifCelsius = ifCelsius;
            this->tunerVal = tunerVal;
        };

        // Returns the manipulated variable given a setpoint and current process value
        float read() {
            // Code from Johnathan's previous project
            reading = analogRead(pin);

            // Convert the ADC Reading to Voltage:
            voltage = (reading / 1024.0) * 5.0;

            // the sensor changes 10 mV per degree
            // the datasheet says there's a 500 mV offset
            // ((voltage - 500 mV) times 100)
            // temperature in Celsius
            temperature = (voltage - tunerVal) * 100;

            if (!ifCelsius) {
                // Convert the Celsius reading to Fahrenheit
                temperature = (temperature * (9.0 / 5.0)) + 32.0;
            }

            return temperature;
        }
        ~Thermocouple() {};

    private:
        int pin;
        bool ifCelsius;
        float tunerVal, reading, voltage, temperature;
};

#endif
