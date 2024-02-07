#include <Arduino.h>

float target = 0.0;

void serial_loop(void) {
        static String str;

        while (Serial.available()) {
                char ch = (char) Serial.read();
                str += ch;

                if (ch == '\n')
                {
                        target = str.toFloat();
                        Serial.print("Target: ");
                        Serial.println(target);
                        str = "";
                }

        }
}

void setup() {
        Serial.begin(115200);
}

void loop() {
        serial_loop();
}
