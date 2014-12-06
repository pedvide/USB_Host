/* USB to Serial - Teensy becomes a USB to Serial converter
   http://dorkbotpdx.org/blog/paul/teensy_as_benito_at_57600_baud

   You must select Serial from the "Tools > USB Type" menu

   This example code is in the public domain.
*/


unsigned long baud = 115200;
const int led_pin = 13;  // 13 = Teensy 3.X, 11 = Teensy 2.0, 6 = Teensy++ 2.0

void setup()
{
	pinMode(led_pin, OUTPUT);
	digitalWrite(led_pin, LOW);
	//Serial.begin(baud);	// USB, communication to PC or Mac
	Serial1.begin(baud);	// communication to hardware serial
}

unsigned char d;

void loop()
{
    unsigned char d;

    if (Serial1.available() > 0) {

        d = Serial1.read();
        if(d=='s') { // status
          Serial1.println("alive");
        }

    }

    digitalWrite(led_pin, !digitalRead(led_pin));

}

