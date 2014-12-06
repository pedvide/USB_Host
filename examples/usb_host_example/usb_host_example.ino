#include "USB_Host.h"

USB_Host usb_host;

const int led_pin = 13;
long led_on_time=0;


void setup() {
      pinMode(led_pin, OUTPUT);

      Serial1.begin(115200);
}

void loop() {
  unsigned char d;

  if (Serial1.available() > 0) {

    d = Serial1.read();
    if(d=='h') { // usb_host
      Serial1.println("usb_host_mode()");
      usb_host.usb_host_mode();
    } else if(d=='s') {
      Serial1.print("USB0_INTEN: ");
      Serial1.println(USB0_INTEN, HEX);
      Serial1.print("USB0_CTL: ");
      Serial1.println(USB0_CTL, HEX);
    } else if(d=='a') { //

    }
  }


  digitalWrite(led_pin, !digitalRead(led_pin));

}
