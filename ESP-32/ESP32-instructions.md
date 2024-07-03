Download the USB driver from here: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads  
I used this one: CP210x Windows Drivers  
Unzip and open the x64 .exe file to install it.  
If it installed correctly, going to device manager->ports should list something something silicon labs

I'm VS Code + Platform IO. The board is ESP32-PICO-KIT-1, listed as ESP32 Pico Kit.
The first project is slow to build but should be <5 minutes.

When the ESP32 is plugged in, the LED on the board should turn on.

Uploading code to the ESP-32 is the same as the bluepill.

If the blink code is working right, an LED connected to the designated LED pin should blink every second, 
and the serial monitor will print text that corresponds to the LED's state.
