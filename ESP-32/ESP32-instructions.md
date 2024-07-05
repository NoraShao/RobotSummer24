Download the USB driver from [here](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads):   
I used this one: CP210x Windows Drivers  
Unzip and open the x64 .exe file to install it.  
If it installed correctly, going to device manager->ports should list something something silicon labs

I'm using VS Code + Platform IO. The board is ESP32-PICO-KIT-1, listed as ESP32 Pico Kit.
The first project is slow to build but should be <5 minutes.

When the ESP32 is plugged in, the LED on the board should turn on.

Uploading code to the ESP-32 is the same as the bluepill.

If the blink code is working right, an LED connected to the designated LED pin should blink every second, 
and the serial monitor will print text that corresponds to the LED's state.

**OTA**  
All code must include the setup code that defines a static IP address, begins the server and connects to wifi. If that's not included, the ESP32 will have to have its next update via wire :(

Also I don't have a tutorial for the newer ElegantOTA so we're using the old one.

To upload code, connect your laptop to the router's wifi and then type the IP address of the ESP32 in your browser.

Duct: 192.168.0.104/update
Scotch: 192.168.0.105/update

In VS Code, click the checkmark on the bottom (build). Next, navigate to projectname/.pio/build/pico32 and upload firmware.bin to the ESP32 using the upload button with firmware selected.