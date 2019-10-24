# INSTALLATION

 * Download Arduino IDE [here](https://www.arduino.cc/en/Main/Software).
 * Choose appropriate version (Windows, Mac, Linux) depending on your computer.
 * Install (some drivers may be installed as well).
 * Open Arduino IDE and create a new Sketch (File-> New).
 * Save the sketch as **tsipouro**.
 * Close Arduino IDE.

# CODE
 * Unzip **tsipouro-distillation-arduino.zip**.
 * Inside there is a tsipouro folder containing a **tsipouro.ino** file.
 * Find where your Arduino folder is (most likely in Documents).
 * Inside it there should be a **tsipouro** folder containing the previously save **tsipouro.ino** file.
 * Replace the tsipouro.ino file with the downloaded tsipouro.ino file.
 * Open again Arduino IDE and open tsipouro sketch (File->Open recent->tsipouro)
 * Be careful not to change anything in the text of the sketch.


# LIBRARIES
 * Inside the unzipped **tsipouro-distillation-arduino** there are 2 other zip files.
 * In Arduino IDE on navigation bar go to Sketch->Include Library->Add ZIP library and choose **Timer.zip** to be included.
 * Repeat the same process with **LiquidCrystal_I2C.zip**.

# DOWNLOAD TO HARDWARE
 * Now, connect the Arduino Uno via USB cable to your computer.
 * Some drivers will be installed if they haven't been installed with the Arduino IDE installation.
 * On Arduino IDE go to Tools->Board and choose Arduino/Genuino Uno.
 * Then go to Tools->Port and choose the only one showing, if more than one then go to device manager (Windows) and check what is right COM port by unplugging/replugging the USB cable.
 * Now you can compile the Code. Press the **verify button** (the one on the left with the tick). If everything goes well, on the bottom it will show *Done compiling.*.
 * Afterwards, press the **upload button** which on the right of the verify button. Once succesfully finished, the program will start running on your Arduino.

