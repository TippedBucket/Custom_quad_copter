# CUSTOM QUADCOPTER
Built a quadcopter from scratch with a custom flight controller, PCB, and 3D printed frame. The following is a quick summary of what I did and how I did it. 
# IMAGES
<img width="1135" height="653" alt="drone_cad" src="https://github.com/user-attachments/assets/305603f1-a768-4526-bec5-df28c89e8a84" />
<img width="1135" height="372" alt="image" src="https://github.com/user-attachments/assets/aa0f86ea-0ae5-442f-a9d8-7fcc8bb83ea0" />


![drone_v1_finished](https://github.com/user-attachments/assets/289871b1-89b2-41e8-b077-453acba0df4d)
![Drone_v1_half](https://github.com/user-attachments/assets/07bd4d18-22e3-4b6d-bea0-040b096e6fef)
![PCB](https://github.com/user-attachments/assets/d68ba087-2db6-45b8-b574-2a88b957198e)



# PARTS LIST
*Note, a lot of these parts were bought in bulk because I didn't have them. You probably have some of these parts lying around
- **FrSky D8R-II Plus**: Old receiver, found it on Facebook Marketplace
- **JRXP303 Radio Controller**: Old controller, found it on Facebook Marketplace
- **FrSky DJT RF Transmitter**: Old transmitter, connects with the JRXP303 controller and sends the RF PWM signal, flashed with updated firmware to run in PWM 
- **Resistor Kit**: For resisting things, of course, lots of use on the drone 
- **Led Kit**: Indicators for drone startup
- **Bread Board**: For prototyping
- **XT30 Plug to JST  Connector**: For battery to PCB connection
- **2300KV Brushless Motor CCW CW**: To move the drone around
- **Teensy 4.0**: Microcontroller, brain of the drone
- **Gyroscope/Accelerometer**: MPU-6050, so I can see how the drone is moving
- **Barometer**: To measure the height of the drone
- **Electronic Speed Controllers**: To move the motors at a variable throttle
- **5" Props** To create upward thrust for movement
- **3S Battery** To power the drone!
- **LiPO Battery Charger**: So we can fly the drone more than once
- **Custom PCB**: See repository files for the CAD
- **GPS and RF receiver/transmitter**: For return to home functionality and transmitting onboard tellmetry
- **Zenre Diode**: This is to protect the Teensy from positive overvoltage and make sure no more than 2.4V are sent to the Teensy during flight
- **Diode**: This is to protect the Teensy for if the battery is connected backwards 
  
# COMPLETED STEPS
*Note, this project is incredibly complex and will have individual nuances based on the parts you buy and how low-level you go. For example, I wanted to try and create my flight controller from scratch because I wanted to understand how all these electrical and physical systems interact with one another to make my drone fly, rather than just build the drone. 

# FLASHING RECEIVER FIRMWARE

This was the longest and most annoying part of the drone build. In an effort to make this project cheap, I bought a second-hand flight controller and set of receivers for 20 bucks, which was a really good deal. Unfortunately, these receivers were so old that they only had firmware supporting Pulse Width Modulation (PWM) instead of Pulse Position Modulation (PPM). I wanted to complete this project using PPM so I wouldn’t need a bunch of unnecessary wires on my drone, and because it’s easier to work with in my opinion.

See this video of my initial motor setup. At this point, I was naïve and thought the receiver was already able to run in PPM mode — this was before I realized it wasn’t.

Luckily for me, FrSky (the company that made these receivers) had released a firmware flash that allows the receivers to operate in PPM instead of PWM, way back in 2010 or so. However, they want you to buy a custom cable that plugs into your PC and receiver to flash the firmware. Because I’m cheap and creative, I decided, "I can do this myself!" So I watched the following videos and scrolled through old internet forums for ages to figure out how to make this work:

- [Flashing FrSky D8R-II Plus for CPPM](https://www.rcgroups.com/forums/showthread.php?2112954-Flashing-Frsky-D8R-II-Plus-for-CPPM-(27ms))  
- [This Video from 12 years ago](https://www.youtube.com/watch?v=NQQJ29dAt5E)  
- [This forum explaining FTDI chip workaround](https://diydrones.com/profiles/blogs/frsky-s-cppm-at-27msec-firmware-update-with-ft-prog-and-ftdi-cabl?id=705844%3ABlogPost%3A1001090)  
- [Link to Manual & Firmware](https://www.frsky-rc.com/d8r-ii-plus/)

---

## FRSky CPPM Firmware Update (27 ms Frame Length)

### Summary
FRSky released a **27 ms CPPM firmware** for D8R-XP and D4R-II receivers (which can also be used on the D8R-II Plus). This increases the frame length, making it compatible with all channels while still providing a usable update rate (~37 Hz), which is more than enough for most drone applications.

The firmware update requires a **USB-to-TTL serial adapter with inverted logic**. FRSky’s official cable has this built in, but you can modify a standard FTDI adapter using FTDI’s `FT_PROG` utility to invert TXD and RXD signals.

---

### Logic Behind the Fix

- **Problem** – The 18 ms CPPM frame couldn’t carry 8 channels + sync pulse without signal issues.  
- **Solution** – Increase the frame length to 27 ms via firmware update.  
- **Challenge** – Updating requires an inverted-TTL adapter; most generic FTDI cables are non-inverted.  
- **Workaround** – Use FTDI’s `FT_PROG` tool to configure a normal FTDI cable to invert TX and RX signals, making it function like FRSky’s official cable.

---

### Steps Followed for FTDI Fix

#### 1. Obtain the Firmware
- Download the official **27 ms CPPM firmware** for your receiver (D8R-XP or D4R-II) from FRSky’s site.  
- Inside the ZIP file, read the included PDF for official flashing instructions.

#### 2. Prepare the FTDI Adapter
- Download [`FT_PROG`](https://ftdichip.com/utilities/#FT_PROG) from FTDI Utilities.  
- Plug in your FTDI cable and open `FT_PROG`.  
- Go to `Devices → Scan and Parse` to detect the adapter.  
- In the **Device Tree**, select **Hardware Specific** and enable inversion for **TXD** and **RXD**.  
- Click the flash icon, then **Program** to save the changes.

#### 3. Connect to the Receiver
- Power the receiver using the FTDI cable’s **5 V** pin.  
- **Cross-connect**: FTDI **TX → RX** on receiver, FTDI **RX → TX** on receiver.  
- Connect **GND → GND**.

#### 4. Flash the Firmware
- Use the FRSky firmware update tool (or equivalent) to load the new firmware to the receiver.

#### 5. (Optional) Revert the FTDI
- Run `FT_PROG` again and uncheck the inversion boxes to restore the adapter to normal operation.

---

I followed these steps but quickly learned that the chip I had was a knock-off and could only read — not write — firmware via FTDI. So I was back to square one. While trying that option, I discovered another method using an old RS232 serial cable from the days of older PCs (1960s–1990s). The problem was that finding a computer with an RS232 port now is like finding a needle in a haystack. So I went online and bought a USB-to-RS232 cable from Amazon and followed the steps below:

---

## Flashing the FrSky D8R-II Plus Receiver for 27 ms CPPM

### Steps

1. **Enter Programming Mode**  
   - Place a jumper between **signal pins 7 and 8** on the receiver.  
   - You can use a female-to-female servo lead with the ground pin removed.  
   <img width="200" height="181" alt="image" src="https://github.com/user-attachments/assets/06a06507-3b7b-4fa3-bf8b-5720eb245861" />

2. **Prepare the Firmware**  
   - Download the `d8rxp_cppm27_build120926` firmware (D8R-XP firmware works on D8R-II Plus).  
   - Install the **FRSky Update Tool** on your PC.

3. **Connect to the Receiver**  
   - Connect your PC’s serial adapter to the receiver’s data port.  
   - Power the receiver using a BEC or battery (e.g., 4-cell NiCad) through a normal servo connector.  
   <img width="200" height="169" alt="image" src="https://github.com/user-attachments/assets/e9c959a1-a10f-4b10-bb07-9386024e372e" />

4. **Flash the Firmware**  
   - Open the FRSky Update Tool.  
   - Select the correct **COM Port**.  
     <img width="200" height="141" alt="image" src="https://github.com/user-attachments/assets/68611623-b40f-4ea3-a4f9-d3108b241a4e" />  
   - Click **File** and select the `.frk` firmware file.  
   - Apply power to the receiver.  
     <img width="200" height="150" alt="image" src="https://github.com/user-attachments/assets/1726774d-ba63-4f07-83f8-f0948bd461d3" />  
   - Wait for numbers to appear in the tool’s status bar (connection confirmed).  
   - Click **Download** to start flashing.

5. **Finish**  
   - Once complete, remove the jumper from pins 7 and 8.  
   - Your D8R-II Plus is now running the **27 ms CPPM firmware**.

---

When my USB-to-RS232 cable arrived, I booted up the firmware updating tool from FrSky and tried to flash the firmware. Unfortunately, the update would hang at 2% and not move after half an hour.  

See this [video of the failed flashing](https://drive.google.com/file/d/1AMVjHAr2IZpz9En8QaMZe7ojqr5RKQkH/view?usp=sharing).

I tried running the update on a Windows XP virtual machine because I read online it could be a driver mismatch issue on modern systems. That still didn’t work. I then suspected the USB-to-RS232 cable was another cheap knock-off. I borrowed a cable from a friend and — on the first try — it worked perfectly. Turns out, yet again, a knock-off cable was the problem.

See this [video showing the updated setup](https://drive.google.com/file/d/1rLf_xIebGQjxXqM2hLLjGMtLDWKhd2ks/view?usp=sharing).

With this complete, I was able to move on to creating my flight controller and starting to assemble parts of the drone.




# BATTERY MANAGEMENT 
There are a few main considerations here:
1. We dont want to fry our flight controller, the Tessny's max allowable voltage is 3.3V. Which is way too low when we are dealing with an 11.1V 3S Battery.
2. We want to protect our battery from a short circuit, no flying flaming drones please.
3. We want to accurately measure our current and voltage to estimate the battery life remaining.
To solve this problem, we are going to use a fancy [transistor](https://www.digikey.ca/en/products/detail/infineon-technologies/BTS500801TMBAKSA1/2080779) for power management, as well as a [Zener Diode](https://www.digikey.ca/en/products/detail/onsemi/BZX79C2V4-T50A/977904) from that nasty high voltage.
Here is a summary of what the transistor does:
- High-side power switch for 12 V systems — replaces relays/fuses.
- Load current sensing – outputs a proportional sense current for monitoring.
- High inrush capability – ISO load current up to 37.5 A, peak short-circuit limit ~90 A.
- Embedded protection:
  - Short circuit & overload protection with latch-off.
  - Multi-step current limiting.
  - Thermal shutdown & auto-restart.
  - Over-voltage & loss-of-ground protection.
Here is a summary of what each pin does:
- **Pins 1,2,6,7(OUT)**: Power output to load. All must be shorted together externally for proper current capacity, clamping, and sensing accuracy.
- **Pin 3(IN)**: Control input. Pulling to ground activates the power switch. Open = switch off.
- **Pin 4(VBB)**: Positive supply voltage from battery (tab is internally tied to this pin).
- **Pin 5(IS)**: Current sense output. Provides a small current proportional to load current; also outputs a fixed “fault” current in error conditions.

## ACCURATE VOLTAGE & CURRENT MEASUREMENT
- When the motors are at full throttle to accelerate the drone upward, the voltage measured on pin 5 will drop significantly due to the high current draw, even though the battery’s actual capacity isn’t that low. Because the motor current draw varies depending on whether the drone is accelerating or simply hovering, we cannot rely solely on a simple voltage measurement (or apply 𝑉=𝐼𝑅) to determine the current, and by extension, the remaining battery capacity.
- To get around this, we need to be able to measure the variable current across the power switch using pin 5.
- According to the [BTS50080-1TMB manual](https://www.infineon.com/assets/row/public/documents/10/49/infineon-bts50080-1tmb-ds-en.pdf?fileId=5546d4625a888733015aa435ca67114f), the ratio between IL(Current Load) and IS(Current Sense) fluctuates between 11000ma to about 14000ma, depending on the load. Because of this, I want to use the typical amount here, which looks to be about 13000
Reference image for context: 
<img width="484" height="321" alt="image" src="https://github.com/user-attachments/assets/66bb8a6c-3adc-4403-a923-33b5b657e782" />

IS= the sense current & IL= the load current
- In layman's terms, that means an amperage of 13A on the IL would be read as 1mA on the IS side, which is much safer to work with
- Since the Teensy cant measure current and only voltage, a resistor is needed to convver the IS current into a measureable voltge
- IS*R=VS
- And IS=IL/KS
- The resistor in this build is 510 Ω, so a 13 A load current (IL) would produce a 1 mA sense current (IS), which in turn creates a 0.51 V output across the resistor. Put simply, this means 13 A corresponds to 0.51 V, giving a conversion ratio of approximately 0.039 V per amp.
- This ratio allows us to interpt remaining battery life while we fly.

## SHORTING PROTECTION
- In order to protect the Teensy microcontroller, we need to use a [Zener Diode](https://www.digikey.ca/en/products/detail/onsemi/BZX79C2V4-T50A/977904). During a short circuit where the current exceeds 85A, (0.039[V/A] * 85 [A]= 3.315[V]) the voltage to the Teensy would exceed the maximum limit and fry our expensive piece of equipment. The Zener diode will shunt excess current to the ground and clamp the voltage so it never rises above 2.4V for this specific one.
- However, we can are still only protected on a overvoltage on the positive side, meaning that if the battery is connected backware, the Zener wont help. To protect for the negative direction a [normal diode](https://www.digikey.ca/en/products/detail/onsemi/1N4007G/1485479) is also in place to block nagative voltages.
  
## BATTERY PROTECTION
- The [BTS50080-1TMB Power Swtich](https://www.infineon.com/assets/row/public/documents/10/49/infineon-bts50080-1tmb-ds-en.pdf?fileId=5546d4625a888733015aa435ca67114f) protects the battery with the following process: 
1. **Current Monitoring Inside the Switch**
   - The switch continuously monitors the drop voltage across its MOSFET
   - Once the drop voltage rises above 3.5V, the device recognizes a fault

2. **Current Limitng**
   - As soon as the short occurs, the device limits the load current to a defined value up to 90 A, depending on the supply voltage, which prevents an uncontrolled current surge that could hurt the battery
3. **Turn Off Fast**
   - If the overcurrent persists (350us-650us) the switch turns off completely and latches the off state, isolating the battery
4. **Thermal Protection as Backup**
   - If the short circuit detection doesn't trigger first, rising internal temps over 175 will trigger a shutdown and turn the switch off.

## BATTERY LIFE
- I used this table to determine the approximate time before the drone's LED would turn red. Theoreticall, I could cycle the battery with an arduino script I wrote for [UBC-Chem-E-Car](https://www.ubcchemecar.com/) to determine the battery voltage capacity relationship but this graph seemed to work for the meantime.<br>
 <p align="center"> <img width="271" height="474" alt="image" src="https://github.com/user-attachments/assets/9064c5e8-2403-4f0e-bd18-a2af95fb0e83" />

# GYROSCOPE DATA 
- Still adding to repository, its a bit of a pain to explain it in a concise matter, it invlov
# FLIGHT CONTROLLER
- See the custom flight controller code. See my calculations [here](). I made an effort to comment on a large part of it to explain what each portion does in the file, but here is a summarized version and model for example:

<img width="1979" height="1580" alt="image" src="https://github.com/user-attachments/assets/2c25126b-cc2d-4fda-8b50-7b8d89ad2dce" />


## Flight Controller Code Overview

### 1. Libraries and Constants
- **Wire.h** – For I²C communication with the MPU6050 IMU.
- **PulsePosition.h** – For decoding PPM signals from the RC receiver.
- Constants for gyro sensitivity, loop time (`Ts`), and hardware addresses.

---

### 2. Global Variables
- **Gyro & Accelerometer Data** – Raw rates, calibration offsets, angles, and accelerations.
- **Receiver Values** – Up to 8 PPM channels stored in `ReceiverValue[]`.
- **Battery & Power Tracking** – Voltage, current, consumed mAh, starting capacity.
- **PID Variables** – Separate P, I, D values for roll, pitch, yaw, and angle control loops.
- **Motor Outputs** – Individual control values for each ESC.
- **Kalman Filter Variables** – For smoothing and combining gyro + accelerometer data to estimate angles.

---

### 3. Helper Functions

#### 3.1 Kalman Filter
- `kalman_1d()` – Estimates roll and pitch angles by combining gyro rates and accelerometer measurements, reducing noise.

#### 3.2 Battery Monitoring
- `battery_voltage()` – Reads ADC pins for battery voltage and current (current from high-side switch sense pin).

#### 3.3 Receiver Input
- `read_receiver()` – Reads PPM signal from the RC receiver and stores it in the `ReceiverValue[]` array.

#### 3.4 IMU Reading
- `gyro_signals()` –
  - Configures MPU6050 low-pass filter, accelerometer range, and gyro sensitivity.
  - Reads raw accelerometer and gyro data via I²C.
  - Converts to physical units (deg/sec, g’s) and calculates pitch/roll angles from accelerometer.

#### 3.5 PID Control
- `pid_equation()` –
  - Calculates P, I, and D terms for a given error.
  - Applies anti-windup limits for I-term and clamps overall output.
  - Returns output, error, and I-term for reuse in the next loop iteration.
- `reset_pid()` – Resets PID state when motors are off to prevent stored error from causing jumps.

#### 3.6 Debug Output
- `printReceiverValues()` – Serial prints motor output values for tuning/debugging.

---

### 4. Setup
- Sets pin modes for LEDs, motor outputs, and safety signals.
- Initializes I²C at 400kHz.
- Wakes MPU6050 from sleep mode.
- Calibrates gyro by averaging 2000 samples.
- Sets motor PWM frequency (250Hz) and resolution (12-bit).
- Reads starting battery voltage and estimates starting capacity.
- Initializes PPM receiver input.
- Starts the loop timer for precise control loop timing.

---

### 5. Main Control Loop (250Hz, every 4ms)

#### 5.1 Sensor Reading & Calibration
- Reads gyro and accelerometer data.
- Applies calibration offsets.
- Runs Kalman filter to get smoothed roll & pitch angles.

#### 5.2 RC Input Processing
- Reads throttle, pitch, roll, yaw from receiver.
- Converts joystick positions to desired angles or rates.

#### 5.3 Angle PID Loop
- Outer loop: Compares desired angles to actual angles (from Kalman filter) to compute desired angular rates for roll & pitch.

#### 5.4 Rate PID Loop
- Inner loop: Compares desired angular rates to measured gyro rates to compute control inputs for motors.

#### 5.5 Motor Mixing
- Combines throttle + roll + pitch + yaw corrections to calculate outputs for each motor.
- Clamps motor outputs to avoid exceeding max throttle or going below minimum sustain power.
- Cuts motors entirely if throttle channel is below cutoff threshold.

#### 5.6 Output to ESCs
- Sends PWM signals to each motor via `analogWrite()`.

#### 5.7 Battery Monitoring & Safety
- Updates consumed mAh based on measured current and loop time.
- Calculates remaining percentage.
- Turns on warning LED if battery is below 30%.

#### 5.8 Loop Timing Control
- Waits until exactly 4ms has passed since loop start to maintain a steady 250Hz control loop rate.

# DRONE BUILDING

I decided to go for a two-layer design, with the top deck housing the custom PCB, and the bottom deck housing the battery, motors, ESC's and receiver

-intially I had a design where I wanted to fit the 

# LEARNINGS 

Key Takeaways From This Build:

One of the biggest lessons I learned is that sensors are never really 100% accurate. The raw MPU6050 data is messy, and I discovered how important filtering is, especially using a Kalman filter to merge accelerometer and gyro readings into something stable enough for flight. I also realized that tuning PIDs is more of an art form than a science. You can’t just drop in random numbers; finding the balance between responsiveness and stability takes patience, and it’s surprisingly easy to make the drone behave unpredictably. And flip it and break a month's worth of progress :(.

Running the control loop at 250Hz showed me how much needs to happen in just 4 milliseconds. Reading sensors, processing inputs, running two layers of PID control, and still hitting timing precisely. I also learned that battery voltage alone doesn’t tell the full story. Voltage drops under high throttle, doesn't  mean the battery is dying, so current sensing and smarter power monitoring became essential to determine whether my drone was going to fall out of they sky.

Safety features turned out to be just as important as flight performance. Cutoff thresholds, LED warnings, and protective diodes aren’t just nice extra, they can prevent damage to ESCs, motors, or the flight controller. When I first started this project I had safety in mind from the beginning. I've worked with experimental batteries before and seeing those things explode in a controlled environment gave me the necessary fear to approach this in a low-risk way, and research and design my build to mitigate for risk. 

Finally, I learned that in a drone, everything affects everything. From the hardware that makes up your drone to small changes in receiver scaling or motor mixing, they ripple through the entire system, so learning to make adjustments in small, careful steps saved me from additional major headaches. And ALWAYS MAKE A BACKUP OF YOUR CODE. I may have lost a chunk of my flight controller do to a saving issue at one point. 

  
# NEXT STEPS
- I'm currently working on implementing the GPS and Return to Home functionality. I took a small break on the project to pursue a couple of other ideas, such as my Pokémon card detector project and my video game idea.
- UI for GPS navigation, live telemetry, and return to home functionality from the following script:
- <img width="1176" height="795" alt="GUI_drone" src="https://github.com/user-attachments/assets/1fcf5431-bb27-4794-847b-5e4d67fc4116" />

-I'm also working on adding Lidar object avoidance and maybe some object detection with my Raspberry Pi & SONYIMX500 Ai Camera that is repurposed from my [Pokemon Card Detector](https://github.com/TippedBucket/Pokemon_card_detector-)


# RESOURCES AND ACCREDITATION 
I did a lot of learning in this project, and this would not have been possible without the following resources and playlists on building drones!
- [CarbonAeruonautics](https://github.com/CarbonAeronautics/Manual-Quadcopter-Drone/tree/main) Electronics, wiring, Flight Controls
- [Imetomi](https://www.instructables.com/Ultimate-Intelligent-Fully-Automatic-Drone-Robot-w/) Idea inspiration
- [Joop Brokking](https://www.youtube.com/@Joop_Brokking) Electronics & GPS
- [Steve Brunton](https://www.youtube.com/watch?v=s_9InuQAx-g) Kalman Filter & PID Controls
