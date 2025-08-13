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
- **Zenre Diode**: This is to protect the Teensy and make sure no more than 2.4V are sent to the Teensy during flight
- **Diode** 
  
# COMPLETED STEPS
*Note, this project is incredibly complex and will have individual nuances based on the parts you buy and how low-level you go. For example, I wanted to try and create my flight controller from scratch because I wanted to understand how all these electrical and physical systems interact with one another to make my drone fly, rather than just build the drone. 

# FLASHING RECEIVER FIRMWARE
This by far, was the longest and most annoying part of the drone. In an effort to this project cheaply, 

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
- In order to protect the Teensy micro controller, we need to use a [Zener Diode](https://www.digikey.ca/en/products/detail/onsemi/BZX79C2V4-T50A/977904). During a short circuit where the current exceeds 85A, (0.039[V/A] * 85 [A]= 3.315[V]) the voltage to the Teensy would exceed the maximum limit and fry our expensive piece of equipment. The Zener diode, will shunt excess current to the ground and clamp the voltage so it never rises above 2.4V for this specific one.
- However, we can are still only protected on a over voltage on the positive side, meaning that if the battery is connected backware, the Zener wont help. To protect for the negative direction a [normal diode](https://www.digikey.ca/en/products/detail/onsemi/1N4007G/1485479) is also in place to block nagative voltages.
  
## BATTERY PROTECTION
- The [BTS50080-1TMB Power Swtich](https://www.infineon.com/assets/row/public/documents/10/49/infineon-bts50080-1tmb-ds-en.pdf?fileId=5546d4625a888733015aa435ca67114f) protects the battery with the following process: 
1. **Current Monitoring Inside the Switch**
   - The switch contrinouslyt monitors the drop voltage across its MOSFET
   - Once the drop voltage rises above 3.5V, the device recognizes a fault

2. **Current Limitng**
   - As soon as the short occurs, the device limites the load current to a defined value up to 90 A depedning on the supply voltage, which prevents an uncontrolled current surge that could hurt that battery
3. **Turn Off Fast**
   - If the overcurrent persists (350us-650us) the switch turns off completly and latches the off state, isolating the battery
4. **Thermal Protection as Backup**
   - If the short circuit detection doesnt trigger first, risning internal temps over 175 will trigger a shutdown and turn the switch off.

## BATTERY LIFE
- I used this table to determine the approximate time before the drones LED would turn red. Theoretically I could cycle the battery with an arduino script I wrote for UBC-Chem-E-Car to determine the battery voltage capacity relationship but this graph seemed to work for the mean time.<br>
 <p align="center"> <img width="271" height="474" alt="image" src="https://github.com/user-attachments/assets/9064c5e8-2403-4f0e-bd18-a2af95fb0e83" />

# GYROSCOPE DATA 
- This part took some time too and involved reading hexidecimal 
# FLIGHT CONTROLLER

# LEARNINGS 


  
# NEXT STEPS
- I'm currently working on implementing the GPS and Return to Home functionality. I took a small break on the project to pursue a couple of other ideas, such as my Pokémon card detector project and my video game idea. 


# RESOURCES AND ACCREDITATION 
I did a lot of learning in this project, and this would not have been possible to create without the following resources and playlists on building drones!
- [CarbonAeruonautics](https://github.com/CarbonAeronautics/Manual-Quadcopter-Drone/tree/main) 
- [Imetomi](https://www.instructables.com/Ultimate-Intelligent-Fully-Automatic-Drone-Robot-w/) 
- [Joop Brokking](https://www.youtube.com/@Joop_Brokking)
