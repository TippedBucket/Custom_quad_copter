# Custom_quad_copter
Built a quadcopter from scratch with a custom flight controller, PCB, and 3D printed frame.
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

## PUTTING IT TOGETHER 

# NEXT STEPS
- I'm currently working on implementing the GPS and Return to Home functionality. I took a small break on the project to pursue a couple of other ideas, such as my Pokémon card detector project and my video game idea. 


# RESOURCES AND ACCREDITATION 
I did a lot of learning in this project, and this would not have been possible to create without the following resources and playlists on building drones!
- [CarbonAeruonautics](https://github.com/CarbonAeronautics/Manual-Quadcopter-Drone/tree/main) 
- [Imetomi](https://www.instructables.com/Ultimate-Intelligent-Fully-Automatic-Drone-Robot-w/) 
- [Joop Brokking](https://www.youtube.com/@Joop_Brokking)
