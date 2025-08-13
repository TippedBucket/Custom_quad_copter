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
  
# COMPLETED STEPS
*Note, this project is incredibly complex and will have individual nuances based on the parts you buy and how low-level you go. For example, I wanted to try and create my flight controller from scratch because I wanted to understand how all these electrical and physical systems interact with one another to make my drone fly, rather then just build the drone. 

# NEXT STEPS
- I'm currently working on implementing the GPS and Return to Home functionality. I took a small break on the project to pursue a couple of other ideas, such as my Pokémon card detector project and my video game idea. 
