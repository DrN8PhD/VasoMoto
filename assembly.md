# Assembly Instructions
## Prior to Beginning:
1) Acquire all parts listed in the "external-parts-list.md" file.
2) Print all required parts in the "VasoMoto Printed Parts" folder. Note that UK and US have different pump heads to print. This is due to differences in parts availability. The UK parts are prototyped, but not tested in practice yet.
3) Every rotary encoder I can find has bent pins that are on the wrong side. You will need to de-solder the 5 bent pins and solder 5 straight pins in their place. Make sure the ones you're replacing are on the **opposite side of the circuit board from the knob.** See below for a clearer picture.
4) This guide assumes you have fabricated or purchased the completed PCB that makes up the "heart" of the VasoMoto controller. A separate manual in the works for how to fabricate the VasoMoto controller from scratch with the "blank" PCB.

##  **Step 1: Lay out all the pieces.**
Everything is shown here, except the pump head and fasteners. They'll be pictured later. The pieces are as follows:
1)  Front plate
2)  Stepper motor NEMA 17 stepper motor
3)  Pump plate
4)  VasoMoto controller PCB
5)  LCD plate
6)  1.8" LCD module
7)  Back plate
8)  Arduino
9)  Rotary encoder and stepper motor driver
10) Motor plate
11) Left side plate
12) Right side plate
13) Base plate
![VasoMoto parts laid out](https://github.com/user-attachments/assets/6dc5eccd-4a30-4fa8-84e6-f93fc99d15f3)

##  **Step 2: Get the fasteners together.**
Fasteners needed are shown below. Where there is flexibility in sizing, I've made that note. We will be assembling efverything __but__ the pump head right now.

1)  1/4" No. 2 sheet metal screws (4). These need to be the 'pointy' kind of screw. They attach the LCD diplay to the LCD plate.
2)  M3 x 4 mm button cap screws (4). These are the 'flat' kind of screws. They attach the motor to the motor plate
3)  M3 x 6 mm button cap screws (4). THey can be longer than 6 mm but not shorter. They attach the pump to the motor plate.
4)  The screws off of the LCD display stand-offs (3). If you got an LCD without these screws, just use 3 more M3x6 screws instead.
![assembly screws](https://github.com/user-attachments/assets/3d4a4ea2-ad37-42a6-88b8-9394a7ada2f0)

##  **Step 3: Assemble VasoMoto controller.**
Insert the arduino in the bottom of the PCB in the appropriate headers. Insert the stepper motor driver in the proper headers on the top. Note that the proper "DIR" and "GND" pins on the header are in the spots marked "DIR" and "GND" on the PCB.
![Assembled PCB](https://github.com/user-attachments/assets/bbc7d8ce-b378-473d-8c02-ac49964f6235)

##  **Step 4: Attach the VasoMoto controller top the base plate.**
Use 3 screws. THe holes aren't threaded, so it might be easier to screw them in without the controller first to cut some threads in the plastic. You might have to pop the stepper driver back out, depending on what size screw and screwdriver you have.
![PCB affixed to baseplate](https://github.com/user-attachments/assets/1d42a998-5f51-4bbc-9d7b-708f9a0df298)

##  **Step 5: Assemble the LCD plate.**
1) If opresent, remove all 4 brass standoffs from the display. no need to connect any cables just yet. Not the direction of the connector -- the LCD only fits in the plate one way. ![LCD in LCD plate](https://github.com/user-attachments/assets/5c2fb6b1-f821-4745-a347-f299e328ed8a)
2) Using 4 No. 2 machine screws, attach the LCD module to the LCD plate. Do NOT overtighten, or the screws will poke through the front of the plate.
3) Insert the rotary econder with the pins placed as noted in the photo. Ignore that extra wire and button -- that's not needed for this build. Attach the rotary encoder using the nut and washer that comes with it.
![LCD and rotary encoder in place](https://github.com/user-attachments/assets/8acad9aa-ea49-4ad1-82d5-8d4b0573bece)

##  **Step 6: Prepare the LCD wiring harness and the rotary encoder wiring harness.**
This part is probably one of the trickier bits since there are a bunch of different ways to do it. I have a crimper and connectors, so I make my own. But this can be done with female-to-female jumpers and a little patience.
