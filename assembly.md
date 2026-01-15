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

##  **Step 6: Prepare and connect the rotary encoder wiring harness.**
These next two parts Are probably the trickier bits since there are a bunch of different ways to do it. I have a crimper and connectors, so I make my own wires. But this can be done with [10 cm female-to-female jumper wires](https://www.amazon.com/dp/B07S2RH6Q4/).
 1)  Connect the roatry encoder to the VasoMoto controller using 5 jumper wires. This is the easy part. Each pin's identity is printed on the front, so the image below tells you what each one is. 

![rotary encoder pinout](https://github.com/user-attachments/assets/7156f319-c305-449b-9a43-47c68035c3f2)
![PCB pinout for encoder](https://github.com/user-attachments/assets/5df0d4fc-f7fe-453c-ad67-da513ea72fb5)

2)  Once connected, it should look like the image below. As you can see I have 5-pin DIN headers and shorter wires, but there is plenty of space in the VasoMoto enclosure to cram the excess 10 cm retail jumper wires.

![connected encoder](https://github.com/user-attachments/assets/7af44a61-46a0-489f-a068-c5ddfcb06e9d)

##  **Step 7: Prepare and connect the LCD wiring harness.
Now for the trickier part. If you can, get some [8-pin female dupont connector housings](https://www.amazon.com/2-54mm-Dupont-Connector-Housing-Female/dp/B0BG7CZ2F9/). This makes connecting and disconnecting a LOT easier....but you can also use the connectors on the LCD harness if you want to save a few bucks. The reaason the order is different on the board compared to the module is...complicated. But it has to be this way.

As you can see, there are 8 wires, labaled and color coded as below:

![LCD wires with labels](https://github.com/user-attachments/assets/d98edd03-ca00-4e63-85b9-dbadb5d9df7b)

The problem is the order they go onto the VasoMoto controller is **DIFFERENT THAN THE ORDER THEY ARE ON THE LCD!**

![board pinout LCD](https://github.com/user-attachments/assets/6eb3e8a6-230c-4d58-9b2c-79292fdd2d61)

So...we either need to make a new 8-pin connector or connect each wire individually to the VasoMoto controller.

**Option 1: Making a new Connector**
1)  Using a small screwdriver, pop the plastic tab on the Dupont connectors on each individual wire. This image, from left to right, shows the intact connector, a connector with the tab bent up, and a wire with the connector removed. They should just slide right off without much force.![removing dupont connector](https://github.com/user-attachments/assets/a43634b8-7fb6-441c-bc8e-c90eec41705a)

2)  Clip all 8 wires into the new 8-pin dupont connector, making sure they are in the proper order. It's the name on the wire that's important, not the color...so if your colors are different than mine, adjust to make the pinout correct.

![8-pin connector](https://github.com/user-attachments/assets/fff58553-9907-4873-84b8-c836c758bff1)

**Option 2: Connect each wire to the board individually.**
This is self-explanatory, but I'll tell you why it's harder. If you need to remove the display, you have to disconnect all 8 wires...and then you have to remember the order that they go in, which is not the same as the order on the LCD module. I make these all the time, and I can't remember what goes to what (other than the white wire is ground). So...spend a few bucks and make the 8-pin connector -- or take lots of photos of the wiring order.

##  **Step 8: Mount the motor to the pump.
1)  Using 4, M3x4 screws, attach the NEMA17 stepper motor to the motor plate. There are 2 sets of motor plate holes, in case someone uses a NEMA 11 instead. Either way, it's stil the same size screws. Make sure the motor is oriented so that the wires point the direction shown below:

![motor wire direction](https://github.com/user-attachments/assets/3df7a598-5fe6-46cd-a120-883bd044cded)

2)  Using 4, M3x6 (or longer) screws, attach the pump to the motor plate. In the photo, the tubing spacer is already in place. THat can stay out for now. No need to connect wires -- we will do that later.

![mounted motor](https://github.com/user-attachments/assets/9b8d0c1c-7fb2-44f7-b250-0c3bfa63396d)

##  **Step 9: Assemble the rest of the VasoMoto controller box.
1) Start with the back, then the sides. Notice the groove on each side plate; it should face inward toward the back plate. The sides shopuld press tightly into the slots on the base plate.

![sides and back in place](https://github.com/user-attachments/assets/ec957305-f7d3-4a1c-9fba-5178b281fa00)

4) Connect the motor wires, in the order shown below, to the screw terminals on the VasoMoto controller. Sometimes the terminals are all screwed the way down, so make sure and loosen the screws first. If you wire colors are different, the proper assignments for each wire are printed on the PCB (e.g., A+, A-, B+, B-). Leave the motor plate off to the side for now.

![motor wiring](https://github.com/user-attachments/assets/10a6ec52-56ca-45fa-81c8-33fc761e5f32)

5)  Connect the LCD module and rotary encoder to the VasoMoto control board, but do not place the LCD on the controller box yet.

![LCD plate connected](https://github.com/user-attachments/assets/0a6fcdf2-450c-4a8a-b6b8-fd24123f80e0)

6)  Put the motor plate in place.

![motor in place](https://github.com/user-attachments/assets/796fb5de-f4a7-4fa8-83a0-810480388b89)

6)  Now you can put the LCD plate in place. Finish it off by cramming the wires inside and putting the front plate on it! Now all that's left is the pump head.

![completed box without pump head](https://github.com/user-attachments/assets/a77d428b-a151-4f3b-97aa-6f375c2dffb4)

##  **Step 10: Assemble and attach the pump head.
1) Gather all the parts. This includes the printed pump head, 3 bushings, 3 acetyl rods, and 3 3/16" set screws. Note: when printing the print head, it's best to print in the orientation shown below (using supports where needed). It is important that the pump head is as circular as possible for proper pump function.

![pump head parts](https://github.com/user-attachments/assets/a57b6c67-b28b-4804-a282-97ba13f3e584)

3) Screw the set screws in place around the pump head. NO need to insert fully -- just 1-2 turns is sufficient for now.

![set screws in place](https://github.com/user-attachments/assets/62aa8362-fe5d-4b28-9aca-805ca11de6e1)

4) Insert rods and bushings. These are just press-fit into place. Do NOT glue the rods in place! Make sure the bushings turn freely. If not, remove the rods, sand the inside surfaces of the print head arms, and re-assemble.

![completed pump head](https://github.com/user-attachments/assets/d8daf70e-c5b1-4ecb-84a1-0ebad54623c1)

5)  Place the pump head on the motor shaft. Align one set screw with the notch on the motor shaft and screw in place. Turn the pump head and repeat for the other two screws.

![tightening pump head](https://github.com/user-attachments/assets/056c3b1e-d8cd-484b-8a25-9625df46f151)

7)  Place the tubing spacer. You need not glue it in place, but you might find you want to after testing the pump later. Insert tubing through the tubing holders and around the pump head. You don't want it too short that it has to stretch over the pump head; nor do you want it too long that it bunches up on either side. You want it just taught enough to stay in place.

![tubing in place](https://github.com/user-attachments/assets/62c8a6df-c895-4893-9ace-f97f911d8fd3)

##  **Step 11: POWER ON!  You're done!

![completed unit](https://github.com/user-attachments/assets/75c4028d-f4e3-4a8f-9aa5-d83d1d270e2f)








