# Assembling

All CAD files and components are mentioned in CAD files and Components directories

## Mechanical Connections
Step 1: Center the cart

Step 2: Stick the slab protector under the bench around proximity of the centered point

Step 3: Attach the clamp on the protector, with the help of M6 nuts and bolts

Step 4: Now, connect the clamp to base through two M6 screws, which in extension conncets the vertical board as well (You can add equal number of washers on each screw to control the distance between the vertical board and clamp)

Step 5: Take the pendulum rod, and attach the elctromagnet grove on it

Step 6: Now, attach the servo motor in the gap on the vertical board with the help of smaller M6 screws (The area is tight enough to work without the screws even)

Step 7: Now attach the rotation shaft of motor on the pendulum rod at nearly the 3rd hole from the top, such that it is in line with the original pole. Use super glue here after caliberating

Step 8: Stick magnetic nuts on the original pendulum rod, and we're good to go



## Electrical Connections
The initial stepper motor setup is untouched. We use an independent power supply to connect both the servo motor and electromagnet grove at ~ 6-7V so that the motor operates at its maximum efficiency. Keep in mind to not let the electromagnet grove get power for longer times as it starts to heat at such high voltage outside its working range. Next, is to connect the common ground to Arduino DUE and correct logic pins for motor and microcontroller as in the code.