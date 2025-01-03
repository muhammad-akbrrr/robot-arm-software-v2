# Preliminary steps before running the app

Before you activate your virtual environment, you need to create a local virtual environment by running "python -m venv /path/to/new/virtual/environment" in the terminal

## On Windows use `venv\Scripts\activate` to activate the virtual enviroment

then run "pip install -r requirements.txt" to install the package dependencies locally

## Converting python file into an .exe file

- To convert main.py python file to an EXE file, open the terminal and type "pip install auto-py-to-exe".
- Next, execute the py-to-exe program by typing "python -mauto_py_to_exe" in the terminal.
- Use this program to create the exe files. Copy all .ico, .gif, and program files from the source code folder into your new exe folder otherwise, exe will not work.

![output root](https://github.com/user-attachments/assets/5671c716-b297-4a10-95ab-accb9bbebc9b)

![output internal](https://github.com/user-attachments/assets/19df871e-3530-499c-8629-edb35d00fcc6)

Alternatively, you can also directly run the python file in terminal and you would still have the same thing

## Information

### Communication

Determine which com port your Arduino is communicating on, you can find this Using Windows device manager, or you can open the Arduino sketch and click on the tools menu, and then click on Port.  Enter this in the "COM PORT" entry field and click "Set Com", you should see the LED on your Arduino blink.  You should only have to do this once as the software will remember which com port the next time you open it.

### Speed / Acceleration / Deceleration

The robot's speed is set as a percentage of max speed.  If you set the speed to 100 this will be the fastest the robot can move.  Typical speed for jogging is between 10% and 25%.  The acceleration and deceleration settings each have 2 parameters: a duration and a percentage.  The duration is the percentage of the move you want the robot to accelerate or decelerate through, for example, if you have a move that is 100mm long and you want the robot to quickly accelerate for only the first 5mm (or 5% of the move) you could set the Dur setting to 5 and then if you wanted the robot to come to a more gradual stop over the last 25mm (or 25% of the move) you could set the deceleration duration setting to 25.  The second value which is percentage is a measure of amplitude, for example, if you set the "speed %" setting to 10% it will accelerate or decelerate within 10% of whatever the speed setting is.  For example, if your overall speed is 25% and your deceleration is 10% the robot will decelerate from 25% down to 2.5%. The Speed, Acc, and Dec setting apply both to jogging and are applied to any positions you teach.

### Jogging

In the degrees to jog box enter the number of degrees you wish the robot to move and then press the corresponding "-" or "+" button to move each joint. When jogging in joint mode You can also jog the robot in individual motor steps by checking the "Jog joint in steps" radio button. The second row of jog buttons allows you to jog the robot in Cartesian coordinates; enter the distance in millimeters you wish the robot to move and press the corresponding "-" or "+" to jog the robot. The third row of jog buttons allows you to jog the robot in tool coordinates; enter the distance in millimeters you wish the robot to move and press the corresponding "-" or "+" to jog the robot - this jogs the robot according to the gripper: think of it as jogging the gripper around depending on whatever direction it is oriented.

You can also jog the robot using an Xbox controller.  Install the Xbox software per the Windows PC Xbox controller adapter directions. To test it In the Windows start menu type joy.cpl and then click on the menu option for the Xbox controller, this will bring up a window where you can verify all the buttons are working.  In this software click the Xbox button, and the indicators on the screen should turn green and allow you to jog the robot per the button list in the next section. When you turn on Xbox control all jog distances are set to start at 5. All jogging is done with the D-pad (analog sticks are not used). There are 3 modes for Xbox jogging - joint, Cartesian, and reorient. The controller starts in joint-mode where the D-pad will start out controlling J1 and J2, you can then press the X button to shift to control J3 and J4, then press X again to shift to J5 and J6.  Press the A button to shift to Cartesian jogging, you then can jog axis X and Y using the D-pad. Press the X button again to shift to axis Z.  Press the B button to control orientation directions.

- jog distance up / down (L & R trigger buttons)
- speed up / down (L & R bumper buttons)
- shift joint mode (X button)
- shift Cartesian mode (A button)
- shift reorient mode (B button)
- jog track (back button)
- jog directions (use D-pad)
- teach position (Y button)  *this will implement whichever move type you have selected from the dropdown
- first DO on/off assignment on Input Output tab (start button)  *typical use for open close gripper

### Programming

#### Teaching Positions

Always select the row in the program window where you want the next move or instruction to be placed. From the move type drop-down button select the move type you wish to insert and then press the "Teach New Position" button to insert the position into your program. All moves will apply the speed, acceleration, and deceleration setting you have set to the move. Position move types are as follows:

- Move J - Move J is a joint move where all joints work together to complete the move, this will not necessarily be a straight line but a sweeping motion with all joints working together.  This is the simplest and most common move to use.  
- OFFS J - this is a joint move that is offset by the values of a stored position.  this move will apply whatever stored position number you have set in the stored position field that is just above the "Teach New Position" button. (stored positions are explained in further detail below in the entry for Move SP).
- Move L - Move L is a linear move, this will execute a perfectly straight line to the position you teach. This program must send a series of waypoints that form the line to the Arduino, there is a couple-second delay before any Move L while all waypoints are being transmitted.
- Move A - Move A is an arc move.  You must teach 3 points to form an arc.  First, select "Move A beg" and teach the start point for your arc - the speed and orientation values for this first point will be applied to the entire arc move.  Second, you need to teach any mid-point on the arc - select "Move A Mid" and teach the second point.  Finally, select "Move A End" and teach the point you want at the end of your arc.  Your command window will now have 3 lines of code in a row for each of the 3 points.  When a "Move A Beg" line of code is executed the program will automatically run the next 2 lines of code to calculate the arc. The move will not work if these are out of order. There is a couple-second delay before the Move A will execute while all waypoints to form the arc are being transmitted to the Arduino.
- Move C - Move C is a circle move.  You must teach 3 points to form a circle.  First, select "Move C center" and teach the center point for your circle - the speed and orientation values for this first point will be applied to the entire circle move.  Second, you need to teach the start point on the circumference of the circle where you want the robot to begin and end the circle - select "Move C Start" and teach the second point.  Finally, select "Move C Plane" and teach a point anywhere on the same plane you want your circle,  this point is only used to know which direction you want the circle to go and this third point defines the plane - in other words just teach another point on the circles circumference - it doesn’t really matter where it is, it’s not an executed point and only used for calculation.  Your command window will now have 3 lines of code in a row for each of the 3 points.  When a "Move C Start" line of code is executed the program will automatically run the next 2 lines of code to calculate the circle. The move will not work if these are out of order. There is a couple-second delay before the Move C will execute while all waypoints to form the arc are being transmitted to the Arduino.
- Move SP - SP stands for stored position.  In the registers tab, there are 16 stored positions you can set.  You can set or save the X,Y,Z,Y,P,R for any position you want to execute later or multiple places in your program.  When you teach a Move SP the robot will move to the position you have entered for the stored position on the register tab.  *Stored positions can also be used for offsets - for example, if you want the robot to come in above your part you may want to use an offset move with a stored position 25mm up in the Z direction - example: (0,0,-25,0,0,0).
- OFFS SP - this moves the robot to a stored position and then offsets that position by the value in another stored position.  This is useful for stacking and placing parts in rows.  This move will use the value in the stored position field that is just above the "Teach New Position" button for the primary move to execute, then for the stored position that it will be offset by it automatically picks the next stored position - but you can use the manual entry field to edit which stored position the move will be offset by (see the section below on editing).
- Teach SP - this move command will insert 6 lines into your program which when executed will store the robot's current position into a stored position register of your choice.  This makes it easier to populate stored positions as you need.

The Stored Position button will allow you to enter lines of code that set individual elements of the X,Y,Z,Y,P,R in a stored position.
The modify position button is only used with Move J and allow you to modify the Move J line in your program that is currently highlighted.
The Delete button allows you to delete any line of code that is currently selected.

#### Pausing

- The wait time button inserts a line that will pause the program for the amount of time entered in seconds.
- The wait input on button will wait for the Arduino input entered in the entry field to come on before moving forward in the program.  This can be used as a way to make the robot wait for something else to happen before proceeding or it can be placed at the beginning of a program as a way to have an automated start signal.
- The wait input off button will wait for the Arduino input entered in the entry field to turn off before moving forward in the program.

#### IO

- The set output on or set output off buttons allow you to insert a line of code that will turn the Arduino IO of your choice on or off (see bottom on input outputs tab for available IO pins on the Arduino Mega). For example, if you have a pneumatic gripper you would hook up your solenoid per the wiring harness manual to output Arduino pin #38 and enter a line of code "Out On = 38" to control your gripper.

#### Navigation

You can create as many program routines as you like. Enter the name of the program you would like to create in the program field and press "load program", if the program does not already exist it will be created, if you have already created a program of that name it will be loaded.  Programs are created in your root directory folder and can be deleted from that file location if no longer needed.

- The "Call Program" button allows you to insert a line of code that calls a program.
- The "Return" button inserts a line of code that will allow the called program to return to the program it came from. 

*note you cannot call another program from within a program that has already been called, you must return to the main program before calling another program.  For example, you will likely want to create a program called "Main" from that program you might call a program called "Pickup Part" At the end of the pickup part you will want to insert a "Return" line to get back to the "Main" program, then you can do other things or call other programs.  You cannot call another program from "Pickup Part" you must first return to the main program.

- the "Create Tab" button allows you to create markers in your program that you can jump or navigate to based on conditions. 

*note you cannot have 2 tabs with the same number - each tab needs a new number. This functionality is very similar to basic programming.

- The "Jump to Tab" button allows you to jump to a tab 

for example, you could put "Tab 1" at the top of your program, and at the bottom put a "Jump to Tab 1" and then your program would loop indefinitely.

The "If Register Jump" button allows you to jump to a tab based on the condition of a register.

For example, you could have a looping program as previously described but then add a line into your program that increments a register and then add a line prior to "Jump to Tab 1" that says "If Register 1 = 5 Jump to Tab 2" and then place a "Tab 2" at the very bottom after "Jump to Tab 1" so that the program will run 5 times and then jump to Tab 2 and stop.

#### Registers

The "Register" button allows you to set a register to a static value or you can add a "++" before the number and the register will then be incremented by the amount.

For example, if you just enter a "1" it will always set that register to a value of 1 but if you enter "++1" it will then increment that register by 1 every time the line is run so that you can use this for counting.  You can enter any number, for example you could enter "++3" and count by 3's if you like.  The same is true for counting down or decrementing - just place a "--" before the number.

#### Servos

The Servo button allows you to control external servos - it’s not for the robot itself, it’s for use if you have a servo gripper or a servo actuator that you want the robot program to control.

For example, if you had a servo gripper that you had hooked up to Arduino pin A0 per the wiring harness manual you could then insert a line of code "Servo number 0 to position: 180" to open the gripper and "Servo number 0 to position: 0" to close your gripper.

#### Editing lines of code

You can select a line of code in your command window and then press the "get selected" button, this will copy that line into the manual entry field. You can now edit the line of code in the manual entry field, some examples might be: changing the stored position number, changing a position, or changing the robot's speed or acceleration. Now with your edited line of code, you can reselect the original line of code in your command window and then press the "replace" button and the old line of code will be replaced with the new edited line. The "insert" button will insert the test from the manual entry field into your program without replacing it - you can use this to insert comments or handwritten lines of code using the insert button, this can be used to copy a line of code from the program and then paste or insert in in numerous places in the program.

### Calibration

- Auto Calibration

Pressing the auto-calibration button will auto-calibrate all axis. The robot will run to its full limit in the default directions and set each of the joint values accordingly. You can also use the individual buttons to calibrate each axis one at a time.

- Force to midrange Calibration

This button allows you to force each axis to be calibrated at its mid-point. This is only used when setting up your robot - for example, if your robot is not yet calibrated and you are trying to jog a joint around and you hit an axis limit this button will allow you to do what you need to do before you can auto-calibrate your robot. Only use this button during construction and setup.

- Fine calibration

This feature allows you to set a reference position, so you can check how true your calibration is given an event where you have a bent limit switch or must replace a part of the robot. 

For example, you can jog the robot to some reference position that you know, perhaps with the robot holding a pointer and going to a known spot, you could then press "teach fine calibration position" to save this position. Then later you can press "Go to fine calibration position" to return to the originally taught position and check accuracy. If it's off, you can jog the robot in small steps to the correct calibration position and then press "execute fine calibration" and your calibration will be adjusted to the original reference position.

- Direction Defaults

the calibration defaults are set to the side of the axis that the limit switch is mounted on. 

If for example, you were building your own custom robot you could change these values to change the direction the robot drives each axis to the calibration limit switch. there are 6 values - one for each joint and they can only be a "0" or "1" for negative direction or positive direction.

The motor direction outputs work in the same way and allow you to change to motor drive direction from clockwise to anticlockwise if you are building a custom robot and your mechanics require a motor to spin the other way. again, there are 6 values - one for each joint and they can only be a "0" or "1" for negative direction or positive direction.

- Robot Calibration Values

The calibration values allow you to input the freedom of motion for each joint and the number of stepper motor steps to make that stroke happen. If building your own custom robot, you could change these values to match your custom range of motion and stepper gearing.

- DH parameters 

these are the Denavit Hartenberg parameters used to calculate the kinematics of the robot given the length and orientation of each arm.

- IO TAB

The buttons on the IO tab are simply a shortcut for you to quickly toggle servos or outputs. For example, if your gripper was wired to Arduino output #38 you could enter 38 into one of the "DO ON / OFF" fields and quickly open and close your gripper without having to execute a line of code from the program console.
