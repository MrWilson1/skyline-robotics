======================================================================
>> README <<
Skyline High School (with Boeing)
Spartabots, Team 2976
Program designed for 2011 FRC Robotics competition - Logomotion
Last updated Saturday, February 12th, 2011.

Table of Contents
  Instructions on how to control the robot
  Explaination of user-interface design choices
  Explaination of source code
  Explaination of how Logomotion (2011's game) is played
======================================================================
Instructions on how to control the robot:

  Summary:
  The robot is controlled with two joysticks, labeled joystick 1 and
  joystick 2.  Joystick 1 is used for moving the robot, whereas the
  other joystick is used for controlling the scissor-lift.
  
  Joystick 1 (Moving):
    Move stick up            - Robot goes forward
    Move stick down          - Robot goes backwards
    Move stick left          - Robot strafes left
    Move stick right         - Robot strafes right
    Amount stick is tilted controls robot speed.
    
    Top-left button (4)      - Robot rotates counter-clockwise
    Top-center button (3)    - Robot rotates clockwise
    Throttle (at bottom)     - Controls rotational speed
    
    Trigger                  - Robot enters fast mode.
    To use fast mode, the trigger MUST be held down.  Releasing it cuts
    the speed in half.
    
    Bottom upper-left (7)    - Enable safety mode (allows fast speed).
    Bottom lower-right (6)   - Disables safety mode (limits speed for safety).
    Safety mode can also be enabled if the scissor lift rises too high
    to prevent the robot from toppling at high speeds.

	Bottom upper-

  Joystick 2 (Scissor Lift):
  	Move stick up            - Lift rises
  	Move stick down          - Lift decreases
  	Top-lower button (2)     - Lift goes to very bottom
  	Top-left button (4)      - Lift goes to the first peg
  	Top-center button (3)    - Lift goes to the second peg
  	Top-right button (5)     - Lift goes to the third peg
    Each of the buttons on the top of the joystick moves the lift to
    a preset value.  Choosing a preset causes the lift to automatically
    move to the value.  Using the joystick or selecting another preset
    overrides any automatic lift movement.
    
    
    Disable safety mode (10) - Allows fast speed
    Enable safety mode (11)  - Disables fast speed (for demos)
    Safety mode can also be enabled if the scissor lift rises too high
    to prevent the robot from toppling at high speeds.
  	
======================================================================
Explaination of user-interface design choices:
  Due to the nature of the game, controlling the robot was divided
  into two roles -- moving and controlling the scissor-lift.  This
  division of roles was initially suggested by Mr. Cooper (one of the
  team's mentors) because it would also allow two drivers to work in
  cooperation.
  
  Toggles were intentionally avoided in the user interface (toggles
  are buttons that alternatively turn something on or off).  The 
  primary reason for this was to increase usability.  If toggles are
  used, the user would have to remember the current state of the robot
  and would have to monitor themselves to make sure they don't
  accidentally toggle something twice.  Removing toggles would in
  theory allow the user to make less mistakes and allow them to focus
  more on the game instead of trying to master the controls.
  
  The button to rotate clockwise is the top-center button, not the
  top-right button.  This is because the distance between the top-left
  and top-right button was too large and would have decreased both
  reaction time and comfort.
======================================================================
Explaination of source code:
  The code for 2011 was based on the previous year's code, which in
  turn was based on 'SimpleTemplate' which was created by FIRST.  
  
  All of the code (for 2011) was placed into a single class named
  'MainRobot'.  The class is called only once, near the end of the
  document.  The class itself is divided into two parts: variable
  declarations and the actual functions.  The variable declarations
  contain a mixture of mutable variables (note that I am somewhat 
  unfamilier with terminology, so may be using incorrect terms) and
  constants.  The 'typedef enum' portions of the code were added by
  Mr. Cooper.  The second part of the code, the functions, all fall
  after the keyword 'public'.  All of the functions were declared
  public (mostly because that was what last year's code was like).
  
  Technically speaking, only three functions are required in the class
   - the constructor, Autonomous(), and OperatorControl(). Depending 
  on the values sent by either the field during the actual competition
  or the driver station while testing, the robot would always start by   
  calling the constructor, and start whichever function was needed. 
  It is important to note that (according to them), the code does not    
  automatically jump from Autonomous() to OperatorControl() -- the     
  program has to detect when it is no longer autonomous, return from  
  that function, then let the robot move on to OperatorControl()      
  
  TODO: Autonomous here
  
  From OperatorControl(), a few things are set up, then the program 
  loops through several pieces of code.  FatalityChecks checks to make
  sure that the robot is not doing anything harmful.  OmniDrive takes 
  in joystick data, and converts it to useful movement.  The function
  to control the scissor-lift was intentionally divided into two --
  the function ScissorManual() is used when the joystick may also 
  control the scissor-lift while ScissorPreset() was created primarily
  so that it would be easier to use the scissor-lift during autonomous
  mode.  UpdateDashboard() uses SmartDashboard (created by FIRST) and 
  creates a simplified dashboard.  The dashboard from last year was 
  not reused because it is apparently no longer compatible with this 
  year's WPI library.  The function GetSign() was created just for
  convienence - after finding that many functions required something
  similar to GetSign(), the function GetSign() was created just to so
  that code could be reused.  
======================================================================
Explaination of how Logomotion (2011's game) is played:
  Logomotion consists of two sides, red and blue, with each side
  comprising of a (temporary) allience between three teams.  There are
  a total of six teams and six robots on the field.  A key feature of
  the gamefield are the four posts -- one in each quarter of the 
  field.
  
  The game can be split into three parts -- autonomous mode, operator
  controlled, and the end-game.  Before autonomous, each robot is 
  lined up slightly beyond the center line, between the two posts on 
  their side of the field.  They start by holding a ubertube. At the 
  start, each robot autonomously moves forward and attempts to place
  the ubertube on a peg (the higher, the more bonus points). This 
  lasts for 10 seconds.  
  
  Then, the operator can control the robot.  The purpose is to drive 
  to the opposite side, grab a tube from a member of the allience, 
  drive back, and place the tube on any of the pegs.  The inflatable
  tubes can either be circular, square, or triangular.  If they are 
  placed on the pegs so that they form the FIRST logo, bonus points 
  are awarded.  No vigorous attacks are allowed, although pinning is 
  permitted.  Apart from the poles, there are no obstacles in the 
  field.  This lasts for 2 minutes.  
  
  In the last 10 seconds, the robot may deploy a minibot on a pole.  
  The first minibot to hit the top of the pole gets the most amount of 
  bonus points.
  
  In short, place the inflatable tubes on the pegs, and build a
  fast-climbing minibot for a bunch of bonus points at the end.
======================================================================

END OF DOCUMENT

