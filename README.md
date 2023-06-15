# procedural-locomotion

This is our branch designed for live modeling of joint angles and trajectories. It also by default loads our territory shown in the demo. 

## Live editing of the joint constraints and trajectories
To edit the joint constraints live while the simulation is running, use the following commands:
| Key  | Functionality |
| :---         |      ---: |
| 0-9 | Press the number keys to enter the joint id (appends digit)|
| # | Reset current joint_id to empty string |
| Shift | Press to switch between editing min, max and both angles |
| + | Increment angle by 0.1 |
| - | Decrement angle by 0.1 |
| e | Print angle constraints of all joints and reset current joint to ""|
| i | Print info of current settings|
| x | Set min equals max|
| p | Edit pelvis trajectory (follow instructions in terminal) |
| l | Edit leg trajectory (follow instructions in terminal). When prompted for input you have to enter four numbers into the terminal each followed by an enter. Those resemble the vales for: lLowerLeg, rLowerLeg, lToes, rToes |
| h | Edit hand trajectory (follow instructions in terminal). When prompted for input you have to enter two numbers into the terminal each followed by an enter. Those resemble the vales for: lHand, rHand|
| k | Edit head trajectory (follow instructions in terminal)|

Next steps: Potentially add real export functionality, potentially add a non-default mode with more fine-grained control.

## References
.obj files for territory creations (e.g. palm trees) taken from: https://www.turbosquid.com/


