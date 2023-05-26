# procedural-locomotion

Code base for the project: Procedural Locomotion Engine For Digital Humans.

## Getting started

We recommend [JetBrains CLion](https://www.jetbrains.com/clion/) for the development.
It is a paid software, but JetBrains has the [student plan](https://www.jetbrains.com/community/education/#students)
that provides free licenses. See [this](https://www.jetbrains.com/help/clion/clion-quick-start-guide.html) for a quick
start guide.

1. Fork this repository and download the code.

2. Build the project (or build `locoApp`). You can build the project in cmake Release mode for realtime performance:
   see [this](https://www.jetbrains.com/help/clion/cmake-profile.html) for a guide about cmake profile for CLion.

3. Run the `locoApp`.

4. Select a model to play with: `Main Menu > Character > Model`. We have `Bob` and `Dog` for examples.

![Screen](img/screen.png)

5. Press the space bar to play the app. You can give joystick command with arrow keys in your keyboard. If it does not
   move, don't freak out. It's normal. You have to implement a base (body) trajectory planning strategy as well as a
   inverse kinematics solver: recall [a1](https://github.com/Digital-Humans-23/a1). Once you've done that, you will see
   something like this.

![Baseline](img/baseline.gif)

6. Of course, we don't want this crappy walking motion. Now, try your best to generate more natural motions for our poor
   Bob! But first, play around the code, and try understanding it. Don't hesitate to contact Dongho (kangd@ethz.ch) if
   you have any question regarding the implementation.

## Immediate Next steps

- Implement an inverse kinematics solver: ```src/libs/loco/include/loco/kinematics/IK_Solver.h```. Complete TODO part.
- Complete a base (body) trajectory planning
  strategy: ```src/libs/loco/include/loco/planner/LocomotionPlannerHelpers.h```
- Design a strategy for arm motions. Modify ```src/libs/loco/include/loco/planner``` files to implement your strategy.
- Review biomechanics, computer graphics and robotics literature for a natural ankle motion. This is the most
  crucial step for successful demo!

## Comments

- Most likely, you wouldn't need to modify the following source code files in
    - ```src/libs/gui``` and ```src/libs/utils```: basic feature implementation for rendering, mathematical operation
      etc.
    - ```src/libs/loco/include/loco/robot``` and corresponding cpp files.
    - But of course, if you need, feel free to do it.
- This repo will be keep updated, so please stay tuned. If you want to sync your repo with the new commits,
  use ```git rebase``` instead of ```git merge```:
  see [this](https://www.atlassian.com/git/tutorials/merging-vs-rebasing) for more details of ```git rebase```.
- Please actively use GitHub issue for questions and reporting issues on the code base!

## Animating a new rigid body

### 1. Check if the rigid body has an end effector
Open `data/robots/bob/bob.rs` and look for the corresponding rigid body definition, e.g. `lUpperArm`:
```
RB
	name lUpperArm
	mesh robots/bob/meshes/lUpperArm.obj
	meshTransformation 1 0 0 0 	-0.16875 -0.39375 0
	mass	2.4027099609375					
	moi	0.02423240929842	0.00285087168216705	0.02423240929842	0	0	0	

	collisionSphere 0 0.16875 0 0.04	
	collisionSphere 0 -0.16875 0 0.04
/End_RB
```
Add an end effector as the desired position relative to the rigid body's origin (usually that's where one of the `collisionSphere`s is located):
```
RB
	name lUpperArm
	mesh robots/bob/meshes/lUpperArm.obj
	meshTransformation 1 0 0 0 	-0.16875 -0.39375 0
	mass	2.4027099609375					
	moi	0.02423240929842	0.00285087168216705	0.02423240929842	0	0	0	

	collisionSphere 0 0.16875 0 0.04	
	collisionSphere 0 -0.16875 0 0.04
    endEffector 0 0.16875 0 0.04
/End_RB
```
Check that the positioning is correct by running `locoApp` > Draw > Endeffectors. End effector appears as green sphere.

### 2. Register the end effector in `menu.h`
Open `src/apps/locoApp/menu.h` and add the rigid body name:
```
...
CRL_DATA_FOLDER "/robots/bob/bob.rbs",  //
{
    {"lLowerLeg", "lLowerLeg"},
    {"rLowerLeg", "rLowerLeg"},
    {"lFoot", "lFoot"},
    {"rFoot", "rFoot"},
    {"lHand", "lHand"},
    {"rHand", "rHand"},
    {"head", "head"},
    {"pelvis", "pelvis"},
    {"<your_actual_RB_name>, <your_personal_RB_name>}
},
0.9,   //
...
```

### 3. Add a new swing phase in `GaitPlanner.h`
A swing phase is a phase where the foot is not in contact with the ground and is an interval of length at most 1. However, for most body parts it does not make sense to define a swing phase, since they never touch the ground. If this is the case, make sure that the swing phase is *nearly* 1, e.g. 0 to 0.999, or -0.5 to 0.499 (if you make it exactly 1, things break).