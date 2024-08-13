**ROBOT LEARNING FRESHMAN RESEARCH INITIATIVE**

*Team Members: Janine, Clarissa, Linh*

This repository contains code that was used to develop our 2-week team project over Rearrangement Planning.

Our initial goals included:
1. Place an item on a flat surface using PyBullet simulation.
2. Place an item near another static object without clashing or overlapping.
3. Place more than one item near more than one static object.
4. Create a policy for a Sawyer robot to arrange items on a table without any human help.
5. Have the Sawyer robot measure objects without predetermined measurements.

Obstacles
- Setting up Pybullet and Packages on lab computers
- Generating Point Clouds: Initially wanted to use Point Clouds as one of our inputs and tried to convert from image to pixels from the depth camera provided by PyBullet, but there were too many issues and too little time so we had to move on.
- Lacked concrete approach to initial goals. Unsure about how to approach our goals and asked for help from professor and teacher assistants, but due to time and difficulty, could not find a solid path.

Redirection
- Create an environment in PyBullet.
- Generate random locations to place plates on within the bounds of a table.
- Determine valid positions by checking for collisions and add them to an array of position values.
- Have the robot create a new arrangement on its own.

This Redirection was much more realistic and achievable given the amount of time we had left. Additionally, we were able to work through schedule conflicts, meet up times, and other outside factors that would hinder our progress on this project.

  *Below are some pictures of our simulations in PyBullet.*

Set Up in PyBullet:
![image](https://github.com/user-attachments/assets/09b4b43c-0cb6-44bd-9477-910ecb06b873)

Invalid Position:
![image](https://github.com/user-attachments/assets/ac12aca1-4197-479f-81f4-f1ee23d3a442)

Valid Position:
![image](https://github.com/user-attachments/assets/4fbd1452-dd52-430c-8a18-99d3aff1561a)


