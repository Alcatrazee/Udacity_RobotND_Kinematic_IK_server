## Project:Kinematics Pick&Place


##### Steps to complete the project:
1.Set up your ROS Workspace  
2.Download or clone [project repository](https://github.com/Alcatrazee/RoboND-Kinematics-Project) into the src directory of your ROS Workspace.  
3.Experiment with the **forward_kinematics** environment and get familiar with the robot.  
4.Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).  
5.Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).  
6.Fill in the **IK_server.py** with your Inverse Kinematics code.
## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points



[image1]: ./repo_pictures/FK.jpg
[image2]: ./repo_pictures/equation.jpg

#### Writeup/README
1.Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.  

This is the one  

#### Kinematic Analysis  

I did not use the frames set from urdf file,instead,I used the original frames set like this.  
 
Then create the DH Table for this robot.  
![alt text][image1]  

Links(i) | α<sub>i-1</sub>| a<sub>i-1</sub>|d<sub>i</sub>|θ<sub>i</sub>|
---|---|---|---|---|
1 | 0|0|0.75|θ<sub>1</sub>|
2 | -pi/2|0.35|0|θ<sub>2</sub>-pi/2|
3 |0|1.25|0|θ<sub>3</sub>|
4 |-pi/2|-0.054|1.5|θ<sub>4</sub>|
5 |pi/2|0|0|θ<sub>5</sub>|
6 |-pi/2|0|0|θ<sub>6</sub>|  
7 |0|0|0.303|0|  

Due to we are calculating the posture of end-effector,we need to add one more row to the DH Table.  
After obtainning the DH Table,we can calculate the transformation matrices between each joint,the form of the matrices is like this.  
![alt_text][image2]
