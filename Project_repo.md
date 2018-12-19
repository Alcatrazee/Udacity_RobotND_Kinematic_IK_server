## Project:Kinematics Pick&Place


##### Steps to complete the project:
1.Set up your ROS Workspace  
2.Download or clone [project repository](https://github.com/Alcatrazee/RoboND-Kinematics-Project) into the src directory of your ROS Workspace.  
3.Experiment with the **forward_kinematics** environment and get familiar with the robot.  
4.Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).  
5.Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).  
6.Fill in the **IK_server.py** with your Inverse Kinematics code.
## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points



[FK]: ./repo_pictures/FK.jpg
[eq_i_ip1]: ./repo_pictures/equation.jpg
[FK_equation]: ./repo_pictures/equation_t.jpg
[theta1]: ./repo_pictures/equation_t1.jpg
[elbow_up_pic]: ./repo_pictures/theta2_elbow_up_true.jpg
[theta2_k0]: ./repo_pictures/theta2_k0_true.jpg
[theta2_kdot]: ./repo_pictures/theta2_k0.jpg
[theta3_k0]: ./repo_pictures/theta3_k0_true.jpg
[theta3_kdot]: ./repo_pictures/theta3_k0.jpg
[t4]: ./repo_pictures/t4.jpg
[t5]: ./repo_pictures/t5.jpg
[t6]: ./repo_pictures/t6.jpg
[rm_456]:./repo_pictures/rotation_matrix_456.jpg
[r36]: ./repo_pictures/R3_6.jpg
[r03]: ./repo_pictures/R_03.jpg


#### Writeup/README
1.Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.  

This is the one  

#### Kinematic Analysis  

I did not use the frames set from urdf file,instead,I used the original frames set like this.  
 
Then create the DH Table for this robot.  
![alt text][FK]  

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
![alt_text][eq_i_ip1]  
Therefore,we can calculate the transformation matrix of end-effector to base in this way.  
![alt_text][FK_equation]    
##### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.  
###### θ<sub>1</sub>  
There are two solutions for θ<sub>1</sub>,because joint2 and joint3 can reach a wide range of angle.  
![alt_text][theta1]  
Most of the base joint of industrial robots can only rotate about 1 round,therefore 2 solutions can satisfy all of the path.

###### θ<sub>2</sub> and θ<sub>3</sub>
There're two solutions for each θ<sub>1</sub>,in the equation of θ<sub>1</sub>,when k=0,robot face toward the wrist center.  
k=0:    
![alt text][elbow_up_pic]  
here're the solutions.  
![alt text][theta2_k0]  
![alt text][theta3_k0]  

k≠0:  
![alt text][theta2_kdot]  
![alt text][theta3_kdot]  
    
###### θ<sub>4</sub>,θ<sub>5</sub> and θ<sub>6</sub>
According to what we have got till now,we can calculate the transformation matrix of the gripper relative to frame 3,besides,we need the rotation matrix only,that'll make the work easier.  
Due to rotation matrix is an element of a group called SO(3),I used transpose operation to replace the inverse operation to make the program faster(I don't see any faster though).  
New that we have θ<sub>1</sub>,θ<sub>2</sub> and θ<sub>3</sub>,we can calculate the rotation matrix of frame 6 relative to frame 3.Which is:  
![alt_text][r36]  
We can obbtain Rotation matrix of frame3 relative to base frame with this:  
![alt_text][r03]  
Given RPY,we can calculate the end-effector's orientation relative to base frame, we need a rotation matrix to transform it to be relative to frame 6,using the end-effector's rotation matrix is ok.Then, we have a target orientation,and that orientation is mapped by a rotation matrix made by R4·R5·R6,where we can extract θ<sub>4</sub>,θ<sub>5</sub> and θ<sub>6</sub>.  
Rotation matrix with θ<sub>4</sub>,θ<sub>5</sub> and θ<sub>6</sub>:
picture:rotation_matrix_456  
Which is R_3_6.Amount all the inverse trigonometric function,atan2 is the best one to retrive the angle,but,in this case,there's a little disavantage in calculation of the angle of over 180 degree,for the function is only able to return -180 degree to 180 degree.  
However,there's more solutions,so we need to calculate them ourselves.Fortunately,it's easy to do that by simply plus or minus 2pi.Therefore,it would be 8 solutions for the orientation which give us a lot of choices. By combinning them to the 3 angles, we can have 32 solutions in total!  
Here I just put the easiest solution.  
![alt_text][t4]  
![alt_text][t5]  
![alt_text][t6]
The other solutions are like θ<sub>4</sub> rotate 180 or 360 degree and θ<sub>5</sub> and θ<sub>6</sub> rotate too.
