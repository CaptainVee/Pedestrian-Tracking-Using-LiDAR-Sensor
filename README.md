# Pedestrian-Tracking-Using-LiDAR-Sensor
## Objective
Utilize sensor data from LIDAR measurements for pedestrian tracking with Kalman Filter.
## State prediction
The x' = Fx + ν equation does these prediction calculations for us.

![image](https://user-images.githubusercontent.com/59261333/72988538-20839e80-3df5-11ea-9831-2b66678a0358.png)

Process noise refers to the uncertainty in the prediction step. We assume the object travels at a constant velocity, but in reality, the object might accelerate or decelerate. The notation ν∼N(0,Q) defines the process noise as a gaussian distribution with mean zero and covariance Q.
when we predict the position one second later, our uncertainty increases. P' = FPFT + Q represents this increase in uncertainty.
Because our state vector only tracks position and velocity, we are modeling acceleration as a random noise. The Q matrix includes time Δt to account for the fact that as more time passes, we become more uncertain about our position and velocity. So, as Δt increases, we add more uncertainty to the state covariance matrix P.
Combining both 2D position and 2D velocity equations previously deducted formulas we have:

![image](https://user-images.githubusercontent.com/59261333/72988947-f9799c80-3df5-11ea-81d9-b175f4f42634.png)

Since the acceleration is unknown, we can add it to the noise component, and this random noise would be expressed analytically as the last terms in the equation derived above. So, we have a random acceleration vector ν in this form:

![image](https://user-images.githubusercontent.com/59261333/72989046-2f1e8580-3df6-11ea-8d80-c4211716b5cd.png)

which is described by a zero mean and a covariance matrix Q, so ν∼N(0, Q).
The vector ν can be decomposed into two components a 4 x 2 matrix G which does not contain random variables and a 2 by 1 matrix a which contains the random acceleration components:

![image](https://user-images.githubusercontent.com/59261333/72989161-62611480-3df6-11ea-8b80-7da747fededd.png)

Δt is computed at each Kalman Filter step and the acceleration is a random vector with zero mean and standard deviations σax and σay.
Based on our noise vector we can define now the new covariance matrix Q. The covariance matrix is defined as the expectation value of the noise vector ν times the noise vector νT. So, let’s write this down:

![image](https://user-images.githubusercontent.com/59261333/72989424-e6b39780-3df6-11ea-8dde-9131d2ec39a1.png)

As G does not contain random variables, we can put it outside the expectation calculation.

![image](https://user-images.githubusercontent.com/59261333/72989506-0a76dd80-3df7-11ea-8740-d6ef982334c9.png)

ax and ay are assumed uncorrelated noise processes. This means that the covariance σaxy in Qν is zero:

![image](https://user-images.githubusercontent.com/59261333/72989546-22e6f800-3df7-11ea-85e0-15785befc0f1.png)

So, after combining everything in one matrix we obtain our 4 by 4 Q matrix:

![image](https://user-images.githubusercontent.com/59261333/72989584-3a25e580-3df7-11ea-8638-2fc0248634f5.png)

## LIDAR measurements
- How does LIDAR measurement look like

![image](https://user-images.githubusercontent.com/59261333/72989721-7d805400-3df7-11ea-94e1-1d610aba8388.png)

The LIDAR will produce 3D measurement px,py,pz. But for the case of driving on the road, we could simplify the pose of the tracked object as: px,py

- The measurement function H

H is the matrix that projects your belief about the object's current state into the measurement space of the sensor. For lidar, this is a fancy way of saying that we discard velocity information from the state variable since the lidar sensor only measures position.
Find the right H matrix to project from a 4D state to a 2D observation space, as follows:

![image](https://user-images.githubusercontent.com/59261333/72990010-11eab680-3df8-11ea-8e1b-f88bbd3c78a6.png)

in order to obtain our measurement (x and y) we need to determine the H matrix. (x and y) is found by multiplying H with the state vector.
So, let’s find out first what is the dimension of our H matrix. Here we have a 2 by 1 matrix, and that came from our m by n H matrix times the four row and one column matrix. Now, from the matrix multiplication we know that the number of columns of the first matrix should be equal with the number of rows of a second matrix, which is 4. And also, the number of rows of the first matrix is the same with the number of rows of the result matrix, which is 2. So, our H is a matrix of 2 rows and 4 columns. And finally, we put ones and zeroes so that the px and py coordinates are propagated to the result Z.
the correct H matrix:

![image](https://user-images.githubusercontent.com/59261333/72990112-40689180-3df8-11ea-95be-d864be05489e.png)

- Measurement Noise Covariance Matrix R

For laser sensors, we have a 2D measurement vector. Each location component px, py are affected by a random noise. So, our noise vector ω has the same dimension as z. And it is a distribution with zero mean and a 2 x 2 covariance matrix which comes from the product of the vertical vector ω and its transpose.

![image](https://user-images.githubusercontent.com/59261333/72990186-65f59b00-3df8-11ea-83e1-c98a1c31b349.png)

where R is the measurement noise covariance matrix; in other words, the matrix R represents the uncertainty in the position measurements we receive from the laser sensor.
Generally, the parameters for the random noise measurement matrix will be provided by the sensor manufacturer.
Remember that the off-diagonal 0s in R indicate that the noise processes are uncorrelated.
