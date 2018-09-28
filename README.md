#**PID Controller Project** 
---

This project is the fouth project of Udacity Self-driving Car Nanodegree Term2. The goals / steps of this project are the following:

* Use PID controller to control the handling and throttle of the car on the simulator
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[video1]: ./results/PID_NoD.mp4 "P controller"
[video2]: ./results/PID_BigD.mp4 "Big D controller"
[video3]: ./results/PID.mp4 "PID controller"
[video3]: ./results/PID_Slowdown.mp4 "PID Slow Down controller"

---

### How to build the program
There is a [guide](https://github.com/simonchu47/CarND-PID-Control-Project/blob/master/HOWTO.md) showing how to prepare the environment and build the program.

### The effects of the P, I, D components

####1. P component effect on handling/throttle

Too large Kp for the handling controller will cause the car serpentine repeadly, and eventually out of control. But inversely, small Kp will let the car response slowly, and usually cause the car to leave the road on a curve. I found that without Kd component support, the handling controller only with Kp component always lead to an unstable situation.

It seems to be easier for throttle controller. Large Kp, 1.0 to 100.0 that I have tried and the target speed 30MPH without Ki for the throttle controller never cause the car's speed to fluctuate too much around the target. But too small Kp(below 1.0) will let the car hardly achieve the speed target.

The following video shows the driving with a PID handling controller only with the P component. It serpentines soon after start.
[video1]


####2. D component effect on handling/throttle

D component plays a role like prediction of the error trend. A suitable Kd value can help the handling controller be convergent rapidly on the cte. But inversely a too large Kd value also leads to an unstable situation.
The D component effect on the throttle is not so significant, for the reason that just the P component could perform very well.

The following video shows the driving with a PID handling controller with a too large D(500 times Kp) component. It also becomes unstable soon after start.
[video2]


####3. I component effect on handling/throttle

I hardly found that Ki can have any effect on both the handling and throttle controllers. I thought that it might be the reason that the simulation situation is not a straight road, and the accumulated error will soon amplified by Ki, which will very easy to let the system become unstable.

###The final hyperparameters

####1. Twiddle algorithm

I design a state machine inside the PID class to perfom the TWIDDLE algorithm. We can decide how long we will let the simulator to run for collecting the error data and reset again and again until the algorithm gets the optimized parameters, which are according to our set condition like the sum of all the k stepping value below some value. 

The following video shows the driving with a handling and a throttle PID controller, and all the k values except ki are derived from the TWIDDLE.
[video3]

####2. Manually Setting

The TWIDDLE algorithm always decides a 0.0 value of Ki, and I will manually give it a very small value.

####3. Slow down at curve

I design an experimental method to mimic the human behavioral model, that we will slow down when the car seems to lose control. When the cte goes high, by the method, the target speed will be set lower. It works, as the following video.

Besides, all the PID value for both the handling and throttle controllers are derived from the TWIDDLE algorithm, except the Ki.
[video4]



