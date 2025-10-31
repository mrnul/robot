
# Experimenting with an esp32c6, OpenCV and automatic control systems
## Equipment
### Set 1
1. A laptop
2. A camera with USB
3. A tripod would be useful

### Set 2
We need to build a 2WD robot car that will be controlled by an esp32c6
1. esp32c6 devkit (has a built-in ws2812 LED)
2. One extra ws2812 LED
3. Jumper wires
4. A motor driver (TB6612FNG) capable of driving 2 motors
5. Chassis and two wheels
6. A power-bank to supply power
7. Optionally a small breadboard


## The Setup
- Camera in a fixed position (tripod) is connected to the laptop via a USB cable
- Laptop acts as a hotspot that accepts connections from the esp32c6
- Robot lies on the floor and exchanges data with laptop

## The Goal
The goal is to make the robot orient itself and move on the floor from the data extracted from camera.

## How
To do that we must locate the robot in the screen from the camera.

In order to locate the robot we are going to recognize the color of the LEDs. One LED at the front of the robot, one LED at the center of the robot.

This way we can have both orientation and location of the robot.

Then we need to find the real coordinates of the robot on the floor.

Having the coordinates we can use a control law to make it move as we like.

All calculations will take place on the laptop because it is easier to debug. The esp32c6 will simply take the result of the control law and apply the appropriate PWM on the motors.


# The Math
## World Coordinates and Camera
Floor is the $XY$ plane.

Camera is at position $c=(0,0,z_c)$ looking down at the floor having an angle $t$ with the $Y$ axis clockwise.

Positive $X$ is on the right of the camera and negative $X$ on the left.

The unit vector at the direction the camera is looking is $n=(0, \cos{t}, -\sin{t})$

It is meaningfull to assume $0 \leq t \leq \frac{\pi}{2}$

## Image Plane
The principal point (center of the image plane) is $pp=c+d \cdot n$, where $d$ is the focal length.

Having a point on the image plane and the vector $n$ we find that the equation of the image plane is $d + (z - z_c) \sin{t}= y \cos{t}$

The unit vectors on the image plane in world coordinates are $ip_x=(1,0,0)$ and $ip_y=(0, \sin{t}, \cos{t})$

## Pixels to Image Plane in 2D
In OpenCV the screen origin is at the top left corner, and positive y means going down on the screen. We need to bring the origin at the center of the screen and make y positive in the up direction, for convenience.

A pixel in screen $(p_x, p_y)$ will have $(u, v)$ coordinates on the image plane.
We assume that this transformation is linear, a simple scaling.

$u=(p_x - \frac{w}{2})s_w$

$v=(\frac{h}{2} - p_y)s_h$

Where $w$ is the screen width in pixels, $h$ is the screen height in pixels and $s_w,s_h$ are the scaling factors.

>Note that in real world this is not a linear transformation since the lenses distort the image. For simplicity we consider it to be linear and accept the fact that we will have some error.

## Image Plane 2D to 3D
Having the $(u, v)$ coordinates on the image plane we can find the 3D coordinates $ip_{xyz}=(x_{ip},y_{ip},z_{ip})$ using the unit vectors of Image Plane 

$ip_{xyz}=pp + u \cdot ip_x + v \cdot ip_y$

## Image Plane 3D to Floor coordinates
The position of the camera $c$ and the point $ip_{xyz}$ define a line of sight with the object.

The line equation is $l=(x_l,y_l,z_l)=c+\lambda(ip_{xyz}-c)$

We need to find the $\lambda$ which makes $z_l=0$, or generally $z_l=a$ with $z_c > a \geq 0$

Turns out that $\lambda=\frac{z_c-a}{-v \cos{t}+ d \sin{t}}$

So the real coordinates on the floor are $(x,y,z)=\lambda(u, d \cos{t}+ v \sin{t}, \frac{a}{\lambda})$

Two constraints arise
1. $-v \cos{t}+ d \sin{t} \neq 0$
2. $\lambda>0$

The first one is obvious since a denominator cannot be zero.

The second one arise from the direction of the line.

Here $\lambda<0$ means that the object is behind the camera which is absurd. $\lambda=0$ breaks things. So the only valid values are for $\lambda>0$

## Visual quick sum-up of the transformations needed
$(p_x, p_y) \rightarrow (u,v) \rightarrow (x_{ip},y_{ip},z_{ip}) \rightarrow (x,y,z)$ 

## Note
It is easy to measure $z_c, t$

We need to find appropriate values for $d,s_w,s_h$ which can be tricky
