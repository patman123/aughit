![Alt text](http://www.robotix.in/Images/rbtx.png)
AugHit : Augmented Image Processing Event
=========================================
This repository contains code for the simulator for the event AugHit conducted during Robotix 2015, Kshitij at IIT Kharagpur. (http://www.robotix.in/aughit)

The simulator has been built using OpenCV on C++ with Box2D Wrapper for Physics and simulations.

**Requirements:**
- OpenCV (2.4.9 or later)
- Box2D
- CMake (>2.6)

To run the game:

```sh
$ git clone https://github.com/mronian/aughit.git
$ cd aughit
$ mkdir build
$ cd build
$ cmake ..
$ make
```
You will find all the required binaries in the **'build'** folder.

##Round 1:

- Simple round with fixed number of static bricks.

**For Semi-Circular:**

```sh
$ ./Round1Circular 1
```

**For Rectangular:**

```sh
$ ./Round1Rectangular 1
```

Where 1 is the **Camera Index**. By default it is 0.


![Alt text](/Files/Images/Screenshots/Round1Circular.png)

####<u>Round 1 : Semi-Circular Paddle</u>

![Alt text](/Files/Images/Screenshots/Round1Rectangular.png)

####<u>Round 1 : Rectangular Paddle</u>

##Round 2

- Has false bricks whose ID's can be passed as command-line arguments to the binary.

**For Semi-Circular:**

```sh
$ ./Round2Circular 0 3 5 7
```

**For Rectangular:**

```sh
$ ./Round2Rectangular 0 3 5 7
```
- Here, as before, 0 is the **Camera Index**. (3, 5, 7) are the ID's of the bricks to be declared as false.

![Alt text](/Files/Images/Screenshots/Round2Circular.png)

####<u>Round 2 : Semi-Circular Paddle</u>

![Alt text](/Files/Images/Screenshots/Round2Rectangular.png)

####<u>Round 2 : Rectangular Paddle</u>


##Round 3

- Has moving bricks also.

**For Semi-Circular:**

```sh
$ ./Round3Circular 0 3 5 7
```

**For Rectangular:**

```sh
$ ./Round3Rectangular 0 3 5 7
```
![Alt text](/Files/Images/Screenshots/Round3Circular1.png)

####<u>Round 3 : Semi-Circular Paddle<u>

![Alt text](/Files/Images/Screenshots/Round3Rectangular1.png)

####<u>Round 3 : Rectangular Paddle</u>

## Miscellaneous
- The dimensions of the game, scaling factor, number of static bricks, number of moving bricks, number of corners, size of the corners are all defined in the *Constants.h* file. Change accordingly to tweak the game!

- Use **ClearVid.sh** to clear the video buffer: (Usage Below)

```sh
$ ./ClearVid.sh
```
