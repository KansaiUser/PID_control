# CarND-Controls-PID
**Self-Driving Car Engineer Nanodegree Program**

David Aliaga

## Reflection

I hereby present my implementation of the PID controller for the vehicle. 
Once built you can use the program in two modes:

* Normal mode:  Just call *./pid*

* Twiddle mode: Call it like *./pid twiddle*

### Normal mode

You should use this mode to see the vehicle drive a lap around the track immediately. 
This mode uses hyparameters chosen manually.

### Twiddle mode

This mode will run first the twiddle algorithm and *it will take time* (more than a couple of hours) to tune the parameters. When it finishes it will run the lap. 
(In the current state the twiddle parameters are fixed, later work may include set the max_n and tolerance by arguments)

## How the parameters were chosen

In the normal mode I chose parameters manually that happened to be quite good and could traverse the lap without much problem. 
The parameters were 0.2,0.0004,3.0

When we apply twiddle two things happend:

* When the length of the try was 500, the parameters were modified but the final parameters 0.537177, 0.000346487, 3.20147 performed horribly, and the vehicle could not complete the lap. 

* When the length of the try was set to 1000, the parameters were modified slightly (only the I parameter) to 0.2,0.0005,3.0 and the vehicle could traverse the lap.

I believe that the poor performance of the first twiddle was because the trial lap was too short so it modified the parameters (specially P) too much, and a bigger P caused the vehicle to go erratically to left and right

The second attempt modified only I so P was stable. 

## Effects of P,I,D

Thanks to trying twiddle I could see that increasing P makes the vehicle go left and right erratically, and sometimes impede it finishing the lap. 

On the other hand a big D stabilize the vehicle. 
I could not see much influence of the I parameter, and once the vehicle could complete the lap with I set to 0.

However I think that when a bias is inherent on the vehicle, I can have a correcting influence to the algorithm

## Video
You can find a video of the normal run (not twiddle) [here](https://www.youtube.com/watch?v=ZYqazWCYQa8)
