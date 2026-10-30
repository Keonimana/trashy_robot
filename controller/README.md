See the top-level README for build and run instructions.



Learning CMAKE: these examples help https://github.com/ttroy50/cmake-examples/tree/master/01-basic




Motor controller library: High level abstraction for I2C calls to PCA9685 
Probably should offer some kind of timing mechanism to see how this is controlled 


Notes: 

PWM range: 
50 Hertz --> 20 ms windows
12 Bit resolution of PCA = 0 - 0xFFFF Where the last digit doesn't matter 
We want to drive this just as radio signals are, which are from 1ms - 2ms with 1.5 ms being the neutral position, which is 5% --> 10%  
What this means in terms of PWM signals is that a 
1ms PWM length --> 0xFFFF / 0x0014 = 0x0ccc = 0x0cc0 
1.5ms PWM length --> 65535 * 0.075 = 4915 = 0x1333 = 0x1330 
2ms PWM length --> 0xFFFF / 0x000A = 0x1999 = 0x1990

Range for forward motion = 0x0666 = 0x0660 = 1632 
Would be nice to remap this into an interval of 0 - 100. 
Each one is approx 16.32 = 0x10 

NOTE: THIS IS A DISCRETE RANGE. 
Gives us a range from 0 - 102. Which is good enough, no need to push it to the very limit. Probably not good enough fine-grain control
but we'll c. 2

Question: How do I design a continuous controller? 
The ultimate limit is simply the resolution of the controller 
which is 12 bits. 2^12 = 4096. 

1/4096 * 20000 us = 4.88us 
A.k.A Each bit increase can give about a 2.4% increase of the entire range? 


How will this translate to speed. Sometimes we'll need the 
upper range of controls depending on what kind of environment we're in? Seems like a far-fetched line of thinking prob don't need to 
worry about it for now. 

