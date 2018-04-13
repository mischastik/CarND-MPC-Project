# Starting point

First I wrote a feature complete version without modelling the delay. It used a reference speed of 40 mph and no weighting of the cost function.
The temporal sampling rate (dt) was set to 0.05s and the number of samples (N) to 25.
It didn't come very far though since the steering oscillated excessively after a few meters.

# Parameter tuning
Weighting the cost parameters improved this behavior. First, the change in subsequent steering commands was penalized stronger.
This helped a lot but caused trouble in the first tight curve after the bridge. The car didn't miss the curve though but overcompensated left the track in the inside of the curve.
So penalizing the use of steering commands already did the trick as the tendency to overcompensate was apparently reduced.
With these two changes, the car was able to nicely drive the whole lap.

# Coping with latency
Next, I enabled the time delay of 100 milliseconds. This again caused massive oscillations and the car left the track after a few meters.
I modelled an additional delay into the constraints and increased the penalties from above even more. This helped but didn't eliminate the oscillations the necessary extent.
That's probably due to the fact that my model can't account for the driving commands "from the past" that cannot be optimized anymore in the current iteration.
It might be possible though, to include the previous steering commands into the constraints.
However, I went for a simpler approach and reduced the temporal resolution of the model as I expected this to alleviate the delay effect. So dt was set to 0.1 (from 0.05) and N to 10 to keep the total simulated time span constant.
This worked very well and the car was again able to take a whole lap without any incidents.

# Fine-tuning
The only annoying property left was an oscillation in the throttle. This isn't critical in this simulation as brake forces are not considered, but it's easy to solve: The use of the throttle and rapid changes in acceleration were also slightly penalized. The latter is probably not a good idea in real life since it penalizes emergency brake maneuvers. :-)
In the simulation, this worked fine but resulted in an overall slower drive and very slow acceleration at the start. A higher penalty for missing the target speed could be a solution here but turned out to have a negative effect on the overall stability and would probably require careful adjustments of the cost weights and dt an N parameters. Instead of tediously fine tuning the parameters to cope with this, I simply increased the target speed to 75 mph which resulted in a very smooth drive at the approximately the same effective speed as before without the annoying erratic brake-accelerate oscillation. However, the actual speed differs significantly from the target speed with this solution.
