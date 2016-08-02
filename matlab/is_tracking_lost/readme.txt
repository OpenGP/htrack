Each line of the file «data.txt« contains two numbers – those are the metrics computed at a given frame. The first metric is the ratio of intersection and union of the model and data silhouette, the second metric is average distance from data to model. 

The entry point is main.m
The array success_and_failure_switch (inside of main.m) contains the numbers of frames at which the system switches between success and failure. I determined these frames but looking at the recording of a tracking sequence and subjectively judging whether the system is failing or nor.

The result of the training is the parameters vector theta. To determine whether the system is failing at a given frame from the two run-time metrics, compute
f = 1.0 / (1.0 + e^(-x’ * theta));
if f < 0.5, the system is failing.
