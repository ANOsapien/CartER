# Handover from internship

This document describes the handover of the project after my (Ananya Priyaroop)
10 week internship.

In the time I had, I automated the cartpole setup, to swing up, and start a feedback loop for PID control and run uninterrupted for many number of trials for data collection. On what data was collected, I performed k-means clustering to identify clusters of optimal PID sets.


## What does not work
Throughout the project, I used 
<a href="https://www.amazon.in/dp/B07P7VZRRX" target="_blank">DS3225MG 25 kg·cm, 180° servo motor</a>, 
which required precise calibration. So, while reproducing the set-up I switched to 
<a href="https://www.amazon.in/dp/B00PNEQKC0" target="_blank">MG996R 360° servo motors</a>, 
unaware that they would need a different tuning approach.  
The setup I had been working on is currently functioning well; however, the three newly assembled systems still require proper motor tuning to achieve optimal performance.
