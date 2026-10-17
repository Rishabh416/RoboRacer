# RoboRacer

[https://youtu.be/pNFWOHAOf94]
Fastest Lap: **7.9s**

The algorithm works using a PD controller for steering using the Lidar for wall and curvature measurements. The lidar readings are converted into points that is used to fit a linear/quadratic curve for the track-width estimates while another set of readings gauge the available forward distance to prevent crashing and allow appropriate time to slow down. With this information, the system calculated the steering from -1 to 1, the approach speed, the target speed and the throttle. Lastly, there are a few checks to ensure the car doesn't crash including checking duration since last sensor message to get fresh data and timeout checks. 
