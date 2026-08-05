# Data Synchronization

NAT-DRIVE utilizes three different sensors: 
- a GPS to capture the driven vehicle trajectory,
- a FMCW radar to record the following vehicle's behavior,
- a dashcam to aid the authors through the data analysis.

As all three sensors run independent from one another it is necessary to synchronize the data after collection. The purpose is to align the events observed in one sensor with the others to ensure that the corresponding follower's reaction aligns with the leader's, and that the follower's time of response is captured.

For synchronization purposes, the authors of this work recorded multiple runs starting from a complete stop and recorded the distance to a fixed wall. The recorded data from GPS, Radar, and Dashcam where then used to calculate the offset between sensors. 

In return, the code slices the corresponding Leader's data based on the time of the processed radar data. Additionally, the data is shifted in time based on the specified time offset between GPS and Radar data.

The "Car-Following Data Processing.ipynb" code was developed to extract the desired data from the Racebox GPS sensor. 

Racebox GPS provides a csv file with the following recorded data:
- Record: 	Recorded element number as an integer staring from 1
- Time: 	Time of the reading in yyyy/mm/dd, hh:mm:ss.000 format
- Latitude
- Longitude
- Altitude
- Speed: 	driven vehicle's speed, units depend on the user's desired setting.
- GForce X:	Accelerometer's forces experienced in the X axis measured in G's.
- GForce Y:	''' in the Y axis '''
- GForce Z:	''' in the Z axis '''
- Lap
- Gyro X:	Gyroscope reading on X axis in Deg.
- Gyro Y:	''' on Y axis in Deg.
- Gyro Z:	''' on Z axis in Deg.

This code outputs multiple files based on the processed data: 
- .csv file containing the sliced data that matches radar event in time.
	- index, Time(dd/mm/yyy hh:mm:ss.000), Latitude, Longitude, Position (m), Speed (km/h), Acceleration (m/s^2)
- Position, Speed, and Acceleration plots of driven car.
- .html file with GPS trajectory from a satellite view. 
