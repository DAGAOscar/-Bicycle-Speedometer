Functional description of the project:

The aim of the project was to create a microcontroller bike counter. The project consists of two main parts, 
namely a microcontroller with a connected Hall sensor and a mobile application launched on the phone. 
The calculation of speed by the microcontroller is based on detecting the presence of a magnetic field near the 
Hall sensor. The source of the magnetic field is a magnet attached to the wheel spoke. The magnetic field is detected 
with each revolution of the wheel. Based on the frequency of the magnetic field, the rotational speed achieved while 
riding is calculated. Then it is necessary to know the wheel diameter to calculate the linear speed. At this step, calibration 
is necessary, which is done by measuring the time needed to travel a distance of 100m and obtaining the average rotational speed. 
Based on this data, the wheel diameter is calculated and then the linear speed. The application then sends the obtained speed to 
the mobile application using the BLE protocol. In the mobile application, it is possible to display the current linear speed and start calibration.
