# python-ins1000
Python driver for INS1000 RTK System


## Main features
- automatically discover a Rover connected to serial port  TODO: make faster and more reliable
- log data to local file
- parse various ouput packets
- run as a webserver for Aceinna Navigation Studio (ANS) web app and show trajectory, satellite signal strength and sky-view info.

## Testing environment 
- Windows10: python2.7 and python 3.7
- cygwin: python2.7 and python 3.6
- Mac OS: python2.7 and python 3.7

## Steps

### pip install
- pyserial
- tornado
- azure
- requests

### Run
Run server.py to run as a webserver for Aceinna Navigation Studio (ANS) web app
```
python server.py
```

### Trajectory and Maps
Open [maps/trajectory webpage](https://developers.aceinna.com/maps) and check trajectory and maps

![maps](/img/maps.png)
![trajectory](/img/trajectory.png)

### sky-view
Open [sky-view webpage](https://developers.aceinna.com/skyview) and check sky-view info

![skyview](/img/skyview.png)
