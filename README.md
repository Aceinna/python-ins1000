# python-ins1000
Python driver for INS1000 RTK System


## main features
- automatically discover a Rover connected to serial port  TODO: make faster and more reliable
- log data to local file
- parse various ouput packets
- run as a webserver for Aceinna Navigation Studio (ANS) web app and show trajectory and sky-view info.

## testing environment 
- Windows10: python2.7 and python 3.7
- cygwin: python2.7 and python 3.6
- Mac OS: python2.7

## steps

### pip install
- pyserial
- tornado

### run
Run server.py to run as a webserver for Aceinna Navigation Studio (ANS) web app
```
python server.py
```

### trajectory and maps
Open [maps/trajectory webpage](https://developers.aceinna.com/maps) and check trajectory and maps

![maps](/img/maps.png)
![trajectory](/img/trajectory.png)

### sky-view
Open [sky-view webpage](https://developers.aceinna.com/skyview) and check sky-view info

![skyview](/img/skyview.png)











