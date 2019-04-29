# python-ins1000
Python driver for INS1000 RTK System


## Main features
- Automatically discover a Rover connected to serial port  TODO: make faster and more reliable
- Log data to local file
- Parse various ouput packets
- Run as a webserver for Aceinna Navigation Studio (ANS) web app and show trajectory, satellite signal strength and sky-view info.

## Testing environment 
- Windows10: python2.7 and python 3.7
- Mac OS: python2.7 and python 3.7

## Steps

### pip install
- pyserial
- tornado
- azure
- requests
- asyncio (If python3)

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
