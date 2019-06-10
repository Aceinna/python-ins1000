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

### Register in ANS
Open [Aceinna Navigation Studio (ANS)](https://developers.aceinna.com/) and registers a new user.
I highly recommend you to use Chrome.

### pip install
- pyserial
- tornado
- azure
- requests
- asyncio (If python3)

### Run
Download the source code of branch Master, then run server.py
```
python server.py
```

Open [ANS](https://developers.aceinna.com/ins), then user can:
- Check the navigation infomation on web GUI
- Log files

![maps](/img/maps.png)
![trajectory](/img/trajectory.png)

Below is the sky-view of satellites:
![skyview](/img/skyview.png)

The log file can be opened by excel as below:
![log_data](/img/log_data.png)

