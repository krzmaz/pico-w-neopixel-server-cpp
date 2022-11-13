# pico-w-neopixel-server
Simple webserver aiming to be a thin remote controller for neopixel strips.

Developed to work with https://github.com/mrozycki/rustmas project.


If your host is capable of using mDNS, then you should be able to connect to the pico board using following hostname:
```
pico-w-neopixel-server.local
```
**WARNING:** There is no implemented conflict resolution - if you plan to use more than one neopixel server, 
consider changing the hostnames to different values for each server.

---
## Dependencies:
- CMake 3.19+
- ARM GNU toolchain

If you don't have them set up, you can use [these instructions](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf#%5B%7B%22num%22%3A39%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C115%2C841.89%2Cnull%5D)

---
## Usage
Just send a POST request to the `/pixels` endpoint with 3 bytes of data per each pixel!

To generate example data you can use:
```bash
NUM_PIXELS=500
temp=0; while [ $temp -lt $NUM_PIXELS ]; do byte=$(expr $temp % 100); printf "\x$byte\x$byte\x$byte" >> tmp.txt; temp=$(expr $temp + 1); done;
```
Then to send it to the device:
```bash
curl --data-binary @tmp.txt pico-w-neopixel-server.local/pixels
```

## FS data
The `my_fsdata.c` file containing the response `res.json` file needs to be generated using [makefsdata](https://github.com/lwip-tcpip/lwip/tree/master/src/apps/http/makefsdata) executable compiled for the host machine.  
Create a folder named `fs` and a `res.json` file with the content of `{}` in it, and run `makefsdata -11 -nossi` in the directory that contains the `fs` directory.

There are issues in the template repository for automating this using Perl/Python scripts, however those lack some functionality that the C program has, and we need, like the `keep-alive` header. 

-----
Generated using https://github.com/krzmaz/pico-w-webserver-example