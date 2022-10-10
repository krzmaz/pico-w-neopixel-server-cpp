# pico-w-neopixel-server
Simple webserver aiming to be a thin remote controller for neopixel strips
## Usage
Just send a POST request to the `/pixels` endpoint with 3 bytes of data per each pixel!

To generate example data you can use:
```bash
NUM_PIXELS=500
echo -n "data=" > tmp.txt; temp=0; while [ $temp -lt $NUM_PIXELS ]; do byte=$(expr $temp % 100); printf "\x$byte\x$byte\x$byte" >> tmp.txt; temp=$(expr $temp + 1); done;
```
Then to send it to the device:
```bash
curl --data-binary @tmp.txt <IP_ADDRESS>/pixels
```
-----
Generated using https://github.com/krzmaz/pico-w-webserver-example