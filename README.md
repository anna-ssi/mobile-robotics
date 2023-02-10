# CMPUT 503: Experimental Mobile Robotics

### Assignment 1

### Assignment 2
Fork the [template](https://github.com/duckietown/template-ros) and add the assignemnet 2 folders in the packages folder.
Change the Dockerfile to your own credentials.

As the code uses a service to change the LED color of the bot, run this command first to start the LED node.
```
dts duckiebot demo --demo_name led_emitter_node --duckiebot_name [VEHICAL NAME] --package_name led_emitter --image duckietown/dt-core:daffy-arm64v8
```

After activating your LED node, run these commands in sequence.

```
dts devel build -f -H [VEHICAL NAME].local
dts devel run -H [VEHICAL NAME].local
```
