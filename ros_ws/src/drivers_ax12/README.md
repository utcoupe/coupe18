# drivers_ax12

This package mainly provides an action server to move an AX-12A to an angle, or to switch it in wheel mode. Both modes need to specify a speed in the action goal.
It comes also with a service to set specific registers of an AX-12A, such as the punch, ...

It uses an old version of the [Dynamixel-SDK](http://en.robotis.com/service/downloadpage.php?cate=sdk) from [this repository](https://github.com/lintangsutawika/Dynmixel-SDK-Linux) to provide a set of low level function.
Wrapping this library is the `Ax12Driver` class, which implements higher-level function such as a scan and methods to switch mode. The table of the AX-12A registers and their addresses is defined in the `ax12_table.h` header.
Finally, the `Ax12Server` class handles the action server and the service.