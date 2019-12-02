# Autonomous SLAM with ROS using the DrRobot X80 and an Intel RealSense Camera

This repository is dedicated to the development of a robotic system that performs [Simultaneous Localization And Mapping](https://github.com/A01371852/ROS_Autonomous_SLAM/projects/1) through ROS.

SLAM is concerned with the problem of building the map of an unknown environment with a mobile robot, while at the same time being able to localize itself within the navigated environment. The system is implemented using the DrRobot X80 platform as a starting point and an Intel Realsense RGB/Depth camera to retrieve visual information for the 3D mapping of closed room.

Navigation demo from 11/16/19: https://youtu.be/bn-o5vQM7YU

<p align="center">
  <img width="460" height="460" src="Evidences/Robot.jpg">
</p>

## Tutorial

The following steps are aimed to help the user replicate the setting up of the project at its current state, as well as giving an overall understanding of the implemented system.

### Prerequisites

#### Hardware prerequisites

* DrRobot X80 platform [[1](#bibliography)]
* Intel Realsense SR300 / BlasterX Senz3D camera [[2](#bibliography)]
* AAEON UP Board [[3](#bibliography)] (or a similar board with a USB 3.0 port and 64bit architecture)
* WiFi-USB module (only if the board does not have an embedded WiFi module already)
* TRENDnet N300 Wireless Router [[4](#bibliography)]
* 11.1V 3S LiPo battery with at least 3000mAh (5200mAh recommended)
* Pololu D15V70F5S3 5V step-down power regulator [[5](#bibliography)] or a regulator with similar specifications
* External PC with WiFi

#### Software prerequisites

* Ubuntu 16.04 Operating System for both the UP Board and the external PC

The process for installing Ubuntu 16.04 in both devices is the following:

* Download the ISO file from [this link](http://releases.ubuntu.com/16.04/) by selecting the ```64-bit PC (AMD64) desktop image```.
* Create a bootable USB image of the ISO file. That process is explained in these links for [Windows](https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-windows#0) and [MacOS](https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-macos#0).
* Installing Ubuntu 16.04 in both computers as explained in [this tutorial](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop-1604#0).

Although a virtual machine with Ubuntu can be used in the external PC, is is recomended that Ubuntu runs natively in both computers. A recommended alternative to using a VM is [installing Ubuntu in a disk partition and using dual boot](https://www.tecmint.com/install-ubuntu-alongside-with-windows-dual-boot/).

### Setup

#### Hardware setup

Setting up the hardware is pretty straightforward. However, it is recommended to check for any short circuit between the terminals of the regulators before connecting the battery, and to measure the output voltage of the step-down regulator before connecting the UP Board.

After the security checks, the charged and balanced LiPo battery is connected to the DrRobot X80 using a male Tamiya connector, and to the 5V power regulator. The X80 can be connected directly to the battery as it already incorporates a low dropout regulator [[6](#bibliography)]. The UP Board will be fed 5V by the step-down regulator through a male DC jack connector. Finally, the camera is connected to the UP Board through the USB 3.0 port; an addapter for the cable might be needed as illustrated in this diagram.

<p align="center">
  <img src="Diagrams/Electrical_design.png">
</p>

#### Software setup

Once Ubuntu 16.04

```
$ Comandos a ejecutar
```
En ocasiones es necesario explicar algún comando dentro de un párrafo, como por ejemplo `mkdir` . Para lograr esto se pone entre apóstrofes invertidos la palabra deseada:


### Demo example



```
$ Comando ejemplificativo o un pedazo de código a editar o modificar antes de compilar.
```



## Recomendaciones para escribir en el README.md

Una lista de opciones se puede crear comenzando la linea con un asterisco y espacio, * + <space>, en cada una de ellas:
* Opción uno
  - Opción 3.1

[Maven](https://maven.apache.org/) - Dependency Management


<p align="center">
  <img width="460" height="300" src="http://www.ros.org/wp-content/uploads/2016/05/kinetic.png">
</p>


| Cantidad | Descripción | Precio Unitario | Subtotal |
| --- | --- | --- | --- |
| 1 | Tarjeta Arduino | 100.00 | 100.00 |
| 2 | Potenciómetros | 10.00 | 20.00 |
| 10 | Resistencias de 1K| 1.00 | 10.00 |


## Authors

* **Marcos Eduardo Castañeda Guzman** (A01372581@itesm.mx) - *Digital Systems and Robotics Engineering*
* **Gerardo Uriel Monroy Vázquez** (A01372286@itesm.mx) - *Digital Systems and Robotics Engineering*
* **Emmanuel Hernández Olvera** (A01371852@itesm.mx) - *Digital Systems and Robotics Engineering*

## Bibliography

1. Dr.Robot. (2006). *WiRobot X80 USER MANUAL*. Retrieved from: [https://www.cs.princeton.edu/courses/archive/fall11/cos495/X80_Manual.pdf](Datasheets/X80_Manual.pdf)
2. Intel Corporation. (2016). *Intel RealSense Camera SR300*. Retrieved from: [https://www.mouser.com/pdfdocs/intel_realsense_camera_sr300.pdf](Datasheets/intel_realsense_camera_sr300.pdf)
3. AAEON. (2019). *UP Datasheet V8.5*. Retrieved from: [http://data-us.aaeon.com/DOWNLOAD/2014%20datasheet/Boards/UPDatasheetV8.5.pdf](Datasheets/AAEON_UP_Board.pdf)
4. TRENDnet. (2012). *N300 Wireless Home Router*. Retrieved from: [https://www.trendnet.com/support/support-detail.asp?prod=230_TEW-731BR](Datasheets/Router_TEW-731BR(1.01).pdf)
5. PololuCorporation. (2019). *Pololu Step-Down Voltage Regulator D15V70F5S3*. Retrieved from: [https://www.pololu.com/product/2111/specs](Datasheets/Pololu_D15V70F5S3%E2%80%8B%E2%80%8B.pdf)
6. Texas Instruments. (2014). LM2940x 1-A Low Dropout Regulator. Retrieved from: [http://www.ti.com/lit/ds/symlink/lm2940-n.pdf](Datasheets/LM2940T-5.0-NOPB.pdf)
