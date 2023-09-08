# WRO 2023 GammaVersion's
**GammaVersion's repo for WRO Future Engineers 2023 season**

***

## Contents

* [Hardware](#hardware)
    * [Models](#models)
    * [Electronics](#electronics)
    * [Assembly](#assembly)

* [Software](#software)

* [Photos](#photos)
    * [Car images](#car-images)
    * [Team images](#team-images)

* [Videos](#videos)
    * [Demonstration videos](#demonstration-videos)

* [Legal](#legal)
    * [License](#license)
    * [Credits](#credits)

***
Building an autonomous car for this challenge involves a meticulous process of reimagining and redesigning various components to achieve precision, safety, and efficiency. In this project, we focused on enhancing the car's steering mechanism, designing a custom-printed circuit board (PCB) to connect all the components seamlessly, and implementing Light Detection and Ranging (LiDAR) technology for superior distance measurement compared to traditional ultrasonic sensors.
## Hardware
    LiDAR Implementation:
    Rather than relying on traditional ultrasonic sensors for obstacle detection, we opted for LiDAR technology. LiDAR, which stands for Light Detection and Ranging, uses laser beams to measure distances to objects with exceptional precision. This technology offers several advantages over ultrasonic sensors:

    a. Greater Range: LiDAR can detect objects at much greater distances, providing the car with more time to react to potential obstacles.

    b. High Precision: LiDAR provides accurate distance measurements, resulting in better navigation and collision avoidance.

    c. 360-Degree Coverage: LiDAR offers a full 360-degree view around the car, ensuring comprehensive situational awareness.

    d. Environmental Adaptability: LiDAR is less affected by environmental factors such as ambient light, making it more reliable in various conditions.

### Models
    3D-Printed Supports and Structural Parts:
    As a crucial part of our project, we designed and 3D-printed custom supports to secure mechanical components and sensors. These supports ensured precise placement and minimized interference. Additionally, we utilized 3D printing to create lightweight yet robust structural components, allowing us to tailor the car's design to our specific needs while maintaining its strength and adaptability.

### Electronics
    Custom PCB Design:
    To seamlessly connect and control all the components of our autonomous car, we designed and printed a custom PCB. The PCB acted as the central hub that allowed us to interface with sensors, motors, microcontrollers, and other electronic components efficiently.

    Our custom PCB design allowed for cleaner wiring, reduced interference, and enhanced reliability. It simplified the process of connecting and configuring various sensors and actuators, enabling smoother integration and troubleshooting during the development phase.

### Assembly
    Steering System Redesign:
    One of the key aspects of our project was reimagining the steering system to achieve a reduced turn ratio. To accomplish this, we implemented Ackermann steering geometry. Ackermann steering geometry is a mechanism that ensures that all four wheels of the car track along different paths during a turn, reducing tire wear and enabling smoother and more precise turns.

    By incorporating Ackermann steering, we were able to optimize the car's turning radius, allowing it to navigate tighter corners with ease. This was achieved by adjusting the angles of the wheels and their pivot points, ensuring that the inner and outer wheels followed distinct paths, greatly enhancing the car's maneuverability.



***

## Software

***

## Photos

### Car images

### Team images

***

## Videos

### Demonstration videos

***

## Legal

### License

The [source code](/code/) in this repository is licensed under the **GNU General Public License v3.0**.

The [hardware](/robot-info/hardware/) exposed in this project is licensed under the **Creative Commons Attribution Share Alike 4.0 International** license.

The documentation of this repository; found in [`robot-info`](/robot-info/), [`vehicle-photos`](/vehicle-photos/), [`videos`](/videos/), [`team-photos`](/team-photos/), as well as this `README.md`; is licensed under the **CERN Open Hardware Licence Version 2 - Strongly Reciprocal** license.

A copy of each license can be found in the [LICENSE](LICENSE) file. More information about the licenses in the specific README.md of each section.

### Credits

***
