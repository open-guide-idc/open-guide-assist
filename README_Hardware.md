# Hardware Report


__Key Components__

___OAK-D___

The OAK-D is the primary component of OpenGuide-Assist that facilitates 3 major functions through its 4K RGB Camera for visual perception, a stereo pair for depth perception and an Intel Myriad X Visual processing Unit to function as the brain/processor capable of running modern neural networks while simultaneously creating a depth map from the stereo pair of images in real time.

__Hardware Specifications:__

The OAK camera uses USB-C cable for communication and power. It supports both USB2 and USB3(5Gbps / 10 Gbps).
| Camera Specs | Color Camera | Stereo Pair|
| ------ | ------ | ------|
| Sensor | IMX378 |OV9282
| DFOV / HFOV / VFOV | 81° / 69° / 55° | 82° / 72° / 50°
| Resolution | 12MP (4032x3040) |1MP (1280x800)
| Max Frame Rate | 60 FPS | 120 FPS|
| Pixel Size |  1.55µm x 1.55µm| 3µm x 3µm

<img src="https://github.com/amg1998/BUSeniorDesign-Opticle-21-22/blob/main/images/oakd.png" width=50% height=50%>


The OAK-D has a 100% detection accuracy when an object is inside in the region of interest as highlighted in the image below provided that the size of the obstacle is at least 46.5 square inches in projected area. The point cloud's region of interest spans 5 feet 7 inches in length, 3 feet 3 inches in width and 3 feet above and below the camera with DFOV angle of 82°.

<img src="https://github.com/amg1998/BUSeniorDesign-Opticle-21-22/blob/main/images/roifinal.png" width=50% height=50%>

___Raspberry Pi 4B___

The Raspberry Pi 4B is the brain of the device that connects all the different components together. Firstly, with 2 USB ports, it is connected to a 10000maH power bank (5V) and the OAK-D camera to receive the video feed and produce actionable feedback such as haptic and auditory output. 

<img src="https://github.com/amg1998/BUSeniorDesign-Opticle-21-22/blob/main/images/raspberry%20pi.png" width=50% height=50%>

___Wrist Mount___

While the OAK-D, Power Source and the Raspberry Pi 4B will be on the chest mount that the user be equipped with, there is also a 2nd wrist mount component that the user will wear like a watch. The wrist mount is equipped with a LRA motor that sits on the base of the mount and a raspberry pi zero that sits in the hollow region between the roof and the base. The raspberry pi zero will interface with the raspberry pi 4B where the LRA motor will vibrate if an object is present inside the point cloud region of 2mx1mx1.7m.

<img src="https://github.com/amg1998/BUSeniorDesign-Opticle-21-22/blob/main/images/wristmount.png" width=20% height=20%>

__Required CAD Files__

Select components were constructed on OnShape(CAD Software) and the files are attached below in the case that the reader requires personal customization.

| Part | CAD File |
| ------ | ------ |
| Wrist Mount | [Wrist Mount CAD](WristMount.step) |
| Pi Holder + Switch | [Pi Holder CAD](piholder.step)|
| PowerBank + Raspberry Pi Holder | [Holder CAD](powerbankholder.step)
| OAK-D Go Pro Mount | [OAK-D Mount CAD ](powerbankholder.step)

__Schematic Diagram__

Below is the overall schematic of OpenGuide-Assist and how the various components interact with each other;
<img src="https://github.com/amg1998/BUSeniorDesign-Opticle-21-22/blob/main/images/schematics%20diagram.png" width=50% height=50%>

__Bill of Materials__


Many of the components were purchased from vendors as the software was the primary focus of this project. Attached below is the bill of materials.
| Part | CAD File |Purpose | Quantity | Price |
| ------ | ------ | ------ | ------ | ------ |
| OAK-D Camera| [OpenCV](https://store.opencv.ai/products/oak-d) | For  visual and depth perception | 1 | $199.00|
| Go Pro Chest Mount | [Amazon](https://www.amazon.com/Chest-Belt-Strap-Harness-Mount/dp/B089LJC6LW/)|To secure components onto the user | 1 | $33.70|
| LRA Motors | [Amazon](https://www.digikey.com/en/products/detail/vybronics-inc/VG0840001D/15220809?s) | To produce haptic  feedback based on video output| 1 | $3.71|
| Raspberry Pi Sugar| [Amazon](https://www.amazon.com/Portable-Pwnagotchi-Raspberry-Accessories-handhold/dp/B09MJ8SCGD/)| Power source for the Raspberry Pi Zero | 1 | $33.99|
| Portable Charger| [Amazon](https://www.amazon.com/10400mAh-Portable-Charger-External-Compatible/dp/B07JYYRT7T/) | To power Raspberry Pi and OAK-D | 1 | $15.99|
| Raspberry Pi Zero| [Amazon](https://www.amazon.com/Raspberry-Pi-Zero-Wireless-model/dp/B06XFZC3BX/)|Placed on the wrist mount and interfaces with primary Pi 4| 1 | $10.00|
| Raspberry Pi 4B | [Amazon](https://www.amazon.com/Raspberry-Pi-Computer-Suitable-Workstation/dp/B0899VXM8F/)|Primary processing unit of the system | 1 | $75.00|
| Barell USB Connector| [Amazon](https://www.amazon.com/2-5mm-Barrel-Connector-Charge-CableCC/dp/B00ZUDXMK2/)|Connect the OAK-D to powersource| 1 | $6.00|
|Total||||$377.39|


__Power Consumption__


| Product | Amps (A) | Capacity (maH) | Volts (V) |Produces/Requires|
| ------ | ------ | ------| ------ | ------|
| OAK-D | 3 | N/A | 5 | Requires
| Raspberry Pi 4B | 5.1 | N/A | 5 | Requires
| PowerBank | 3 | 10400 | 5 | Produces
| Raspberry Pi Zero | 0.8 | N/A | 5 | Requires
| Raspberry Pi Sugar | 2 | 1200 | 5 | Produces

__Power Capacity Calculations__

| Part | Capacity |
| ------ | ------ |
| Power Bank  | 10000mAH / 3000mA  = 3.33 Hours ~ 3 Hours and 20 Minutes |
| Raspberry Pi Sugar  | 1200 mAH / 800 mA = 1.5 Hours ~ 1 Hour and 30 Minutes|
| Titanium AfterShockz | Roughly 6 Hours on Full Charge|

Since the system cant produce haptic feedback without the Raspberry Pi zero interfacing with the Raspberry Pi 4, the total duration that the system can remain portable is **1 Hour and 30 Minutes.**


__Weight of Product__

|Item | Weight(g)|
| ------ | ------ |
|OAK-D Camera | 42.52|
|Go Pro Chest Mount |229.06|
|Raspberry Pi 4B |45.36|
|Raspberry Pi Zero |8.79|
|Pi Sugar |39.50|
|10000 maH Power Bank| 178.62|
|Titanium AfterShockz Headphones| 36.20|
|LRA motor |2.50|
|Total |580.50|

__Final Assembly of Product__ 

Below is a visual assembly of product OpenGuide-Assist from a front and back perspective.

<img src="https://github.com/amg1998/BUSeniorDesign-Opticle-21-22/blob/main/images/assembledmount%20diagram.png" width=450px height=450px> <img  src="images/backview.png" width=400px height=450px>



