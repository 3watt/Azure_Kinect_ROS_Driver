# Azure Kinect ROS Driver

##설치

- git clone 을 해주어야 한다. [Azure-Kinect-Sensor-SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK) 와 [Azure_kinect_ros_driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver)

이후, catkin make 를 하면, k4a가 없다는 에러가 뜰 것이다. 그냥 sudo apt get install 을 하면, k4a 가 나오지 않을 것이다!! 
sudo get install 에서 k4a pkg 를 불러오려면 아래의 과정을 해주어야 한다.
```
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
sudo apt-get update

```
그 이후에, 아래의 lib 를 설치해주면 된다.
```
sudo apt-get install k4a-tools
sudo apt-get install libk4a1.3
sudo apt-get install libk4a1.3-dev 
```

그리고, Azure-Kinect-Sensor-SDK/scripts 에 있는 rules 파일을 /etc/udev/rules.d/로  cp 해주자!

```
 cp ~/catkin_ws/src/Azure-Kinect-Sensor-SDK/scripts/99-k4a.rules /etc/udev/rules.d/
 
```
그런다음, 카메라를 전원선과 C_type_usb선 두개 모두 꽂아서 사용하면 된다!! 둘 중 하나라도 꽂아 있지 않으면,
Failed to open a K4A device. Cannot continue.
에러가 발생 한다.




This project is a node which publishes sensor data from the [Azure Kinect Developer Kit](https://azure.microsoft.com/en-us/services/kinect-dk/) to the [Robot Operating System (ROS)](http://www.ros.org/). Developers working with ROS can use this node to connect an Azure Kinect Developer Kit to an existing ROS installation.

This repository uses the [Azure Kinect Sensor SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK) to communicate with the Azure Kinect DK. It supports both Linux and Windows installations of ROS.

[![Build Status](https://dev.azure.com/ms/Azure_Kinect_ROS_Driver/_apis/build/status/microsoft.Azure_Kinect_ROS_Driver?branchName=melodic)](https://dev.azure.com/ms/Azure_Kinect_ROS_Driver/_build/latest?definitionId=166&branchName=melodic)

## Features

This ROS node outputs a variety of sensor data, including:

- A PointCloud2, optionally colored using the color camera
- Raw color, depth and infrared Images, including CameraInfo messages containing calibration information
- Rectified depth Images in the color camera resolution
- Rectified color Images in the depth camera resolution
- The IMU sensor stream
- A TF2 model representing the extrinsic calibration of the camera

The camera is fully configurable using a variety of options which can be specified in ROS launch files or on the command line.

However, this node does ***not*** expose all the sensor data from the Azure Kinect Developer Kit hardware. It does not provide access to:

- Microphone array

For more information about how to use the node, please see the [usage guide](docs/usage.md).

## Status

This code is provided as a starting point for using the Azure Kinect Developer Kit with ROS. Community developed features are welcome.

For information on how to contribute, please see our [contributing guide](CONTRIBUTING.md).

## Building

The Azure Kinect ROS Driver uses catkin to build. For instructions on how to build the project please see the 
[building guide](docs/building.md).

## Join Our Developer Program

Complete your developer profile [here](https://aka.ms/iwantmr) to get connected with our Mixed Reality Developer Program. You will receive the latest on our developer tools, events, and early access offers.

## Code of Conduct

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/).
For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or
contact [opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.

## Reporting Security Issues
Security issues and bugs should be reported privately, via email, to the
Microsoft Security Response Center (MSRC) at <[secure@microsoft.com](mailto:secure@microsoft.com)>.
You should receive a response within 24 hours. If for some reason you do not, please follow up via
email to ensure we received your original message. Further information, including the
[MSRC PGP](https://technet.microsoft.com/en-us/security/dn606155) key, can be found in the
[Security TechCenter](https://technet.microsoft.com/en-us/security/default).

## License

[MIT License](LICENSE)
