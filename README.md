# Robot-arm
## Robot arm simulation
![image](https://github.com/iron-kang/Robot-arm/blob/main/images/robot-arm-1.png)

## Requirement
* Qt Creator >= 4.5.1
* Armadillo library
```shell=
sudo apt install libopenblas-dev liblapack-dev libboost-dev
sudo apt-get install libarmadillo-dev
```
## Funtions
|<img src="https://github.com/iron-kang/Robot-arm/blob/main/resource/image/axis-dis.png" width="50%"/>|<img src="https://github.com/iron-kang/Robot-arm/blob/main/resource/image/home.png" width="50%"/>|<img src="https://github.com/iron-kang/Robot-arm/blob/main/resource/image/record.png" width="50%"/>|<img src="https://github.com/iron-kang/Robot-arm/blob/main/resource/image/stop.png" width="50%"/>|<img src="https://github.com/iron-kang/Robot-arm/blob/main/resource/image/xbox-dis.png" width="50%"/>|
|:---:|:---:|:---:|:---:|:---:|
|<center>Display 3D Cartesian coordinate</center>|<center>Home return</center>|Record waypoint|Replay record|Joystick control|

## Kinematics
### D-H table
|$Link$|${a_i}$|${\alpha_i}$|${d_i}$|${\theta_i}$|
|:---:|:---:|:---:|:---:|:---:|
|1|12|90|61.5|${\theta_1}$|
|2|94|0|0|${\theta_2}$|
|3|89.98|0|0|${\theta_3}$|
|4|27.2|90|0|${\theta_4}$|
|5|0|0|46.66|${\theta_5}$|
|6|0|0|90|0|
