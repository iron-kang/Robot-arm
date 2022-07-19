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
|<center>Display 3D axis</center>|<center>Home return</center>|Record waypoint|Replay record|Joystick control|
### Joystick control
|<img src="https://github.com/iron-kang/Robot-arm/blob/main/images/xbox-controller.jpg" width="70%"/>|<img src="https://github.com/iron-kang/Robot-arm/blob/main/images/xbox-controller-back.jpg" width="70%"/>|
|:--:|:--:|
<pre>
1. Move horzontally and vertically
2. Joint 5 rotation CCW
6. Replay record
7. Joint 5 rotation CW
8. Veiwport control
10. Joint 5 Rotation / Move forward
11. Speed decrease
14. Speed increase
A. Add waypoint
B. Gripper open/close
</pre>

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
### Forward kinematics

![](http://latex.codecogs.com/svg.latex?{A_1}=\\begin{bmatrix}cos{\\theta_1}&0&sin\\theta_1&a_1cos\\theta_1\\\\sin\\theta_1&0&-cos\\theta_1&a_1sin\\theta_1\\\\0&1&0&d_1\\\\0&0&0&1\\\\\end{bmatrix}) ![](http://latex.codecogs.com/svg.latex?{A_2}=\\begin{bmatrix}cos{\\theta_2}&-sin\\theta_2&0&a_2cos\\theta_2\\\\sin\\theta_2&cos\\theta_2&0&a_2sin\\theta_2\\\\0&0&1&0\\\\0&0&0&1\\\\\end{bmatrix}) ![](http://latex.codecogs.com/svg.latex?{A_3}=\\begin{bmatrix}cos{\\theta_3}&-sin\\theta_3&0&a_3cos\\theta_3\\\\sin\\theta_3&cos\\theta_3&0&a_3sin\\theta_3\\\\0&0&1&0\\\\0&0&0&1\\\\\end{bmatrix})

