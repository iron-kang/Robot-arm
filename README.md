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
1. Move vertically and Joint 1 rotation
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
<img src="https://github.com/iron-kang/Robot-arm/blob/main/images/3d_dimensional.png" width="70%"/>

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
![](http://latex.codecogs.com/svg.latex?{A_4}=\\begin{bmatrix}cos{\\theta_4}&0&sin\\theta_4&a_4cos\\theta_4\\\\sin\\theta_4&0&-cos\\theta_4&a_4sin\\theta_4\\\\0&1&0&0\\\\0&0&0&1\\\\\end{bmatrix}) ![](http://latex.codecogs.com/svg.latex?{A_5}=\\begin{bmatrix}cos{\\theta_5}&-sin\\theta_5&0&0\\\\sin\\theta_5&cos\\theta_5&0&0\\\\0&0&1&d_5\\\\0&0&0&1\\\\\end{bmatrix}) ![](http://latex.codecogs.com/svg.latex?{A_6}=\\begin{bmatrix}1&0&0&0\\\\0&1&0&0\\\\0&0&1&d_6\\\\0&0&0&1\\\\\end{bmatrix})

### Inverse kinematics

![](http://latex.codecogs.com/svg.latex?O_6=\\begin{bmatrix}x_6\\\y_6\\\\z_6\\\\\end{bmatrix})
![](http://latex.codecogs.com/svg.latex?R=\\begin{bmatrix}R_{11}&R_{12}&R_{13}\\\R_{21}&R_{22}&R_{23}\\\\R_{31}&R_{32}&R_{33}\\\\\end{bmatrix})

![](http://latex.codecogs.com/svg.latex?P_4=O_6-(d_5+d_6)R\\begin{bmatrix}0\\\0\\\\1\\\\\end{bmatrix}=\\begin{bmatrix}x_6-(d_5+d_6)R_{02}\\\\y_6-(d_5+d_6)R_{12}\\\\z_6-(d_5+d_6)R_{22}\\\\\end{bmatrix})

![](http://latex.codecogs.com/svg.latex?A^{1}_{3}A_4=\\begin{bmatrix}cos\\theta_1cos(\\theta_2+\\theta_3)&-cos\\theta_1sin(\\theta_2+\\theta_3)&sint\\theta_1&x_c\\\sin\\theta_1cos(\\theta_2+\\theta_3)&-sin\\theta_1sin(\\theta_2+\\theta_3)&-cos\\theta_1&y_c\\\\sin(\\theta_2+\\theta_3)&cos(\\theta_2+\\theta_3)&0&z_c\\\\0&0&0&1\\\\\end{bmatrix}\\begin{bmatrix}cos{\\theta_4}&0&sin\\theta_4&a_4cos\\theta_4\\\\sin\\theta_4&0&-cos\\theta_4&a_4sin\\theta_4\\\\0&1&0&0\\\\0&0&0&1\\\\\end{bmatrix})

![](http://latex.codecogs.com/svg.latex?\\begin{cases}-cos\\theta_1sin(\\theta_2+\\theta_3+\\theta_4)=R_{02}\\\\-sin\\theta_1sin(\\theta_2+\\theta_3+\\theta_4)=R_{12}\\\\cos(\\theta_2+\\theta_3+\\theta_4)=R_{22}\\end{cases}=>\\begin{cases}sin(\\theta_2+\\theta_3+\\theta_4)=\\sqrt{R_{02}^2+R_{12}^2}\\\\\\theta_1=atan2(R_{12},R_{02})\\\\cos\\theta_1cos(\\theta_2+\\theta_3+\\theta_4)=cos\\theta_1R_{22}\\\\sin\\theta_1cos(\\theta_2+\\theta_3+\\theta_4)=sin\\theta_1R_{22}\\end{cases})

![](http://latex.codecogs.com/svg.latex?\\begin{bmatrix}x_c\\\y_c\\\\z_c\\\\\end{bmatrix}=\\begin{bmatrix}x_6-(d_5+d_6)R_{02}-a_4cos\\theta_1cos(\\theta_2+\\theta_3+\\theta_4)\\\y_6-(d_5+d_6)R_{12}-a_4sin\\theta_1cos(\\theta_2+\\theta_3+\\theta_4)\\\\z_6-(d_5+d_6)R_{22}-a_4sin(\\theta_2+\\theta_3+\\theta_4)\\\\\end{bmatrix})

![](http://latex.codecogs.com/svg.latex?D=(x_c-a_1cos\\theta_1)^2+(y_c-a_1sin\\theta_1)^2+(z_c-d_1)^2)

![](http://latex.codecogs.com/svg.latex?\\theta_1=atan2(y_c,x_c))

![](http://latex.codecogs.com/svg.latex?\\theta_3=cos^{-1}({D^{2}-(a_2^2+a_3^2)\over2a_2a_3}))

![](http://latex.codecogs.com/svg.latex?\\theta_2=atan2(p_z,\\sqrt{p_x^2+p_y^2})-atan2(a_3sin\\theta_3,a_2+a_3cos\\theta_3))

![](http://latex.codecogs.com/svg.latex?R^{1}_{3}=\\begin{bmatrix}cos\\theta_1cos(\\theta_2+\\theta_3)&-cos\\theta_1sin(\\theta_2+\\theta_3)&sin\\theta_1\\\sin\\theta_1cos(\\theta_2+\\theta_3)&-sin\\theta_1sin(\\theta_2+\\theta_3)&-cos\\theta_1\\\\sin(\\theta_2+\\theta_3)&cos(\\theta_2+\\theta_3)&0\\\\\end{bmatrix})

![](http://latex.codecogs.com/svg.latex?R^{3}_{6}={R^{1}_{3}}^{T}R=\\begin{bmatrix}cos\\theta_4cos\\theta_5&-cos\\theta_4sin\\theta_5&sin\\theta_4\\\\sin\\theta_4cos\\theta_5&-sin\\theta_4sin\\theta_5&-cos\\theta_4\\\\sin\\theta_5&cos\\theta_5&0\\\\\end{bmatrix}=\\begin{bmatrix}cos\\theta_1cos(\\theta_2+\\theta_3)&sin\\theta_1cos(\\theta_2+\\theta_3)&sin(\\theta_2+\\theta_3)\\\\-cos\\theta_1sin(\\theta_2+\\theta_3)&-sin\\theta_1sin(\\theta_2+\\theta_3)&cos(\\theta_2+\\theta_3)\\\\sin\\theta_1&-cos\\theta_1&0\\\\\end{bmatrix}\\begin{bmatrix}R_{11}&R_{12}&R_{13}\\\R_{21}&R_{22}&R_{23}\\\\R_{31}&R_{32}&R_{33}\\\\\end{bmatrix})

![](http://latex.codecogs.com/svg.latex?\\theta_4=cos^{-1}(cos\\theta_1sin(\\theta_2+\\theta_3)R_{13}+sin\\theta_1sin(\\theta_2+\\theta_3)R_{23}-cos(\\theta_2+\\theta_3)R_{33}))

![](http://latex.codecogs.com/svg.latex?\\theta_5=sin^{-1}(sin\\theta_1R_{11}-cos\\theta_1R_{21}))
