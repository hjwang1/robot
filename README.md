# 1.视觉感知
> Research about robot perceptions. Author: *hjwang1@163.com*, 此视觉感知算法，可以做3D人脸识别、手势识别，可延及一般性的物体识别、分割、曲面重建;这里开源的是视觉感知算法-学术研发版本，可达到实时性的商业版不在这里开源，有商业需求的朋友可以邮件联系我们。


#### 1.1.本套算法是基于3D Sensor之上的视觉感知算法：
* 输入：PLY格式的物体或场景点云数据
* 中间输出：算法运行中会临时输出一些三角剖分的物体mesh数据，供分析、可视化查看
* 输出：JSON格式的视觉感知数据

#### 1.2.点云数据由3D Sensor采集而来：
* 3D结构光Sensor
* TOF Sensor
* 双目视觉Sensor

#### 1.3.视觉感知结果
![image](https://raw.githubusercontent.com/hjwang1/robot/master/img/perceptions.png)

#### 1.4.算法的概要处理流程
* 已经拿到物体或场景的三维点云数据，格式是PLY
* 降噪滤波等预处理
* 曲面triangle mesh重建
* 多分辨率视觉感知处理
* matlab可视化查看处理结果
> 备注：本人是用Orbbec的3D结构光Sensor采集，3D Sensor与OPPO Find X手机同款，也可以使用微软的kinect、Intel的RealSense等，也可以用业界公开的点云数据进行测试使用；曲面重建的方法可以用Delauney三角剖分、Poisson等方法，有能力研究者可以使用更好的曲面重建算法自行替代；matlab可视化查看源码是简单些，有特别需求的研究者可自行根据所求编写。


# 2.使用步骤
#### 2.1. 准备编译环境
* ubuntu16.04

#### 2.2. 必要与可选库的安装
* vcglib下载，可参考本目录下的子目录vcglib，里有详细说明
* pcl，本目录下的子目录distrib里自带了1.8版本, BSD license
* jsoncpp，本目录下的子目录distrib里自带了1.8版本，MIT license
* openmesh，本目录下的子目录distrib里自带了6.3版本，BSD license
* eigen3，本目录下的子目录distrib里自带了，MPL2 license

#### 2.3. 编译（compile and build）
已经自带了CMakeList.txt文件，通过cmake工具编译构建
<p>进入robot目录</p>
> <pre><code>mkdir build
cd build
cmake ..
</code></pre>

# 视觉感知算法原理
