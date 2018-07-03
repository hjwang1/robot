# 1.视觉感知
> 一般性的物体识别、分割、曲面重建，Research about robot perceptions. Author: *hjwang1@163.com*, 此视觉感知算法，可以做3D人脸识别、手势识别，可延及一般性的物体识别、分割、曲面重建;这里开源的是视觉感知算法-学术研发版本，可达到实时性的商业版不在这里开源，有商业需求的朋友可以邮件联系我们。


#### 1.1.本套算法是基于3D Sensor之上的视觉感知算法：
* 输入：PLY格式的物体或场景点云数据
* 中间输出：算法运行中会临时输出一些三角剖分的物体mesh数据，供分析、可视化查看
* 输出：JSON格式的视觉感知数据

#### 1.2.点云数据由3D Sensor采集而来：
* 3D结构光Sensor
* TOF Sensor
* 双目视觉Sensor

#### 1.3.人脸曲面重建(Surface reconstruction)
![image](https://raw.githubusercontent.com/hjwang1/robot/master/img/snapshotHead00.png)
![image](https://raw.githubusercontent.com/hjwang1/robot/master/img/snapshotHead02.png)

#### 1.4.手势曲面重建(Surface reconstruction)
![image](https://raw.githubusercontent.com/hjwang1/robot/master/img/snapshotopen00.png)

#### 1.5.一般性物体曲面重建(Surface reconstruction)
![image](https://raw.githubusercontent.com/hjwang1/robot/master/img/chair.jpg)

#### 1.6.视觉感知结果
![image](https://raw.githubusercontent.com/hjwang1/robot/master/img/perceptions.png)
* 不仅仅可以做3D人脸识别、手势识别，可延及一般性的物体识别、分类
> 备注：上图中，红色曲线代表“一位妈妈的3D Face”，蓝色曲线代表“妈妈的孩子的3D Face”，曲线不同代表不同的人脸，绿色曲线代表玩具篮球，篮球的曲线与两条人脸曲线差距很大，从而可以把物体分类，人脸曲线的不同代表了不同的人，但是分布又比较近，可见是同一类。

#### 1.7.算法的概要处理流程
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
* vcglib下载安装，可参考本目录下的子目录vcglib，里有详细说明
* pcl，本目录下的子目录distrib里自带了1.8版本, BSD license
* jsoncpp，本目录下的子目录distrib里自带了1.8版本，MIT license
* openmesh，本目录下的子目录distrib里自带了6.3版本，BSD license
* eigen3下载安装，可参考本目录下的子目录eigen3，里有详细说明，MPL2 license
* poisson reconstruction10.02，本目录下的子目录distrib里自带了，MIT license
* libaiethan.a，视觉感知的必要数据结构与基本算法库
* libaiethantool.a，视觉感知必要算法库
* triangle，可选库libtriangle_shared

#### 2.3. 编译（compile and build）
已经自带了CMakeList.txt文件，通过cmake工具编译构建
* 进入robot目录
<pre><code>
mkdir build
cd build
cmake ..
</code></pre>
* 编译后生成可执行程序：visdetectpoisson

#### 2.4. 配置文件
###### 2.4.1.scancrawler.json
* srcdir: 待处理3D点云文件的所在目录
* src: 字符串，3D点云文件的前辍，格式是：前辍+序号+“.ply”，
* filterdir: 滤波后的3D点云文件的所在目录
* filtered: 滤波后的3D点云文件的前辍，格式是：前辍+序号+“.ply”，
* importdir: 最终处理结果JSON文件的所在目录
* import: 最终处理结果JSON文件的前辍，格式是：前辍+序号+时间戳+“.json”，
* startindex: 3D点云文件批量文件的连续序号的最小值
* endindex: 3D点云文件批量文件的连续序号的最大值
* directed_by: 3D点云文件的创造者
* id: 3D点云文件的唯一识别号
* initial_release_date: 3D点云文件的生成日期
* name: 3D点云文件所代表的物体的名称
* content: 3D点云文件所代表物体的概要描述
* lenth_d: 3D点云文件所代表物体的长度
* width_d: 3D点云文件所代表物体的宽度
* height_d: 3D点云文件所代表物体的高度
* vsize_d: 3D点云文件所代表物体的体积
* vprefix_s: 3D点云文件所代表物体的类别号
* vsuffix_s: 3D点云文件所代表物体的类别号
* shap: 3D点云文件所代表物体的形状
* color: 3D点云文件所代表物体的颜色
* material: 3D点云文件所代表物体的材质
* phase: 3D点云文件所代表物体的相态
* note: 其他备注
> 备注：对于如下的文件名“hjwhand-10004.ply”，前辍是“hjwhand-”，序号是“10004”

###### 2.4.2.scaninfo.json
* threshold: 连通点云的最小顶点数
* filter_*: 物体3D点云的三维Box切割
* distance: triangle mesh中，edge的最大长度，在部分算法中使用此值
* Niters: 对mesh平滑操作的次数，在部分算法中使用此值
* key_i: 用哪一类特征数据参与算法计算，在部分算法中使用此值
* inf_i: 最小一级视觉感知分辨率，分辨率范围[3,10]
* sup_i: 最大一级视觉感知分辨率，分辨率范围[3,10]

#### 2.5. 程序分解说明
* 程序的详细说明请参考源码注释
* 编译运行之前，请务必对源程序里的相关配置进行合理设置

#### 2.6. 运行测试
* 根据实际的待处理3D点云文件，对两个配置文件进行合理的配置
* 运行可执行程序visdetectpoisson
> 备注：运行时，要注意lib库的链接路径设置，运行完成后，会在指定目录生成与3D点云文件相对应的视觉感知结果，以JSON文件存储感知结果

#### 2.7. 视觉感知结果分析
* 在Matlab R2013b(或以上)打开diffdis.m文件
* 在diffdis.m文件中合理设置视觉感知结果JSON文件的路径
* 在Matlab中运行diffdis.m，就可以得到如1.3节中所示的图
* 每条曲线代表一个物体，从而进一步做物体识别、分类等后续工作

# 视觉感知算法原理
