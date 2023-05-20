## 上场说明

### 角度标定

​	角度标定了自动瞄准才可能能用

- 将红点瞄准前哨站， 记录当前距离和角度
- 将红点瞄准前哨站左边缘，记录当前距离和角度
- 将红点瞄准前哨站右边缘，记录当前距离和角度
- 将红点瞄准基地， 记录当前距离和角度
- 将红点瞄准基地左边缘， 记录当前距离和角度
- 将红点瞄准基地右边缘， 记录当前距离和角度

### 发射准备操作

​	装填序列：R0 R1 B0  <------飞镖架发射方向

- PT：当前发射架pitch角度
- FT：当前摩擦轮转速目标值
- YL：当前解算后发射架yaw偏角
- DS：测距仪返回值(若始终为零先用手挡在前方20cm处， 若更新解决， 若不更新需关开电池)

1. 将飞镖架搬至飞镖发射站
2. 打开电池并将发射站滑台推回发射站主体内
3. 使用调试模式将红点瞄准待击打目标中心
4. 可用测距值校准摆放位置是否与上次一致
5. 调整到上场模式， 更新测距解算摩擦轮转速
6. 将yaw偏置向右调整0.4度， 即开关8下
7. 根据情况调整转速偏置
8. 使用大于60fps录制落点视频用于校射

## 操作说明

![1815493-20191011175632763-327473721](README.assets/1815493-20191011175632763-327473721.png)

![1815493-20191011175632763-327473720](README.assets/1815493-20191011175632763-327473720.png)

### S1左拨杆拨上(1) 为上场模式， 

#### 摩擦轮转速控制

- ch0右摇杆右方向拨到底 + S2右拨杆由任意方向向中切换→更新测距解算摩擦轮转速；

- S2右拨杆由下向中切换→摩擦轮偏置转速-10rpm， （偏置为零时即为测距解算摩擦轮转速， 未更新时默认6000转）
- S2右拨杆由上向中切换→摩擦轮偏置转速+10rpm； 
- ch3左摇杆下方向拨到底 + S2右拨杆由任意方向向中切换→摩擦轮转速偏置归零；

#### yaw轴控制

- ch2左摇杆左右方向拨到底 + S2右拨杆由任意方向向中切换→对应方向yaw角度+-0.05°(解算后发射架角度)；  
- ch3左摇杆上方向拨到底 + S2右拨杆由任意方向向中切换→yaw轴方向重置为S1左拨杆切换为上场模式时的yaw； 
- ditl拨轮向上启动朝向基地的辅助瞄准(未完成标定), 拨轮向下启动朝向前哨站的辅助瞄准(该操作尚未完成标定先不使用)。

### S1左拨杆拨中(3)为零电流模式，

 即所有电机都不会有驱动电流， 任意电机的角度可以被随意拨动。

![1815493-20191011175632763-327473719](README.assets/1815493-20191011175632763-327473719.png)

### S1左拨杆拨下(2)为调试模式， 

- ch2左摇杆左右控制发射架yaw， 
- ch1右摇杆上下控制输弹机装填； 
- S2右拨杆拨中摩擦轮零电流； 
- S2右拨杆拨上摩擦轮低速旋转用于调试输弹机， 此时ditl拨轮向上摩擦轮正转可以弹出飞镖，向下摩擦轮反转可以装填飞镖； 
- S2右拨杆拨下摩擦轮高速旋转用于发射飞镖调试， 此时ditl拨轮向上摩擦轮转速升高， 反之降低。