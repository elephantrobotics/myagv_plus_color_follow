# MyAGV Plus 颜色跟随案例

## 1. 代码拉取
```bash
git clone https://github.com/elephantrobotics/myagv_plus_color_follow.git
```



## 2. 注意事项
在复现案例前，需要插入HDMI欺骗器，加速窗口的渲染，否则会导致窗口的渲染速度变慢，跟随效果会不佳，另外目前只支持跟随40mmX40mX40mm的绿色和黄色木块。**需要以VNC的方式访问AGV，运行案例复现指令，才能使用硬件加速画面渲染**，基于无线wifi连接的方式，VNC的跟随窗口会出现卡帧的情况，这是属实网络传输波动的问题，不影响跟随功能

## 3. 案例复现

打开终端，输入下面指令，启动底盘节点
```bash
ros2 launch myagv_plus_controller controller.launch.py
```

再打开一个新的终端，输入以下指令，启动跟随任务
```bash
cd ~/myagv_plus_color_follow
```

```bash
python color_detection.py
```

**功能说明**：
启动跟随任务后，在窗口中，框选要跟随的颜色木块后，AGV便会开始跟随，在英文输入法下，按下键盘r可取消跟随，重新框选，即可再次跟随。AGV只支持向前跟随，左平移跟随，右平移跟随，若AGV相距木块大概15cm左右，AGV便会停止运动，向后移动木块，AGV便会又开始跟随，在移动木块时，速度要缓慢一点，

**案例效果视频参考**：
https://www.bilibili.com/video/BV1Rj5X6GEua/?spm_id_from=333.337.search-card.all.click


