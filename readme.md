# MyAGV Plus 颜色跟随案例

## 1. 代码拉取
```bash
git clone 
```

## 2. 案例复现

打开终端，输入下面指令，启动底盘节点
```bash
ros2 launch myagv_plus_controller controller.launch.py
```

再打开一个新的终端，输入以下指令，启动跟随任务
```bash
cd 
```

```bash
python color_detection.py
```

**功能说明**：
启动跟随任务后，在窗口中，框选要跟随的颜色木块后，AGV便会开始跟随，在英文输入法下，按下键盘r可取消跟随


## 3. 注意事项
在复现案例前，需要插入HDMI欺骗器，加速窗口的渲染，否则会导致窗口的渲染速度变慢，跟随效果会不佳，另外目前只支持跟随40mmX40mX40mm的绿色和黄色木块

