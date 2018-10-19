# SerialPort

## 一、简要说明

serialport串口通信封装模块，为机器人串口通信提供底层支持。属于RMdemo子模块

* 本模块参考了**UESTC Robomaster 2017代码** 。

### a. 运行环境：

* ubuntu 16.04

### b.特性

* 模块化（接口化），通信底层实现与通信协议逻辑分离。
* 支持32位定长数据包发送
* 接口松动，可自动重连

### c.文件结构

| 文件名                         | 功能说明                                        |
| ------------------------------ | ----------------------------------------------- |
| serial_port.cpp/h              | ubuntu 下通用串口功能封装（无需改动）           |
| serial_packet.cpp/h            | 串口数据包（32字节定长数据包）封装（无需改动）  |
| serial_interface.cpp/h（核心） | 协议接口封装（根据通信协议改动）                |
| serial_port_debug.cpp/h        | 串口协议通信自检测试API封装（根据通信协议改动） |

### d.数据包封装说明

* 数据包为32字节定长数据包

数据包创建：

```c++
SerialPacket sendPacket;//创建数据包
sendPacket.creatPacket(0x01);//命令指令
/***************自定义装载数据***************/
void setUncharInBuffer(unsigned char data,int locationInBuffer);//unchar类型数据
void setShortIntInBuffer(short int data,int locationInBuffer);//short int类型数据
void setIntInBuffer(int data,int locationInBuffer);//int类型数据
void setFloatInBuffer(float data,int locationInBuffer);//float类型数据
//填充数据函数，第二个参数为转载的字节位置2-30（0，31为校验字节，1为命令字节）
```

数据包解析：

```c++
/*******自定义解析数据************/
unsigned char getUncharInBuffer(int locationInBuffer);//unchar类型数据
int getIntInBuffer(int locationInBuffer);//short int类型数据
short int getShortIntInBuffer(int locationInBuffer);//int类型数据
float getFloatInBuffer(int locationInBuffer);//float类型数据
```

### c.可能出现的问题：

* 接口松动可能会引起串口号的变化，可使用udev固定串口设备路径。

## 二、基本使用

创建，及初始化：

```c++
SerialInterface serialInterface;

serialInterface.init("/dev/ttyUSB0");//串口设备路径
```

发送信息：（不经过封装）

```c++
SerialPacket sendPacket;//创建数据包
sendPacket.creatPacket(0x01);//cmd
sendPacket.setFloatInBuffer(100.2,2);//填充数据包
serialInterface.dataSend(sendPacket);//发送
```

发送信息：（经过协议封装后，以发送云台角度为例）

```c++
serialInterface.YunTaiDeltaSet(10,5);//发送pitch,yaw角度
```

接收消息：

```c++
SerialPacket recvPacket;//创建数据包
serialInterface.dataRecv(recvPacket)；
unsigned char cmd;
float data;
cmd=recvPacket.getCMD();//解析数据包命令
data=recvPacket.getFloatInBuffer(2);//解析数据包数据位
```

## 三、版权及许可证

uestc robomaster2018

- gezp email:1350824033@qq.com

SerialPort is provided under the [GPL-v3](COPYING).

