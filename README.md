# DFRobot_SHT3x
SHT3x系列温湿度传感器是SHT2x系列的继承者，它包括低成本版本SHT30、标准版本SHT31，以及高端版本SHT35<br>
SHT3x系列温湿度传感器通过IIC通信，使用比较方便，工作电压范围宽(2.15至5.5 V)，DFN封装的占位面积为2.5 × 2.5 mm2，<br>
高度为0.9 mm,这有助于SHT3x集成到多种应用,适合各类应用<br>
SHT3x建立在全新和优化的CMOSens® 芯片之上，进一步提高了产品可靠性和精度规格。SHT3x提供了一系列新功能，<br>
如增强信号处理、两个独特和用户可选I2C地址、一个可编程温湿度极限的报警模式，以及高达1 MHz的通信速度。<br>
SHT3x芯片提供两种工作模式:<br>
1.单次测量模式，此模式下空闲状态电流为0.2微安(测量数据时600 微安).<br>
2.周期测量模式，此模式下空闲状态电流为45微安(测量数据时600 微安).<br>
以下是芯片典型的测量精度：<br>
版本号                | 典型温度精度 (°C)    | 典型湿度精度 (%RH)  |   -----
------------------ | :----------: | :----------: | :---------
SHT30        |    ±0.2 @0-65 °C |        ±2 @10-90% RH     |     
SHT31       |     ±0.2  @0-90 °C   |        ±2 @0-100% RH     |  
SHT35       |     ±0.1  @20-60 °C  |          ±1.5 @0-80% RH  |  

![正反面svg效果图](https://github.com/ouki-wang/DFRobot_Sensor/raw/master/resources/images/SEN0245svg1.png)

## 产品链接（链接到英文商城）
    SKU：SHT3x 温度湿度传感器
   
## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary

   1.在单次测量模式下读取环境温湿度，用户可以选择测量的可重复性(芯片在两次相同测量条件下测量到的数据的差值)<br>
   2.在周期测量模式下读取环境温湿度,用户可以选择测量的可重复性和测量频率(0.5Hz,1Hz,2Hz,4Hz,10Hz)<br>
   3.利用ALERT引脚和Arduino的中断引脚达到温湿度超阈值报警的效果，用户可自定义阈值大小<br>


## Installation

To use this library, first download the library file, paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in the folder.

## Methods

```C++
/*!
 * @brief 构造函数
 * @param pWire I2C总线指针对象，构造设备，可传参数也可不传参数，默认Wire。
 * @param address 芯片IIC地址,共有两个可选地址0x44、0x45(默认为0x44)。
 * @param RST 芯片复位引脚，默认为4.
 * @n IIC地址是由芯片上的引脚addr决定。
 * @n 当addr与VDD连接,芯片IIC地址为：0x45。
 * @n 当addr与VSS连接,芯片IIC地址为：0x44。
 */
DFRobot_SHT3x(TwoWire *pWire = &Wire, uint8_t address = 0x44,uint8_t RST = 4);

/**
 * @brief 初始化函数
 * @return 返回0表示初始化成功，返回其他值表示初始化失败，返回错误码
 */
int begin();

/**
 * @brief 通过IIC发送命令复位，进入芯片的默认模式单次测量模式，关闭加热器，并清除ALERT引脚的警报。
 * @return 通过读取状态寄存器来判断命令是否成功被执行，返回true则表示成功
 */
bool softReset();

/**
 * @brief 通过芯片的复位引脚进行复位，进入芯片的默认模式单次测量模式，并清除ALERT引脚的警报。
 * @return 状态寄存器有一数据位能检测芯片是否进行了复位，返回true则表示成功
 */
bool  pinReset();

/**
 * @brief 在单次测量模式下获取温湿度数据
 * @param repeatability 设置读取温湿度数据的可重复性，eRepeatability_t类型的数据
 * @return  返回包含有温度(°C)、湿度(%RH)、状态码的结构体
 * @n 状态码为0则表明数据正确
 */
sRHAndTemp_t readTempAndHumidity(eRepeatability_t repeatability);

/**
 * @brief 进入周期测量模式，并设置可重复性、读取频率。
 * @param repeatability 读取温湿度数据的可重复性，eRepeatability_t类型的数据
 * @param measureFreq   读取数据的频率，eMeasureFrequency_t类型的数据
 * @return 通过读取状态寄存器来判断命令是否成功被执行，返回true则表示成功
 */          
bool setMeasurementMode(eRepeatability_t repeatability,eMeasureFrequency_t measureFreq);

/**
 * @brief 在周期测量模式下获取温湿度数据.
 * @return  返回包含有温度(°C)、湿度(%RH)、状态码的结构体.
 * @n 状态码为0则表明数据正确.
 */
sRHAndTemp_t readTempAndHumidity();

/**
 * @brief 从周期读取数据模式退出。
 * @return 通过读取状态寄存器来判断命令是否成功被执行，返回true则表示成功
 */
bool stopPeriodicMode();

/**
 * @brief 打开芯片里面的加热器.
 * @return 通过读取状态寄存器来判断命令是否成功被执行，返回true则表示成功
 */
bool heaterEnable();

/**
 * @brief 关闭芯片里面的加热器.
 * @return 通过读取状态寄存器来判断命令是否成功被执行，返回true则表示成功
 */
bool heaterDisable();
/**
 * @brief All flags (Bit 15, 11, 10, 4) in the status register can be cleared (set to zero)
 * @n  把bit：15 设置为0后ALERT引脚才能正常工作，否则将一直处于高电平。
 */
void clearStatusRegister();

/**
 * @brief 设置温度阈值温度和警报清除温度
 * @param highset 高温报警点，当温度大于此值时ALERT引脚产生报警信号。
 * @param highClear 高温警报清除点，当温度大于highset产生报警信号，而温度小于此值报警信号则被清除。
 * @param lowclear 低温警报清除点，当温度小于lowset产生报警信号，而温度大于此值时报警信号则被清除。
 * @param lowset 低温报警点，当温度小于此值时ALERT引脚产生报警信号。
 * @return 返回0则表示设置成功.
 */
uint8_t  setTemperatureLimitC(float highset,float highclear,float lowclear, float lowset);

/**
 * @brief 设置相对湿度阈值温度和警报清除湿度
 * @param highset 高湿度报警点，当相对湿度大于此值时ALERT引脚产生报警信号。
 * @param highClear 高湿度警报清除点，当相对湿度于highset产生报警信号，而相对湿度小于此值报警信号则被清除。
 * @param lowclear 低湿度警报清除点，当相对湿度小于lowset产生报警信号，而相对湿度大于此值时报警信号则被清除。
 * @param lowset 低湿度报警点，当相对湿度小于此值时ALERT引脚产生报警信号。
 * @return 返回0则表示设置成功.
 */
uint8_t  setHumidityLimitRH(float highset,float highclear,float lowclear, float lowset);

/**
 * @brief 读取温度阈值温度和警报清除温度
 * @return slimitData_t类型的结构体里面包含了高温报警点、高温警报清除点、低温警报清除点、低温报警点,状态码
 */
slimitData_t readTemperatureLimitC();

/**
 * @brief 读取相对湿度阈值温度和警报清除湿度
 * @return slimitData_t类型的结构体里面包含了高湿度报警点、高湿度警报清除点、低湿度警报清除点、低湿度报警点,状态码
 */
slimitData_t readHumidityLimitRH();


```

## Compatibility

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | -----
Arduino Uno        |      √       |              |             | 
Mega2560        |      √       |              |             | 
Leonardo        |      √       |              |             | 
ESP32        |      √       |              |             | 
micro:bit        |      √       |              |             | 


## History

- data 2019-8-25
- version V0.1


## Credits

Written by fengli(li.feng@dfrobot.com), 2019.8.25 (Welcome to our [website](https://www.dfrobot.com/))





