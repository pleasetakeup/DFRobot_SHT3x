/*!
 * @file DFRobot_SHT3x.h
 * @brief 定义DFRobot_SHT3x 类的基础结构
 * @n 这是一个数字温度湿度传感器的库，用来驱动SHT3x系列SHT30、SHT31和SHT35，读取环境温度
 * @n 和相对湿度。
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [fengli](li.feng@dfrobot.com)
 * @version  V1.0
 * @date  2019-08-19
 * @get from https://www.dfrobot.com
 * @url https://github.com/DFRobot/DFRobot_SHT3x
 */
 
#ifndef DFROBOT_SHT3X_H
#define DFROBOT_SHT3X_H
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <Wire.h>



#define ENABLE_DBG

#ifdef ENABLE_DBG
#define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define DBG(...)
#endif
#define CMD_READ_SERIAL_NUMBER             (0x3780)// 读取芯片序列号
#define CMD_GETDATA_H_CLOCKENBLED          (0x2C06)// measurement:high repeatability
#define CMD_GETDATA_M_CLOCKENBLED          (0x2C0D)//measurement: medium repeatability
#define CMD_GETDATA_L_CLOCKENBLED          (0x2C10)// measurement: low repeatability

#define CMD_SETMODE_H_FREQUENCY_HALF_HZ    (0x2032)//measurement: periodic 0.5 mps, high repeatability
#define CMD_SETMODE_M_FREQUENCY_HALF_HZ    (0x2024)// measurement: periodic 0.5 mps, medium
#define CMD_SETMODE_L_FREQUENCY_HALF_HZ    (0x202F)//measurement: periodic 0.5 mps, low repeatability
#define CMD_SETMODE_H_FREQUENCY_1_HZ       (0x2130)// measurement: periodic 1 mps, high repeatability
#define CMD_SETMODE_M_FREQUENCY_1_HZ       (0x2126)// measurement: periodic 1 mps, medium repeatability
#define CMD_SETMODE_L_FREQUENCY_1_HZ       (0x212D)// measurement: periodic 1 mps, low repeatability
#define CMD_SETMODE_H_FREQUENCY_2_HZ       (0x2236)// measurement: periodic 2 mps, high repeatability
#define CMD_SETMODE_M_FREQUENCY_2_HZ       (0x2220)// measurement: periodic 2 mps, medium repeatability
#define CMD_SETMODE_L_FREQUENCY_2_HZ       (0x222B)// measurement: periodic 2 mps, low repeatability
#define CMD_SETMODE_H_FREQUENCY_4_HZ       (0x2334)// measurement: periodic 4 mps, high repeatability
#define CMD_SETMODE_M_FREQUENCY_4_HZ       (0x2322)// measurement: periodic 4 mps, medium repeatability
#define CMD_SETMODE_L_FREQUENCY_4_HZ       (0x2329)// measurement: periodic 4 mps, low repeatability
#define CMD_SETMODE_H_FREQUENCY_10_HZ      (0x2737)// measurement: periodic 10 mps, high repeatability
#define CMD_SETMODE_M_FREQUENCY_10_HZ      (0x2721)// measurement: periodic 10 mps, medium
#define CMD_SETMODE_L_FREQUENCY_10_HZ      (0x272A)// measurement: periodic 10 mps, low repeatability
#define CMD_GETDATA                        (0xE000)// readout measurements for periodic mode

#define CMD_STOP_PERIODIC_ACQUISITION_MODE (0x3093)
#define CMD_SOFT_RESET                     (0x30A2)// soft reset
#define CMD_HEATER_ENABLE                  (0x306D)// enabled heater
#define CMD_HEATER_DISABLE               (0x3066)// disable heater
#define CMD_READ_STATUS_REG                (0xF32D)// read status register
#define CMD_CLEAR_STATUS_REG               (0x3041)// clear status register

#define CMD_READ_HIGH_ALERT_LIMIT_SET      (0xE11F)// read alert limits, high set
#define CMD_READ_HIGH_ALERT_LIMIT_CLEAR    (0xE114)// read alert limits, high clear
#define CMD_READ_LOW_ALERT_LIMIT_CLEAR     (0xE109)// read alert limits, low clear
#define CMD_READ_LOW_ALERT_LIMIT_SET       (0xE102)// read alert limits, low set
#define CMD_WRITE_HIGH_ALERT_LIMIT_SET     (0x611D)// write alert limits, high set
#define CMD_WRITE_HIGH_ALERT_LIMIT_CLEAR   (0x6116)// write alert limits, high clear
#define CMD_WRITE_LOW_ALERT_LIMIT_CLEAR    (0x610B)// write alert limits, low clear
#define CMD_WRITE_LOW_ALERT_LIMIT_SET      (0x6100)// write alert limits, low set
class DFRobot_SHT3x
{
public:
  #define ERR_OK             0      //无错误
  #define ERR_DATA_BUS      -1      //数据总线错误
  #define ERR_IC_VERSION    -2      //芯片版本不匹配
  
  /*!
   The status register contains information on the operational status of the heater, the alert mode and on the execution status of 
   the last command and the last write sequence. 
   
   ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
            b15       |       b14 |      b13   |      b12 |        b11   |        b10       |  b5~b9    |    b4              |    b2~b3 |    b1       |       b0              |
   ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    alertPendingStatus| reserved3 | heaterStaus|reserved2 |humidityAlert | temperatureAlert | reserved1 | systemResetDeteced |reserved0 |commendStatus|writeDataChecksumStatus|
   ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   writeDataChecksumStatus:   '0' : checksum of last write transfer wascorrect
                              '1' : checksum of last write transfer failed
   commendStatus              '0' : last command executed successfully
                              '1' : last command not processed. 
   systemResetDeteced         '0' : no reset detected since last ‘clear status register’ command
                              '1' : reset detected (hard reset, soft reset command or supply fail)
   temperatureAlert           '0' : no alert
                              '1' : alert
   humidityAlert              '0' : no alert
                              '1' : alert
   heaterStaus                '0' : Heater OFF
                              '1' : Heater ON
   alertPendingStatus         '0' : no pending alerts
                              '1' : at least one pending alert
  */
  typedef struct {
    uint8_t writeDataChecksumStatus : 1;
    uint8_t commendStatus : 1;
    uint8_t reserved0 : 2;
    uint8_t systemResetDeteced : 1;
    uint8_t reserved1 : 5;
    uint8_t temperatureAlert : 1;
    uint8_t humidityAlert : 1;
    uint8_t reserved2 : 1;
    uint8_t heaterStaus :1;
    uint8_t reserved3 :1;
    uint8_t alertPendingStatus :1;
  }__attribute__ ((packed)) sStatusRegister_t;
  /*!
    我们可以选择芯片测量温湿度数据的可重复性，可供选择中、高、低三档的可重复性。
  */
  typedef enum{
    eRepeatability_High,/**<高可重复性模式下，湿度的可重复性为0.10%RH，温度的可重复性为0.06°C*/
    eRepeatability_Medium,/**<中等可重复性模式下，湿度的可重复性为0.15%RH，温度的可重复性为0.12°C*/
    eRepeatability_Low,/**<低可重复性模式下，湿度的可重复性为0.25%RH，温度的可重复性为0.24°C*/
  } eRepeatability_t;
  /*!
    在the periodic data acquisition 模式下，我们可以选择芯片测量温湿度数据的频率，
    可供选择的频率有，0.5Hz、1Hz、2Hz、4Hz、10Hz.
   */
  typedef enum{
    eMeasureFreq_Hz5,
    eMeasureFreq_1Hz,
    eMeasureFreq_2Hz,
    eMeasureFreq_4Hz,
    eMeasureFreq_10Hz
  } eMeasureFrequency_t;

  /**
    用来存储温度和相对湿度的结构体
  */
  typedef struct{
    float TemperatureC;
    float Humidity;
    float TemperatureF;
    int ERR;
  }sRHAndTemp_t;
  typedef struct{
    float highSet;/**<自定义温度(C)/湿度(%RH)范围上阈值，大于此值时会ALERT会产生高电平报警>*/
    float highClear;/**<小于此温度(C)/湿度(%RH)值报警信号则会清除>*/
    float lowClear;/**<大于此温度(C)/湿度(%RH)值报警信号则会清除>*/
    float lowSet;/**<自定义温度(C)/湿度(%RH)范围下阈值，小于此值时会ALERT会产生高电平报警>*/
    int  ERR;
  } slimitData_t;
public:
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
   * @brief 进入周期测量模式，并设置可重复性(芯片在两次相同测量条件下测量到的数据的差值)、读取频率。
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

private:

  /**
   * @brief 向传感器芯片写指令.
   * @param cmd  芯片指令.
   * @param size  指令数据的个数，八位为一个数据.
   * @return 返回0表示发送指令成功，返回其他值表示未能成功发送
   */
  uint8_t  writeCommand(uint16_t cmd,size_t size);
  /**
   * @brief 读取状态寄存器里面的数据.
   * @return  返回包含如加热器是否开启，alert引脚的状态，重置状态等，前一次命令是否执行。
   */
  sStatusRegister_t readStatusRegister();
  
  /**
   * @brief 写入阈值数据.
   * @param cmd  发送阈值数据的芯片指令.
   * @param limitData 需要发送的温度和湿度的原始数据(湿度占7位，温度占11位).
   * @return 返回0表示发送指令成功，返回其他值表示未能成功发送
   */
  uint8_t writeLimitData(uint16_t cmd,uint16_t limitData);
  
  /**
   * @brief 读取阈值数据.
   * @param cmd  读取阈值数据的芯片指令.
   * @param *pBuf 存储读到的数据
   * @return 返回0表示返回数据正确否则错误
   */
  uint8_t readLimitData(uint16_t cmd,uint16_t *pBuf);
  /**
   * @brief 向传感器芯片写指令.
   * @param pBuf  指令中包含的数据.
   * @param size  指令数据的个数.
   * @return  返回0表示读取成功，返回其他值表示未能正确读取.
   */
  uint8_t readData(void *pBuf,size_t size);
  
  /**
   * @brief CRC校验.
   * @param data[] 需要校验的数据.
   * @return 得到的校验码.
   */
  uint8_t checkCrc(uint8_t data[]);
  
  /**
   * @brief 将从传感器返回的数据转化为摄氏温度.
   * @param 从传感器得到的温度数据.
   * @return 摄氏温度.
   */
  float convertTemperature(uint8_t rawTemperature[]);
  /**
   * @brief 将从传感器返回的数据转化为相对湿度.
   * @param 从传感器得到的数据.
   * @return 相对湿度.
   */
  float convertHumidity(uint8_t rawHumidity[]);
  
  /**
   * @brief 将要写入的温度数据转化为芯片需要的数据.
   * @param 需要写入的温度.
   * @return 写入传感器的数据.
   */
  uint16_t convertRawTemperature(float value);
  /**
   * @brief 将要写入的相对湿度数据转化为芯片需要的数据.
   * @param 需要写入的湿度.
   * @return 写入传感器的数据.
   */
  uint16_t convertRawHumidity(float value);
private:
  TwoWire *_pWire;
  uint8_t _address;
  uint8_t _RST;
};   
#endif