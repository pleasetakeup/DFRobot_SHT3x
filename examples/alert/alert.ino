/*!
  *@file alert.ino
  *@brief 温湿度超阈值报警
  *@n 实验现象:用户自定义设置温度和湿度的阈值，当温湿度超出了自定义的阈值时，ALERT引脚就会产生
  *@n 报警信号
  *@n 使用注意：在使用此功能时应当将传感器上的ALERT引脚与Arduino上的中断引脚pin 2或者pin 3相连
  *@copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  *@licence     The MIT License (MIT)
  *@author [fengli](li.feng@dfrobot.com)
  *@version  V1.0
  *@date  2019-08-26
  *@get from https://www.dfrobot.com
  *@url https://github.com/DFRobot/DFRobot_SHT3x
*/

#include <DFRobot_SHT3x.h>
  /*!
   * @brief 构造函数
   * @param pWire I2C总线指针对象，构造设备，可传参数也可不传参数，默认Wire。
   * @param address 芯片IIC地址,共有两个可选地址0x44、0x45(默认为0x44)。
   * @param RST 芯片复位引脚，默认为4.
   * @n IIC地址是由芯片上的引脚addr决定。
   * @n 当addr与VDD连接,芯片IIC地址为：0x45。
   * @n 当addr与VSS连接,芯片IIC地址为：0x44。
   */
#define RST  4
//DFRobot_SHT3x sht3x(&Wire,0x44,RST);

DFRobot_SHT3x sht3x;
//alert引脚的非报警状态为低电平
volatile  int alertState = 0;
void alert(){
  alertState = 1 - alertState;
}
void setup() {

  Serial.begin(9600);
  //使用arduino中断,使用中断0号脚(即是数字引脚2)，,CHANGE表示在电平改变时就会产生中断
  attachInterrupt(0,alert,CHANGE);
    //初始化芯片,检测是否能正常通信

  while (sht3x.begin() != 0) {
    Serial.println("初始化芯片失败，请确认芯片连线是否正确");
    delay(1000);
  }
  /**
   * softReset：通过IIC发送命令复位，进入芯片的默认模式单次测量模式，关闭加热器，并清除ALERT引脚的警报。
   * @return 通过读取状态寄存器来判断命令是否成功被执行，返回true则表示成功
   */
  if(!sht3x.softReset()){
     Serial.println("芯片复位失败....");
   }
  /**
   * @brief All flags (Bit 15, 11, 10, 4) in the status register can be cleared (set to zero)
   * @n  把bit：15 设置为0后ALERT引脚才能正常工作，否则将一直处于高电平。
   */
  sht3x.clearStatusRegister();
  /**
   * heaterEnable()： 打开芯片里面的加热器.作用是使传感器在潮湿的环境也能有准确的湿度数据
   * @return 通过读取状态寄存器来判断命令是否成功被执行，返回true则表示成功
   */
  if(!sht3x.heaterEnable()){
     Serial.println("加热器打开失败....");
  }
  /**
   * setMeasurementMode ：进入周期测量模式，并设置可重复性、读取频率，只有在此模式下ALERT才能工作。
   * @param repeatability 读取温湿度数据的可重复性，eRepeatability_t类型的数据
   * @note  可选择的参数：
               eRepeatability_High /**高可重复性模式下，湿度的可重复性为0.10%RH，温度的可重复性为0.06°C
               eRepeatability_Medium,/**中等可重复性模式下，湿度的可重复性为0.15%RH，温度的可重复性为0.12°C
               eRepeatability_Low, /**低可重复性模式下，湿度的可重复性为0.25%RH，温度的可重复性为0.24°C
   * @param measureFreq   读取数据的频率，eMeasureFrequency_t类型的数据
   * @note  可选择的参数：
               eMeasureFreq_Hz5,   /**芯片每2秒采集一次数据
               eMeasureFreq_1Hz,   /**芯片每1秒采集一次数据
               eMeasureFreq_2Hz,   /**芯片每0.5秒采集一次数据
               eMeasureFreq_4Hz,   /**芯片每0.25采集一次数据
               eMeasureFreq_10Hz   /**芯片每0.1采集一次数据
   * @return 通过读取状态寄存器来判断命令是否成功被执行，返回true则表示成功
   */          
  sht3x.setMeasurementMode(sht3x.eRepeatability_High,sht3x.eMeasureFreq_10Hz);
  /**
   * setTemperatureLimitC:设置温度阈值温度和警报清除温度
   * @param highset 高温报警点，当温度大于此值时ALERT引脚产生报警信号。
   * @param highClear 高温警报清除点，当温度大于highset产生报警信号，而温度小于此值报警信号则被清除。
   * @param lowclear 低温警报清除点，当温度小于lowset产生报警信号，而温度大于此值时报警信号则被清除。
   * @param lowset 低温报警点，当温度小于此值时ALERT引脚产生报警信号。
   * @note 填入的数值应该为整数。 
   */
  sht3x.setTemperatureLimitC(35,34,20,18);
  /**
   * setHumidityLimitRH: 设置相对湿度阈值温度和警报清除湿度
   * @param highset 高湿度报警点，当相对湿度大于此值时ALERT引脚产生报警信号。
   * @param highClear 高湿度警报清除点，当相对湿度于highset产生报警信号，而相对湿度小于此值报警信号则被清除。
   * @param lowclear 低湿度警报清除点，当相对湿度小于lowset产生报警信号，而相对湿度大于此值时报警信号则被清除。
   * @param lowset 低湿度报警点，当相对湿度小于此值时ALERT引脚产生报警信号。
   * @note 填入的数值应该为整数。 
   */
  sht3x.setHumidityLimitRH(60,58,20,19);
  Serial.println("----------------------警报检测-------------------------------");
  Serial.println("--------------当温湿度超出阈值范围就会产生警报---------------");
  Serial.println("----------------------温度限制(°C)---------------------------");
  /**
   * @brief 读取相对湿度阈值温度和警报清除湿度
   * @return slimitData_t类型的结构体里面包含了高湿度报警点、高湿度警报清除点、低湿度警报清除点、低湿度报警点,状态码
   */
  DFRobot_SHT3x::slimitData_t humidityLimit=sht3x.readHumidityLimitRH();
  Serial.print("high set:");
  Serial.print(humidityLimit.highSet);
  Serial.print("               low clear:");
  Serial.println(humidityLimit.lowClear);
  Serial.print("high clear:");
  Serial.print(humidityLimit.highClear);
  Serial.print("               low set:");
  Serial.println(humidityLimit.lowSet);
   
  /**
   * @brief 读取温度阈值温度和警报清除温度
   * @return slimitData_t类型的结构体里面包含了高温报警点、高温警报清除点、低温警报清除点、低温报警点,状态码
   */
  Serial.println("----------------------湿度限制(%RH)--------------------------");
  DFRobot_SHT3x::slimitData_t temperatureLimit=sht3x.readTemperatureLimitC();
  Serial.print("high set:");
  Serial.print(temperatureLimit.highSet);
  Serial.print("               low clear:");
  Serial.println(temperatureLimit.lowClear);
  Serial.print("high clear:");
  Serial.print(temperatureLimit.highClear);
  Serial.print("               low set:");
  Serial.println(temperatureLimit.lowSet);
  Serial.println("--------------------------------------------------------");
  /**
   * readAlertState: 读取ALERT引脚的状态.
   * @return 高电平则返回1，低电平则返回0.
   */
   //此判断的作用是，初始化ALERT的状态
  if(readAlertState() == 1){
    alertState==1;
  } else {
    alertState==0;
  }
}
void loop() {
  /**
   * @brief 在周期测量模式下获取温湿度数据.
   * @return  返回包含有温度(°C)、湿度(%RH)、状态码的结构体.
   * @n 状态码为0则表明数据正确.
   */
  DFRobot_SHT3x::sRHAndTemp_t data=sht3x.readTempAndHumidity();
  if(data.ERR == 0){
    Serial.print("环境温度(°C):");
    Serial.print(data.TemperatureC);
    Serial.print(" C        ");
    Serial.print("相对湿度(%RH):");
    Serial.print(data.Humidity);
    Serial.println(" %RH");
  }
  //读取数据的频率应该大于芯片采集数据的频率，否则返回的数据就会出错。
  delay(1000);
  if(alertState==1){
    Serial.println("温度或湿度超出阈值范围");
  }
  else{
    Serial.println("温湿度处于正常范围,警报已清除");
  }
}
