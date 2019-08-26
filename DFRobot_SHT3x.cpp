/*!
 * @file DFRobot_SHT3x.h
 * @brief 定义DFRobot_SHT3x 类的基础结构，基础方法的实现
 * 
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [fengli](li.feng@dfrobot.com)
 * @version  V1.0
 * @date  2019-08-20
 * @get from https://www.dfrobot.com
 * @url https://github.com/DFRobot/DFRobot_SHT3x
 */

#include <DFRobot_SHT3x.h>
#include"math.h"
DFRobot_SHT3x::DFRobot_SHT3x(TwoWire *pWire, uint8_t address,uint8_t RST)
{
  _pWire = pWire;
  _address = address;
  _RST = RST;
}

int DFRobot_SHT3x::begin() 
{
  _pWire->begin();
  uint8_t data[2];
  writeCommand(CMD_READ_SERIAL_NUMBER,2);
  if(readData(data,2) != 2){
    DBG("");
    DBG("bus data access error"); DBG("");
    return ERR_DATA_BUS;
   }
  return ERR_OK;
}

bool DFRobot_SHT3x::softReset(){
  writeCommand(CMD_READ_SERIAL_NUMBER,2);
  DFRobot_SHT3x::sStatusRegister_t registerRaw;
  delay(10);
  registerRaw = readStatusRegister();
  if(registerRaw.commendStatus == 0)
    return true;
  else 
    return false;
}
bool DFRobot_SHT3x::pinReset()
{
  DFRobot_SHT3x::sStatusRegister_t registerRaw;
  clearStatusRegister();
  pinMode(_RST,OUTPUT);
  digitalWrite(_RST,1);
  delay(10);
  digitalWrite(_RST,0);
  delay(1);
  digitalWrite(_RST,1);
  delay(10);
  registerRaw = readStatusRegister();
  if(registerRaw.systemResetDeteced == 1)
    return true;
  else 
    return false;
}
bool DFRobot_SHT3x::stopPeriodicMode()
{  
  DFRobot_SHT3x::sStatusRegister_t registerRaw;
  writeCommand(CMD_STOP_PERIODIC_ACQUISITION_MODE,2);
  registerRaw = readStatusRegister();
  if(registerRaw.commendStatus == 0)
    return true;
  else 
    return false;
}
bool DFRobot_SHT3x::heaterEnable()
{
  DFRobot_SHT3x::sStatusRegister_t registerRaw;
  writeCommand(CMD_HEATER_ENABLE,2);
  registerRaw = readStatusRegister();
  if(registerRaw.heaterStaus == 1)
    return true;
  else 
    return false;
}
bool DFRobot_SHT3x::heaterDisable()
{
  DFRobot_SHT3x::sStatusRegister_t registerRaw;
  writeCommand( CMD_HEATER_DISABLE,2);
  registerRaw = readStatusRegister();
  if(registerRaw.heaterStaus == 1)
    return true;
  else 
    return false;
}
void DFRobot_SHT3x::clearStatusRegister(){
  writeCommand(CMD_CLEAR_STATUS_REG,2);
  delay(10);
}
DFRobot_SHT3x::sRHAndTemp_t DFRobot_SHT3x::readTempAndHumidity(eRepeatability_t repeatability)
{
  uint8_t rawData[6];
  uint8_t rawTemperature[3];
  uint8_t rawHumidity[3];
  DFRobot_SHT3x::sRHAndTemp_t tempRH;
  tempRH.ERR = 0;
      switch(repeatability){
        case eRepeatability_High:writeCommand(CMD_GETDATA_H_CLOCKENBLED,2);break;
        case eRepeatability_Medium:writeCommand(CMD_GETDATA_M_CLOCKENBLED,2);break;
        case eRepeatability_Low:writeCommand(CMD_GETDATA_L_CLOCKENBLED,2);break;
    }
  delay(15);
  readData(rawData,6);
  memcpy(rawTemperature,rawData,3);
  memcpy(rawHumidity,rawData+3,3);
  if((checkCrc(rawTemperature) != rawTemperature[2]) || (checkCrc(rawHumidity) != rawHumidity[2])){
     tempRH.ERR = -1;
  }
  tempRH.TemperatureC = convertTemperature(rawTemperature);
  tempRH.TemperatureF = (9/5)*tempRH.TemperatureC+32;
  tempRH.Humidity = convertHumidity(rawHumidity);
  return tempRH;
}
bool DFRobot_SHT3x::setMeasurementMode(eRepeatability_t repeatability,eMeasureFrequency_t measureFreq)
{
  DFRobot_SHT3x::sStatusRegister_t registerRaw;
  switch(measureFreq)
  {
    case eMeasureFreq_Hz5:
      switch(repeatability){
        case eRepeatability_High:writeCommand(CMD_SETMODE_H_FREQUENCY_HALF_HZ,2);break;
        case eRepeatability_Medium:writeCommand(CMD_SETMODE_M_FREQUENCY_HALF_HZ,2);break;
        case eRepeatability_Low:writeCommand(CMD_SETMODE_L_FREQUENCY_HALF_HZ,2);break;
    };break;
    case eMeasureFreq_1Hz:
      switch(repeatability){
        case eRepeatability_High:writeCommand(CMD_SETMODE_H_FREQUENCY_1_HZ,2);break;
        case eRepeatability_Medium:writeCommand(CMD_SETMODE_M_FREQUENCY_1_HZ,2);break;
        case eRepeatability_Low:writeCommand(CMD_SETMODE_L_FREQUENCY_1_HZ,2);break;
    };break;
    case eMeasureFreq_2Hz:
      switch(repeatability){
        case eRepeatability_High:writeCommand(CMD_SETMODE_H_FREQUENCY_2_HZ,2);break;
        case eRepeatability_Medium:writeCommand(CMD_SETMODE_M_FREQUENCY_2_HZ,2);break;
        case eRepeatability_Low:writeCommand(CMD_SETMODE_L_FREQUENCY_2_HZ,2);break;
    };break;
    case eMeasureFreq_4Hz:
      switch(repeatability){
        case eRepeatability_High:writeCommand(CMD_SETMODE_H_FREQUENCY_4_HZ,2);break;
        case eRepeatability_Medium:writeCommand(CMD_SETMODE_M_FREQUENCY_4_HZ,2);break;
        case eRepeatability_Low:writeCommand(CMD_SETMODE_L_FREQUENCY_4_HZ,2);break;
    };break;
    case eMeasureFreq_10Hz:
      switch(repeatability){
        case eRepeatability_High:writeCommand(CMD_SETMODE_H_FREQUENCY_10_HZ,2);break;
        case eRepeatability_Medium:writeCommand(CMD_SETMODE_M_FREQUENCY_10_HZ,2);break;
        case eRepeatability_Low:writeCommand(CMD_SETMODE_L_FREQUENCY_10_HZ,2);break;
    };break;
  }
  registerRaw = readStatusRegister();
  if(registerRaw.commendStatus == 0)
    return true;
  else 
    return false;
}
DFRobot_SHT3x::sStatusRegister_t DFRobot_SHT3x::readStatusRegister(){
  uint8_t register1[3];
  uint16_t data;
  DFRobot_SHT3x::sStatusRegister_t registerRaw;
  writeCommand(CMD_READ_STATUS_REG,2);
  
  readData(register1,3);
  data = (register1[0]<<8) | register1[1];
  memcpy(&registerRaw,&data,2);
  /*Serial.println(registerRaw.writeDataChecksumStatus);
  Serial.println(registerRaw.commendStatus);
  Serial.println(registerRaw.reserved0);
  Serial.println(registerRaw.systemResetDeteced);
  Serial.println(registerRaw.reserved1);
  Serial.println(registerRaw.temperatureAlert);
  Serial.println(registerRaw.humidityAlert);
  Serial.println(registerRaw.reserved2);
  Serial.println(registerRaw.heaterStaus);
  Serial.println(registerRaw.reserved3);
  Serial.println(registerRaw.alertPendingStatus);
  */
  return registerRaw;

}
DFRobot_SHT3x::sRHAndTemp_t DFRobot_SHT3x::readTempAndHumidity()
{
  uint8_t rawData[6];
  uint8_t rawTemperature[3];
  uint8_t rawHumidity[3];
  DFRobot_SHT3x::sRHAndTemp_t tempRH;
  tempRH.ERR = 0;
  writeCommand(CMD_GETDATA,2);
  readData(rawData,6);
  memcpy(rawTemperature,rawData,3);
  memcpy(rawHumidity,rawData+3,3);
  if((checkCrc(rawTemperature) != rawTemperature[2]) || (checkCrc(rawHumidity) != rawHumidity[2])){
     tempRH.ERR = -1;
  }
  tempRH.TemperatureC = convertTemperature(rawTemperature);
  tempRH.Humidity = convertHumidity(rawHumidity);
  tempRH.TemperatureF = (9/5)*tempRH.TemperatureC+32;
  return tempRH;
}
uint8_t  DFRobot_SHT3x::setTemperatureLimitC(float highset,float highclear,float lowclear, float lowset)
{
  uint16_t _highset ,_highclear,_lowclear,_lowset,limit[1];
  _highset = convertRawTemperature(highset);
  if(readLimitData(CMD_READ_HIGH_ALERT_LIMIT_SET,limit) != 0) {
	    DBG("");
    return 1;
  }
  DBG("");
  _highset = (_highset >> 7) |(limit[1] & 0xfe00);
  writeLimitData(CMD_WRITE_HIGH_ALERT_LIMIT_SET,_highset);
  _highclear = convertRawTemperature(highclear);
  if(readLimitData(CMD_READ_HIGH_ALERT_LIMIT_CLEAR,limit) != 0){
    return 1;
  }
  DBG("");
  _highclear = (_highclear >> 7) |(limit[1] & 0xfe00);
  writeLimitData(CMD_WRITE_HIGH_ALERT_LIMIT_CLEAR,_highclear);
  _lowclear = convertRawTemperature(lowclear);
  if(readLimitData(CMD_READ_LOW_ALERT_LIMIT_CLEAR,limit) != 0){
    return 1;
  }
  DBG("");
  _lowclear = (_lowclear >> 7) |(limit[1] & 0xfe00);
  writeLimitData(CMD_WRITE_LOW_ALERT_LIMIT_CLEAR,_lowclear);
  _lowset = convertRawTemperature(lowset);
  if(readLimitData(CMD_READ_LOW_ALERT_LIMIT_SET,limit) != 0){
    return 1;
  }
  DBG("");
  _lowset = (_lowset >> 7) |(limit[1] & 0xfe00);
  writeLimitData(CMD_WRITE_LOW_ALERT_LIMIT_SET,_lowset);
  return 0;
}
uint8_t DFRobot_SHT3x::setHumidityLimitRH(float highset,float highclear,float lowclear, float lowset)
{
  uint16_t _highset ,_highclear,_lowclear,_lowset,limit[1];
  _highset = convertRawHumidity(highset);
  if(readLimitData(CMD_READ_HIGH_ALERT_LIMIT_SET,limit) != 0) {
    return 1;
  }
  _highset = (_highset & 0xFE00) |(limit[1] & 0x1FF);
  writeLimitData(CMD_WRITE_HIGH_ALERT_LIMIT_SET,_highset);
  _highclear = convertRawHumidity(highclear);
  if(readLimitData(CMD_READ_HIGH_ALERT_LIMIT_CLEAR,limit) != 0){
    return 1;
  }
  _highclear = (_highclear & 0xFE00) |(limit[1] & 0x1FF);
  writeLimitData(CMD_WRITE_HIGH_ALERT_LIMIT_CLEAR,_highclear);
  _lowclear = convertRawHumidity(lowclear);
  if(readLimitData(CMD_READ_LOW_ALERT_LIMIT_CLEAR,limit) != 0){
    return 1;
  }
  _lowclear = (_lowclear & 0xFE00) |(limit[1] & 0x1FF);
  writeLimitData(CMD_WRITE_LOW_ALERT_LIMIT_CLEAR,_lowclear);
  _lowset = convertRawHumidity(lowset);
  if(readLimitData(CMD_READ_LOW_ALERT_LIMIT_SET,limit) != 0){
    return 1;
  }
  _lowset = (_lowset & 0xFE00) |(limit[1] & 0x1FF);
  writeLimitData(CMD_WRITE_LOW_ALERT_LIMIT_SET,_lowset);
  return 0;
}
DFRobot_SHT3x::slimitData_t DFRobot_SHT3x::readTemperatureLimitC(){

  slimitData_t limitData;
  uint16_t limit[1] ;
  if(readLimitData(CMD_READ_HIGH_ALERT_LIMIT_SET,limit) != 0) {
    limitData.ERR = -1;
  }
  limit[1] = limit[1] << 7;
  limit[1] = limit[1] & 0xFF80;
  limit[1] = limit[1] | 0x1A;
  limitData.highSet = round(175 * (float)limit[1] / 65535 - 45);
  if(readLimitData(CMD_READ_HIGH_ALERT_LIMIT_CLEAR,limit) != 0){
    limitData.ERR = -1;
  }
  limit[1] = limit[1] << 7;
  limit[1] = limit[1] & 0xFF80;
  limit[1] = limit[1] | 0x1A;
  limitData.highClear = round(175 * (float)limit[1] / 65535 - 45);
  if(readLimitData(CMD_READ_LOW_ALERT_LIMIT_CLEAR,limit) != 0){
    limitData.ERR = -1;
  }
  limit[1] = limit[1] << 7;
  limit[1] = limit[1] & 0xFF80;
  limit[1] = limit[1] | 0x1A;
  limitData.lowClear = round(175 * (float)limit[1] / 65535 - 45);
  if(readLimitData(CMD_READ_LOW_ALERT_LIMIT_SET,limit) != 0){
    limitData.ERR = -1;
  }
  limit[1] = limit[1] << 7;
  limit[1] = limit[1] & 0xFF80;
  limit[1] = limit[1] | 0x1A;
  limitData.lowSet = round(175 * (float)limit[1] / 65535 - 45);
  return limitData;
}
uint8_t DFRobot_SHT3x::readLimitData(uint16_t cmd,uint16_t *pBuf)
{ 
  uint8_t rawData[3];
  uint8_t crc;

  writeCommand(cmd,2);
  readData(rawData,3);
  crc = rawData[2];
  if(checkCrc(rawData) != crc){
    return 1 ;
  }
  pBuf[1] = rawData[0];
  pBuf[1] = (pBuf[1] << 8) | rawData[1];
  return 0;
}
DFRobot_SHT3x::slimitData_t DFRobot_SHT3x::readHumidityLimitRH(){

  slimitData_t limitData;
  uint16_t limit[1];
  if(readLimitData(CMD_READ_HIGH_ALERT_LIMIT_SET,limit) != 0) {
    limitData.ERR = -1;
  }
  limit[1] = limit[1] & 0xFE00;
  limit[1] = limit[1] | 0xCD;
  limitData.highSet = round(100 * (float)limit[1] / 65535) ;
  
  if(readLimitData(CMD_READ_HIGH_ALERT_LIMIT_CLEAR,limit) != 0){
    limitData.ERR = -1;
  }
  limit[1] = limit[1] & 0xFE00;
  limit[1] = limit[1] | 0xCD;
  limitData.highClear = round(100 * (float)limit[1] / 65535);
  if(readLimitData(CMD_READ_LOW_ALERT_LIMIT_CLEAR,limit) != 0){
    limitData.ERR = -1;
  }
  limit[1] = limit[1] & 0xFE00;
  limit[1] = limit[1] | 0xCD;
  limitData.lowClear = round(100 * (float)limit[1] / 65535);
  if(readLimitData(CMD_READ_LOW_ALERT_LIMIT_SET,limit) != 0){
    limitData.ERR = -1;
  }
  limit[1] = limit[1] & 0xFE00;
  limit[1] = limit[1] | 0xCD;
  limitData.lowSet = round(100 * (float)limit[1] / 65535);
  return limitData;
}
float DFRobot_SHT3x::convertTemperature(uint8_t rawTemperature[])
{
  uint16_t rawValue ;
  rawValue = rawTemperature[0];
  rawValue = (rawValue << 8) | rawTemperature[1];
  return 175.0f * (float)rawValue / 65535.0f - 45.0f;
}

float DFRobot_SHT3x::convertHumidity(uint8_t rawHumidity[])
{
  uint16_t rawValue ;
  rawValue = rawHumidity[0];
  rawValue = (rawValue << 8) | rawHumidity[1];
  return 100.0f * (float)rawValue / 65535.0f;
}

uint16_t DFRobot_SHT3x::convertRawTemperature(float value)
{
  return (value + 45.0f) / 175.0f * 65535.0f;
}

uint16_t DFRobot_SHT3x::convertRawHumidity(float value)
{
  return value / 100.0f * 65535.0f;
}
uint8_t DFRobot_SHT3x::checkCrc(uint8_t data[])
{
    uint8_t bit;
    uint8_t crc = 0xFF;

    for (uint8_t dataCounter = 0; dataCounter < 2; dataCounter++)
    {
        crc ^= (data[dataCounter]);
        for (bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x131;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}
uint8_t DFRobot_SHT3x::writeLimitData(uint16_t cmd,uint16_t limitData){
  uint8_t _pBuf[4];
  uint8_t ERR;
  _pBuf[0] = cmd >>8;
  _pBuf[1] = cmd & 0xff;
  _pBuf[2] = limitData >> 8;
  _pBuf[3] = limitData & 0xff;

  _pWire->beginTransmission(_address);
  _pWire->write(_pBuf[0]);
  _pWire->write(_pBuf[1]);
  _pWire->write(_pBuf[2]);
  _pWire->write(_pBuf[3]);
  _pWire->write(checkCrc(_pBuf+2));
  ERR =_pWire->endTransmission();
  return ERR;
}
uint8_t DFRobot_SHT3x::writeCommand(uint16_t cmd,size_t size)
{
  uint8_t _pBuf[2];
  uint8_t ERR;
  _pBuf[0] = cmd >> 8;
  _pBuf[1] = cmd & 0xFF;
  _pWire->beginTransmission(_address);
  for (uint8_t i = 0; i < size; i++) {
    _pWire->write(_pBuf[i]);
  }
  ERR =_pWire->endTransmission();
    
  return ERR;
}
uint8_t DFRobot_SHT3x::readData(void *pBuf, size_t size) {
  if (pBuf == NULL) {
    DBG("pBuf ERROR!! : null pointer");
  }
  delay(10);
  uint8_t * _pBuf = (uint8_t *)pBuf;
  //读取芯片返回的数据
  _pWire->requestFrom(_address,size);
  uint8_t len = 0;
  for (uint8_t i = 0 ; i < size; i++) {
    _pBuf[i] = _pWire->read();
    len++;
    
  }
  return len;
}