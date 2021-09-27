#include "Motor_driver.h"

MotorDriver::MotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION),
  dxl_id_(DXL_ID)//,
  //dxl_id2_(DXL_ID2)
{
}

MotorDriver::~MotorDriver()
{
  closeDynamixel();
}

bool MotorDriver::init(void)
{
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort())
  {
    ERROR_PRINT("Port is opened");
  }
  else
  {
    ERROR_PRINT("Port couldn't be opened");

    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_))
  {
    ERROR_PRINT("Baudrate is set");
  }
  else
  {
    ERROR_PRINT("Baudrate couldn't be set");

    return false;
  }

  // Enable Dynamixel Torque
  setTorque(dxl_id_, true);
  //setTorque(dxl_id2_, true);
  
  groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  groupSyncReadEncoder_   = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  groupBulkWrite_ = new dynamixel::GroupBulkWrite(portHandler_, packetHandler_);
  return true;
}

bool MotorDriver::setTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->getRxPacketError(dxl_error);
  }
}

bool MotorDriver::readEncoder(int32_t& value/*, int32_t& value2*/)
{
  int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result

  // Set parameter
  dxl_addparam_result = groupSyncReadEncoder_->addParam(dxl_id_);
  if (dxl_addparam_result != true)
    return false;
    
  // Syncread present position
  dxl_comm_result = groupSyncReadEncoder_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));

  // Check if groupSyncRead data of Dynamixels are available
  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(dxl_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  /*
  dxl_getdata_result = groupSyncReadEncoder_->isAvailable(dxl_id2_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;*/
    
  // Get data
  value = groupSyncReadEncoder_->getData(dxl_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  //value2 = groupSyncReadEncoder_->getData(dxl_id2_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  groupSyncReadEncoder_->clearParam();
  return true;
}

void MotorDriver::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setTorque(dxl_id_, false);
  //setTorque(dxl_id2_, false);
  
  // Close port
  portHandler_->closePort();
}

bool MotorDriver::VelControl(int8_t id, int64_t value)
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  if(id == DXL_ID){
    dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(dxl_id_, (uint8_t*)&value);
    if (dxl_addparam_result_ != true)
      return false;
  }
  /*
  if(id == DXL_ID2){
    dxl_addparam_result_ = groupSyncWriteVelocity_->addParam(dxl_id2_, (uint8_t*)&value);
    if (dxl_addparam_result_ != true)
      return false;
  }*/
  dxl_comm_result_ = groupSyncWriteVelocity_->txPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result_);
    return false;
  }

  groupSyncWriteVelocity_->clearParam();
  return true;
}

bool MotorDriver::PosControl(int8_t id, int64_t value)
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  if(id == DXL_ID){
    dxl_addparam_result_ = groupBulkWrite_->addParam(dxl_id_, ADDR_X_GOAL_POSITION, LEN_X_GOAL_POSITION, (uint8_t*)&value);
    if (dxl_addparam_result_ != true)
      return false; 
  }
  /*if(id == DXL_ID2){
    dxl_addparam_result_ = groupBulkWrite_->addParam(dxl_id2_, ADDR_X_GOAL_POSITION, LEN_X_GOAL_POSITION, (uint8_t*)&value);
    if (dxl_addparam_result_ != true)
      return false; 
  }*/
  dxl_comm_result_ = groupBulkWrite_->txPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->getTxRxResult(dxl_comm_result_);
    return false;
  }

  groupBulkWrite_->clearParam();
  return true;
}
