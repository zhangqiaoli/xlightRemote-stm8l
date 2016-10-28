#include "_global.h"
#include "delay.h"
#include "rf24l01.h"
#include "timer.h"
#include "button.h"
#include "MyMessage.h"
#include "ProtocolParser.h"

/*
Xlight Remoter Program
License: MIT

Auther: Baoshi Sun
Email: bs.sun@datatellit.com, bs.sun@uwaterloo.ca
Github: https://github.com/sunbaoshi1975
Please visit xlight.ca for product details

RF24L01 connector pinout:
GND    VCC
CE     CSN
SCK    MOSI
MISO   IRQ

Connections:
  PB4 -> CE
  PD4 -> CSN
  PB5 -> SCK
  PB6 -> MOSI
  PB7 -> MISO
  PD5 -> IRQ

*/

// Xlight Application Identification
#define XLA_VERSION               0x01
#define XLA_ORGANIZATION          "xlight.ca"               // Default value. Read from EEPROM
#define XLA_PRODUCT_NAME          "XRemote"                 // Default value. Read from EEPROM

// RF channel for the sensor net, 0-127
#define RF24_CHANNEL	   		71
#define ADDRESS_WIDTH                   5
#define PLOAD_WIDTH                     32

// Unique ID for STM8L151x4
#define     UNIQUE_ID_ADDRESS         (0x4926)

const UC RF24_BASE_RADIO_ID[ADDRESS_WIDTH] = {0x00,0x54,0x49,0x54,0x44};

// Public variables
Config_t gConfig;
DeviceStatus_t gDevStatus[NUM_DEVICES];
MyMessage_t msg;
uint8_t *pMsg = (uint8_t *)&msg;
bool gIsChanged = FALSE;
uint8_t _uniqueID[UNIQUE_ID_LEN];

// Moudle variables
uint8_t mutex;
uint8_t rx_addr[ADDRESS_WIDTH] = {0x11, 0x11, 0x11, 0x11, 0x11};
uint8_t tx_addr[ADDRESS_WIDTH] = {0x11, 0x11, 0x11, 0x11, 0x11};

static void clock_init(void)
{
  CLK_DeInit();
  CLK_HSICmd(ENABLE);
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);
}

void Flash_ReadBuf(uint32_t Address, uint8_t *Buffer, uint16_t Length) {
  assert_param(IS_FLASH_ADDRESS(Address));
  assert_param(IS_FLASH_ADDRESS(Address+Length));
  
  for( uint16_t i = 0; i < Length; i++ ) {
    Buffer[i] = FLASH_ReadByte(Address+i);
  }
}

void Flash_WriteBuf(uint32_t Address, uint8_t *Buffer, uint16_t Length) {
  assert_param(IS_FLASH_DATA_EEPROM_ADDRESS(Address));
  assert_param(IS_FLASH_DATA_EEPROM_ADDRESS(Address+Length));
  
  // Init Flash Read & Write
  FLASH_SetProgrammingTime(FLASH_ProgramMode_Standard);

  FLASH_Unlock(FLASH_MemType_Data);
  /* Wait until Data EEPROM area unlocked flag is set*/
  while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET);
  
  uint8_t WriteBuf[FLASH_BLOCK_SIZE];
  uint16_t nBlockNum = (Length - 1) / FLASH_BLOCK_SIZE + 1;
  uint16_t block;
  for( block = 0; block < nBlockNum; block++ ) {
    memset(WriteBuf, 0x00, FLASH_BLOCK_SIZE);
    for( uint16_t i = 0; i < FLASH_BLOCK_SIZE; i++ ) {
      WriteBuf[i] = Buffer[block * FLASH_BLOCK_SIZE + i];
    }
    FLASH_ProgramBlock(block, FLASH_MemType_Data, FLASH_ProgramMode_Standard, WriteBuf);
    FLASH_WaitForLastOperation(FLASH_MemType_Data);
  }
  /* Alternative option, write byte by byte
  for( uint16_t i = 0; i < Length; i++ ) {
    FLASH_ProgramByte(Address+i, Buffer[i]);
  }*/
  
  FLASH_Lock(FLASH_MemType_Data);
}

uint8_t *Read_UniqueID(uint8_t *UniqueID, uint16_t Length)  
{
  Flash_ReadBuf(UNIQUE_ID_ADDRESS, UniqueID, Length);
  return UniqueID;
}

// Save config to Flash
void SaveConfig()
{
  if( gIsChanged ) {
    Flash_WriteBuf(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS, (uint8_t *)&gConfig, sizeof(gConfig));
    gIsChanged = FALSE;
  }
}

// Init Device Status
void InitDeviceStatus()
{
  DEVST_Present(0) = 0;
  DEVST_OnOff(0) = 0;
  DEVST_Bright(0) = 50;
  DEVST_WarmCold(0) = CT_MIN_VALUE;
  for(UC i = 1; i < NUM_DEVICES; i++ ) {
    gDevStatus[i] = gDevStatus[0];
  }
}

// Load config from Flash
void LoadConfig()
{
    // Load the most recent settings from FLASH
    Flash_ReadBuf(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS, (uint8_t *)&gConfig, sizeof(gConfig));
    if( gConfig.version > XLA_VERSION || gConfig.indDevice >= NUM_DEVICES || 
        !IS_VALID_REMOTE(gConfig.type) || gConfig.rfPowerLevel > RF24_PA_MAX ) {
      memset(&gConfig, 0x00, sizeof(gConfig));
      gConfig.version = XLA_VERSION;
      gConfig.indDevice = 0;
      gConfig.present = 0;
      gConfig.type = remotetypRFStandard;
      gConfig.rfPowerLevel = RF24_PA_MAX;
      memcpy(CurrentNetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH);
      sprintf(gConfig.Organization, "%s", XLA_ORGANIZATION);
      sprintf(gConfig.ProductName, "%s", XLA_PRODUCT_NAME);

      // Set device info
      NodeID(0) = BASESERVICE_ADDRESS;       // NODEID_MIN_REMOTE; BASESERVICE_ADDRESS; NODEID_DUMMY
      DeviceID(0) = NODEID_MAINDEVICE;
      DeviceType(0) = devtypUnknown;
      gConfig.devItem[1] = gConfig.devItem[0];
      gConfig.devItem[2] = gConfig.devItem[0];
      gConfig.devItem[3] = gConfig.devItem[0];
    
      gIsChanged = TRUE;
      SaveConfig();
    }
}

void UpdateNodeAddress() {
  memcpy(rx_addr, CurrentNetworkID, ADDRESS_WIDTH);
  rx_addr[0] = CurrentNodeID;
  memcpy(tx_addr, CurrentNetworkID, ADDRESS_WIDTH);
  tx_addr[0] = (CurrentNodeID >= BASESERVICE_ADDRESS ? BASESERVICE_ADDRESS : NODEID_GATEWAY);
  RF24L01_setup(tx_addr, rx_addr, RF24_CHANNEL, BROADCAST_ADDRESS);     // With openning the boardcast pipe
}

void EraseCurrentDeviceInfo() {
  CurrentNodeID = BASESERVICE_ADDRESS;
  CurrentDeviceID = NODEID_MAINDEVICE;
  CurrentDeviceType = devtypUnknown;
  CurrentDevicePresent = 0;
  memcpy(CurrentNetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH);
  gIsChanged = TRUE;
  SaveConfig();
}

bool WaitMutex(uint32_t _timeout) {
  while(_timeout--) {
    if( mutex > 0 ) return TRUE;
  }
  return FALSE;
}

// Send message and switch back to receive mode
bool SendMyMessage() {
  if( bMsgReady ) {
    mutex = 0;
    RF24L01_set_mode_TX();
    RF24L01_write_payload(pMsg, PLOAD_WIDTH);

    WaitMutex(0x1FFFF);
    if (mutex != 1) {
      //The transmission failed, Notes: mutex == 2 doesn't mean failed
      //It happens when rx address defers from tx address
      //asm("nop"); //Place a breakpoint here to see memory
    }
    
    // Switch back to receive mode
    bMsgReady = 0;
    RF24L01_set_mode_RX();
  }

  return(mutex > 0);
}

bool SayHelloToDevice(bool infinate) {

  uint8_t _count = 0;
  UpdateNodeAddress();
  while(1) {
    if( IS_NOT_REMOTE_NODEID(CurrentNodeID) || _count++ > 5 ) {
      // Try RequestNodeID message once for a while, 
      // even if we have a valid nodeID, in case the controller is changed.
      _count = 0;
      Msg_RequestNodeID();
    } else {
      Msg_Presentation();
    }
    
    if( SendMyMessage() ) break;
    if( !infinate ) return FALSE;
    
    // Failed or Timeout, then repeat init-step
    delay_ms(10000);   // Delay 10 seconds
  }
  
  return TRUE;
}

// ToDo: change to async operation and blink LED during pairing
uint8_t ChangeCurrentDevice(uint8_t _newDev) {
  if( _newDev >= NUM_DEVICES ) _newDev = 0;
  
  uint8_t currentDev = gConfig.indDevice;
  if( currentDev != _newDev ) {
    SelectDeviceLED(_newDev);
    gConfig.indDevice = _newDev;
    UpdateNodeAddress();
    if( !SayHelloToDevice(FALSE) ) {
      // Switch back to prevoius device
      SelectDeviceLED(currentDev);
      gConfig.indDevice = currentDev;
      UpdateNodeAddress();
    }
  }
  
  return gConfig.indDevice;
}

int main( void ) {

  // Init clock, timer and button
  clock_init();
  timer_init();
  button_init();
  
  // Go on only if NRF chip is presented
  RF24L01_init();
  while(!NRF24L01_Check());
  
  // Load config from Flash
  FLASH_DeInit();
  Read_UniqueID(_uniqueID, UNIQUE_ID_LEN);
  LoadConfig();
  gIsChanged = TRUE;
  SaveConfig();

  // Init Device Status Buffer
  InitDeviceStatus();
  
  delay_ms(3000);   // about 3 sec
  
  // Update RF addresses and Setup RF environment
  //gConfig.nodeID = 0x11; // test
  //gConfig.nodeID = NODEID_MIN_REMOTE;   // test
  //gConfig.nodeID = BASESERVICE_ADDRESS;   // test
  //memcpy(gConfig.NetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH); // test
  //UpdateNodeAddress();

  // NRF_IRQ
  disableInterrupts();
  GPIO_Init(GPIOD, GPIO_Pin_5, GPIO_Mode_In_FL_IT);
  EXTI_SelectPort(EXTI_Port_D);
  EXTI_SetPinSensitivity(EXTI_Pin_5, EXTI_Trigger_Falling);  
  enableInterrupts();

  // Must establish connection firstly
  SayHelloToDevice(TRUE);

  while (1) {
    
    // Send message if ready
    SendMyMessage();
    
    // Save Config if Changed
    SaveConfig();
    
    // Delay for while
    //delay_ms(500);
  }
}

void RF24L01_IRQ_Handler() {
  if(RF24L01_is_data_available()) {
    //Packet was received
    RF24L01_clear_interrupts();
    RF24L01_read_payload(pMsg, PLOAD_WIDTH);
    bMsgReady = ParseProtocol();
    return;
  }
 
  uint8_t sent_info;
  if (sent_info = RF24L01_was_data_sent()) {
    //Packet was sent or max retries reached
    RF24L01_clear_interrupts();
    mutex = sent_info;
    return;
  }

   RF24L01_clear_interrupts();
}