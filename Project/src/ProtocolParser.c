#include "ProtocolParser.h"
#include "_global.h"
#include "MyMessage.h"

uint8_t bMsgReady = 0;
char strOutput[50];

// Assemble message
void build(uint8_t _destination, uint8_t _sensor, uint8_t _command, uint8_t _type, bool _enableAck, bool _isAck)
{
    msg.header.version_length = PROTOCOL_VERSION;
    msg.header.sender = CurrentNodeID;
    msg.header.destination = _destination;
    msg.header.sensor = _sensor;
    msg.header.type = _type;
    miSetCommand(_command);
    miSetRequestAck(_enableAck);
    miSetAck(_isAck);
}

void UpdateCurrentDeviceOnOff(bool _OnOff) {
  CurrentDeviceOnOff = _OnOff;
  /*
  if( _OnOff ) {
    // Automatically clear the flag, so that the remote resumes controlling the lights
    gConfig.inPresentation = 0;
  }*/
}

uint8_t ParseProtocol(){
  if( msg.header.destination != CurrentNodeID && msg.header.destination != BROADCAST_ADDRESS ) return 0;
  
  uint8_t _cmd = miGetCommand();
  uint8_t _sender = msg.header.sender;  // The original sender
  uint8_t _type = msg.header.type;
  uint8_t _sensor = msg.header.sensor;
  bool _needAck = (bool)miGetRequestAck();
  bool _isAck = (bool)miGetAck();
  
  switch( _cmd ) {
  case C_INTERNAL:
    if( _type == I_ID_RESPONSE ) {
      // Device/client got nodeID from Controller
      uint8_t lv_nodeID = _sensor;
      if( lv_nodeID == NODEID_GATEWAY || lv_nodeID == NODEID_DUMMY ) {
      } else {
        sprintf(strOutput, "Get NodeId: %d, networkId: %X:%X:%X:%X:%X:%X:%X:%X", lv_nodeID, 
                msg.payload.data[0], msg.payload.data[1], msg.payload.data[2], msg.payload.data[3], 
                msg.payload.data[4], msg.payload.data[5], msg.payload.data[6], msg.payload.data[7]);
        CurrentNodeID = lv_nodeID;
        gIsChanged = TRUE;
        memcpy(CurrentNetworkID, msg.payload.data, sizeof(CurrentNetworkID));
        UpdateNodeAddress();
        Msg_Presentation();
        return 1;
      }
    }    
    break;
      
  case C_PRESENTATION:
    if( _type == S_DIMMER ) {
      if( _isAck ) {
        // Device/client got Response to Presentation message, ready to work
        gConfig.token = msg.payload.uiValue;
        gConfig.present = (gConfig.token >  0);
        gIsChanged = TRUE;
        sprintf(strOutput, "Got Presentation Ack Node:%d token:%d", CurrentNodeID, gConfig.token);
        
        // REQ device status
        Msg_RequestDeviceStatus(CurrentDeviceID);
        return 1;
      }
    }
    break;
  
  case C_REQ:
  case C_SET:
    if( _isAck ) {
      if( _type == V_STATUS ) {
        bool _OnOff = msg.payload.bValue;
        if( _OnOff != CurrentDeviceOnOff ) {
          UpdateCurrentDeviceOnOff(_OnOff);
          gIsChanged = TRUE;
          // ToDo: change On/Off LED
        }
      } else if( _type == V_PERCENTAGE ) {
        if( msg.payload.data[1] != CurrentDeviceBright || msg.payload.data[0] != CurrentDeviceOnOff) {
          CurrentDeviceBright = msg.payload.data[1];
          UpdateCurrentDeviceOnOff(msg.payload.data[0]);
          gIsChanged = TRUE;
          // ToDo: change On/Off LED
        }        
      } else if( _type == V_LEVEL ) { // CCT
        uint16_t _CCTValue = msg.payload.data[1] * 256 + msg.payload.data[0];
        if( _CCTValue != CurrentDeviceCCT ) {
          CurrentDeviceCCT = _CCTValue;
          gIsChanged = TRUE;
        }
      } else if( _type == V_RGBW ) {
        if( msg.payload.data[0] ) { // Success
          CurrentDeviceType = msg.payload.data[1];
          CurrentDevicePresent = msg.payload.data[2];
          CurrentDeviceOnOff = msg.payload.data[3];
          CurrentDeviceBright = msg.payload.data[4];
          if( IS_SUNNY(CurrentDeviceType) ) {
            uint16_t _CCTValue = msg.payload.data[6] * 256 + msg.payload.data[5];
            CurrentDeviceCCT = _CCTValue;
          }
          gIsChanged = TRUE;
          // ToDo: change On/Off LED
        }
      }
    }    
    break;
  }
  
  return 0;
}

void Msg_RequestNodeID() {
  // Request NodeID for remote
  build(BASESERVICE_ADDRESS, NODE_TYP_REMOTE, C_INTERNAL, I_ID_REQUEST, 1, 0);
  miSetPayloadType(P_ULONG32);
  miSetLength(UNIQUE_ID_LEN);
  memcpy(msg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  bMsgReady = 1;
}

// Prepare device presentation message
void Msg_Presentation() {
  build(NODEID_GATEWAY, gConfig.type, C_PRESENTATION, S_DIMMER, 1, 0);
  miSetPayloadType(P_ULONG32);
  miSetLength(UNIQUE_ID_LEN);
  memcpy(msg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  bMsgReady = 1;
}

// Enquiry Device Status
void Msg_RequestDeviceStatus(UC _nodeID) {
  build(_nodeID, CurrentNodeID, C_REQ, V_RGBW, 1, 0);
  bMsgReady = 1;
}

// Set current device On/Off
void Msg_DevOnOff(bool _sw) {
  build(CurrentDeviceID, CurrentNodeID, C_SET, V_STATUS, 1, 0);
  miSetLength(1);
  miSetPayloadType(P_BYTE);
  msg.payload.bValue = _sw;
  bMsgReady = 1;
}

// Set current device brightness
void Msg_DevBrightness(uint8_t _br) {
  build(CurrentDeviceID, CurrentNodeID, C_SET, V_PERCENTAGE, 1, 0);
  miSetLength(1);
  miSetPayloadType(P_BYTE);
  msg.payload.bValue = _br;
  bMsgReady = 1;
}

// Set current device CCT
void Msg_DevCCT(uint16_t _cct) {
  build(CurrentDeviceID, CurrentNodeID, C_SET, V_LEVEL, 1, 0);
  miSetLength(2);
  miSetPayloadType(P_UINT16);
  //msg.payload.uiValue = _cct;
  msg.payload.data[0] = _cct % 256;
  msg.payload.data[1] = _cct / 256;
  bMsgReady = 1;
}

// Set current device brightness & CCT
void Msg_DevBR_CCT(uint8_t _br, uint16_t _cct) {
  build(CurrentDeviceID, CurrentNodeID, C_SET, V_RGBW, 1, 0);
  miSetLength(4);
  miSetPayloadType(P_CUSTOM);
  msg.payload.data[0] = 1;
  msg.payload.data[1] = _br;
  msg.payload.data[2] = _cct % 256;
  msg.payload.data[3] = _cct / 256;
  bMsgReady = 1;
}
