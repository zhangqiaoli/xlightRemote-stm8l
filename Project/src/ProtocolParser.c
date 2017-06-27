#include "ProtocolParser.h"
#include "_global.h"
#include "MyMessage.h"
#include "xliNodeConfig.h"
#include "button.h"

uint8_t bMsgReady = 0;

// Assemble message
void build(uint8_t _destination, uint8_t _sensor, uint8_t _command, uint8_t _type, bool _enableAck, bool _isAck)
{
    sndMsg.header.version_length = PROTOCOL_VERSION;
    sndMsg.header.sender = CurrentNodeID;
    sndMsg.header.destination = _destination;
    sndMsg.header.sensor = _sensor;
    sndMsg.header.type = _type;
    moSetCommand(_command);
    moSetRequestAck(_enableAck);
    moSetAck(_isAck);
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
  if( rcvMsg.header.destination != CurrentNodeID && rcvMsg.header.destination != BROADCAST_ADDRESS ) return 0;
  
  uint8_t _cmd = miGetCommand();
  uint8_t _sender = rcvMsg.header.sender;  // The original sender
  uint8_t _type = rcvMsg.header.type;
  uint8_t _sensor = rcvMsg.header.sensor;
  uint8_t _lenPayl = miGetLength();
  bool _isAck = (bool)miGetAck();
  
  switch( _cmd ) {
  case C_INTERNAL:
    if( _type == I_ID_RESPONSE ) {
      // Device/client got nodeID from Controller
      uint8_t lv_nodeID = _sensor;
      if( IS_NOT_REMOTE_NODEID(lv_nodeID) && !IS_GROUP_NODEID(lv_nodeID) ) {
      } else {
        if( _lenPayl > 8 ) {
          // Verify _uniqueID        
          if(!isIdentityEqual(_uniqueID, rcvMsg.payload.data+8, UNIQUE_ID_LEN)) {
            return 0;
          }
        }
        CurrentNodeID = lv_nodeID;
        memcpy(CurrentNetworkID, rcvMsg.payload.data, sizeof(CurrentNetworkID));
        UpdateNodeAddress();
        gIsChanged = TRUE;
        gDelayedOperation = DELAY_OP_PAIRED;
        Msg_Presentation();
        return 1;
      }
    } else if( _type == I_CONFIG ) {
      // Node Config
      switch( _sensor ) {
      case NCF_QUERY:
        // Inform controller with version & NCF data
        Msg_NodeConfigData(_sender);
        return 1;
        break;

      case NCF_DEV_ASSOCIATE:
        CurrentDeviceID = rcvMsg.payload.data[0];
        CurrentDevSubID = rcvMsg.payload.data[1];
        break;

      case NCF_DEV_SET_SUBID:
        CurrentSubNID = rcvMsg.payload.data[0];
        break;

      case NCF_DEV_SET_RELAY_NODE:
        gConfig.relayKey.deviceID = rcvMsg.payload.data[0];
        gConfig.relayKey.subDevID = rcvMsg.payload.data[1];
        break;

      case NCF_DEV_SET_RELAY_KEYS:
        {
          uint8_t _len = (_lenPayl > NUM_RELAY_KEYS ? NUM_RELAY_KEYS : _lenPayl);
          memset(gConfig.relayKey.keys, 0x00, NUM_RELAY_KEYS);
          for( uint8_t i = 0; i < _len; i++ ) {
            gConfig.relayKey.keys[i] = rcvMsg.payload.data[i];
          }
        }
        break;
        
      case NCF_DEV_CONFIG_MODE:
        SetConfigMode(rcvMsg.payload.data[0], rcvMsg.payload.data[1]);
        break;
        
      case NCF_DEV_MAX_NMRT:
        gConfig.rptTimes = rcvMsg.payload.data[0];
        break;
        
      case NCF_DATA_FN_SCENARIO:
        {
          uint8_t fn_id = rcvMsg.payload.data[0] & 0x0F;
          if( fn_id < 4 ) {
            gConfig.fnScenario[fn_id].bmDevice = (rcvMsg.payload.data[0] >> 4);
            gConfig.fnScenario[fn_id].scenario = rcvMsg.payload.data[1];
          } else {
            return 0;
          }
        }
        break;
      case NCF_DATA_FN_HUE:
        {
          uint8_t fn_id = rcvMsg.payload.data[0] & 0x0F;
          if( fn_id < 4 ) {
            gConfig.fnScenario[fn_id].bmDevice = (rcvMsg.payload.data[0] >> 4);
            gConfig.fnScenario[fn_id].scenario = 0;
            memcpy(&(gConfig.fnScenario[fn_id].hue), rcvMsg.payload.data+1, sizeof(Hue_t));
          } else {
            return 0;
          }
        }
        break;
      }
      gIsChanged = TRUE;
      Msg_NodeConfigAck(_sender, _sensor);
      return 1;
    }
    break;
      
  case C_PRESENTATION:
    if( _sensor == S_DIMMER ) {
      if( _isAck ) {
        // Device/client got Response to Presentation message, ready to work
        gConfig.token = rcvMsg.payload.uiValue;
        gConfig.present = (gConfig.token >  0);
        if(!IS_NOT_DEVICE_NODEID(_type) || IS_GROUP_NODEID(_type) ) {
          CurrentDeviceID = _type;
        }
        gIsChanged = TRUE;
        if( gDelayedOperation != DELAY_OP_PAIRED ) gDelayedOperation = DELAY_OP_CONNECTED;
        
        // REQ device status
        Msg_RequestDeviceStatus();
        return 1;
      }
    }
    break;
  
  case C_REQ:
  case C_SET:
    if( _isAck ) {
      if( _type == V_STATUS ) {
        bool _OnOff = rcvMsg.payload.bValue;
        if( _OnOff != CurrentDeviceOnOff ) {
          UpdateCurrentDeviceOnOff(_OnOff);
          gIsChanged = TRUE;
          // ToDo: change On/Off LED
        }
      } else if( _type == V_PERCENTAGE ) {
        if( rcvMsg.payload.data[1] != CurrentDeviceBright || rcvMsg.payload.data[0] != CurrentDeviceOnOff) {
          CurrentDeviceBright = rcvMsg.payload.data[1];
          UpdateCurrentDeviceOnOff(rcvMsg.payload.data[0]);
          gIsChanged = TRUE;
          // ToDo: change On/Off LED
        }        
      } else if( _type == V_LEVEL ) { // CCT
        uint16_t _CCTValue = rcvMsg.payload.data[1] * 256 + rcvMsg.payload.data[0];
        if( _CCTValue != CurrentDeviceCCT ) {
          CurrentDeviceCCT = _CCTValue;
          gIsChanged = TRUE;
        }
      } else if( _type == V_RGBW ) {
        if( rcvMsg.payload.data[0] ) { // Success
          CurrentDeviceType = rcvMsg.payload.data[1];
          CurrentDevicePresent = rcvMsg.payload.data[2];
          // uint8_t _RingID = rcvMsg.payload.data[3];     // No use for now
          CurrentDeviceOnOff = rcvMsg.payload.data[4];
          CurrentDeviceBright = rcvMsg.payload.data[5];
          if( IS_SUNNY(CurrentDeviceType) ) {
            uint16_t _CCTValue = rcvMsg.payload.data[7] * 256 + rcvMsg.payload.data[6];
            CurrentDeviceCCT = _CCTValue;
          } else if( IS_RAINBOW(CurrentDeviceType) || IS_MIRAGE(CurrentDeviceType) ) {
            // Set RGBW
            CurrentDeviceCCT = rcvMsg.payload.data[6];
            CurrentDevice_R = rcvMsg.payload.data[7];
            CurrentDevice_G = rcvMsg.payload.data[8];
            CurrentDevice_B = rcvMsg.payload.data[9];
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

void Msg_NodeConfigAck(uint8_t _to, uint8_t _ncf) {
  build(_to, _ncf, C_INTERNAL, I_CONFIG, 0, 1);

  sndMsg.payload.data[0] = 1;      // OK
  moSetPayloadType(P_BYTE);
  moSetLength(1);
  bMsgReady = 1;
}

// Prepare NCF query ack message
void Msg_NodeConfigData(uint8_t _to) {
  uint8_t payl_len = 0;
  build(_to, NCF_QUERY, C_INTERNAL, I_CONFIG, 0, 1);

  sndMsg.payload.data[payl_len++] = gConfig.version;
  sndMsg.payload.data[payl_len++] = gConfig.type + 224;
  sndMsg.payload.data[payl_len++] = ((gConfig.indDevice << 5) | (gConfig.inPresentation << 4) | gConfig.rptTimes);
  sndMsg.payload.data[payl_len++] = 0;     // Reservered
  sndMsg.payload.data[payl_len++] = 0;     // Reservered
  sndMsg.payload.data[payl_len++] = 0;     // Reservered
  for(uint8_t _index = 0; _index < NUM_DEVICES; _index++ ) {
    sndMsg.payload.data[payl_len++] = DeviceID(_index);
  }
  sndMsg.payload.data[payl_len++] = gConfig.fnScenario[0].scenario;
  sndMsg.payload.data[payl_len++] = gConfig.fnScenario[1].scenario;
  sndMsg.payload.data[payl_len++] = gConfig.fnScenario[2].scenario;
  sndMsg.payload.data[payl_len++] = gConfig.fnScenario[3].scenario;
  
  moSetLength(payl_len);
  moSetPayloadType(P_CUSTOM);
  bMsgReady = 1;
}

void Msg_RequestNodeID() {
  // Request NodeID for remote
  build(BASESERVICE_ADDRESS, NODE_TYP_REMOTE, C_INTERNAL, I_ID_REQUEST, 1, 0);
  moSetPayloadType(P_ULONG32);
  moSetLength(UNIQUE_ID_LEN);
  memcpy(sndMsg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  bMsgReady = 1;
}

// Prepare device presentation message
void Msg_Presentation() {
  build(NODEID_GATEWAY, S_DIMMER, C_PRESENTATION, gConfig.type, 1, 0);
  moSetPayloadType(P_ULONG32);
  moSetLength(UNIQUE_ID_LEN);
  memcpy(sndMsg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  bMsgReady = 1;
}

// Enquiry Device Status
void Msg_RequestDeviceStatus() {
  build(CurrentDeviceID, CurrentDevSubID, C_REQ, V_RGBW, 1, 0);
  moSetLength(1);
  moSetPayloadType(P_BYTE);
  sndMsg.payload.bValue = RING_ID_ALL;
  bMsgReady = 1;
}

// Set current device 1:On; 0:Off; 2:toggle
void Msg_DevOnOff(uint8_t _sw) {
  SendMyMessage();
  build(CurrentDeviceID, CurrentDevSubID, C_SET, V_STATUS, 1, 0);
  moSetLength(1);
  moSetPayloadType(P_BYTE);
  sndMsg.payload.bValue = _sw;
  bMsgReady = 1;
}

// Set relay key (e.g. pot light) 1:On; 0:Off; 2:toggle
void Msg_RelayOnOff(uint8_t _sw) {
  if( gConfig.relayKey.deviceID > 0 ) {
    SendMyMessage();
    if( _sw == DEVICE_SW_TOGGLE ) {
      gConfig.relayKey.state = 1 - gConfig.relayKey.state;
    } else {
      gConfig.relayKey.state = (_sw == DEVICE_SW_ON);
    }
    uint8_t _len = 0;
    build(gConfig.relayKey.deviceID, gConfig.relayKey.subDevID, C_SET, (gConfig.relayKey.state ? V_RELAY_ON : V_RELAY_OFF), 0, 0);
    moSetPayloadType(P_BYTE);
    for( _len = 0; _len < NUM_RELAY_KEYS; _len++ ) {
      if( gConfig.relayKey.keys[_len] == 0 ) break;
      sndMsg.payload.data[_len] = gConfig.relayKey.keys[_len];
    }
    moSetLength(_len);
    bMsgReady = 1;
  }
}

// Set current device brightness
void Msg_DevBrightness(uint8_t _op, uint8_t _br) {
  SendMyMessage();
  build(CurrentDeviceID, CurrentDevSubID, C_SET, V_PERCENTAGE, 1, 0);
  moSetLength(2);
  moSetPayloadType(P_BYTE);
  sndMsg.payload.data[0] = _op;
  sndMsg.payload.data[1] = _br;
  bMsgReady = 1;
}

// Set current device CCT
void Msg_DevCCT(uint8_t _op, uint16_t _cct) {
  SendMyMessage();
  build(CurrentDeviceID, CurrentDevSubID, C_SET, V_LEVEL, 1, 0);
  moSetLength(3);
  moSetPayloadType(P_UINT16);
  sndMsg.payload.data[0] = _op;
  sndMsg.payload.data[1] = _cct % 256;
  sndMsg.payload.data[2] = _cct / 256;
  bMsgReady = 1;
}

// Set current device brightness & CCT
void Msg_DevBR_CCT(uint8_t _br, uint16_t _cct) {
  SendMyMessage();
  build(CurrentDeviceID, CurrentDevSubID, C_SET, V_RGBW, 1, 0);
  moSetLength(5);
  moSetPayloadType(P_CUSTOM);
  sndMsg.payload.data[0] = RING_ID_ALL;      // Ring ID: 0 means all rings
  sndMsg.payload.data[1] = 1;                // State: On
  sndMsg.payload.data[2] = _br;
  sndMsg.payload.data[3] = _cct % 256;
  sndMsg.payload.data[4] = _cct / 256;
  bMsgReady = 1;
}

// Set current device brightness & RGBW
void Msg_DevBR_RGBW(uint8_t _br, uint8_t _r, uint8_t _g, uint8_t _b, uint8_t _w) {
  SendMyMessage();
  build(CurrentDeviceID, CurrentDevSubID, C_SET, V_RGBW, 1, 0);
  moSetLength(7);
  moSetPayloadType(P_CUSTOM);
  sndMsg.payload.data[0] = RING_ID_ALL;      // Ring ID: 0 means all rings
  sndMsg.payload.data[1] = 1;                // State: On
  sndMsg.payload.data[2] = _br;
  sndMsg.payload.data[3] = _w;
  sndMsg.payload.data[4] = _r;
  sndMsg.payload.data[5] = _g;
  sndMsg.payload.data[6] = _b;
  bMsgReady = 1;
}

// Change scenario on current device
void Msg_DevScenario(uint8_t _scenario) {
  SendMyMessage();
  build(CurrentDeviceID, CurrentDevSubID, C_SET, V_SCENE_ON, 1, 0);
  moSetLength(1);
  moSetPayloadType(P_BYTE);
  sndMsg.payload.bValue = _scenario;
  bMsgReady = 1;
}

// PPT Object Action
void Msg_PPT_ObjAction(uint8_t _obj, uint8_t _action) {
  SendMyMessage();
  build(CurrentDeviceID, NODEID_PROJECTOR, C_SET, V_STATUS, 1, 0);
  moSetLength(4);
  moSetPayloadType(P_STRING);
  sndMsg.payload.data[0] = _obj;
  sndMsg.payload.data[1] = _action;
  sndMsg.payload.data[2] = '0';      // Reserved
  sndMsg.payload.data[3] = '0';      // Reserved
  bMsgReady = 1;
}
