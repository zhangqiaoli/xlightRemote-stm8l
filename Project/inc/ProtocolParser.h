#ifndef __PROTOCOL_PARSER_H
#define __PROTOCOL_PARSER_H

#include "_global.h"

extern uint8_t bMsgReady;

uint8_t ParseProtocol();
void build(uint8_t _destination, uint8_t _sensor, uint8_t _command, uint8_t _type, bool _enableAck, bool _isAck);
void Msg_RequestNodeID();
void Msg_Presentation();
void Msg_RequestDeviceStatus(UC _nodeID);
void Msg_DevOnOff(uint8_t _sw);
void Msg_DevBrightness(uint8_t _op, uint8_t _br);
void Msg_DevCCT(uint8_t _op, uint16_t _cct);
void Msg_DevBR_CCT(uint8_t _br, uint16_t _cct);

#endif /* __PROTOCOL_PARSER_H */