/*
 xlight remoter button functions

- Dimmer keys:
      PD2 -> keyUp
      PD3 -> keyDown
      PD0 -> keyLeft
      PD1 -> KeyRight
      PB0 -> KeyCenter

  - Functuon keys:
      PB1 -> Fn1
      PB2 -> Fn2
      PB3 -> Fn3
      //PD6 -> Fn4

LEDs
  Flashlight (White LED) -> PC1
  On/Off (Green LED) -> PC6
  Device Selection (Red LEDs) -> PC0 to PC3

*/

#include "_global.h"
#include "PPTCtrl_Protocol.h"
#include "stm8l15x.h"
#include "stm8l15x_gpio.h"
#include "stm8l15x_exti.h"

#include "timer.h"
#include "button.h"
#include "ProtocolParser.h"

//---------------------------------------------------
// PIN Map
//---------------------------------------------------
// LED pin map
#define LEDS_PORT               (GPIOC)
#define LED_PIN_LASERPEN        (GPIO_Pin_0)
#define LED_PIN_FLASHLIGHT      (GPIO_Pin_1)

// Button pin map
#define BUTTONS_PORT1           (GPIOD)
#define BUTTONS_PORT2           (GPIOB)

// Port 1    
#define BUTTON_PIN_UP           (GPIO_Pin_0)
#define BUTTON_PIN_DOWN         (GPIO_Pin_1)
#define BUTTON_PIN_LEFT         (GPIO_Pin_2)
#define BUTTON_PIN_RIGHT        (GPIO_Pin_3)

#ifndef PCB_10_BUTTONS
#define BUTTON_PIN_SB           (GPIO_Pin_4)
#endif

#define BUTTON_PIN_FN4          (GPIO_Pin_6)
#define BUTTON_PIN_SA           (GPIO_Pin_7)

// Port 2
#define BUTTON_PIN_CENTER       (GPIO_Pin_0)
#define BUTTON_PIN_FN1          (GPIO_Pin_1)
#define BUTTON_PIN_FN2          (GPIO_Pin_2)
#define BUTTON_PIN_FN3          (GPIO_Pin_3)
//---------------------------------------------------

// Set LED pin status
#define ledSetPin(x, pin)       GPIO_WriteBit(LEDS_PORT, pin, ((x) > 0 ? SET : RESET))
#define ledFlashLight(x)        ledSetPin(x, LED_PIN_FLASHLIGHT)
#define ledToggleFlashLight     GPIO_ToggleBits(LEDS_PORT, LED_PIN_FLASHLIGHT)
#define ledLaserPen(x)          ledSetPin(x, LED_PIN_LASERPEN)
#define ledToggleLaserPen       GPIO_ToggleBits(LEDS_PORT, LED_PIN_LASERPEN)

// Get Button pin input
#define pinKeyUp                ((BitStatus)(BUTTONS_PORT1->IDR & (uint8_t)BUTTON_PIN_UP))
#define pinKeyDown              ((BitStatus)(BUTTONS_PORT1->IDR & (uint8_t)BUTTON_PIN_DOWN))
#define pinKeyLeft              ((BitStatus)(BUTTONS_PORT1->IDR & (uint8_t)BUTTON_PIN_LEFT))
#define pinKeyRight             ((BitStatus)(BUTTONS_PORT1->IDR & (uint8_t)BUTTON_PIN_RIGHT))
#define pinKeyCenter            ((BitStatus)(BUTTONS_PORT2->IDR & (uint8_t)BUTTON_PIN_CENTER))

#define pinKeyFn1               ((BitStatus)(BUTTONS_PORT2->IDR & (uint8_t)BUTTON_PIN_FN1))
#define pinKeyFn2               ((BitStatus)(BUTTONS_PORT2->IDR & (uint8_t)BUTTON_PIN_FN2))
#define pinKeyFn3               ((BitStatus)(BUTTONS_PORT2->IDR & (uint8_t)BUTTON_PIN_FN3))
#define pinKeyFn4               ((BitStatus)(BUTTONS_PORT1->IDR & (uint8_t)BUTTON_PIN_FN4))

#define pinKeySA                ((BitStatus)(BUTTONS_PORT1->IDR & (uint8_t)BUTTON_PIN_SA))

#ifdef BUTTON_PIN_SB
#define pinKeySB                ((BitStatus)(BUTTONS_PORT1->IDR & (uint8_t)BUTTON_PIN_SB))
#endif

#define pinLEDFlashlight        ((BitStatus)(LEDS_PORT->IDR & (uint8_t)LED_PIN_FLASHLIGHT))
#define pinLEDLaserPen          ((BitStatus)(LEDS_PORT->IDR & (uint8_t)LED_PIN_LASERPEN))

#define BUTTON_DEBONCE_DURATION                 3       // The unit is 10 ms, so the duration is 30 ms.
#define BUTTON_WAIT_2S                          100     // The unit is 10 ms, so the duration is 2 s.
#define BUTTON_WAIT_3S                          300     // The unit is 10 ms, so the duration is 3 s.
#define BUTTON_DOUBLE_BTN_DURATION              50      // The unit is 10 ms, so the duration is 500 ms.
#define BUTTON_DOUBLE_BTN_TRACK_DURATION        300     // The unit is 10 ms, so the duration is 3 s.

// Button behavior
#define BTN_STEP_PAGES          5
#ifdef BATCH_TEST
#define BTN_STEP_SHORT_BR       50
#define BTN_STEP_SHORT_CCT      1900
#define BTN_STEP_LONG_BR        50
#define BTN_STEP_LONG_CCT       1900
#define BTN_BR_LOW              5
#else
#define BTN_STEP_SHORT_BR       10
#define BTN_STEP_SHORT_CCT      300
#define BTN_STEP_LONG_BR        25
#define BTN_STEP_LONG_CCT       800
#define BTN_BR_LOW              10
#endif

uint8_t lastswitch = 1;

static button_timer_status_t  m_btn_timer_status[keylstDummy] = {BUTTON_STATUS_INIT};
static bool detect_double_btn_press[keylstDummy] = {FALSE};
static bool btn_is_pushed[keylstDummy] = {FALSE};
static uint16_t btn_bit_postion[keylstDummy];

static uint8_t m_timer_id_btn_detet[keylstDummy];
static uint8_t m_timer_id_double_btn_detet[keylstDummy];

static bool double_button_track = FALSE;
static uint16_t button_status = 0xFFFF;
static uint16_t button_first_detect_status = 0xFFFF;

static uint8_t m_timer_id_debonce_detet;

void app_button_event_handler(uint8_t _btn, button_event_t button_event);
void button_push(uint8_t _btn);
void button_release(uint8_t _btn);

// Switch PPT mode
void SwitchPPTMode(bool _sw) {
#ifdef ENABLE_PRESENTATION_MODE  
  gConfig.inPresentation = _sw;
  if( gDelayedOperation == 0 ) {
    gDelayedOperation = _sw ? DELAY_OP_PPTMODE_ON : DELAY_OP_PPTMODE_OFF;
  }
#endif  
}

// Blink LED to indicate starting
void LED_Blink(bool _flash, bool _fast) {
  if( _flash )
    SetFlashlight(DEVICE_SW_ON);
  else
    SetLasterBeam(DEVICE_SW_ON);
  //delay_ms(_fast ? 200 : 500);
  WaitMutex(_fast ? 0x4FFF : 0xBFFF);
  if( _flash )
    SetFlashlight(DEVICE_SW_OFF);
  else
    SetLasterBeam(DEVICE_SW_OFF);
  //delay_ms(_fast ? 200 : 500);
  WaitMutex(_fast ? 0x4FFF : 0xBFFF);
}

// Change device selection LEDs according to current device index
void SelectDeviceLED(uint8_t _dev) {
  switch(_dev) {
  case 1:
    //ledDevice0(0);
    //ledDevice1(1);
    //ledDevice2(0);
    //ledDevice3(0);
    break;
    
  case 2:
    //ledDevice0(0);
    //ledDevice1(0);
    //ledDevice2(1);
    //ledDevice3(0);
    break;

  default:
    //ledDevice0(1);
    //ledDevice1(0);
    //ledDevice2(0);
    //ledDevice3(0);
    break;
  }
}

static void btn_duration_timeout_handler(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  button_event_t button_event = BUTTON_INVALID;
  switch (m_btn_timer_status[_btn]) {
  case BUTTON_STATUS_INIT:
    break;
    
  case BUTTON_STATUS_LESS_2S:
    button_event = BUTTON_LONG_HOLD;
    timer_start(m_timer_id_btn_detet[_btn], BUTTON_WAIT_3S);
    m_btn_timer_status[_btn] = BUTTON_STATUS_MORE_2S;
    break;
    
  case BUTTON_STATUS_MORE_2S:
    button_event = BUTTON_VERY_LONG_HOLD;
    m_btn_timer_status[_btn] = BUTTON_STATUS_MORE_5S;
    break;
    
  case BUTTON_STATUS_MORE_5S:
    break;
    
  case BUTTON_STATUS_DOUBLE_TRACK:
    button_event = DOUBLE_BTN_TRACK;
    m_btn_timer_status[_btn] = BUTTON_STATUS_INIT;
    break;
    
  default:
    break;
  }
  
  if( button_event != BUTTON_INVALID ) {
    app_button_event_handler(_btn, button_event);
  }
}

void double_btn_timeout_handler(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  button_event_t button_event = BUTTON_SHORT_PRESS;
  detect_double_btn_press[_btn] = FALSE;
  m_btn_timer_status[_btn] = BUTTON_STATUS_INIT;
  timer_stop(m_timer_id_double_btn_detet[_btn]);
  app_button_event_handler(_btn, button_event);
}

void btn_debonce_timeout_handler(uint8_t _tag)
{
  uint16_t valid_button;
  uint16_t current_button;
  uint16_t changed_button;
  
  current_button = GPIO_ReadInputData(BUTTONS_PORT1);
  current_button <<= 8;
  current_button |= GPIO_ReadInputData(BUTTONS_PORT2);

  valid_button = ~(current_button ^ button_first_detect_status);    
  changed_button = ((current_button^button_status) & valid_button);
  button_status = current_button;
  
  // Scan all buttons
  uint8_t _btn;
  for( _btn = 0; _btn < keylstDummy; _btn++ ) {    
    if ((changed_button & btn_bit_postion[_btn]) != 0)
    {
      timer_stop(m_timer_id_btn_detet[_btn]);
      if ((current_button & btn_bit_postion[_btn]) == 0)
      {
        button_push(_btn);
      }
      else
      {
        button_release(_btn);
      }
    }
  }
}

void button_init()
{
  uint8_t _btn;
  
  // Set button bit postion
  btn_bit_postion[keylstLeft] = BUTTON_PIN_LEFT;
  btn_bit_postion[keylstLeft] <<= 8;
  btn_bit_postion[keylstRight] = BUTTON_PIN_RIGHT;
  btn_bit_postion[keylstRight] <<= 8;
  btn_bit_postion[keylstUp] = BUTTON_PIN_UP;
  btn_bit_postion[keylstUp] <<= 8;
  btn_bit_postion[keylstDown] = BUTTON_PIN_DOWN;
  btn_bit_postion[keylstDown] <<= 8;
  btn_bit_postion[keylstFn4] = BUTTON_PIN_FN4;
  btn_bit_postion[keylstFn4] <<= 8;
  btn_bit_postion[keylstFLASH] = BUTTON_PIN_SA;
  btn_bit_postion[keylstFLASH] <<= 8;

#ifdef BUTTON_PIN_SB
  btn_bit_postion[keylstLASER] = BUTTON_PIN_SB;
  btn_bit_postion[keylstLASER] <<= 8;
#endif
  
  btn_bit_postion[keylstCenter] = BUTTON_PIN_CENTER;
  btn_bit_postion[keylstFn1] = BUTTON_PIN_FN1;
  btn_bit_postion[keylstFn2] = BUTTON_PIN_FN2;
  btn_bit_postion[keylstFn3] = BUTTON_PIN_FN3;
  
  // Setup Interrupts
  disableInterrupts();
  //GPIO_Init(LEDS_PORT, (LED_PIN_ONOFF | LED_PIN_DEV0 | LED_PIN_DEV1 | LED_PIN_DEV2 | LED_PIN_DEV3), GPIO_Mode_Out_PP_Low_Fast);
#ifdef ENABLE_FLASHLIGHT_LASER  
  GPIO_Init(LEDS_PORT, (LED_PIN_FLASHLIGHT | LED_PIN_LASERPEN), GPIO_Mode_Out_PP_Low_Fast);
#endif
#ifdef BUTTON_PIN_SB
  GPIO_Init(BUTTONS_PORT1, (BUTTON_PIN_LEFT | BUTTON_PIN_RIGHT | BUTTON_PIN_UP | BUTTON_PIN_DOWN | BUTTON_PIN_FN4 | BUTTON_PIN_SA | BUTTON_PIN_SB), GPIO_Mode_In_PU_IT);
#else
  GPIO_Init(BUTTONS_PORT1, (BUTTON_PIN_LEFT | BUTTON_PIN_RIGHT | BUTTON_PIN_UP | BUTTON_PIN_DOWN | BUTTON_PIN_FN4 | BUTTON_PIN_SA), GPIO_Mode_In_PU_IT);
#endif  
  
  GPIO_Init(BUTTONS_PORT2, (BUTTON_PIN_CENTER | BUTTON_PIN_FN1 | BUTTON_PIN_FN2 | BUTTON_PIN_FN3), GPIO_Mode_In_PU_IT);
  EXTI_DeInit();
  EXTI_SelectPort(EXTI_Port_D);
  EXTI_SetPinSensitivity(EXTI_Pin_0, EXTI_Trigger_Rising_Falling);
  EXTI_SetPinSensitivity(EXTI_Pin_1, EXTI_Trigger_Rising_Falling);
  EXTI_SetPinSensitivity(EXTI_Pin_2, EXTI_Trigger_Rising_Falling);
  EXTI_SetPinSensitivity(EXTI_Pin_3, EXTI_Trigger_Rising_Falling);
#ifdef BUTTON_PIN_SB
  EXTI_SetPinSensitivity(EXTI_Pin_4, EXTI_Trigger_Rising_Falling);
#endif
  EXTI_SetPinSensitivity(EXTI_Pin_6, EXTI_Trigger_Rising_Falling);
  EXTI_SetPinSensitivity(EXTI_Pin_7, EXTI_Trigger_Rising_Falling);
  
  EXTI_SelectPort(EXTI_Port_B);
  EXTI_SetPinSensitivity(EXTI_Pin_0, EXTI_Trigger_Rising_Falling);
  EXTI_SetPinSensitivity(EXTI_Pin_1, EXTI_Trigger_Rising_Falling);
  EXTI_SetPinSensitivity(EXTI_Pin_2, EXTI_Trigger_Rising_Falling);
  EXTI_SetPinSensitivity(EXTI_Pin_3, EXTI_Trigger_Rising_Falling);
  enableInterrupts();

  // Create all timers
  for( _btn = 0; _btn < keylstDummy; _btn++ ) {
    timer_create(&m_timer_id_btn_detet[_btn], _btn, btn_duration_timeout_handler);
    timer_create(&m_timer_id_double_btn_detet[_btn], _btn, double_btn_timeout_handler);
  }
  timer_create(&m_timer_id_debonce_detet, 0, btn_debonce_timeout_handler);
}

void SetFlashlight(uint8_t _st)
{
#ifdef ENABLE_FLASHLIGHT_LASER
  if( _st == DEVICE_SW_ON ) {
    ledFlashLight(SET);
  } else if( _st == DEVICE_SW_OFF ) {
    ledFlashLight(RESET);
  } else {
    ledToggleFlashLight;
  }
#endif  
}

void SetLasterBeam(uint8_t _st)
{
#ifdef ENABLE_FLASHLIGHT_LASER
  if( _st == DEVICE_SW_ON ) {
    ledLaserPen(SET);
  } else if( _st == DEVICE_SW_OFF ) {
    ledLaserPen(RESET);
  } else {
    ledToggleLaserPen;
  }
#endif  
}

void FN_Button_Action(uint8_t _fn) {
  uint8_t lv_curInd = gConfig.indDevice;
    
  for( uint8_t idx = 0; idx < NUM_DEVICES; idx++ ) {
    if( gConfig.fnScenario[_fn].bmDevice > 0 ) {
      // Verify deviceID and switch current device if matched
      if( gConfig.fnScenario[_fn].bmDevice & (0x01 << idx) ) {
        if( gConfig.devItem[idx].deviceID > 0 ) {
          gConfig.indDevice = idx;
        } else {
          continue;
        }
      } else {
        continue;
      }
    }
    
    if( gConfig.fnScenario[_fn].effect > 0 ) {
      CurrentDeviceOnOff = DEVICE_SW_ON;
      Msg_DevSpecialEffect(gConfig.fnScenario[_fn].effect);
    } else if( gConfig.fnScenario[_fn].hue.State == DEVICE_SW_TOGGLE ) {
      CurrentDeviceOnOff = 1 - CurrentDeviceOnOff;
      Msg_DevOnOff(CurrentDeviceOnOff);
    } else if( gConfig.fnScenario[_fn].hue.State == DEVICE_SW_OFF ) {
      Msg_DevOnOff(DEVICE_SW_OFF);
    } else if( gConfig.fnScenario[_fn].hue.State == DEVICE_SW_ON ) {
      Msg_DevOnOff(DEVICE_SW_ON);

      if( gConfig.fnScenario[_fn].hue.CCT ==0 && gConfig.fnScenario[_fn].hue.R == 0 && gConfig.fnScenario[_fn].hue.G == 0 && gConfig.fnScenario[_fn].hue.B==0)
      {// if only adjust br 
        uint8_t br = gConfig.fnScenario[_fn].hue.BR>BR_MIN_VALUE ? gConfig.fnScenario[_fn].hue.BR : BR_MIN_VALUE;
        Msg_DevBrightness(OPERATOR_SET,br);
      }
      else if( gConfig.fnScenario[_fn].hue.CCT >= CT_MIN_VALUE ) {
        Msg_DevBR_CCT(gConfig.fnScenario[_fn].hue.BR, gConfig.fnScenario[_fn].hue.CCT);
      } else {
        Msg_DevBR_RGBW(gConfig.fnScenario[_fn].hue.BR, gConfig.fnScenario[_fn].hue.R, gConfig.fnScenario[_fn].hue.G, gConfig.fnScenario[_fn].hue.B, (uint8_t)gConfig.fnScenario[_fn].hue.CCT);
      }
    }
    
    if( gConfig.fnScenario[_fn].scenario > 0 ) {
      // Delay sending
      gSendDelayTick = 0;
      gSendScenario = gConfig.fnScenario[_fn].scenario;
    }
    
    if( gConfig.fnScenario[_fn].bmDevice == 0 ) break;
  }
  
  // Restore Device Index
  gConfig.indDevice = lv_curInd;
}

void FavoriteDevStatus(uint8_t index)
{
/*  if(index > NUM_FAVORITE) return;
  if(gConfig.favoritesDevStat[index].reserved == 0)
  { // favorite
    gLastFavoriteIndex = index;
    Msg_RequestDeviceStatus();  
  }
  else
  { // restore favorite status
    if( gConfig.favoritesDevStat[index].ring.CCT >= CT_MIN_VALUE ) {
      Msg_DevBR_CCT(gConfig.favoritesDevStat[index].ring.BR, gConfig.favoritesDevStat[index].ring.CCT);
    } else {
      Msg_DevBR_RGBW(gConfig.favoritesDevStat[index].ring.BR, gConfig.favoritesDevStat[index].ring.R, gConfig.favoritesDevStat[index].ring.G, gConfig.favoritesDevStat[index].ring.B, gConfig.favoritesDevStat[index].ring.CCT);
    }   
  }*/ 
#ifdef NUM_FAVORITE
  if(index > NUM_FAVORITE) return;
  SaveFavoriteDevStatus();
  gLastFavoriteIndex = index;
  gLastFavoriteTick = 0;
  Msg_RequestDeviceStatus(); 
#endif
}

void RestoreFavoriteDevStatus(uint8_t index)
{
#ifdef NUM_FAVORITE
  if(index >= NUM_FAVORITE) return;
 // restore favorite status
  if( gConfig.favoritesDevStat[index].ring.CCT >= CT_MIN_VALUE ) {
    Msg_DevBR_CCT(gConfig.favoritesDevStat[index].ring.BR, gConfig.favoritesDevStat[index].ring.CCT);
  } else {
    Msg_DevBR_RGBW(gConfig.favoritesDevStat[index].ring.BR, gConfig.favoritesDevStat[index].ring.R, gConfig.favoritesDevStat[index].ring.G, gConfig.favoritesDevStat[index].ring.B, gConfig.favoritesDevStat[index].ring.CCT);
  }  
#endif
}

void btn_short_button_press(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  switch( _btn ) {
  case keylstLeft:
    if( gConfig.inPresentation ) {
      // Turn screen black or restore
      Msg_PPT_ObjAction(PPT_OBJ_SCREEN, DEVICE_SW_TOGGLE);
    } else {
      // Reduce CCT and get warmmer white
      Msg_DevCCT(OPERATOR_SUB, BTN_STEP_SHORT_CCT);
    }
    break;
    
  case keylstRight:
    if( gConfig.inPresentation ) {
      // Start play or stop play
      Msg_PPT_ObjAction(PPT_OBJ_PAGE, DEVICE_SW_TOGGLE);
    } else {
      // Increase CCT and get colder white
      Msg_DevCCT(OPERATOR_ADD, BTN_STEP_SHORT_CCT);
    }
    break;
    
  case keylstUp:
    if( gConfig.inPresentation ) {
      // Page Up
      Msg_PPT_ObjAction(PPT_OBJ_PAGE, CONTENT_GO_PREV);
    } else {
      // more bright
      Msg_DevBrightness(OPERATOR_ADD, BTN_STEP_SHORT_BR);
      break;
    }
    
  case keylstDown:
    if( gConfig.inPresentation ) {
      // Page Down
      Msg_PPT_ObjAction(PPT_OBJ_PAGE, CONTENT_GO_NEXT);
    } else {
      // less bright
      Msg_DevBrightness(OPERATOR_SUB, BTN_STEP_SHORT_BR);
    }
    break;
    
  case keylstCenter:
    if( gConfig.inPresentation ) {
      // Next Page
      Msg_PPT_ObjAction(PPT_OBJ_PAGE, CONTENT_GO_NEXT);
    } else {
      // Toggle lights on/off
#if defined (ENABLE_SDTM) || defined (BATCH_TEST)
      //Msg_DevOnOff(CurrentDeviceOnOff == 0);
      Msg_DevOnOff(lastswitch == 0);
      lastswitch = (lastswitch == 0);
#else
      Msg_DevOnOff(DEVICE_SW_TOGGLE);
#endif
    }
    break;
    
  case keylstFn1:
#ifdef BATCH_TEST
    Msg_DevBrightness(OPERATOR_SET,50);
#else
#ifdef HOME_VERSION
    FN_Button_Action(0);
#else
#ifdef ENABLE_SDTM
    // Pure Red
    Msg_DevBR_RGBW(60, 255, 0, 0, 0);
#else
    FN_Button_Action(0);
    Msg_SpecialDevOnOff(129,8,1);
#endif
#endif
#endif

    break;
    
  case keylstFn2:
#ifdef BATCH_TEST
    Msg_DevCCT(OPERATOR_SET,4600);
#else
#ifdef HOME_VERSION
    FN_Button_Action(1);   
    Msg_DevOnOffDelay(DEVICE_SW_OFF,MINUTE_UNIT,SLEEP_TIME);
#else
#ifdef ENABLE_SDTM
    // Pure Green
    Msg_DevBR_RGBW(60, 0, 255, 0, 0);
#else
    FN_Button_Action(1);
#ifdef CLASS_ROOM_TYPE    
    FN_Button_Action(2);
    Msg_SpecialDevOnOff(129,8,1);
#endif
#endif
#endif
#endif
    break;
    
  case keylstFn3:
#ifdef HOME_VERSION
    FN_Button_Action(2);
#else
#ifdef ENABLE_SDTM
    // Pure Blue
    Msg_DevBR_RGBW(60, 0, 0, 255, 0);
#else
    FN_Button_Action(2);
#ifdef CLASS_ROOM_TYPE    
    FN_Button_Action(4);
    Msg_SpecialDevOnOff(129,8,1);
#endif       
#endif
#endif    
    break;
    
  case keylstFn4:
#ifdef HOME_VERSION
    FN_Button_Action(3);
#else
#ifdef ENABLE_SDTM
    // Pure White
    Msg_DevBR_RGBW(60, 0, 0, 0, 255);
#else
#ifdef CLASS_ROOM_TYPE    
    FN_Button_Action(2);
#endif
    FN_Button_Action(3);
    Msg_SpecialDevOnOff(129,8,0);
    //Msg_DevOnOff(DEVICE_SW_TOGGLE);
#endif  
#endif  
    break;

  case keylstFLASH:
#ifdef BATCH_TEST
    Msg_DevOnOff(DEVICE_SW_ON);
#else
#ifdef HOME_VERSION
    //Favorites1
    RestoreFavoriteDevStatus(0);
#else
#ifdef ENABLE_FLASHLIGHT_LASER    
    // Toggle the flash light
    ledToggleFlashLight;
#else
    //Msg_RelayOnOff(DEVICE_SW_TOGGLE);
    FN_Button_Action(5);
#endif 
#endif
#endif
    break;
    
  case keylstLASER:
#ifdef BATCH_TEST
    Msg_DevOnOff(DEVICE_SW_OFF);
#else
#ifdef HOME_VERSION
    //Favorites2
    RestoreFavoriteDevStatus(1);
#else
#ifdef ENABLE_FLASHLIGHT_LASER    
    // Toggle laser pen
    ledToggleLaserPen;
#else
    //Msg_RelayOnOff(DEVICE_SW_TOGGLE);
    FN_Button_Action(6);
#endif
#endif   
#endif
    break;
    
  default:
    break;
  }
}

void btn_double_button_press(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  switch( _btn ) {
  case keylstLeft:
    if( gConfig.inPresentation ) {
      // Replay media
      Msg_PPT_ObjAction(PPT_OBJ_MEDIA, CONTENT_REPLAY);
    } else {
      Msg_DevCCT(OPERATOR_SET, CT_MIN_VALUE);
    }
    break;
    
  case keylstRight:
    if( gConfig.inPresentation ) {
      // Play media or stop play
      Msg_PPT_ObjAction(PPT_OBJ_MEDIA, DEVICE_SW_TOGGLE);
    } else {
      Msg_DevCCT(OPERATOR_SET, CT_MAX_VALUE);
    }
    break;
    
  case keylstUp:
    if( gConfig.inPresentation ) {
      // Last Page
      Msg_PPT_ObjAction(PPT_OBJ_PAGE, CONTENT_GO_LAST);
    } else {
      Msg_DevBrightness(OPERATOR_SET, 100);
    }
    break;
    
  case keylstDown:
    if( gConfig.inPresentation ) {
      // First Page
      Msg_PPT_ObjAction(PPT_OBJ_PAGE, CONTENT_GO_FIRST);
    } else {
      Msg_DevBrightness(OPERATOR_SET, BTN_BR_LOW);
    }
    break;
    
  case keylstCenter:
    // Toggle In-presentation flag
#ifdef ENABLE_SDTM
    Msg_DevOnOff(DEVICE_SW_ON);
#else
    SwitchPPTMode(!gConfig.inPresentation);
#endif
    break;
    
  case keylstFn1:
    break;
    
  case keylstFn2:
    break;
    
  case keylstFn3:
    break;
    
  case keylstFn4:
    break;

  case keylstFLASH:
    break;
    
  case keylstLASER:
    break;
    
  default:
    break;
  }  
}

void btn_long_hold_button_press(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  switch( _btn ) {
  case keylstLeft:
    break;
    
  case keylstRight:
    break;
    
  case keylstUp:
    break;
    
  case keylstDown:
    break;
    
  case keylstCenter:
#ifdef ENABLE_SDTM
    Msg_DevOnOff(DEVICE_SW_OFF);
#else
    // Turn on the laser
    SetLasterBeam(DEVICE_SW_ON);
#endif
    break;
    
  case keylstFn1:
    break;
    
  case keylstFn2:
    break;
    
  case keylstFn3:
    break;
    
  case keylstFn4:
    break;
    
  case keylstFLASH:
    FavoriteDevStatus(0);
    break;
    
  case keylstLASER:
    FavoriteDevStatus(1);
    break;
    
  default:
    break;
  }  
}

void btn_long_button_press(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  switch( _btn ) {
  case keylstLeft:
    if( gConfig.inPresentation ) {
      // Media FB (Fast Backward)
      Msg_PPT_ObjAction(PPT_OBJ_MEDIA, CONTENT_FAST_BACKWARD);
    } else {
      // Reduce CCT and get warmmer white
      Msg_DevCCT(OPERATOR_SUB, BTN_STEP_LONG_CCT);
    }
    break;
    
  case keylstRight:
    if( gConfig.inPresentation ) {
      // Media FF (Fast Forward)
      Msg_PPT_ObjAction(PPT_OBJ_MEDIA, CONTENT_FAST_FORWARD);
    } else {
      // Increase CCT and get colder white
      Msg_DevCCT(OPERATOR_ADD, BTN_STEP_LONG_CCT); 
    }
    break;
    
  case keylstUp:
    if( gConfig.inPresentation ) {
      // n Pages Up
      Msg_PPT_ObjAction(PPT_OBJ_PAGE, CONTENT_FAST_BACKWARD);
    } else {
      // more bright
      Msg_DevBrightness(OPERATOR_ADD, BTN_STEP_LONG_BR);
    }
    break;
    
  case keylstDown:
    if( gConfig.inPresentation ) {
      // n Pages Down
      Msg_PPT_ObjAction(PPT_OBJ_PAGE, CONTENT_FAST_FORWARD);
    } else {
      // less bright
      Msg_DevBrightness(OPERATOR_SUB, BTN_STEP_LONG_BR);
    }
    break;
    
  case keylstCenter:
    // Turn off the laser
    SetLasterBeam(DEVICE_SW_OFF);
    break;
    
  case keylstFn1:
    // Toggle In-presentation flag
    //SwitchPPTMode(!gConfig.inPresentation);
    break;
    
  case keylstFn2:
    break;
    
  case keylstFn3:
    break;
    
  case keylstFn4:
    break;
    
  case keylstFLASH:
    break;
    
  case keylstLASER:
#ifdef CLASS_ROOM_TYPE
    Msg_SpecialDevOnOff(255,0,0);
#endif
    break;
    
  default:
    break;
  }
}

void btn_very_long_hold_button_press(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  switch( _btn ) {
  case keylstLeft:
    break;
    
  case keylstRight:
    break;
    
  case keylstUp:
    break;
    
  case keylstDown:
    break;
    
  case keylstCenter:
    break;
    
  case keylstFn1:
    break;
    
  case keylstFn2:
    // Soft reset
    //WWDG->CR = 0x80;
    break;
    
  case keylstFn3:
    // Erase current device infomation
    //EraseCurrentDeviceInfo();
    break;
    
  case keylstFn4:
    break;
  
  case keylstFLASH:
    // Soft reset
    WWDG->CR = 0x80;
    break;
  
  case keylstLASER:
    break;
    
  default:
    break;
  }
}

void btn_very_long_button_press(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  switch( _btn ) {
  case keylstLeft:
    break;
    
  case keylstRight:
    break;
    
  case keylstUp:
    break;
    
  case keylstDown:
    break;
    
  case keylstCenter:
    // Turn off the laser
    SetLasterBeam(DEVICE_SW_OFF);
    break;
    
  case keylstFn1:
    break;
    
  case keylstFn2:
    break;
    
  case keylstFn3:
    break;
    
  case keylstFn4:
    break;
    
  case keylstFLASH:
    break;
    
  case keylstLASER:
    break;
    
  default:
    break;
  }
}

void btn_double_long_hold_press(uint8_t _btn1, uint8_t _btn2)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn1) || !IS_VALID_BUTTON(_btn2) ) return;
  
  /*
  if( _btn1 == keylstCenter && _btn2 == keylstRight ) {
    // Change current device
    ChangeCurrentDevice(gConfig.indDevice+1);
  }*/
  if( _btn1 == keylstFLASH ) {
    if( _btn2 == keylstFn1 ) {
      // Erase current device infomation
      EraseCurrentDeviceInfo();      
    } else if( _btn2 == keylstFn2 ) {
      // Switch to Config Mode (no entering low power mode for a while)
      SetConfigMode(TRUE, gConfig.indDevice);
    } else if( _btn2 == keylstFn4 ) {
      // Toggle Simple Direct Test Mode
      ToggleSDTM();
    }
  }
}

void app_button_event_handler(uint8_t _btn, button_event_t button_event)
{
  uint8_t sec_btn = keylstDummy;
  
  switch (button_event)
  {
  case BUTTON_INVALID:
    break;
    
  case BUTTON_SHORT_PRESS:
    btn_short_button_press(_btn);
    break;
    
  case BUTTON_DOUBLE_PRESS:
    btn_double_button_press(_btn);
    break;
    
  case BUTTON_LONG_HOLD:
    btn_long_hold_button_press(_btn);
    break;
    
  case BUTTON_LONG_PRESS:
    btn_long_button_press(_btn);
    break;
    
  case BUTTON_VERY_LONG_HOLD:
    btn_very_long_hold_button_press(_btn);
    break;
    
  case BUTTON_VERY_LONG_PRESS:
    btn_very_long_button_press(_btn);
    break;
    
  case DOUBLE_BTN_TRACK:
    if( btn_is_pushed[keylstFn1] ) sec_btn = keylstFn1;
    else if( btn_is_pushed[keylstFn2] ) sec_btn = keylstFn2;
    else if( btn_is_pushed[keylstFn3] ) sec_btn = keylstFn3;
    else if( btn_is_pushed[keylstFn4] ) sec_btn = keylstFn4;
    if( sec_btn < keylstDummy )
      btn_double_long_hold_press(keylstFLASH, sec_btn);
    
  default:
    break;
  }
}

// Only use button1_timer to track double button long hold.
void check_track_double_button(void)
{
  if( btn_is_pushed[keylstFLASH] == TRUE && btn_is_pushed[keylstFn1] == TRUE )
  {
    timer_stop(m_timer_id_btn_detet[keylstFn1]);  // Disable btn2_timer when tracking double hold.
    m_btn_timer_status[keylstFLASH] = BUTTON_STATUS_DOUBLE_TRACK;
    m_btn_timer_status[keylstFn1] = BUTTON_STATUS_INIT;
    double_button_track = TRUE;
    timer_start(m_timer_id_btn_detet[keylstFn1], BUTTON_DOUBLE_BTN_TRACK_DURATION);  //3 s
  } else if( btn_is_pushed[keylstFLASH] == TRUE && btn_is_pushed[keylstFn4] == TRUE ) {
    timer_stop(m_timer_id_btn_detet[keylstFn4]);  // Disable btn2_timer when tracking double hold.
    m_btn_timer_status[keylstFLASH] = BUTTON_STATUS_DOUBLE_TRACK;
    m_btn_timer_status[keylstFn4] = BUTTON_STATUS_INIT;
    double_button_track = TRUE;
    timer_start(m_timer_id_btn_detet[keylstFn4], BUTTON_DOUBLE_BTN_TRACK_DURATION);  //3 s
  } else {
    if (double_button_track == TRUE)
    {
      m_btn_timer_status[keylstFLASH] = BUTTON_STATUS_INIT;
      m_btn_timer_status[keylstFn1] = BUTTON_STATUS_INIT;
      m_btn_timer_status[keylstFn4] = BUTTON_STATUS_INIT;
      double_button_track = FALSE;
      timer_stop(m_timer_id_btn_detet[keylstFn1]);
      timer_stop(m_timer_id_btn_detet[keylstFn4]);
    }
  }
}

void button_push(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  btn_is_pushed[_btn] = TRUE;
  check_track_double_button();
  
  if (double_button_track == FALSE)
  {
    m_btn_timer_status[_btn] = BUTTON_STATUS_LESS_2S;
    timer_start(m_timer_id_btn_detet[_btn], BUTTON_WAIT_2S);
  }
}

void button_release(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  btn_is_pushed[_btn] = FALSE;
  button_event_t button_event = BUTTON_INVALID;
  
  check_track_double_button();
  
  switch (m_btn_timer_status[_btn])
  {
  case BUTTON_STATUS_INIT:
    break;
    
  case BUTTON_STATUS_LESS_2S:
    if (detect_double_btn_press[_btn] == FALSE)
    {
      detect_double_btn_press[_btn] = TRUE;
      timer_start(m_timer_id_double_btn_detet[_btn], BUTTON_DOUBLE_BTN_DURATION);  // 500ms
    }
    else
    {
      button_event = BUTTON_DOUBLE_PRESS;
      detect_double_btn_press[_btn] = FALSE;
      timer_stop(m_timer_id_double_btn_detet[_btn]);
    }
    break;
    
  case BUTTON_STATUS_MORE_2S:
    button_event = BUTTON_LONG_PRESS;
    break;
    
  case BUTTON_STATUS_MORE_5S:
    button_event = BUTTON_VERY_LONG_PRESS;
    break;
    
  default:
    break;
  }
  
  m_btn_timer_status[_btn] = BUTTON_STATUS_INIT;
  if (button_event != BUTTON_INVALID) {
    app_button_event_handler(_btn, button_event);
  }
}

void button_event_handler(uint8_t _pin)
{
  tmrIdleDuration = 0;
  button_first_detect_status = GPIO_ReadInputData(BUTTONS_PORT1);
  button_first_detect_status <<= 8;
  button_first_detect_status |= GPIO_ReadInputData(BUTTONS_PORT2);
  timer_start(m_timer_id_debonce_detet, BUTTON_DEBONCE_DURATION);
  
  // Check Flashlight
  /*
  static BitStatus keyFL = 0;
  if( _pin == GPIO_Pin_0 ) {
    if( keyFL != pinKeySA ) {
      keyFL = pinKeySA;
      // Set Flashlight
      ledFlashLight(keyFL);
    }
  }*/
}
