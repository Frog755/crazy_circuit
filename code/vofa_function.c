
#include "vofa_function.h"
#include <stdio.h>
#include "base_transfer.h"
#include "vofa_uart.h"
#include "zf_common_headfile.h"
#include "pid.h"

/*VOFA type structure*/
vofaJustFloatFrame JustFloat_Data;
vofaCommand        vofaCommandData;

/**
* @param vofaJFFframe: 鍖呭惈鏁版嵁甯х殑缁撴瀯浣�
* @return void
*/
void vofaSendJustFloat(vofaJustFloatFrame *vofaJFFrame)
{
	uint8_t i;
	uint8_t u8Array[4];
	for (i = 0; i < CH_COUNT; i++)
	{
		float2uint8Array(u8Array, &vofaJFFrame->fdata[i], 0);
		uartSendData(u8Array, sizeof(u8Array));
	}
	uartSendData(vofaJFFrame->frametail, FRAME_TAIL_SIZE);
}

/**
* @param fdata: 鎸囧悜瑕佸彂閫佺殑娴偣鏁版嵁鐨勬寚閽�
* @param ulSize锛� 瑕佸彂閫佺殑鏁版嵁涓暟
* @return void
*/
void vofaSendFirewater(float *fdata, uint32_t ulSize)
{
	uint32_t i;
	for (i = 0; i < ulSize - 1; i++)
	{
		printf("%.6f,", *(fdata + i));
	}
	printf("%.6f\n", *(fdata + i));
}

/**
* @param pData: 鎸囧悜瑕佸彂閫佺殑鍗曞瓧鑺傛暟鎹殑鎸囬拡
* @param ulSize锛� 瑕佸彂閫佺殑鏁版嵁涓暟
* @return void
*/
void vofaSendRawdata(uint8_t *pData, uint32_t ulSize)
{
	uint32_t i;
	for (i = 0; i < ulSize; i++)
	{
		uartSendByte(*(pData + i));
	}
}

/**
* @brief 鍒濆鍖朖ustFloat甯х粨鏋勪綋
*/
void vofaJustFloatInit(void)
{
	vofaCommandData.cmdID                   = INVALID;
	vofaCommandData.cmdType                 = INVALID;
	vofaCommandData.completionFlag          = 0;
	JustFloat_Data.frametail[0] = 0x00;
	JustFloat_Data.frametail[1] = 0x00;
	JustFloat_Data.frametail[2] = 0x80;
	JustFloat_Data.frametail[3] = 0x7f;
}

/**
* @brief 灏嗕覆鍙ｆ敹鍒扮殑鏁版嵁鍒ゆ柇骞跺瓨鍏ユ暟鎹寘涓紝骞舵瘮瀵瑰抚鎺у埗鎺ユ敹瀹屾垚鏍囧織浣嶇疆浣�
* @param byte_data锛� 涓插彛鎺ユ敹鍒扮殑瀛楄妭鏁版嵁
*/
void uartCMDRecv(uint8_t byte_data) //姝ゅ嚱鏁版斁鍦ㄤ覆鍙ｄ腑鏂腑
{
	vofaCommandData.uartRxPacket[vofaRxBufferIndex] = byte_data;

	if (vofaCommandData.uartRxPacket[vofaRxBufferIndex - 1] == '!' && vofaCommandData.uartRxPacket[vofaRxBufferIndex] == '#')
	{
		vofaCommandData.completionFlag = 1;
		vofaRxBufferIndex  = 0;
	}

	else if (vofaRxBufferIndex > (CMD_FRAME_SIZE - 1))
	{
		vofaCommandData.completionFlag = 0;
		vofaRxBufferIndex  = 0;
		memset(vofaCommandData.uartRxPacket, 0, 10);
	}

	else
	{
		vofaRxBufferIndex++;
	}
}

/**
* @brief vofa鍛戒护甯цВ鏋�
*/
void vofaCommandParse(void)
{
	uint8_t* pRxPacket;
	pRxPacket = vofaCommandData.uartRxPacket;

	if (vofaCommandData.uartRxPacket[0] != '@' || vofaCommandData.uartRxPacket[3] != '=' || vofaCommandData.uartRxPacket[CMD_FRAME_SIZE - 2] != '!' || vofaCommandData.
		uartRxPacket[CMD_FRAME_SIZE - 1] != '#')
	{
		memset(vofaCommandData.uartRxPacket, 0, CMD_FRAME_SIZE);
		return;
	}

	switch (vofaCommandData.uartRxPacket[1])
	{
		case 'P': vofaCommandData.cmdType = KP;
			break;
                case 'A': vofaCommandData.cmdType = KP2;
			break;        
		case 'D': vofaCommandData.cmdType = KD;
			break;
                case 'd': vofaCommandData.cmdType = KD2;
			break;       
                case 'L': vofaCommandData.cmdType = KPL;
                        break;
                case 'I': vofaCommandData.cmdType = KIL;
                        break;
                case 'R': vofaCommandData.cmdType = KPR;
                        break;
                case 'i': vofaCommandData.cmdType = KIR;
                        break;
		default: vofaCommandData.cmdType = INVALID;
			break;
	}

	switch (vofaCommandData.uartRxPacket[2])
	{
		case '1': vofaCommandData.cmdID = Direct_Assignment;
			break;
		case '2': vofaCommandData.cmdID = Increase;
			break;
		case '3': vofaCommandData.cmdID = Decrease;
			break;
		default: vofaCommandData.cmdID = INVALID;
			break;
	}
	memcpy(vofaCommandData.validData, pRxPacket + 4, 4);

	vofaCommandData.floatData = uint8Array2Float(vofaCommandData.validData, 0);

	pRxPacket = NULL;
	memset(vofaCommandData.validData, 0, 4);
	memset(vofaCommandData.uartRxPacket, 0, CMD_FRAME_SIZE);
}



void vofa_uart_proc(void)
{
       if (vofaCommandData.completionFlag == 1)
       {
        vofaCommandData.completionFlag= 0;
        vofaCommandParse();
        switch (vofaCommandData.cmdType){
        case KP:
          switch (vofaCommandData.cmdID){
          case Direct_Assignment:
            turn_pid.Kp = vofaCommandData.floatData;
            break;
          }
          break;
        case KP2:
          switch (vofaCommandData.cmdID){
          case Direct_Assignment:
            turn_pid.Kp2 = vofaCommandData.floatData;
            break;
          }
          break;
        case KD:
          switch (vofaCommandData.cmdID){
          case Direct_Assignment:
            turn_pid.Kd = vofaCommandData.floatData;
            break;
          }
          break;
        case KD2:
          switch (vofaCommandData.cmdID){
          case Direct_Assignment:
            turn_pid.Kd2 = vofaCommandData.floatData;
            break;
          }
          break;     
        case KPL:
          switch (vofaCommandData.cmdID){
          case Direct_Assignment:
            pid_L.Kp = vofaCommandData.floatData;
            break;
          }
          break;    
        case KIL:
          switch (vofaCommandData.cmdID){
          case Direct_Assignment:
            pid_L.Ki = vofaCommandData.floatData;
            break;
          }
          break; 
        case KPR:
          switch (vofaCommandData.cmdID){
          case Direct_Assignment:
            pid_R.Kp  = vofaCommandData.floatData;
            break;
          }
          break;
        case KIR:
          switch (vofaCommandData.cmdID){
          case Direct_Assignment:
            pid_R.Ki = vofaCommandData.floatData;
            break;
          }
          break;
        }
//      JustFloat_Data.fdata[0] = turn_pid.Kp;
//      JustFloat_Data.fdata[1] = turn_pid.Kp2;
//      JustFloat_Data.fdata[2] = turn_pid.Kd;
//      JustFloat_Data.fdata[3] = turn_pid.Kd2;
      JustFloat_Data.fdata[4] = 0;
      JustFloat_Data.fdata[5] = 0;
      JustFloat_Data.fdata[6] = 0;
      JustFloat_Data.fdata[7] = 0;
      vofaSendJustFloat(&JustFloat_Data);
      }
}
