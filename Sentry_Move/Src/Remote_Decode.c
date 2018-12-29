#include "Remote_Decode.h"
#include "Remote_Ctrl.h"

RemoteCtrl_t RemoteCtrlData;       //遥控器输入
uint8_t isRevRemoteData = 0;			//遥控器接收标志位

/**
  * @brief	根据遥控器协议对进行接收到的数据进行处理
  * @param	pData:	一个指向8位数据的指针
  * @retval	None
  */
void RC_DataHandle(uint8_t *pData)
{
	if (pData == NULL)
    {
        return;
    }
	
	/* pData[0]为ch0的低8位，Data[1]的低3位ch0的高3位 */
	RemoteCtrlData.remote.ch0 = (uint16_t)(pData[0] | pData[1] << 8) & 0x07FF;
	
	/* pData[1]的高5位为ch1的低5位，pData[2]的低6位为ch1的高6位 */
	RemoteCtrlData.remote.ch1 = (uint16_t)(pData[1] >> 3 | pData[2] << 5) & 0x07FF;
	
	/* pData[2]的高2位为ch2的低2位, pData[3]为ch2的中8位，pData[4]的低1位为ch2的高1位 */
	RemoteCtrlData.remote.ch2 = (uint16_t)(pData[2] >> 6 | pData[3] << 2 | pData[4] << 10) & 0x07FF;
	
	/* pData[4]的高7位为ch3的低7位，pData[5]的低4位为ch3的高4位 */
	RemoteCtrlData.remote.ch3 = (uint16_t)(pData[4] >> 1 | pData[5] << 7) & 0x07FF;

	/* pData[5]的高2位为s1 */
	RemoteCtrlData.remote.s1  = ((pData[5] >> 6) & 0x03);
	
	/* pData[6]的6，7位为s2 */
	RemoteCtrlData.remote.s2  = ((pData[5] >> 4) & 0x03);
	
	/* pData[6],pData[7]为x */
	RemoteCtrlData.mouse.x    = (int16_t)(pData[6] | pData[7] << 8);
	
	/* pData[8],pData[9]为y */
	RemoteCtrlData.mouse.y    = (int16_t)(pData[8] | pData[9] << 8);
	
	/* pData[10],pData[11]为z */
	RemoteCtrlData.mouse.z    = (int16_t)(pData[10] | pData[11] << 8);
	
	/* pData[12]为左键 */
	RemoteCtrlData.mouse.press_l = pData[12];
	
	/* pData[13]为右键 */
	RemoteCtrlData.mouse.press_r = pData[13];
	
	/* pData[14],pData[15]为键盘值 */
	RemoteCtrlData.key.v 		 = (int16_t)(pData[14] | pData[15] << 8);
	
	/* 拨动开关进行解码 */
	Remote_Process();
}
