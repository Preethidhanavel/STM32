

#ifndef INC_STM32L4XX_I2C_DRIVER_C_
#define INC_STM32L4XX_I2C_DRIVER_C_

#include"stm32l4xx.h"
typedef struct{
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
}I2C_Config_t;

typedef struct{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState;
	uint8_t DevAddr;
	uint32_t RxSize;
	uint8_t Sr;
}I2C_Handle_t;

#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM	400000
#define I2C_SCL_SPEED_FM2K  200000

#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

#define I2C_DISABLE_SR       0
#define I2C_ENABLE_SR        1

#define I2C_READY 					0
#define I2C_BUSY_IN_RX 				1
#define I2C_BUSY_IN_TX 				2

#define I2C_EV_TX_CMPLT  	 	0
#define I2C_EV_RX_CMPLT  	 	1
#define I2C_EV_STOP       		2
#define I2C_ERROR_BERR 	 		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*Init and De-init*/
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/*Data Send and Receive*/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);
void I2C_BurstWrite(I2C_Handle_t*, uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint32_t len);
void I2C_BurstRead(I2C_Handle_t*, uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint32_t len);

uint8_t I2C_BurstWriteIT(I2C_Handle_t*, uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint32_t len);
uint8_t I2C_BurstReadIT(I2C_Handle_t*, uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint32_t len);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);




/*IRQ Configuration and ISR handling*/

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

/*callback*/
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);

#endif /* INC_STM32L4XX_I2C_DRIVER_C_ */


