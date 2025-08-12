#include"stm32l4xx_i2c_driver.h"


static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle );
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle );

void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR2 |=(1<<13);
}
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR2 |= ( 1 << 14);
}
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	pI2Cx->CR2 &= ~(0x3FFU << 0);
	pI2Cx->CR2 |= ((uint32_t)SlaveAddr << 1) & (0x3FFU << 0);
	pI2Cx->CR2 &= ~(1U << 10);
}


static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	pI2Cx->CR2 &= ~(0x3FFU << 0);
	pI2Cx->CR2 |= ((uint32_t)SlaveAddr << 1) & (0x3FFU << 0);
	pI2Cx->CR2 |= (1U << 10);
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << 0);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << 0);
	}

}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}

	}

}
/*Init */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg=0;
	tempreg |=pI2CHandle->I2C_Config.I2C_ACKControl<<15;
	pI2CHandle->pI2Cx->CR2=tempreg;

	pI2CHandle->pI2Cx->TIMINGR |= 0x00411313;  // 100kHz

	tempreg=pI2CHandle->I2C_Config.I2C_DeviceAddress<<1;
	pI2CHandle->pI2Cx->OAR1 |=tempreg;

}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer,uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{


    while (((pI2CHandle->pI2Cx->ISR >>15)&1));

   
    uint32_t cr2 = 0;
    pI2CHandle->pI2Cx->CR2 = (SlaveAddr << 1) |(Len << 16);

    if (Sr == I2C_DISABLE_SR)
    {
        cr2 |= (1<<25);
    }

    pI2CHandle->pI2Cx->CR2 |= cr2;
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
    while (Len > 0)
    {
        while (!(pI2CHandle->pI2Cx->ISR & (1<<0)));
        pI2CHandle->pI2Cx->TXDR = *pTxBuffer;
        pTxBuffer++;
        Len--;
    }

    if (Sr == I2C_DISABLE_SR)
    {
        while (!(pI2CHandle->pI2Cx->ISR & (1<<5)));
        pI2CHandle->pI2Cx->ICR |= (1<<5);
    }
    else
    {
        while (!(pI2CHandle->pI2Cx->ISR & (1<<6)));
    }
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{


    while (((pI2CHandle->pI2Cx->ISR >>15)&1)==0);

    
    uint32_t cr2 = 0;
       cr2 |= (SlaveAddr << 1);
       cr2 |= (Len << 16);
       cr2 |= (0 << 10);
       if (Sr == I2C_DISABLE_SR)
       {
           cr2 |= (1<<25);
       }

       pI2CHandle->pI2Cx->CR2 |= cr2;
	   I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    while (Len > 0)
    {
        while (!(pI2CHandle->pI2Cx->ISR & (1<<2)));
        *pRxBuffer = (uint8_t)pI2CHandle->pI2Cx->RXDR;
        pRxBuffer++;
        Len--;
    }

    if (Sr == I2C_DISABLE_SR)
    {
        while (!(pI2CHandle->pI2Cx->ISR & (1<<5)));
        pI2CHandle->pI2Cx->ICR |= (1<<5);
    }
    else
    {
        while (!(pI2CHandle->pI2Cx->ISR & (1<<6)));
    }
}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;
		
		 while (((pI2CHandle->pI2Cx->ISR >>15)&1)==0);
		 uint32_t cr2 = 0;
		       cr2 |= (SlaveAddr << 1);
		       cr2 |= (Len << 16);
		       cr2 |= (0 << 10);
		       if (Sr == I2C_DISABLE_SR)
		       {
		           cr2 |= (1<<25);
		       }
		       pI2CHandle->pI2Cx->CR2|=cr2;
			   I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		       pI2CHandle->pI2Cx->CR1 |= ((1<<1) | (1<<6) | (1<<7));
	}

	return busystate;
}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer,uint8_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;

    if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        pI2CHandle->pRxBuffer = pRxBuffer;
        pI2CHandle->RxLen     = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
        pI2CHandle->RxSize    = Len;
        pI2CHandle->DevAddr   = SlaveAddr;
        pI2CHandle->Sr        = Sr;
    
        pI2CHandle->pI2Cx->CR2 = 0;
        pI2CHandle->pI2Cx->CR2 |= (SlaveAddr << 1);
        pI2CHandle->pI2Cx->CR2 |= (Len << 16);
        pI2CHandle->pI2Cx->CR2 |= (1 << 10);
		pI2CHandle->pI2Cx->CR2 |= (1 << 13);
        if (Sr == 0)
            pI2CHandle->pI2Cx->CR2 |= (1 << 25);

        pI2CHandle->pI2Cx->CR1 |= ((1<<1) | (1<<6) | (1<<7));
    }

    return busystate;
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle )
{

	if(pI2CHandle->TxLen > 0)
	{
		pI2CHandle->pI2Cx->TXDR = *(pI2CHandle->pTxBuffer);
		pI2CHandle->TxLen--;
		pI2CHandle->pTxBuffer++;

	}

}
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle )
{
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->RXDR;
		pI2CHandle->RxLen--;

	}

	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
		}

			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->RXDR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0 )
	{
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		I2C_CloseReceiveData(pI2CHandle);
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
	}
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << 1);

	pI2CHandle->pI2Cx->CR2 &= ~( 1 <<6);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}

}
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	pI2CHandle->pI2Cx->CR2 &= ~( 1 <<  1);
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << 6);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t isr = pI2CHandle->pI2Cx->ISR;
    uint32_t cr1 = pI2CHandle->pI2Cx->CR1;

    if ((isr & (1U << 3)) && (cr1 & (1U << 3)))
    {
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        }
        else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        }
    }

    if (isr & (1U << 3))
    {
        pI2CHandle->pI2Cx->ICR |= (1U << 3);
    }

    if ((isr & (1U << 6)) && (cr1 & (1U << 6)))
    {
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            if (pI2CHandle->TxLen == 0)
            {
                if (pI2CHandle->Sr == I2C_DISABLE_SR)
                    pI2CHandle->pI2Cx->CR2 |= (1U << 14);

                I2C_CloseSendData(pI2CHandle);
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
            }
        }
    }

    if (isr & (1U << 6))
    {
        pI2CHandle->pI2Cx->ICR |= (1U << 6);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
    }

    if ((isr & (1U << 1)) && (cr1 & (1U << 1)))
    {
        if (isr & (1U << 16))
        {
            if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
            {
                I2C_MasterHandleTXEInterrupt(pI2CHandle);
            }
        }
        else
        {
            if (isr & (1U << 17))
            {
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
            }
        }
    }

    if ((isr & (1U << 2)) && (cr1 & (1U << 2)))
    {
        if (isr & (1U << 16))
        {
            if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
            {
                I2C_MasterHandleRXNEInterrupt(pI2CHandle);
            }
        }
        else
        {
            if (!(isr & (1U << 17)))
            {
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
            }
        }
    }
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t isr = pI2CHandle->pI2Cx->ISR;
    uint32_t cr1 = pI2CHandle->pI2Cx->CR1;

    if ((isr & (1U << 8)) && (cr1 & (1U << 7)))
    {
        pI2CHandle->pI2Cx->ICR |= (1U << 8);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
    }

    if ((isr & (1U << 9)) && (cr1 & (1U << 7)))
    {
        pI2CHandle->pI2Cx->ICR |= (1U << 9);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
    }

    if ((isr & (1U << 10)) && (cr1 & (1U << 7)))
    {
        pI2CHandle->pI2Cx->ICR |= (1U << 10);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
    }

    if ((isr & (1U << 11)) && (cr1 & (1U << 7)))
    {
        pI2CHandle->pI2Cx->ICR |= (1U << 11);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
    }

    if ((isr & (1U << 12)) && (cr1 & (1U << 7)))
    {
        pI2CHandle->pI2Cx->ICR |= (1U << 12);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
    }
}



