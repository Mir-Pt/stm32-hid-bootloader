/*
* STM32 HID Bootloader - USB HID bootloader for STM32F10X
* Copyright (c) 2018 Bruno Freitas - bruno@brunofreitas.com
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stm32f10x.h>
#include <stdlib.h>

#include "usb.h"
#include "bitwise.h"
/* USB Buffer */
USB_RxTxBuf_t RxTxBuffer[MAX_EP_NUM];
/* 全局变量和描述符定义 */
volatile uint8_t DeviceAddress = 0;
volatile uint16_t DeviceConfigured = 0, DeviceStatus = 0;
void (*_EPHandler)(uint16_t) = NULL;
void (*_USBResetHandler)(void) = NULL;

/* USB字符串描述符 */
/* 注:USB字符串使用Unicode编码，每个字符后跟一个0字节。 */
const uint8_t sdVendor[] = {
	0x2A, // Size,
	0x03, // Descriptor type 
	'w', 0, 'w', 0, 'w', 0, '.', 0, 'b', 0, 'r', 0, 'u', 0, 'n', 0, 'o', 0,
	'f', 0, 'r', 0, 'e', 0, 'i', 0, 't', 0, 'a', 0, 's', 0, '.', 0, 'c', 0,
	'o', 0, 'm', 0
};

const uint8_t sdProduct[] = {
	0x2C, // Size,
	0x03, // Descriptor type
	'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, 'F', 0, ' ', 0, 'H', 0, 'I', 0,
	'D', 0, ' ', 0, 'B', 0, 'o', 0, 'o', 0, 't', 0, 'l', 0, 'o', 0, 'a', 0,
	'd', 0, 'e', 0, 'r', 0
};

const uint8_t sdSerial[] = {
	0x16, // Size,
	0x03, // Descriptor type
	'1',0,'2',0,'3',0,'4',0,'5',0,'6',0,'7',0,'8',0,'9',0,'0',0
};

const uint8_t sdLangID[] = {
		0x04, // Size,
		0x03, // Descriptor type
		0x09, 0x04
};
/*  USB端点处理程序 */
void USB_PMA2Buffer(uint8_t EPn) {
	uint8_t Count = RxTxBuffer[EPn].RXL = (_GetEPRxCount(EPn) & 0x3FF);//获取接收数据长度（低10位有效）
	uint32_t *Address = (uint32_t *) (PMAAddr + _GetEPRxAddr(EPn) * 2);//PMA地址需要乘以2，因为STM32的USB PMA是16位寻址
	uint16_t *Destination = (uint16_t *) RxTxBuffer[EPn].RXB;
	//  从PMA复制数据到用户缓冲区
	for (uint8_t i = 0; i < Count; i++) {
		*(uint16_t *) Destination = *(uint16_t *) Address;
		Destination++;
		Address++;
	}
}
/*  从用户缓冲区复制数据到PMA */
void USB_Buffer2PMA(uint8_t EPn) {
	uint32_t *Destination;
	uint8_t Count;

	Count = RxTxBuffer[EPn].TXL <= RxTxBuffer[EPn].MaxPacketSize ? RxTxBuffer[EPn].TXL : RxTxBuffer[EPn].MaxPacketSize;
	_SetEPTxCount(EPn, Count);

	Destination = (uint32_t *) (PMAAddr + _GetEPTxAddr(EPn) * 2);

	for (uint8_t i = 0; i < (Count + 1) / 2; i++) {
		*(uint32_t *) Destination = *(uint16_t *) RxTxBuffer[EPn].TXB;
		Destination++;
		RxTxBuffer[EPn].TXB++;
	}

	RxTxBuffer[EPn].TXL -= Count;
}
/* 发送数据到主机 */
void USB_SendData(uint8_t EPn, uint16_t *Data, uint16_t Length) {

	if (EPn > 0 && !DeviceConfigured) {
		return;// 端点0以外的端点需要设备配置后才能使用
	}

	RxTxBuffer[EPn].TXL = Length;

	RxTxBuffer[EPn].TXB = Data;

	if (Length > 0) {
		USB_Buffer2PMA(EPn);// 将数据从应用缓冲区复制到PMA
	} else {
		_SetEPTxCount(EPn, 0);// 零长度包
	}

	_SetEPTxValid(EPn);// 设置端点TX状态为VALID，启动传输
}
/* Shutdown USB 关闭USB */
//USB关闭遵循STM32的标准流程：禁用中断→清除标志→关闭USB宏单元→模拟断开连接→禁用USB时钟。
void USB_Shutdown() {
	bit_set(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);

	// 禁用USB中断
	NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
	_SetISTR(0);

	DeviceConfigured = DeviceStatus = 0;

	_EPHandler = NULL;

	_USBResetHandler = NULL;

	// 关闭USB宏单元(FRES + PWDN)
	_SetCNTR(0x03);

	// 配置PA12为开漏输出并拉低（模拟断开连接） (b01)
	bit_set(GPIOA->CRH, GPIO_CRH_CNF12_0);
	bit_clear(GPIOA->CRH, GPIO_CRH_CNF12_1);

	// Set PA_12 to output
	bit_set(GPIOA->CRH, GPIO_CRH_MODE12);// PA_12 set as: Output mode, max speed 50 MHz.

	//拉低PA12
	GPIOA->BRR = GPIO_BRR_BR12;

	// 禁用USB时钟 APB1
	bit_clear(RCC->APB1ENR, RCC_APB1ENR_USBEN);
}
/* Turn on USB 打开USB */
static void USB_TurnOn() {
	bit_set(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);

	// PA_12 output mode: General purpose Input Float (b01)	
	bit_set(GPIOA->CRH, GPIO_CRH_CNF12_0);
	bit_clear(GPIOA->CRH, GPIO_CRH_CNF12_1);

	// Set PA_12 to Input mode
	bit_clear(GPIOA->CRH, GPIO_CRH_MODE12);
}
/* Initialize USB 初始化USB */
//USB初始化遵循STM32的标准流程：时钟使能→强制复位→等待复位完成→清除标志→设置中断。
void USB_Init(void (*EPHandlerPtr)(uint16_t), void (*ResetHandlerPtr)(void)) {

	//  重置所有端点的RxTxBuffer结构内的RX和TX长度
	for(int i = 0; i < MAX_EP_NUM; i++) {
		RxTxBuffer[i].RXL = RxTxBuffer[i].TXL = 0;
	}

	_EPHandler = EPHandlerPtr;
	_USBResetHandler = ResetHandlerPtr;

	USB_TurnOn();//配置USB相关的GPIO

	DeviceConfigured = DeviceStatus = 0;

	bit_set(RCC->APB1ENR, RCC_APB1ENR_USBEN); // 使能USB时钟
	NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn); // 使能USB中断

	/*** CNTR_PWDN = 0 ***/
	_SetCNTR(CNTR_FRES);// 强制USB复位

	/* The following sequence is recommended:
	 1- FRES = 0
	 2- Wait until RESET flag = 1 (polling)
	 3- clear ISTR register */

	/*** CNTR_FRES = 0 ***/
	_SetCNTR(0);

	/* 等待RESET标志置位 flag = 1 (polling) */
	while (!(_GetISTR() & ISTR_RESET));

	/***   清除挂起的中断 ***/
	_SetISTR(0);

	/***   设置中断掩码 ***/
	_SetCNTR(CNTR_CTRM | CNTR_RESETM | CNTR_SUSPM | CNTR_WKUPM);
}
/* Is device configured 检查设备是否配置 */
uint16_t USB_IsDeviceConfigured() {
	return DeviceConfigured;
}
/*  USB低优先级或CAN RX0中断处理程序 */
void USB_LP_CAN1_RX0_IRQHandler() {
	// 处理Reset中断
	if (_GetISTR() & ISTR_RESET) {
		_SetISTR(_GetISTR() & CLR_RESET);
		if(_USBResetHandler) {
			_USBResetHandler();// 调用上层复位处理函数
		}
		return;
	}

	// 处理端点数据传输中断
	if (_GetISTR() & ISTR_CTR) { // Handle data on EP 处理EP上的数据
		if(_EPHandler) {
			_EPHandler(_GetISTR());// 调用端点数据处理函数
		}
		_SetISTR(_GetISTR() & CLR_CTR);
		return;
	}

	// Handle DOVR 处理DOVR
	if (_GetISTR() & ISTR_DOVR) {
		_SetISTR(_GetISTR() & CLR_DOVR);
		return;
	}

	// Handle Suspend 处理挂起
	if (_GetISTR() & ISTR_SUSP) {
		_SetISTR(_GetISTR() & CLR_SUSP);

		// If device address is assigned, then reset it 如果分配了设备地址，则重置它
		if (_GetDADDR() & 0x007f) {
			_SetDADDR(0);
			_SetCNTR(_GetCNTR() & ~CNTR_SUSPM);
		}

		return;
	}

	// Handle Error 处理错误
	if (_GetISTR() & ISTR_ERR) {
		_SetISTR(_GetISTR() & CLR_ERR);
		return;
	}

	// Handle Wakeup 处理唤醒
	if (_GetISTR() & ISTR_WKUP) {
		_SetISTR(_GetISTR() & CLR_WKUP);
		return;
	}

	// Handle SOF 处理SOF
	if (_GetISTR() & ISTR_SOF) {
		_SetISTR(_GetISTR() & CLR_SOF);
		return;
	}

	// Handle ESOF 处理ESOF
	if (_GetISTR() & ISTR_ESOF) {
		_SetISTR(_GetISTR() & CLR_ESOF);
		return;
	}

	// Default to clear all interrupt flags 默认清除所有中断标志
	_SetISTR(0);
}

