#include<stm32f10x_rcc.h>
#include<stm32f10x_gpio.h>
#include "stm32f10x_usart.h"

#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include <misc.h>
int16_t tmp,rx,tx_end;

PIN_CONFIGURATION(A, 0, HIGH, INPUT_PULL_UP);
PIN_CONFIGURATION(A, 1, HIGH, INPUT_PULL_UP);

void USART1_IRQHandler(void)
{
	send_to_uart('J');
	//Receive Data register not empty interrupt
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		rx=1;
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        tmp=USART_ReceiveData (USART1);

        switch(tmp)
        { //И выполняем определённое действие...
            case '0':
            	if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_9))
            	{
            		GPIO_WriteBit(GPIOC,GPIO_Pin_9,Bit_RESET);
            	}
            	else
            	{
            		GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);
            	}
            	//GPIOC->ODR^=GPIO_Pin_9;
            	break;
            case '1':
            	GPIOC->ODR^=GPIO_Pin_8;
            	break;
        }

	}
    //Transmission complete interrupt
	if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_TC);
		tx_end=1;
	}
}


void Delay(volatile uint32_t nCount);

GPIO_InitTypeDef GPIO_InitStructure;
NVIC_InitTypeDef  NVIC_InitStructure; // interruptions

void send_to_uart(uint8_t data)
{
	while(!(USART1->SR & USART_SR_TC)); //Ждем пока бит TC в регистре SR станет 1
	USART1->DR=data; //Отсылаем байт через UART
}



volatile int main(void)
{
	uint8_t uart_rx_data;

	// init for GPIO (LED)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN , ENABLE); //for irq
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9 ;       // two LED (guess on what pin!!)
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitTypeDef PORTA_init_struct;

	// Включаем тактирование порта А и USART1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
	GPIO_InitTypeDef gpio_port;

	// Настраиваем ногу TxD (PA9) как выход push-pull c альтернативной функцией
	PORTA_init_struct.GPIO_Pin = GPIO_Pin_9;
	PORTA_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	PORTA_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &PORTA_init_struct);

	// Настраиваем ногу PA10 как вход UARTа (RxD)
	gpio_port.GPIO_Pin   = GPIO_Pin_10;
	gpio_port.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gpio_port);

	/*//Настраиваем UART
	USART1->BRR=0x9c4; //BaudRate 9600
	USART1->CR1 |= USART_CR1_UE; //Разрешаем работу USART1
	USART1->CR1 |= USART_CR1_TE; //Включаем передатчик*/
	USART_InitTypeDef uart_struct;
	uart_struct.USART_BaudRate            = 9600;
	uart_struct.USART_WordLength          = USART_WordLength_8b;
	uart_struct.USART_StopBits            = USART_StopBits_1;
	uart_struct.USART_Parity              = USART_Parity_No ;
	uart_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	uart_struct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

	//Инициализируем UART
	USART_Init(USART1, &uart_struct);
	//Включаем UART
	USART_Cmd(USART1, ENABLE);


    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    while(1)
    {
        /*GPIO_WriteBit(GPIOC,GPIO_Pin_8,Bit_RESET);
        GPIO_WriteBit(GPIOC,GPIO_Pin_9,Bit_SET);
        Delay(8000000);
        GPIO_WriteBit(GPIOC,GPIO_Pin_9,Bit_RESET);
        GPIO_WriteBit(GPIOC,GPIO_Pin_8,Bit_SET);*/
        Delay(8000000);
    	send_to_uart('P');
        send_to_uart('i');
        send_to_uart('z');
        send_to_uart('d');
        send_to_uart('e');
        send_to_uart('t');
        send_to_uart('s');
        send_to_uart(':');
        send_to_uart(')');
        send_to_uart('\n');
        send_to_uart('\r');
//        if (USART1->SR & USART_SR_RXNE)
//        { // ... не пришло ли что-то в UART ?
//            uart_rx_data=USART1->DR; //Считываем то что пришло в переменную...
        /*    switch(uart_rx_data)
            { //И выполняем определённое действие...
                case '0':
                	if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_9))
                	{
                		GPIO_WriteBit(GPIOC,GPIO_Pin_9,Bit_RESET);
                	}
                	else
                	{
                		GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);
                	}
                	//GPIOC->ODR^=GPIO_Pin_9;
                	break;
                case '1':
                	GPIOC->ODR^=GPIO_Pin_8;
                	break;
            }*/
      //  }
    }
}

//-------
void Delay(volatile uint32_t nCount)
{
    for (; nCount > 0; nCount--);
}
