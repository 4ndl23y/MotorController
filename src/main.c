
#include "main.h"

int main(void)
{
	for(uint32_t d = 0; d < 800000; d++); 	// just wait for voltage stabilization
	
	RCC_Init();
	TIM_Init();
	EXTI_Init();
    display_Init();	
    
	#ifdef SERIALDEBUG 
        UART_Init(); 
    #endif
    	
	__enable_irq();  
	
	
	PIDset();
	
	TIM3->CR1 |= TIM_CR1_CEN;		// Hall sensor start
	TIM16->CR1 |= TIM_CR1_CEN;		// Hall wachdog timer start
	
	while(1)
	{
		if(m.calcPID){
			PIDcalculate();  	// calculate new motorPulse value
			m.calcPID = 0;
		}
        
        printValue();   //print Task
        
	}
} 

//___Functions______________________________________________________________________________________
void PIDcalculate() 
{
	int32_t RPMerror;
	int32_t PIDoutput;
	
	if(disp.mode == 6){	
		//PIDreset();
        arm_pid_init_f32(&m.PID, 1);
		disp.mode = 0;
	}
	
	RPMerror = HallDeltaTime - 30000000 / (m.rpmSet + 1);	// PID error
	
	PIDoutput = (int32_t)(arm_pid_f32(&m.PID, RPMerror));	// calculate
    
	if(PIDoutput > m.PIDoutMax){				// check MIN/MAX
		PIDoutput = m.PIDoutMax;
        m.PID.state[2] = m.PIDoutMax;
	}
	else if(PIDoutput < m.PIDoutMin){
		PIDoutput = m.PIDoutMin;
        m.PID.state[2] = m.PIDoutMin;
	}
    
	#ifdef TEST     // for functional testing define TEST
        MotorPulse = (uint16_t)(RPMset / 100);
    #else
        m.pulse = (uint16_t)(PIDoutput);
    #endif
        
    m.on = (m.rpmSet >= MINRPM ? 1 : 0);		// check ON/OFF
    
    #ifdef SERIALDEBUG
        if(UARTonFlag){
            USARTprint("$%d %d, %d, %d;", m.rpmSet, 30000000/HallDeltaTime, RPMerror, m.pulse);
        }
    #endif
}

void printValue(){
    static uint16_t i = 0;
    if(i > 6000){
        switch(disp.mode){		// Menu
            case 0:  displayPrint(m.rpmSet / 1000, 0);		    break;
            case 1:  displayPrint((uint8_t)(m.PID.Kp * 10), 1);	break;
            case 2:  displayPrint((uint8_t)(m.PID.Ki * 100), 2);	break;
            case 3:  displayPrint((uint8_t)(m.PID.Kd * 100), 3);	break;
            case 4:  displayPrint(m.savePID, 1);			        break;
            default: displayPrint(99, 3);
        }
        i = 0;
    }
    i++;
}

void displayPrint(uint8_t number, uint8_t dpPosition)		
{
	static uint8_t digit = 0;

	if(digit){						// write to 1 digit
		number = number / 10;
		GPIOA->BSRR = NumbRegA;		// reset all
		GPIOB->BSRR = NumbRegB;
		GPIOA->BSRR = ((uint32_t)(NumbRegAset[number]) << 16); 	// print number to digit
		GPIOB->BSRR = ((uint32_t)(NumbRegBset[number]) << 16) 
				| GPIO_BSRR_BR_6 
				| GPIO_BSRR_BS_5
				| ((dpPosition & 0x2) ? GPIO_BSRR_BR_12 : GPIO_BSRR_BS_12);
	}else{							// write to 2 digit
		number = number % 10;
		GPIOA->BSRR = NumbRegA;
		GPIOB->BSRR = NumbRegB;
		GPIOA->BSRR = ((uint32_t)(NumbRegAset[number]) << 16);
		GPIOB->BSRR = ((uint32_t)(NumbRegBset[number]) << 16) 
				| GPIO_BSRR_BS_6 
				| GPIO_BSRR_BR_5
				| ((dpPosition & 0x1)  ? GPIO_BSRR_BR_12 : GPIO_BSRR_BS_12);
	}
	digit = ~digit;
}

void PIDreset()
{
	m.PID.Kp = 1.0;
	m.PID.Ki = 0.0;
	m.PID.Kd = 0.0;
	m.PIDoutMax = 9000; 			// WARN: absolute max 10000!
	m.PIDoutMin = 0;
	arm_pid_init_f32(&m.PID, 1);		// DSP lib PID init
}

void PIDset()
{
	readVal(&m.PID.Kp, 0); 
	readVal(&m.PID.Ki, 8);
	readVal(&m.PID.Kd, 16);
	m.PIDoutMax = 9000; 			// WARN: absolute max 10000!
	m.PIDoutMin = 0;
	arm_pid_init_f32(&m.PID, 1);		// DSP lib PID init
}

//___save_settings_to_flash_________________________________
void PIDsave()
{
    m.savePID++;
    if(m.savePID > 5){
		m.savePID = 0;
		disp.mode = 0;
		
        while(FLASH->SR & FLASH_SR_BSY);   // Whaiting, if flash not ready
        if(FLASH->SR & FLASH_SR_EOP){     
            FLASH->SR = FLASH_SR_EOP;
        }
        FLASH->CR |= FLASH_CR_PER;
        FLASH->AR = CONFIGFLASHADDR;       // Any address, that belongs to erasable page
        FLASH->CR |= FLASH_CR_STRT;
        while(!(FLASH->SR & FLASH_SR_EOP));
        FLASH->SR = FLASH_SR_EOP;
        FLASH->CR &= ~FLASH_CR_PER;
        while(FLASH->SR & FLASH_SR_BSY);
        if(FLASH->SR & FLASH_SR_EOP){
            FLASH->SR = FLASH_SR_EOP;
        }
        FLASH->CR |= FLASH_CR_PG;

		writeVal(m.PID.Kp, 0);
		writeVal(m.PID.Ki, 8);
		writeVal(m.PID.Kd, 16);
		
        FLASH->SR = FLASH_SR_EOP;
        FLASH->CR &= ~(FLASH_CR_PG);
    }
}

void writeVal(float32_t k, uint8_t addr)
{
	union float32ToUint16{
		float32_t k;
		uint16_t kChar[2];
	} kToFlash;

	kToFlash.k = k;
	for(uint8_t i = 0; i < 2; i++){
        *(volatile uint16_t*)(CONFIGFLASHADDR + addr + i * 2) = kToFlash.kChar[i];        
        while(!(FLASH->SR & FLASH_SR_EOP));
        FLASH->SR = FLASH_SR_EOP;
	}
}

void readVal(float32_t* k, uint8_t addr)
{
	union float32ToUint16{
		float32_t k;
		uint16_t kChar[2];
	} kFromFlash;
	
	for(uint8_t i = 0; i < 2; i++){
		kFromFlash.kChar[i] = *(uint16_t*)(CONFIGFLASHADDR + addr + i * 2);
	}
		
	*k = kFromFlash.k;
}
//___save_settings_to_flash_end_____________________________

//___serial_debug___________________________________________
#ifdef SERIALDEBUG
void USARTprint(char* format, ...)
{
	va_list dataList;
	va_start(dataList, format);
	vsprintf(txBuff, format, dataList);
	va_end(dataList);

	UARTdataSend(txBuff);
}

void UARTdataSend(char* str)
{
	uint8_t i = 0;
	while(str[i])
	{
		if(USART1->ISR & USART_ISR_TXE){ // if tx empty
			USART1->TDR = str[i];
			if(str[i] == '\r'){
				return;
			}
			i++;
		}
	}
}
#endif
//___serial_debug_end______________________________________

//___Handler________________________________________________________________________________________
//_____Hall_input_capture_tim_interrupt_____________________________________________________________
void TIM3_IRQHandler(void)
{	 
	HallDeltaTime = TIM3->CCR1;	    // get hall input capture (delta time)
	TIM3->CNT = 0;					// reset hall tim
	TIM16->CNT = 0;					// reset hall wachdog TIM16
	TIM3->SR &= ~TIM_SR_CC1OF;
	TIM3->SR &= ~TIM_SR_UIF;
}
//_____tim_tinterrupt__________________________________________________________________
void TIM17_IRQHandler()	
{
	if(but1.on && but2.on){
        disp.mode = (disp.mode < 4 ? disp.mode+1 : 0);
        m.savePID = 0;
    }
    else if(but2.on){
        switch(disp.mode){
			case 0: m.rpmSet < 100000 ? m.rpmSet += 1000 : 0; break; // speed++
			case 1: m.PID.Kp += 0.1;    break;
			case 2: m.PID.Ki += 0.01;   break;
			case 3: m.PID.Kd += 0.01;   break;
            case 4: PIDsave();          break;  // Press 5 times to save Kp, Ki, Kd
		}
    }
    else if(but1.on){
        switch(disp.mode){
			case 0: m.rpmSet > 1000 ? m.rpmSet -= 1000 : 0; break; // speed--
			case 1: m.PID.Kp -= 0.1;    break;
			case 2: m.PID.Ki -= 0.01;   break;
			case 3: m.PID.Kd -= 0.01;   break;
            case 4: PIDset();           break;
		}
    }
    but1.on = 0;
    but2.on = 0;
    TIM17->CR1 &= ~TIM_CR1_CEN;
	TIM17->SR &= ~TIM_SR_CC1IF;
	TIM17->SR &= ~TIM_SR_UIF;
}
//_____Hall_wachdog_tim_tinterrupt__________________________________________________________________
void TIM16_IRQHandler()	
{
	HallDeltaTime = HALLDELTAMAX;
	TIM16->SR &= ~TIM_SR_CC1IF;
	TIM16->SR &= ~TIM_SR_UIF;
}

//_____Zero_Cross_detection_handler_by_falling_and_rising___________________________________________
void EXTI0_IRQHandler(void)				
{			
	if(m.on){
		if(!(GPIOB->IDR & GPIO_IDR_0)){         // First or second zero crossing per period
			TIM2->CCR4 = WAVELEN - m.pulse - MOTORPULSELEN;	// time delay before pulse
			TIM2->ARR = WAVELEN - m.pulse;		
		}else{
			TIM2->CCR4 = WAVELEN - m.pulse - KF - MOTORPULSELEN;
			TIM2->ARR = WAVELEN - m.pulse - KF;
		}			
		TIM2->CNT = 0;
		TIM2->EGR |= TIM_EGR_UG;		// update timer settings
		TIM2->CR1 |= TIM_CR1_CEN; 		// start motorPulse timer
	}
	m.calcPID = 1;				// calculate new PID value flag
	EXTI->PR |= EXTI_PR_PR0;
}

//_____Button1_handler______________________________________________________________________________
void EXTI2_TSC_IRQHandler(void)	
{		
	but1.on = 1;
    TIM17->CR1 |= TIM_CR1_CEN;
    EXTI->PR |= EXTI_PR_PR2;
}

//_____Button2_handler______________________________________________________________________________
void EXTI15_10_IRQHandler(void)
{		
    but2.on = 1;
    TIM17->CR1 |= TIM_CR1_CEN;
	EXTI->PR |= EXTI_PR_PR10;
}
//___Handler_end____________________________________________________________________________________

//_____Peripheral_Initialization____________________________________________________________________
void RCC_Init()
{
	//_____Flash_configuration__________________________________________________________________
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_2;		// latency for 64 MHz
	FLASH->ACR |= FLASH_ACR_PRFTBE;         // Allow buffer
	
	FLASH->KEYR = 0x45670123; // Magic numbers from STMicroelectronics
	FLASH->KEYR = 0xCDEF89AB; //  which allows to change flash mem (look RM0316 page 68)
	
	//_____Clock_configuration_64_MHz___________________________________________________________
	RCC->CR |= RCC_CR_HSION;
	while((RCC->CR & RCC_CR_HSIRDY) == 0);
	RCC->CR &= ~RCC_CR_PLLON;
	while((RCC->CR & RCC_CR_PLLRDY) != 0);	
	
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_DIV2; 	// set HSI as PLL source
	RCC->CFGR |= RCC_CFGR_PLLMUL16			// x16
		      | RCC_CFGR_HPRE_DIV1			// AHB PRE = 1
		      | RCC_CFGR_PPRE1_DIV2			// APB1 PRESCKALER = 2
		      | RCC_CFGR_PPRE2_DIV1;		// APB2 PRESCKALER = 1
	
	RCC->CFGR3 |= RCC_CFGR3_USART1SW_SYSCLK;// USART
	
	RCC->CR |= RCC_CR_PLLON; 
	while((RCC->CR & RCC_CR_PLLRDY) == 0); 
	RCC->CFGR |= RCC_CFGR_SW_PLL;			// PLL selected as system clock
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
	SystemCoreClockUpdate();
	
	//_____RCC_configuration____________________________________________________________________
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;	
	RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
}

void TIM_Init()
{
	//_____Motor_Pulse_onePulse_mode_tim2_ch4_(GPIOB_11)________________________________________
	GPIOB->MODER |= AF << GPIO_MODER_MODER11_Pos;
	GPIOB->OSPEEDR |= HS << GPIO_OSPEEDER_OSPEEDR11_Pos;
	GPIOB->PUPDR |= PULL_DOWN << GPIO_PUPDR_PUPDR11_Pos;
	GPIOB->AFR[1] |= 0x01 << GPIO_AFRH_AFRH3_Pos; 	//AF1	//11 = 8 + 3
	
	TIM2->PSC = 64 - 1;				// FIXME: change 64 for (SystemCoreClock / 1000000)
	TIM2->CCR4 = WAVELEN - MOTORPULSELEN - 1;	// time delay before toggle
	TIM2->ARR = WAVELEN - 1;			// reset time, pulse length = ARR - CCR1
	TIM2->EGR |= TIM_EGR_UG;
	TIM2->CR1 |= TIM_CR1_OPM;
	TIM2->CCMR2 |= TIM_CCMR2_OC4PE;			// preload enable
	TIM2->CCMR2 |= TIM_CCMR2_OC4M;			// 111: output compare pwm2 mode
	TIM2->CCER |= TIM_CCER_CC4E;			// compare output channel 4
	TIM2->BDTR |= TIM_BDTR_MOE;
	
	//_____Hall_sensor_input_capture_mode_tim3_ch1(GPIOB_4)_____________________________________
	GPIOB->MODER &= ~GPIO_MODER_MODER4;
	GPIOB->MODER |= AF << GPIO_MODER_MODER4_Pos;	// Hall tim3 ch1 GPIOB_4
	GPIOB->OSPEEDR |= HS << GPIO_OSPEEDER_OSPEEDR4_Pos;
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR4;// WARN: GPIOB_4 pull up by default!
	GPIOB->AFR[0] |= 0x02 << GPIO_AFRL_AFRL4_Pos; 	// AF2
	
	TIM3->PSC = 64 - 1;
	TIM3->ARR = 60000 - 1;
	TIM3->CCMR1 |= TIM_CCMR1_CC1S_0			// CC1 channel, IC1 is mapped on TI1 (01)
                | TIM_CCMR1_IC1F_1
                | TIM_CCMR1_IC1F_3;			// filter (0011)
	TIM3->CCER |= TIM_CCER_CC1P;			// by rising edge
	TIM3->DIER |= TIM_DIER_CC1IE;			// Capture/Compare interrupt enable
	TIM3->CCER |= TIM_CCER_CC1E;			// capture compare enable
	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM3_IRQn, 2);

    //_____Task_timer____counter_mode_no_output_TIM15__________________________________________
	TIM17->PSC = 64000 - 1;
	TIM17->ARR = 200;	
	TIM17->DIER |= TIM_DIER_CC2IE			// interrupt enable
                | TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM17_IRQn);
	NVIC_SetPriority(TIM17_IRQn, 3);
    
	//_____HALL_wachdog_counter_mode_no_output_TIM16____________________________________________
	TIM16->PSC = 64 - 1;
	TIM16->ARR = HALLDELTAMAX - 1;		
	TIM16->DIER |= TIM_DIER_CC2IE			// interrupt enable
                | TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM16_IRQn);
	NVIC_SetPriority(TIM16_IRQn, 2);
}

void EXTI_Init()
{
	//_____Zerro_Cross_interrupt_(GPIOB_0)______________________________________________________
	GPIOB->MODER &= ~GPIO_MODER_MODER0;
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB;
	EXTI->RTSR |= EXTI_RTSR_TR0;			// by rising
	EXTI->FTSR |= EXTI_FTSR_TR0; 			// and by falling
	EXTI->IMR |= EXTI_IMR_MR0;
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_SetPriority(EXTI0_IRQn, 1);
	
	//_____Button1_interrupt_(GPIOA_2)__________________________________________________________
	GPIOA->MODER &= ~GPIO_MODER_MODER2;
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PA;
	EXTI->RTSR |= EXTI_RTSR_TR2;			// only by rising
	EXTI->IMR |= EXTI_IMR_MR2;
	NVIC_EnableIRQ(EXTI2_TSC_IRQn);
	NVIC_SetPriority(EXTI2_TSC_IRQn, 3);
	
	//_____Button2_interrupt_(GPIOB_10)_________________________________________________________
	GPIOB->MODER &= ~GPIO_MODER_MODER10;
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PB;
	EXTI->RTSR |= EXTI_RTSR_TR10; 
	EXTI->IMR |= EXTI_IMR_MR10;
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_SetPriority(EXTI15_10_IRQn, 3);
}

void UART_Init()
{	
	//___TX_(GPIOA9)_alternate_func_high_speed_______________________
	GPIOA->MODER |= AF << GPIO_MODER_MODER9_Pos;
	GPIOA->OSPEEDR |= HS << GPIO_OSPEEDER_OSPEEDR9_Pos;
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_9;
	GPIOA->PUPDR |= PULL_UP << GPIO_PUPDR_PUPDR9_Pos;
	
	//___RX_(GPIOA10)_input_floating_______________________________
	GPIOA->MODER |= AF << GPIO_MODER_MODER10_Pos;
	GPIOA->OSPEEDR |= HS << GPIO_OSPEEDER_OSPEEDR10_Pos;
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_10; 
	GPIOA->PUPDR |= PULL_UP << GPIO_PUPDR_PUPDR10_Pos;
	
	//___AF7_for_GPIOA9_and_GPIOA10________________________
	GPIOA->AFR[1] |= 0x07 << GPIO_AFRH_AFRH1_Pos;	// GPIOA9   9 = 8 + 1
	GPIOA->AFR[1] |= 0x07 << GPIO_AFRH_AFRH2_Pos;	// GPIOA10 10 = 8 + 2
	
	//___Baudrate__________________________________________
	USART1->BRR = 0x22 << USART_BRR_DIV_MANTISSA_Pos	// 64 000 000 / 115200 = 555.55 
		        | 0x0B << USART_BRR_DIV_FRACTION_Pos;	// 556d - 1 = 22Bh  
	USART1->CR1 = 0;	
	USART1->CR2 = 0;			
	USART1->CR1 |= USART_CR1_TE 	// TX enable
		        | USART_CR1_RE;		// RX enable
	USART1->CR1 |=  USART_CR1_UE;   // USART enable
}

void display_Init()
{
	//_____7-segment_2-digit_Display_GPIO_Init__________________________________________________
	GPIOB->MODER |= OUTPUT << GPIO_MODER_MODER5_Pos;	// BP1
	GPIOB->MODER |= OUTPUT << GPIO_MODER_MODER6_Pos;	// BP2
	GPIOA->MODER |= OUTPUT << GPIO_MODER_MODER8_Pos;	// A
	GPIOA->MODER |= OUTPUT << GPIO_MODER_MODER11_Pos;	// B
	GPIOB->MODER |= OUTPUT << GPIO_MODER_MODER14_Pos;	// C
	GPIOA->MODER |= OUTPUT << GPIO_MODER_MODER6_Pos;	// D
	GPIOB->MODER |= OUTPUT << GPIO_MODER_MODER1_Pos;	// E
	GPIOB->MODER |= OUTPUT << GPIO_MODER_MODER7_Pos;	// F
	GPIOA->MODER |= OUTPUT << GPIO_MODER_MODER4_Pos;	// G
	GPIOB->MODER |= OUTPUT << GPIO_MODER_MODER12_Pos;	// DP
}
//_____Peripheral_Initialization_end________________________________________________________________
