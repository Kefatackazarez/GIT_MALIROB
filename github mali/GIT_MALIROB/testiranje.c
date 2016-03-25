/*
 * testiranje.c
 *
 * Created: 16/11/15 14:36:10
 *  Author: marko
 */ 

#include <avr/io.h>
#include "Headers/testiranje.h"
#include "Headers/avr_compiler.h"
#include "Headers/usart_driver.h"
#include "Headers/port_driver.h"
#include "Headers/adc_driver.h"
#include "math.h"
#include "Headers/globals.h"
#include "Headers/mechanism.h"
#include "Headers/hardware.h"

static char step1 = 0;
static char flag1 = 0;

volatile signed long
X_pos,
Y_pos,
X_cilj,
Y_cilj,
teta,
teta_cilj,
teta_cilj_final;



void nuliraj_poziciju_robota(void)
{
	X_pos = X_cilj = 0;
	Y_pos = Y_cilj = 0;
	teta = teta_cilj = teta_cilj_final = 0;
	smer_zadati = smer_trenutni = 2; //Idi pravo
	TCD0.CNT = 0;			//Desni enkoder
	TCD1.CNT = 0;			//Levi enkoder
	count_L = 0;
	count_R = 0;
	last_count_R = 0;
	last_count_L = 0;
}

void zadaj_X_Y_teta(signed long x, signed long y, signed long teta_des, unsigned char dir)
{
	X_cilj = x * scale_factor_for_mm;
	Y_cilj = y * scale_factor_for_mm;
	teta_cilj_final = (teta_des * krug360) / 360;
	smer_zadati = dir;
}

void zadaj_X_Y(signed long x, signed long y, unsigned char dir)
{
	X_cilj = x * scale_factor_for_mm;
	Y_cilj = y * scale_factor_for_mm;
	smer_zadati = dir;
}

void zadaj_teta(signed long teta_des, unsigned char dir)
{
	teta_cilj_final = (teta_des * krug360) / 360;
	smer_zadati = dir;
}

void idi_pravo(signed long x, signed long y, unsigned long ugao){
	//zadaj_X_Y_teta(-500,0,0,2);
	X_cilj = -x * scale_factor_for_mm;
	Y_cilj = -y * scale_factor_for_mm;
	teta_cilj_final = (ugao * krug360) / 360;
	smer_zadati = 2;
}

void idi_unazad(signed long x, signed long y, unsigned long ugao){
	//zadaj_X_Y_teta(0,0,0,1);
	X_cilj = -x * scale_factor_for_mm;
	Y_cilj = -y * scale_factor_for_mm;
	teta_cilj_final = (ugao * krug360) / 360;
	smer_zadati = 1;
}

void delay1 (unsigned long x){
	sys_time=0;
	while (sys_time<x/3)
	{
	}	
	
}

void rotiraj(){
	
}

void inicijalizuj_bluetooth()
{
	//USARTE1, PE7 -> USARTE1_TX, PE6 -> USARTE1_RX
	PORTE.DIR |= (1 << 7);		//set pin PE7 as output
	PORTE.DIR &= ~(1 << 6);		//set pin PE6 as input
	
	USARTE1.CTRLA |= (1 << 4 | 1 << 2);		//enable receiver and transmitter interrupts at low level
	USARTE1.CTRLB |= (1 << 4 | 1 << 3);		//enable receiver and transmitter
	USARTE1.CTRLC |= (1 << 1 | 1 << 0);		//no parity, 1 stop bit, 8 bit data size
	
	USARTE1.BAUDCTRLA = 12;
	USARTE1.BAUDCTRLB |= (2 << 4);
	
	PMIC.CTRL |= (1 << 0);		//enable low level interrupts
	sei();						//global interrupts enabled
}

void sendMsg(char *poruka)
{
	while(*poruka != '\0'){
		sendChar(*poruka);
		poruka++;
	}
}

void sendChar(char c)
{
	USARTE0.DATA = c;
	while(!(USARTE0.STATUS & (1 << 5)));
}

void inicijalizuj_servo_tajmer_20ms()
{
	PORTF.DIR |= (1 << 0);	//servo 1
	
	//Clock source = 32/4 MHz = 8 MHz
	TCF0.CTRLA |= (1 << 2 | 1 << 0);						//Set presclaer to 64, PER 2500 = 20 ms
	TCF0.CTRLB |= (0x0F << 4 | 0x03 << 0);					//Enable Capture/compare A,B,C,D and select single slope PWM
	TCF0.INTCTRLA |= (1 << 0);								//Enable low level overflow interrupt
	TCF0.INTCTRLB |= (1 << 0 | 1 << 2 | 1 << 4 | 1 << 6);	//Enable Capture/compare low level interrupts
	TCF0.PER = 2500;
}

void pomeri_servo_1(uint16_t deg)
{
	uint16_t res = (uint16_t)(deg*(250/180));	//250 cycles for 180 degree turn
	if(res <= 0)
		res = 125;								//125 cycles for 0 degree turn
	else if(res > 250)
		res = 250;
	TCF0.CCA = res;
}

ISR(TCF0_CCA_vect)
{
	PORTF.OUT |= (1 << 0);
}

ISR(TCF0_OVF_vect)
{
	PORTF.OUT &= ~(1 << 0);
}

// demo 1 napad na velikog robota
 void demo_1(void)
 {
	switch(step1)
	{
		case 0:
		if(flag1 == 0){
			stigao_flag = 0;
			flag1 = 1;
			idi_pravo(500,0,90);
			//zadaj_teta(180,1);
			sendChar('0');
			
		}
		else if(stigao_flag == 1){
			step1++;
			flag1 = 0;
			sys_time=0;
		}
		break;
		
		 case 1:
		 if (sys_time>999)
		 {
			 if(flag1 == 0){
				 stigao_flag = 0;
				 flag1 = 1;
				 idi_unazad(500,-500,0);
				 // zadaj_X_Y(-500,0,2);
				 sendChar('0');
			 }
			 else if(stigao_flag == 1){
				 step1++;
				 flag1 = 0;
				 sys_time=0;
			 }
		 }
		 break;
		 
		 case 2:
		 if (sys_time>999)
		 {
			 if(flag1 == 0){
				 stigao_flag = 0;
				 flag1 = 1;
				 idi_pravo(2500,-500,90);
				 // zadaj_X_Y(-500,0,2);
				 sendChar('0');
			 }
			 else if(stigao_flag == 1){
				 step1++;
				 flag1 = 0;
				 sys_time=0;
			 }
		 }
		 break;
		 
		  case 3:
		  if (sys_time>999)
		  {
			  if(flag1 == 0){
				  stigao_flag = 0;
				  flag1 = 1;
				  idi_pravo(2500,0,0);
				  // zadaj_X_Y(-500,0,2);
				  sendChar('0');
			  }
			  else if(stigao_flag == 1){
				  step1++;
				  flag1 = 0;
				  sys_time=0;
			  }
		  }
		  break;
		
	
		
		
		default:
		break;
		
 	}
 }
 
 void demo_2(void)
 {
 		switch(step1)
 		{
 			case 0:
 			if(flag1 == 0){
 				stigao_flag = 0;
 				flag1 = 1;
 				idi_pravo(250,0,270);
				//zadaj_X_Y(-500,0,2);
				sendChar('0');
 			}
 			else if(stigao_flag == 1){
 				step1++;
 				flag1 = 0;
				
 			}
 			break;

 			case 1:
			 //zeljena_pravolinijska_brzina = 250;
 			if (sys_time>999)
 			{
			 if(flag1 == 0){
	 			stigao_flag = 0;
	 			flag1 = 1;
	 			idi_unazad(250,400,270);
				// zadaj_X_Y(-500,-500,2);
	 			sendChar('1');
 			}
 			else if(stigao_flag == 1){
	 			step1++;
	 			flag1 = 0;
 			}
			 }
 			break;
			 
			 	case 2:
			 	zeljena_pravolinijska_brzina = 200;
			 	if (sys_time>1332)
			 	{
				 	if(flag1 == 0){
					 	stigao_flag = 0;
					 	flag1 = 1;
					 	idi_unazad(250,650,270);
					 	// zadaj_X_Y(-500,-500,2);
					 	sendChar('1');
				 	}
				 	else if(stigao_flag == 1){
					 	step1++;
					 	flag1 = 0;
				 	}
			 	}
			 	break;
				 
				 	case 3:
				 	//zeljena_pravolinijska_brzina = 250;
				 	if (sys_time>2000)
				 	{
						 zeljena_pravolinijska_brzina = 500;
					 	if(flag1 == 0){
						 	stigao_flag = 0;
						 	flag1 = 1;
						 	idi_pravo(250,300,0);
						 	// zadaj_X_Y(-500,-500,2);
						 	sendChar('1');
					 	}
					 	else if(stigao_flag == 1){
						 	step1++;
						 	flag1 = 0;
					 	}
				 	}
				 	break;
					 
					 
					 	case 4:
					 	//zeljena_pravolinijska_brzina = 250;
					 	if (sys_time>999)
					 	{
						 	if(flag1 == 0){
							 	stigao_flag = 0;
							 	flag1 = 1;
							 	idi_pravo(450,300,270);
							 	// zadaj_X_Y(-500,-500,2);
							 	sendChar('1');
						 	}
						 	else if(stigao_flag == 1){
							 	step1++;
							 	flag1 = 0;
						 	}
					 	}
					 	break;
						 
						  	case 5:
						  	zeljena_pravolinijska_brzina = 250;
						  	if (sys_time>999)
						  	{
							  	if(flag1 == 0){
								  	stigao_flag = 0;
								  	flag1 = 1;
								  	idi_pravo(450,650,270);
								  	// zadaj_X_Y(-500,-500,2);
								  	sendChar('1');
							  	}
							  	else if(stigao_flag == 1){
								  	step1++;
								  	flag1 = 0;
							  	}
						  	}
						  	break;
							  
							   	case 6:
							   	zeljena_pravolinijska_brzina = 500;
							   	if (sys_time>2000)
							   	{
								   	if(flag1 == 0){
									   	stigao_flag = 0;
									   	flag1 = 1;
									   	idi_pravo(450,300,270);
									   	// zadaj_X_Y(-500,-500,2);
									   	sendChar('1');
								   	}
								   	else if(stigao_flag == 1){
									   	step1++;
									   	flag1 = 0;
								   	}
							   	}
							   	break;
			 
			 //case 2:
			 //zeljena_pravolinijska_brzina = 500;
			 //if(flag1 == 0){
				 //stigao_flag = 0;
				 //flag1 = 1;
				 //idi_unazad(0,0,0);
				 //// zadaj_X_Y(-500,-500,2);
				 //sendChar('2');
			 //}
			 //else if(stigao_flag == 1){
				 //step1++;
				 //flag1 = 0;
			 //}
			 //break;
 			
 			//case 2:
 			//if(flag1 == 0){
	 			//stigao_flag = 0;
	 			//flag1 = 1;
	 			//idi_pravo(0,500,0);
				 ////zadaj_X_Y(0,-500,2);
	 			//sendChar('2');
 			//}
 			//else if(stigao_flag == 1){
	 			//step1++;
	 			//flag1 = 0;
 			//}
 			//break;
 			//
 			//case 3:
 			//if(flag1 == 0){
	 			//stigao_flag = 0;
	 			//flag1 = 1;
	 			//idi_pravo(0,0,0);
				//// zadaj_X_Y(0,0,2);
	 			//sendChar('3');
 			//}
 			//else if(stigao_flag == 1){
	 			//step1++;
	 			//flag1 = 0;
 			//}
 			//break;
			 //
			//case 4:
			//if(flag1 == 0){
				//stigao_flag = 0;
				//flag1 = 1;
				////idi_pravo(0,0,0);
				//zadaj_teta(0,0);
				//sendChar('4');
			//}
			//else if(stigao_flag == 1){
				//step1++;
				//flag1 = 0;
			//}
			//break;
			 
 			
 			default:
 			break;
 		}

 		
 }
 
   void demo_3(void)
   {
	   
	   switch(step1)
	   {
		   case 0:
		   if (sys_time>666)
		   {
			   if(flag1 == 0){
				   stigao_flag = 0;
				   flag1 = 1;
				   idi_pravo(500,0,90);
				   // zadaj_X_Y(-500,0,2);
				   sendChar('0');
			   }
			   else if(stigao_flag == 1){
				   step1++;
				   flag1 = 0;
				   sys_time=0;
			   }
		   }
		   break;
		   
		   case 1:
		   if (sys_time>1332)
		   {
			   if(flag1 == 0){
				   stigao_flag = 0;
				   flag1 = 1;
				   idi_pravo(500,500,180);
				   
			   }
			   else if(stigao_flag == 1){
				   step1++;
				   flag1 = 0;
				   sys_time=0;
				   
			   }
		   }
		   break;
		   
		   case 2:
		   if (sys_time>1332)
		   {
			   if(flag1 == 0){
				   stigao_flag = 0;
				   flag1 = 1;
				   idi_pravo(0,500,270);
				   // zadaj_X_Y(-500,0,2);
				   sendChar('2');
			   }
			   else if(stigao_flag == 1){
				   step1++;
				   flag1 = 0;
				   sys_time=0;
				   
			   }
		   }
		   break;
		   
		   case 3:
		   if (sys_time>1332)
		   {
			   if(flag1 == 0){
				   stigao_flag = 0;
				   flag1 = 1;
				   idi_pravo(0,0,0);
				   // zadaj_X_Y(-500,0,2);
				   sendChar('3');
			   }
			   else if(stigao_flag == 1){
				   step1++;
				   flag1 = 0;
				   sys_time=0;
				   
			   }
		   }
		   break;
		   
		   default:
		   break;
	   }

	   
   }
 
 
  //void demo_3(void)
  //{
	  //
	  //switch(step1)
	  //{
		  //case 0:
		  //if (sys_time>666)
		  //{
			  //if(flag1 == 0){
				  //stigao_flag = 0;
				  //flag1 = 1;
				  //idi_pravo(500,0,0);
				  //// zadaj_X_Y(-500,0,2);
				  //sendChar('0');
			  //}
			  //else if(stigao_flag == 1){
				  //step1++;
				  //flag1 = 0;
				  //sys_time=0;
			  //}
		  //}
		  //break;
		  //
		  //case 1:
		  //if (sys_time>1332)
		  //{
			  //if(flag1 == 0){
				  //stigao_flag = 0;
				  //flag1 = 1;
				  //idi_unazad(500,500,0);
				  //// zadaj_X_Y(-500,0,2);
				  //sendChar('1');
			  //}
			  //else if(stigao_flag == 1){
				  //step1++;
				  //flag1 = 0;
				  //sys_time=0;
				  //
			  //}
		  //}
		  //break;
		  //
		   //case 2:
		   //if (sys_time>1332)
		   //{
			   //if(flag1 == 0){
				   //stigao_flag = 0;
				   //flag1 = 1;
				   //idi_unazad(0,500,0);
				   //// zadaj_X_Y(-500,0,2);
				   //sendChar('2');
			   //}
			   //else if(stigao_flag == 1){
				   //step1++;
				   //flag1 = 0;
				   //sys_time=0;
				   //
			   //}
		   //}
		   //break;
		   //
		    //case 3:
		    //if (sys_time>1332)
		    //{
			    //if(flag1 == 0){
				    //stigao_flag = 0;
				    //flag1 = 1;
				    //idi_unazad(0,0,0);
				    //// zadaj_X_Y(-500,0,2);
				    //sendChar('3');
			    //}
			    //else if(stigao_flag == 1){
				    //step1++;
				    //flag1 = 0;
				    //sys_time=0;
				    //
			    //}
		    //}
		    //break;
		  //
		  //default:
		  //break;
	  //}
//
	  //
  //}

void demo_6(void)
{
	switch(step1)
	{
		case 0:
		if(flag1 == 0)
		{
			stigao_flag = 0;
			flag1 = 1;
			send_Msg("00");
			zadaj_X_Y_teta(500,0,0,2);
		}
		else if(stigao_flag == 1)
		{
			send_Msg("01");
			step1++;
			flag1 = 0;
		}
		break;
		case 1:
		if(flag1 == 0)
		{
			stigao_flag = 0;
			flag1 = 1;
			zadaj_X_Y_teta(0,0,0,1);
			send_Msg("10");
		}
		else if(stigao_flag == 1)
		{
			//step1++;
			flag1 = 0;
			send_Msg("11");
		}
		break;
		case 2:
		if(flag1 == 0)
		{
			_delay_ms(1000);
			stigao_flag = 0;
			flag1 = 1;
			USART_TXBuffer_PutByte(&USART_E0_data, '2');
			_delay_ms(200);
			USART_TXBuffer_PutByte(&USART_E0_data, '2');
			zadaj_teta(180,0);
		}
		else if(stigao_flag == 1)
		{
			step1++;
			flag1 = 0;
			//_delay_ms(8000);
		}
		//break;
		//case 2:
		//if(flag1 == 0){
			//stigao_flag = 0;
			//flag1 = 1;
			//zadaj_X_Y_teta(750, 0, 0, 1);
			//} else if(stigao_flag == 1){
			//step1++;
			//flag1 = 0;
		//}
		//break;
		//case 3:
		//if(flag1 == 0){
			//stigao_flag = 0;
			//flag1 = 1;
			//zadaj_X_Y_teta(0, 0, 0, 1);
			//} else if(stigao_flag == 1){
			//step1++;
			//flag1 = 0;
		//}
		//break;
		default:
		//do nothing
		break;
	}
}

 void demo_4(void)
 {
 	switch(step1)
 	{
 		case 0:
 		if(flag1 == 0){
 			USART_TXBuffer_PutByte(&USART_E0_data, '0');
 			stigao_flag = 0;
 			flag1 = 1;
 			zadaj_X_Y(200, 0, 1);
 		}
 		else if(stigao_flag == 1){
 			step1++;
 			flag1 = 0;
 		}
 		break;
 		case 1:
 		if(flag1 == 0){
 			_delay_ms(1000);
 			USART_TXBuffer_PutByte(&USART_E0_data, '1');
 			stigao_flag = 0;
 			flag1 = 1;
 			zadaj_X_Y(500, 500, 2);
 		}
 		else if(stigao_flag == 1){
 			step1++;
 			flag1 = 0;
 		}
 		break;
 // 		case 1:
 // 		if(flag1 == 0){
 // 			stigao_flag = 0;
 // 			flag1 = 1;
 // 			zadaj_teta(180,1);
 // 			} else if(stigao_flag == 1){
 // 			step1++;
 // 			flag1 = 0;
 // 		}
 // 		break;
  		default:
  		break;
 	}
 }