#include "rf_in.h"
#include "common.h"
#include "system.h"
#include "bsp_gpio.h"
#include "cmt2300a_hal.h"
#include "cmt2300a_hal_rf315.h"
#include "cmt2300a_hal_rf330.h"
#include "cmt2300a_hal_rf433.h"

#include "bsp_ioce.h"
#include "gui.h"
//#include "font.h" 
#include "lcd.h" 

#include "delay.h"


extern uchar  flag;
extern unsigned char jmnx ;
extern unsigned char rf_en ;
extern unsigned char da1527[2][4];		//????
extern unsigned char key_d;			//????
extern unsigned int short_k ;     	//???????,?????????????????????
//extern unsigned char short_width ; //??????????
extern unsigned int delay_fac_us ;
extern unsigned char T19_S ; //??????????
extern unsigned char T19_L ;
unsigned char ii = 0;

//bit jmnx = 0;  				//为0是2262，1是1527
//unsigned char rf_en = 0;
//unsigned char da1527[2][3];		//地址缓存
//unsigned char key_d;			//键值缓存
//unsigned char short_k = 0;     	//高电平脉冲宽度,在发射前可用此数据初始成与接收相同脉冲宽度
unsigned char short_width = 60; //初始发射的短脉冲宽度





extern unsigned char CESHI_RFIN;
extern unsigned char CESHI_heak;
extern unsigned char CESHI_j;
extern unsigned char CESHI_short_k;
void buzzer_init(void)
{
    SET_GPIO_L(BUZZER_GPIO);
    SET_GPIO_OUT(BUZZER_GPIO);
    SET_GPIO_L(BUZZER_GPIO);
}

void buzzer_on(void)
{
    SET_GPIO_H(BUZZER_GPIO);
}

void buzzer_off(void)
{
    SET_GPIO_L(BUZZER_GPIO);
}

void buzzer_onAutoOff_proc(void)
{
    buzzer_off();
}

void buzzer_onAutoOff(u32 ms)
{
    time_server_setDelayRunTask(DELAY_RUN_TASK_BUZZER, buzzer_onAutoOff_proc, ms);
    buzzer_on();
}

//void delay_us(__IO u32 nTime)
//{ 
//	u32 TimingDelay = nTime;	
//  uint8_t CLOCK=0;
//	while(nTime--)
//	{
//		 CLOCK=3;
//		while(CLOCK--);	
//	}
//}

void RF_IN(void) 
{
	
	
	//unsigned char ii = 0;
	unsigned char j = 0;
	unsigned char k = 0;
//	unsigned char rep = 0;
	unsigned char u,i,i1;
  unsigned int head_k=0;      //高电平
	 CESHI_heak=head_k; 
   CESHI_short_k=short_k;
  	if(rf_en==0)
    {
//			 for(i1=0;i1<2;i1++)
//	     {
//	       for(i=0;i<4;i++)
//	          da1527[i1][i]=0;
//	     }
    	//-------------------------------????-----------------------------------------
    	short_k = 0;
		  while(RFIN && j<252) 			 //高电平 short_K
		  {
			  delay_us(5);
			  short_k++;
		  } 	
	  	while(!RFIN) 					 //低电平 heak_K
		  {
		  	delay_us(5);
		  	head_k++;
		  } 							 
      	if(((short_k*22)<head_k) && (head_k<(short_k*38)))   	//???????????32?
        {	
        	 // for(rep=0;rep<2;rep++)	 
           // {  
				      for(ii=0;ii<3;ii++)										//3??  3个字节
	        	  {
	        		   for(k=0;k<8;k++)									//????8?
	        		  {        	 	  	   
						       j = 0;
						       while(RFIN && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       } 		 
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[0][ii]&=~(1<<((7-k)));	           //写0                          
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//写1
						            	da1527[0][ii]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }	
									head_k = 0; 
						      while(!RFIN && j<255)	   		//延长了一点倍数？？？？？
						      {
							      delay_us(5);
							      head_k++;
						      }   
                 if((head_k>(short_k*22)) && (head_k<(short_k*38)))    //23?,T19S
				         {
									 //da1527[0][2]|=(1<<(0));
									 T19_S=1;                                //T19 短码---------------------------T19S------短码
									 break;
				         }
                 else
								 {
									 T19_S=0;		 
								 }									 			
					    }
							
	        	}
    
        if(T19_S==1)             //T19 短码
				{
					 for(ii=0;ii<3;ii++)										//3??
	        {
	        		   for(k=0;k<8;k++)									//????8?
	        		  {        	 	  	   
						       j = 0;
						       while(RFIN && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       } 	
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[1][ii]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[1][ii]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }
									head_k = 0;
						      while(!RFIN && j<255)	   		//?????
						      {
							      delay_us(5);
							      head_k++;
						      }   
									 
                 if((head_k>(short_k*22)) && (head_k<(short_k*38)))    //23?,T19S
				         {
									 //da1527[1][2]|=(1<<(0));
									 T19_S=1;
									 da1527[0][3]=0;
						       da1527[1][3]=0;
									 break;
				         }
                 else
								 {
									 T19_S=0;		 
								 }									 			
					    }
							
	        	}
				}	
				 j=0;
				while(RFIN && (j<200))					//???????????
				{
					delay_us(5);
					j++;
					//j1++;
				}            			   
			    head_k = 0;
				while(!RFIN) 						   //???????????
				{
					delay_us(5);
					head_k++;
				}
					if(((short_k*22)<head_k) && (head_k<(short_k*38))&&(T19_S==0))   	//???????????32?
					{
						 for(ii=0;ii<3;ii++)										//3??
	        	  {
	        		   for(k=0;k<8;k++)									//????8?
	        		  {        	 	  	   
						       j = 0;
						       while(RFIN && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       } 	
									 head_k = 0;
						       while(!RFIN && j<255)	   		//?????
						       {
							       delay_us(5);
							       head_k++;
						       }   
									 
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[1][ii]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[1][ii]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }									   		   	
					    }
							
	        	} 
						da1527[0][3]=0;
						da1527[1][3]=0;
					}
					else  if((j<(short_k*5))&&(T19_S==0))    //T19_L判断条件
					{
						  k=0;
						 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[0][3]&=~(1<<((7-k)));	                            
	        	 else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[0][3]|=(1<<(7-k)); 				        			        
	           else 											//????-
						 {	
							  rf_en = 0;
							  return;
						 }									
						   for(k=1;k<8;k++)									//????8?
	        		 {        	 	  	   
						       j = 0;
						       while(RFIN && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       }							
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[0][3]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[0][3]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }	
									 head_k = 0;
						       while(!RFIN && j<255)	   		//?????
						       {
							       delay_us(5);
							       head_k++;
						       } 
								 if((head_k>(short_k*22)) && (head_k<(short_k*38)))    //23?,T19S
				         {
									 //da1527[1][2]|=(1<<(0));
									 T19_L=1;
									// da1527[0][3]=0;
						       //da1527[1][3]=0;
									 break;
				         }
                 else
								 {
									 T19_L=0;		 
								 }						
					    }
				if(T19_L==1)
				{
					 for(ii=0;ii<4;ii++)										//接4个字节
	        {
	        		   for(k=0;k<8;k++)									//????8?
	        		   {        	 	  	   
						       j = 0;
						       while(RFIN && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       } 	
									 
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[1][ii]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[1][ii]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }									
	        		    head_k = 0;
						      while(!RFIN && j<255)	   		//?????
						     {
							      delay_us(5);
							      head_k++;
						     } 
								  if((head_k>(short_k*22)) && (head_k<(short_k*38)))    //23?,T19S
				         {
									 //da1527[1][2]|=(1<<(0));
									 T19_L=1;
									// da1527[0][3]=0;
						       //da1527[1][3]=0;
									 break;
				         }
                 else
								 {
									 T19_L=0;		 
								 }					
					     }
					  }										
					}	
			   	/*while(RFIN && (j<200))					//???????????
				  {
					  delay_us(5);
					  j++;
				  }            			   
			    head_k = 0;
				  while(!RFIN) 						   //???????????
				  {
					  delay_us(5);
					  head_k++;
				  }   			
					if(((short_k*22)<head_k) && (head_k<(short_k*38)))   	//???????????32?
					{
						  for(ii=0;ii<4;ii++)										//4??
	        	  {
	        		   for(k=0;k<8;k++)									//????8?
	        		   {        	 	  	   
						       j = 0;
						       while(RFIN && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       } 	
									 
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[1][ii]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[1][ii]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }									
	        		    head_k = 0;
						      while(!RFIN && j<255)	   		//?????
						     {
							      delay_us(5);
							      head_k++;
						     }      			      		   	
					      }
					   }										
				}
				else
				{
					for(ii=0;ii<4;ii++)		
			      da1527[1][ii]=0;
				}*/
			
		}
				
				
				
           	//} 
        	//+++++++++++++++++++++++++2262?1527??????++++++++++++++++++++++++++++++++++++++++ 
		   if(T19_S==1)
			{
				     for(i=0;i<3;i++)  			//??2262?T19S
             {
                for(u=0;u<4;u++) 
					      {
						        if(((da1527[0][i]>>(u*2)) & 3)==2) 
						        {
							          i=80; break;
						        }
					      }  						//?10??T19S"H"
					        if(i==80) break;
             }
             if(i==80)  					//T19S
             {
              // key_d=da1527[1][2] & 0x0f;      //??1527????
               //da1527[0][2]=da1527[1][2]>>4; 	//??1527??4???
					     jmnx = 2;         				//?0??2262 ,1?T19S
             } 
            else
            {
							 jmnx = 0;			
						}	
				    rf_en = 4;                			//????
						 //jmnx = 0;	
			}
			else if(T19_L==1)
			{
				 jmnx = 3;//T19 
				 rf_en = 4;                			//????
			}
		
		  else	if((da1527[0][0]==da1527[1][0]) && (da1527[0][1]==da1527[1][1]) && (da1527[0][2]==da1527[1][2]))	//?????????????
	    {	       
													 for(i=0;i<4;i++)  			//??2262?1527
													 {
																for(u=0;u<4;u++) 
																{
																	if(((da1527[0][i]>>(u*2)) & 3)==2) 
																	{
																			i=80; break;
																	}
																}  						//?10??1527"H"
																if(i==80) break;
													 }
													 if(i==80)  					//1527
													 {
														 key_d=da1527[1][2] & 0x0f;      //??1527????
														// da1527[0][2]=da1527[1][2]>>4; 	//??1527??4???
														 jmnx = 1;         				//?0??2262 ,1?1527
													 }     
													 else      							//2262
													{
														 key_d=0;
														for(i=0;i<4;i++)	  			//???2262????
														{
																if(((da1527[0][2]>>(i*2))&3)==3) 
																 key_d|=1<<i;
														 }                                    
																//	da1527[0][2] = 0; 				//2262??4???,??0
														jmnx = 0;         				//?0?2262,1?1527
												 } 
					
                rf_en = 4;                			//????		
			  }	
		 }       	  
	 }  
 }

void RF315_IN(void) 
{
	
	
	//unsigned char ii = 0;
	unsigned char j = 0;
	unsigned char k = 0;
//	unsigned char rep = 0;
	unsigned char u,i,i1;
  unsigned int head_k=0;      //
	 CESHI_heak=head_k; 
   CESHI_short_k=short_k;
  	if(rf_en==0)
    {
//			 for(i1=0;i1<2;i1++)
//	     {
//	       for(i=0;i<4;i++)
//	          da1527[i1][i]=0;
//	     }
    	//-------------------------------????-----------------------------------------
    	short_k = 0;
		  while(RFIN_RF315&& j<252) 			 //??????????????
		  {
			  delay_us(5);
			  short_k++;
		  } 	
	  	while(!RFIN_RF315 ) 					 //??????????
		  {
		  	delay_us(5);
		  	head_k++;
		  } 							 
      	if(((short_k*22)<head_k) && (head_k<(short_k*38)))   	//???????????32?
        {	
        	 // for(rep=0;rep<2;rep++)	 
           // {  
				      for(ii=0;ii<3;ii++)										//3??
	        	  {
	        		   for(k=0;k<8;k++)									//????8?
	        		  {        	 	  	   
						       j = 0;
						       while(RFIN_RF315  && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       } 		 
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[0][ii]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[0][ii]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }	
									head_k = 0; 
						      while(!RFIN_RF315  && j<255)	   		//?????
						      {
							      delay_us(5);
							      head_k++;
						      }   
                 if((head_k>(short_k*22)) && (head_k<(short_k*38)))    //23?,T19S
				         {
									 //da1527[0][2]|=(1<<(0));
									 T19_S=1;
									 break;
				         }
                 else
								 {
									 T19_S=0;		 
								 }									 			
					    }
							
	        	}
    
        if(T19_S==1)
				{
					 for(ii=0;ii<3;ii++)										//3??
	        {
	        		   for(k=0;k<8;k++)									//????8?
	        		  {        	 	  	   
						       j = 0;
						       while(RFIN_RF315  && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       } 	
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[1][ii]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[1][ii]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }
									head_k = 0;
						      while(!RFIN_RF315  && j<255)	   		//?????
						      {
							      delay_us(5);
							      head_k++;
						      }   
									 
                 if((head_k>(short_k*22)) && (head_k<(short_k*38)))    //23?,T19S
				         {
									 //da1527[1][2]|=(1<<(0));
									 T19_S=1;
									 da1527[0][3]=0;
						       da1527[1][3]=0;
									 break;
				         }
                 else
								 {
									 T19_S=0;		 
								 }									 			
					    }
							
	        	}
				}	
				 j=0;
				while(RFIN_RF315  && (j<200))					//???????????
				{
					delay_us(5);
					j++;
					//j1++;
				}            			   
			    head_k = 0;
				while(!RFIN_RF315 ) 						   //???????????
				{
					delay_us(5);
					head_k++;
				}
					if(((short_k*22)<head_k) && (head_k<(short_k*38))&&(T19_S==0))   	//???????????32?
					{
						 for(ii=0;ii<3;ii++)										//3??
	        	  {
	        		   for(k=0;k<8;k++)									//????8?
	        		  {        	 	  	   
						       j = 0;
						       while(RFIN_RF315  && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       } 	
									 head_k = 0;
						       while(!RFIN_RF315  && j<255)	   		//?????
						       {
							       delay_us(5);
							       head_k++;
						       }   
									 
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[1][ii]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[1][ii]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }									   		   	
					    }
							
	        	} 
						da1527[0][3]=0;
						da1527[1][3]=0;
					}
					else  if((j<(short_k*5))&&(T19_S==0))//T19_L
					{
						  k=0;
						 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[0][3]&=~(1<<((7-k)));	                            
	        	 else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[0][3]|=(1<<(7-k)); 				        			        
	           else 											//????
						 {	
							  rf_en = 0;
							  return;
						 }									
						   for(k=1;k<8;k++)									//????8?
	        		 {        	 	  	   
						       j = 0;
						       while(RFIN_RF315  && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       }							
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[0][3]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[0][3]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }	
									 head_k = 0;
						       while(!RFIN_RF315  && j<255)	   		//?????
						       {
							       delay_us(5);
							       head_k++;
						       } 
								 if((head_k>(short_k*22)) && (head_k<(short_k*38)))    //23?,T19S
				         {
									 //da1527[1][2]|=(1<<(0));
									 T19_L=1;
									// da1527[0][3]=0;
						       //da1527[1][3]=0;
									 break;
				         }
                 else
								 {
									 T19_L=0;		 
								 }						
					    }
				if(T19_L==1)
				{
					 for(ii=0;ii<4;ii++)										//4??
	        {
	        		   for(k=0;k<8;k++)									//????8?
	        		   {        	 	  	   
						       j = 0;
						       while(RFIN_RF315  && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       } 	
									 
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[1][ii]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[1][ii]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }									
	        		    head_k = 0;
						      while(!RFIN_RF315  && j<255)	   		//?????
						     {
							      delay_us(5);
							      head_k++;
						     } 
								  if((head_k>(short_k*22)) && (head_k<(short_k*38)))    //23?,T19S
				         {
									 //da1527[1][2]|=(1<<(0));
									 T19_L=1;
									// da1527[0][3]=0;
						       //da1527[1][3]=0;
									 break;
				         }
                 else
								 {
									 T19_L=0;		 
								 }					
					     }
					  }										
					}	
			   	/*while(RFIN && (j<200))					//???????????
				  {
					  delay_us(5);
					  j++;
				  }            			   
			    head_k = 0;
				  while(!RFIN) 						   //???????????
				  {
					  delay_us(5);
					  head_k++;
				  }   			
					if(((short_k*22)<head_k) && (head_k<(short_k*38)))   	//???????????32?
					{
						  for(ii=0;ii<4;ii++)										//4??
	        	  {
	        		   for(k=0;k<8;k++)									//????8?
	        		   {        	 	  	   
						       j = 0;
						       while(RFIN && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       } 	
									 
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[1][ii]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[1][ii]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }									
	        		    head_k = 0;
						      while(!RFIN && j<255)	   		//?????
						     {
							      delay_us(5);
							      head_k++;
						     }      			      		   	
					      }
					   }										
				}
				else
				{
					for(ii=0;ii<4;ii++)		
			      da1527[1][ii]=0;
				}*/
			
		}
				
				
				
           	//} 
        	//+++++++++++++++++++++++++2262?1527??????++++++++++++++++++++++++++++++++++++++++ 
		   if(T19_S==1)
			{
				     for(i=0;i<3;i++)  			//??2262?T19S
             {
                for(u=0;u<4;u++) 
					      {
						        if(((da1527[0][i]>>(u*2)) & 3)==2) 
						        {
							          i=80; break;
						        }
					      }  						//?10??T19S"H"
					        if(i==80) break;
             }
             if(i==80)  					//T19S
             {
              // key_d=da1527[1][2] & 0x0f;      //??1527????
               //da1527[0][2]=da1527[1][2]>>4; 	//??1527??4???
							  key_d=da1527[1][2] & 0x0f;      //??1527????
               da1527[0][2]=da1527[1][2]>>4; 	//??1527??4???
					     jmnx = 2;         				//?0??2262 ,1?T19S
             } 
            else
            {
							 jmnx = 0;			
						}	
				    rf_en = 4;                			//????
						 //jmnx = 0;	
			}
			else if(T19_L==1)
			{
				   key_d=da1527[1][2]>>1;      //??1527????
				   key_d=key_d&0x0f;
				   key_d=key_d<<4;
           //da1527[0][2]=key_d; 	//??1527??4???
				
				
				jmnx = 3;//T19_L
				 rf_en = 4;                			//????
			}
		
		  else	if((da1527[0][0]==da1527[1][0]) && (da1527[0][1]==da1527[1][1]) && (da1527[0][2]==da1527[1][2]))	//?????????????
	    {	       
             for(i=0;i<4;i++)  			//??2262?1527
             {
                	for(u=0;u<4;u++) 
					        {
						        if(((da1527[0][i]>>(u*2)) & 3)==2) 
						        {
							          i=80; break;
						        }
					        }  						//?10??1527"H"
					        if(i==80) break;
             }
             if(i==80)  					//1527
             {
               key_d=da1527[1][2] & 0x0f;      //??1527????
               da1527[0][2]=da1527[1][2]>>4; 	//??1527??4???
					     jmnx = 1;         				//?0??2262 ,1?1527
             }     
             else      							//2262
            {
               key_d=0;
              for(i=0;i<4;i++)	  			//解析 2262的 按键值
					    {
						      if(((da1527[0][2]>>(i*2))&3)==3) 
					         key_d|=1<<i;
					     }                                    
                    da1527[0][2] = 0; 				//2262??4???,??0
					    jmnx = 0;         				//?0?2262,1?1527
           } 
					
                rf_en = 4;                			//????		
			  }	
		 }       	  
	 }  
 }

//----------------------------------------------330-------------------------------------------------------//
 void RF330_IN(void) 
{
	
	
	//unsigned char ii = 0;
	unsigned char j = 0;
	unsigned char k = 0;
//	unsigned char rep = 0;
	unsigned char u,i,i1;
  unsigned int head_k=0;      //
	 CESHI_heak=head_k; 
   CESHI_short_k=short_k;
  	if(rf_en==0)
    {
//			 for(i1=0;i1<2;i1++)
//	     {
//	       for(i=0;i<4;i++)
//	          da1527[i1][i]=0;
//	     }
    	//-------------------------------????-----------------------------------------
    	short_k = 0;
		  while(RFIN_RF330&& j<252) 			 //??????????????
		  {
			  delay_us(5);
			  short_k++;
		  } 	
	  	while(!RFIN_RF330 ) 					 //??????????
		  {
		  	delay_us(5);
		  	head_k++;
		  } 							 
      	if(((short_k*22)<head_k) && (head_k<(short_k*38)))   	//???????????32?
        {	
        	 // for(rep=0;rep<2;rep++)	 
           // {  
				      for(ii=0;ii<3;ii++)										//3??
	        	  {
	        		   for(k=0;k<8;k++)									//????8?
	        		  {        	 	  	   
						       j = 0;
						       while(RFIN_RF330  && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       } 		 
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[0][ii]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[0][ii]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }	
									head_k = 0; 
						      while(!RFIN_RF330  && j<255)	   		//?????
						      {
							      delay_us(5);
							      head_k++;
						      }   
                 if((head_k>(short_k*22)) && (head_k<(short_k*38)))    //23?,T19S
				         {
									 //da1527[0][2]|=(1<<(0));
									 T19_S=1;
									 break;
				         }
                 else
								 {
									 T19_S=0;		 
								 }									 			
					    }
							
	        	}
    
        if(T19_S==1)
				{
					 for(ii=0;ii<3;ii++)										//3??
	        {
	        		   for(k=0;k<8;k++)									//????8?
	        		  {        	 	  	   
						       j = 0;
						       while(RFIN_RF330  && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       } 	
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[1][ii]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[1][ii]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }
									head_k = 0;
						      while(!RFIN_RF330  && j<255)	   		//?????
						      {
							      delay_us(5);
							      head_k++;
						      }   
									 
                 if((head_k>(short_k*22)) && (head_k<(short_k*38)))    //23?,T19S
				         {
									 //da1527[1][2]|=(1<<(0));
									 T19_S=1;
									 da1527[0][3]=0;
						       da1527[1][3]=0;
									 break;
				         }
                 else
								 {
									 T19_S=0;		 
								 }									 			
					    }
							
	        	}
				}	
				 j=0;
				while(RFIN_RF330  && (j<200))					//???????????
				{
					delay_us(5);
					j++;
					//j1++;
				}            			   
			    head_k = 0;
				while(!RFIN_RF330 ) 						   //???????????
				{
					delay_us(5);
					head_k++;
				}
					if(((short_k*22)<head_k) && (head_k<(short_k*38))&&(T19_S==0))   	//???????????32?
					{
						 for(ii=0;ii<3;ii++)										//3??
	        	  {
	        		   for(k=0;k<8;k++)									//????8?
	        		  {        	 	  	   
						       j = 0;
						       while(RFIN_RF330  && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       } 	
									 head_k = 0;
						       while(!RFIN_RF330  && j<255)	   		//?????
						       {
							       delay_us(5);
							       head_k++;
						       }   
									 
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[1][ii]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[1][ii]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }									   		   	
					    }
							
	        	} 
						da1527[0][3]=0;
						da1527[1][3]=0;
					}
					else  if((j<(short_k*5))&&(T19_S==0))//T19_L
					{
						  k=0;
						 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[0][3]&=~(1<<((7-k)));	                            
	        	 else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[0][3]|=(1<<(7-k)); 				        			        
	           else 											//????
						 {	
							  rf_en = 0;
							  return;
						 }									
						   for(k=1;k<8;k++)									//????8?
	        		 {        	 	  	   
						       j = 0;
						       while(RFIN_RF330  && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       }							
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[0][3]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[0][3]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }	
									 head_k = 0;
						       while(!RFIN_RF330  && j<255)	   		//?????
						       {
							       delay_us(5);
							       head_k++;
						       } 
								 if((head_k>(short_k*22)) && (head_k<(short_k*38)))    //23?,T19S
				         {
									 //da1527[1][2]|=(1<<(0));
									 T19_L=1;
									// da1527[0][3]=0;
						       //da1527[1][3]=0;
									 break;
				         }
                 else
								 {
									 T19_L=0;		 
								 }						
					    }
				if(T19_L==1)
				{
					 for(ii=0;ii<4;ii++)										//4??
	        {
	        		   for(k=0;k<8;k++)									//????8?
	        		   {        	 	  	   
						       j = 0;
						       while(RFIN_RF330  && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       } 	
									 
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[1][ii]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[1][ii]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }									
	        		    head_k = 0;
						      while(!RFIN_RF330  && j<255)	   		//?????
						     {
							      delay_us(5);
							      head_k++;
						     } 
								  if((head_k>(short_k*22)) && (head_k<(short_k*38)))    //23?,T19S
				         {
									 //da1527[1][2]|=(1<<(0));
									 T19_L=1;
									// da1527[0][3]=0;
						       //da1527[1][3]=0;
									 break;
				         }
                 else
								 {
									 T19_L=0;		 
								 }					
					     }
					  }										
					}	
			   	/*while(RFIN && (j<200))					//???????????
				  {
					  delay_us(5);
					  j++;
				  }            			   
			    head_k = 0;
				  while(!RFIN) 						   //???????????
				  {
					  delay_us(5);
					  head_k++;
				  }   			
					if(((short_k*22)<head_k) && (head_k<(short_k*38)))   	//???????????32?
					{
						  for(ii=0;ii<4;ii++)										//4??
	        	  {
	        		   for(k=0;k<8;k++)									//????8?
	        		   {        	 	  	   
						       j = 0;
						       while(RFIN && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       } 	
									 
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[1][ii]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[1][ii]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }									
	        		    head_k = 0;
						      while(!RFIN && j<255)	   		//?????
						     {
							      delay_us(5);
							      head_k++;
						     }      			      		   	
					      }
					   }										
				}
				else
				{
					for(ii=0;ii<4;ii++)		
			      da1527[1][ii]=0;
				}*/
			
		}
				
				
				
           	//} 
        	//+++++++++++++++++++++++++2262?1527??????++++++++++++++++++++++++++++++++++++++++ 
		   if(T19_S==1)
			{
				     for(i=0;i<3;i++)  			//??2262?T19S
             {
                for(u=0;u<4;u++) 
					      {
						        if(((da1527[0][i]>>(u*2)) & 3)==2) 
						        {
							          i=80; break;
						        }
					      }  						//?10??T19S"H"
					        if(i==80) break;
             }
             if(i==80)  					//T19S
             {
              // key_d=da1527[1][2] & 0x0f;      //??1527????
               //da1527[0][2]=da1527[1][2]>>4; 	//??1527??4???
							  key_d=da1527[1][2] & 0x0f;      //??1527????
               da1527[0][2]=da1527[1][2]>>4; 	//??1527??4???
					     jmnx = 2;         				//?0??2262 ,1?T19S
             } 
            else
            {
							 jmnx = 0;			
						}	
				    rf_en = 4;                			//????
						 //jmnx = 0;	
			}
			else if(T19_L==1)
			{
				   key_d=da1527[1][2]>>1;      //??1527????
				   key_d=key_d&0x0f;
				   key_d=key_d<<4;
           //da1527[0][2]=key_d; 	//??1527??4???
				
				
				jmnx = 3;//T19_L
				 rf_en = 4;                			//????
			}
		
		  else	if((da1527[0][0]==da1527[1][0]) && (da1527[0][1]==da1527[1][1]) && (da1527[0][2]==da1527[1][2]))	//?????????????
	    {	       
             for(i=0;i<4;i++)  			//??2262?1527
             {
                	for(u=0;u<4;u++) 
					        {
						        if(((da1527[0][i]>>(u*2)) & 3)==2) 
						        {
							          i=80; break;
						        }
					        }  						//?10??1527"H"
					        if(i==80) break;
             }
             if(i==80)  					//1527
             {
               key_d=da1527[1][2] & 0x0f;      //??1527????
               da1527[0][2]=da1527[1][2]>>4; 	//??1527??4???
					     jmnx = 1;         				//?0??2262 ,1?1527
             }     
             else      							//2262
            {
               key_d=0;
              for(i=0;i<4;i++)	  			//解析 2262的 按键值
					    {
						      if(((da1527[0][2]>>(i*2))&3)==3) 
					         key_d|=1<<i;
					     }                                    
                    da1527[0][2] = 0; 				//2262??4???,??0
					    jmnx = 0;         				//?0?2262,1?1527
           } 
					
                rf_en = 4;                			//????		
			  }	
		 }       	  
	 }  
 }
 
 
 
 
//----------------------------------------------330emd-----------------------------------------------------//
 void RF433_IN(void) 
{
	
	
	//unsigned char ii = 0;
	unsigned char j = 0;
	unsigned char k = 0;
//	unsigned char rep = 0;
	unsigned char u,i,i1;
  unsigned int head_k=0;      //
	 CESHI_heak=head_k; 
   CESHI_short_k=short_k;
  	if(rf_en==0)
    {
//			 for(i1=0;i1<2;i1++)
//	     {
//	       for(i=0;i<4;i++)
//	          da1527[i1][i]=0;
//	     }
    	//-------------------------------????-----------------------------------------
    	short_k = 0;
		  while(RFIN_RF433&& j<252) 			 //??????????????
		  {
			  delay_us(5);
			  short_k++;
		  } 	
	  	while(!RFIN_RF433 ) 					 //??????????
		  {
		  	delay_us(5);
		  	head_k++;
		  } 							 
      	if(((short_k*22)<head_k) && (head_k<(short_k*38)))   	//???????????32?
        {	
        	 // for(rep=0;rep<2;rep++)	 
           // {  
				      for(ii=0;ii<3;ii++)										//3??
	        	  {
	        		   for(k=0;k<8;k++)									//????8?
	        		  {        	 	  	   
						       j = 0;
						       while(RFIN_RF433  && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       } 		 
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[0][ii]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[0][ii]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }	
									head_k = 0; 
						      while(!RFIN_RF433  && j<255)	   		//?????
						      {
							      delay_us(5);
							      head_k++;
						      }   
                 if((head_k>(short_k*22)) && (head_k<(short_k*38)))    //23?,T19S
				         {
									 //da1527[0][2]|=(1<<(0));
									 T19_S=1;
									 break;
				         }
                 else
								 {
									 T19_S=0;		 
								 }									 			
					    }
							
	        	}
    
        if(T19_S==1)
				{
					 for(ii=0;ii<3;ii++)										//3??
	        {
	        		   for(k=0;k<8;k++)									//????8?
	        		  {        	 	  	   
						       j = 0;
						       while(RFIN_RF433  && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       } 	
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[1][ii]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[1][ii]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }
									head_k = 0;
						      while(!RFIN_RF433  && j<255)	   		//?????
						      {
							      delay_us(5);
							      head_k++;
						      }   
									 
                 if((head_k>(short_k*22)) && (head_k<(short_k*38)))    //23?,T19S
				         {
									 //da1527[1][2]|=(1<<(0));
									 T19_S=1;
									 da1527[0][3]=0;
						       da1527[1][3]=0;
									 break;
				         }
                 else
								 {
									 T19_S=0;		 
								 }									 			
					    }
							
	        	}
				}	
				 j=0;
				while(RFIN_RF433  && (j<200))					//???????????
				{
					delay_us(5);
					j++;
					//j1++;
				}            			   
			    head_k = 0;
				while(!RFIN_RF433 ) 						   //???????????
				{
					delay_us(5);
					head_k++;
				}
					if(((short_k*22)<head_k) && (head_k<(short_k*38))&&(T19_S==0))   	//???????????32?
					{
						 for(ii=0;ii<3;ii++)										//3??
	        	  {
	        		   for(k=0;k<8;k++)									//????8?
	        		  {        	 	  	   
						       j = 0;
						       while(RFIN_RF433  && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       } 	
									 head_k = 0;
						       while(!RFIN_RF433  && j<255)	   		//?????
						       {
							       delay_us(5);
							       head_k++;
						       }   
									 
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[1][ii]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[1][ii]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }									   		   	
					    }
							
	        	} 
						da1527[0][3]=0;
						da1527[1][3]=0;
					}
					else  if((j<(short_k*5))&&(T19_S==0))//T19_L
					{
						  k=0;
						 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[0][3]&=~(1<<((7-k)));	                            
	        	 else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[0][3]|=(1<<(7-k)); 				        			        
	           else 											//????
						 {	
							  rf_en = 0;
							  return;
						 }									
						   for(k=1;k<8;k++)									//????8?
	        		 {        	 	  	   
						       j = 0;
						       while(RFIN_RF433  && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       }							
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[0][3]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[0][3]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }	
									 head_k = 0;
						       while(!RFIN_RF433  && j<255)	   		//?????
						       {
							       delay_us(5);
							       head_k++;
						       } 
								 if((head_k>(short_k*22)) && (head_k<(short_k*38)))    //23?,T19S
				         {
									 //da1527[1][2]|=(1<<(0));
									 T19_L=1;
									// da1527[0][3]=0;
						       //da1527[1][3]=0;
									 break;
				         }
                 else
								 {
									 T19_L=0;		 
								 }						
					    }
				if(T19_L==1)
				{
					 for(ii=0;ii<4;ii++)										//4??
	        {
	        		   for(k=0;k<8;k++)									//????8?
	        		   {        	 	  	   
						       j = 0;
						       while(RFIN_RF433  && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       } 	
									 
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[1][ii]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[1][ii]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }									
	        		    head_k = 0;
						      while(!RFIN_RF433  && j<255)	   		//?????
						     {
							      delay_us(5);
							      head_k++;
						     } 
								  if((head_k>(short_k*22)) && (head_k<(short_k*38)))    //23?,T19S
				         {
									 //da1527[1][2]|=(1<<(0));
									 T19_L=1;
									// da1527[0][3]=0;
						       //da1527[1][3]=0;
									 break;
				         }
                 else
								 {
									 T19_L=0;		 
								 }					
					     }
					  }										
					}	
			   	/*while(RFIN && (j<200))					//???????????
				  {
					  delay_us(5);
					  j++;
				  }            			   
			    head_k = 0;
				  while(!RFIN) 						   //???????????
				  {
					  delay_us(5);
					  head_k++;
				  }   			
					if(((short_k*22)<head_k) && (head_k<(short_k*38)))   	//???????????32?
					{
						  for(ii=0;ii<4;ii++)										//4??
	        	  {
	        		   for(k=0;k<8;k++)									//????8?
	        		   {        	 	  	   
						       j = 0;
						       while(RFIN && j<245) 
						       {
							       delay_us(5);								//16us(6mhz:2~5)
							         j++;
						       } 	
									 
	        	 	  	 if(j>(short_k-short_k/2-short_k/3)&&j<(short_k*1.96)) 
							        da1527[1][ii]&=~(1<<((7-k)));	                            
	        	  	   else if(j>(short_k*1.96)&&j<(short_k*5))		//%25 ?????????3?
						            	da1527[1][ii]|=(1<<(7-k)); 				        			        
	                 else 											//????
						      {	
							       rf_en = 0;
							       return;
						      }									
	        		    head_k = 0;
						      while(!RFIN && j<255)	   		//?????
						     {
							      delay_us(5);
							      head_k++;
						     }      			      		   	
					      }
					   }										
				}
				else
				{
					for(ii=0;ii<4;ii++)		
			      da1527[1][ii]=0;
				}*/
			
		}
				
				
				
           	//} 
        	//+++++++++++++++++++++++++2262?1527??????++++++++++++++++++++++++++++++++++++++++ 
		   if(T19_S==1)
			{
				     for(i=0;i<3;i++)  			//??2262?T19S
             {
                for(u=0;u<4;u++) 
					      {
						        if(((da1527[0][i]>>(u*2)) & 3)==2) 
						        {
							          i=80; break;
						        }
					      }  						//?10??T19S"H"
					        if(i==80) break;
             }
             if(i==80)  					//T19S
             {
              // key_d=da1527[1][2] & 0x0f;      //??1527????
               //da1527[0][2]=da1527[1][2]>>4; 	//??1527??4???
							  key_d=da1527[1][2] & 0x0f;      //??1527????
               da1527[0][2]=da1527[1][2]>>4; 	//??1527??4???
					     jmnx = 2;         				//?0??2262 ,1?T19S
             } 
            else
            {
							 jmnx = 0;			
						}	
				    rf_en = 4;                			//????
						 //jmnx = 0;	
			}
			else if(T19_L==1)
			{
				   key_d=da1527[1][2]>>1;      //??1527????
				   key_d=key_d&0x0f;
				   key_d=key_d<<4;
           //da1527[0][2]=key_d; 	//??1527??4???
				
				
				jmnx = 3;//T19_L
				 rf_en = 4;                			//????
			}
		
		  else	if((da1527[0][0]==da1527[1][0]) && (da1527[0][1]==da1527[1][1]) && (da1527[0][2]==da1527[1][2]))	//?????????????
	    {	       
             for(i=0;i<4;i++)  			//??2262?1527
             {
                	for(u=0;u<4;u++) 
					        {
						        if(((da1527[0][i]>>(u*2)) & 3)==2) 
						        {
							          i=80; break;
						        }
					        }  						//?10??1527"H"
					        if(i==80) break;
             }
             if(i==80)  					//1527
             {
               key_d=da1527[1][2] & 0x0f;      //??1527????
               da1527[0][2]=da1527[1][2]>>4; 	//??1527??4???
					     jmnx = 1;         				//?0??2262 ,1?1527
             }     
             else      							//2262
            {
               key_d=0;
              for(i=0;i<4;i++)	  			//解析 2262的 按键值
					    {
						      if(((da1527[0][2]>>(i*2))&3)==3) 
					         key_d|=1<<i;
					     }                                    
                    da1527[0][2] = 0; 				//2262??4???,??0
					    jmnx = 0;         				//?0?2262,1?1527
           } 
					
                rf_en = 4;                			//????		
			  }	
		 }       	  
	 }  
 }
 
 
 
 
 
 
 
 //-----------------------------------------433------------------------------------------------------------------//
 
 
 
 
 
 
 
 //-----------------------------------433 END----------------------------------------------------------------------//
//---------------------------------------------RF发射------------------------------------------------------
//wid发射脉冲宽度，
//发送8次数据的时间为 120ms = (0.96+0.34)*12*8
//dat1,dat2,dat3为2262与1527地址，dat4为按键值,wid为发射脉冲的宽度一般为60

void out0()						//发射0  
{
  RF1_ON;
	delay_us(short_width*5);    		//延时320us.
	RF1_OFF;
	delay_us(short_width*15);   	//长脉衝信號是短脉衝寬度的3倍.	 
}
void out1()						//发射1
{
   RF1_ON;
	delay_us(short_width*15);   	//延时960us.
	RF1_OFF;
	delay_us(short_width*5);    		//延时320us.	 
} 

void send_bat(unsigned char dat)//发送一个字节
{
	unsigned char a;

    for (a=0;a<8;a++)			//发送一个字节数据
	{
    	if((dat>>(7-a))&1==1)	
			out1();	       
		else 
			out0();		
	}
}

void send_rf(unsigned char dat1,unsigned char dat2,unsigned char dat3,unsigned char dat4,unsigned char wid)//1527编码发射 
{ 
	unsigned char dd,b; 

	 __set_PRIMASK(1);  // 关除了EMI和FAULT中断
	short_width = wid; 
    // ---------------------2262与1527自动按键值处理-------------------------------------
    for(b=0;b<4;b++){if(((dat1>>(b*2)) & 3)==2) {b=80;break;}}  			//有10则为1527
    if(b!=80)for(b=0;b<4;b++){if(((dat2>>(b*2)) & 3)==2) {b=80;break;}}  	//有10则为1527
    if(b!=80)for(b=0;b<2;b++){if((((dat3)>>(b*2)) & 3)==2) {b=80;break;}} 	//有10则为1527
    if(b==80) dat3=dat3*16+dat4;  	//1527
    else  							//2262
    {
		dd = 0;
        for(b=0;b<4;b++) if((dat4>>b)&1==1)  dd|=3<<(b*2);  				//还原按键值的2262编码
        dat3 = dd;
    }       
    //------------------------------------------------------------------------------------     
	for(dd=0;dd<8;dd++)         	//发送相同的8组码
	{	     
        send_bat(dat1);
		send_bat(dat2);
		send_bat(dat3);
		RF1_ON;
		delay_us(short_width*5); 
		RF1_OFF;
		delay_us(short_width*32*5);     //这几句是发射结束码及头信号	             
	}	
	 __set_PRIMASK(0);  // 打开了EMI和FAULT中断
}



//显示 按键 信息 

void RF_Display(uchar address1,uchar address2,uchar address3,uchar address4,uchar chip_ID,uchar key_value,uchar zhouqi) //地址1 地址2 数据 周期1 周期2   前面3个字节为地址和数据 周期1，2合起来成uint
{
uchar i,flag;
uchar f=0;
uchar wei1,wei2,wei3,wei4;
uint cycle[2],r;
//uchar key_ID;
flag=1;
	
	wei1=address1;
	wei2=address2;
	wei3=address3;
	wei4=address4;
//---------------------------------------------------------------------------------------------------------------------//

																	//":"
	                              LCD_Clear_Rectangle(70+48+32,50,70+48+32+80,50+16,BLACK);
                                switch(chip_ID)
                                {
																  //case 0:  break;
																	case 0:Show_Str(70+48+32,50,WHITE,BLACK,"xx2262",16,0);	break;
														      case 1:Show_Str(70+48+32,50,WHITE,BLACK,"xx1527",16,0);	break;
															    case 2:Show_Str(70+48+32,50,WHITE,BLACK,"T19-L",16,0);	break;	
																  case 3:Show_Str(70+48+32,50,WHITE,BLACK,"T19-S",16,0);	break;
																	default:break;
																}
//---------------------------------------------------------------------------------------------------------------------//
                             //显示ID码
                            	
                              // Show_Str(70,70,WHITE,BLACK,"ID",16,0);	
															// Show_Str(70+16,70,WHITE,BLACK,"    码",16,0);
															// Show_Str(70+64,70,WHITE,BLACK,":",16,0);
																
																			//////////////////////////////////////////////////////////////
                                      
																			//////////////////////////////////////////////////////////////
																        LCD_Clear_Rectangle(70+64+8,70,70+64+8+80,70+16,BLACK);
                                      //-----------------------data------------------------------------------------------------------------------//
																				for(i=0;i<4;i++)//中8位地址 把它显示为3态所以就变成4个
																					 {	  				  	
																						 if(i==0)
																						 {
																						  
																												 if((wei1&0xC0)==0x40)   LCD_ShowChar(70+64+8, 70, WHITE, BLACK, 'F', 16, 0); //01悬空F，15代码的F字母
																											 else if ((wei1&0xC0)==0xC0)  LCD_ShowChar(70+64+8, 70, WHITE, BLACK, '1', 16, 0); //11 高，0x31是液晶显示1
																											 else if ((wei1&0xC0)==0x80)  LCD_ShowChar(70+64+8, 70, WHITE, BLACK, '2', 16, 0); //10 高，0x31是液晶显示H
																											 else	                        LCD_ShowChar(70+64+8, 70, WHITE, BLACK, '0', 16, 0); //00 低，0x31是液晶显示0
																						 
																						 
																						 
																						 }else  if(i==1)

                                             {
																						         if((wei1&0xC0)==0x40)   LCD_ShowChar(70+64+16, 70, WHITE, BLACK, 'F', 16, 0); //01悬空F，15代码的F字母
																											 else if ((wei1&0xC0)==0xC0)  LCD_ShowChar(70+64+16, 70, WHITE, BLACK, '1', 16, 0); //11 高，0x31是液晶显示1
																											  else if ((wei1&0xC0)==0x80)  LCD_ShowChar(70+64+16, 70, WHITE, BLACK, '2', 16, 0); //10 高，0x31是液晶显示H
																											 else	                        LCD_ShowChar(70+64+16, 70, WHITE, BLACK, '0', 16, 0); //00 低，0x31是液晶显示0
																						 
																						 
																						 
																						 }else  if(i==2)	
                                             {
																						         if((wei1&0xC0)==0x40)   LCD_ShowChar(70+64+24, 70, WHITE, BLACK, 'F', 16, 0); //01悬空F，15代码的F字母
																											 else if ((wei1&0xC0)==0xC0)  LCD_ShowChar(70+64+24, 70, WHITE, BLACK, '1', 16, 0); //11 高，0x31是液晶显示1
																											 else if ((wei1&0xC0)==0x80)  LCD_ShowChar(70+64+24, 70, WHITE, BLACK, '2', 16, 0); //10 高，0x31是液晶显示H
																											 else	                        LCD_ShowChar(70+64+24, 70, WHITE, BLACK, '0', 16, 0); //00 低，0x31是液晶显示0
																						 
																						 
																						 }else
																							{
																											if((wei1&0xC0)==0x40)   LCD_ShowChar(70+64+32, 70, WHITE, BLACK, 'F', 16, 0); //01悬空F，15代码的F字母
																											 else if ((wei1&0xC0)==0xC0)  LCD_ShowChar(70+64+32, 70, WHITE, BLACK, '1', 16, 0); //11 高，0x31是液晶显示1
																											 else if ((wei1&0xC0)==0x80)  LCD_ShowChar(70+64+32, 70, WHITE, BLACK, '2', 16, 0); //10 高，0x31是液晶显示H
																											 else	                        LCD_ShowChar(70+64+32, 70, WHITE, BLACK, '0', 16, 0); //00 低，0x31是液晶显示0


																							}																						 
																						
																					 wei1=wei1<<2;
																					 }

                                        for(i=0;i<4;i++)//中8位地址 把它显示为3态所以就变成4个
																					 {	  				  	
																														if(i==0)
																												 {
																													
																																		 if((wei2&0xC0)==0x40)   LCD_ShowChar(70+64+8+32, 70, WHITE, BLACK, 'F', 16, 0); //01悬空F，15代码的F字母
																																	 else if ((wei2&0xC0)==0xC0)  LCD_ShowChar(70+64+8+32, 70, WHITE, BLACK, '1', 16, 0); //11 高，0x31是液晶显示1
																																	 else if ((wei2&0xC0)==0x80)  LCD_ShowChar(70+64+8+32, 70, WHITE, BLACK, 'H', 16, 0); //10 高，0x31是液晶显示H
																																	 else	                        LCD_ShowChar(70+64+8+32, 70, WHITE, BLACK, '0', 16, 0); //00 低，0x31是液晶显示0
																												 
																												 
																												 
																												 }else  if(i==1)

																												 {
																																 if((wei2&0xC0)==0x40)   LCD_ShowChar(70+64+16+32, 70, WHITE, BLACK, 'F', 16, 0); //01悬空F，15代码的F字母
																																	 else if ((wei2&0xC0)==0xC0)  LCD_ShowChar(70+64+16+32, 70, WHITE, BLACK, '1', 16, 0); //11 高，0x31是液晶显示1
																																	 else if ((wei2&0xC0)==0x80)  LCD_ShowChar(70+64+16+32, 70, WHITE, BLACK, 'H', 16, 0); //10 高，0x31是液晶显示H
																																	 else	                        LCD_ShowChar(70+64+16+32, 70, WHITE, BLACK, '0', 16, 0); //00 低，0x31是液晶显示0
																												 
																												 
																												 
																												 }else  if(i==2)	
																												 {
																																 if((wei2&0xC0)==0x40)   LCD_ShowChar(70+64+24+32, 70, WHITE, BLACK, 'F', 16, 0); //01悬空F，15代码的F字母
																																	 else if ((wei2&0xC0)==0xC0)  LCD_ShowChar(70+64+24+32, 70, WHITE, BLACK, '1', 16, 0); //11 高，0x31是液晶显示1
																																	 else if ((wei2&0xC0)==0x80)  LCD_ShowChar(70+64+24+32, 70, WHITE, BLACK, 'H', 16, 0); //10 高，0x31是液晶显示H
																																	 else	                        LCD_ShowChar(70+64+24+32, 70, WHITE, BLACK, '0', 16, 0); //00 低，0x31是液晶显示0
																												 
																												 
																												 }else
																													{
																																	if((wei2&0xC0)==0x40)   LCD_ShowChar(70+64+32+32, 70, WHITE, BLACK, 'F', 16, 0); //01悬空F，15代码的F字母
																																	 else if ((wei2&0xC0)==0xC0)  LCD_ShowChar(70+64+32+32, 70, WHITE, BLACK, '1', 16, 0); //11 高，0x31是液晶显示1
																																	 else if ((wei2&0xC0)==0x80)  LCD_ShowChar(70+64+32+32, 70, WHITE, BLACK, 'H', 16, 0); //10 高，0x31是液晶显示H
																																	 else	                        LCD_ShowChar(70+64+32+32, 70, WHITE, BLACK, '0', 16, 0); //00 低，0x31是液晶显示0


																													}																						 
																												
																											 wei2=wei2<<2;
																					 }
																			 //-----------------------display------------------------------------------------------------------------------//
																			///////////////////////////////////////////////////////////////////////////////////////////////
																
 //-------------------------------------------------------------------------------------------------------------------//
																//显示按键码 																
                              
                              // Show_Str(70,90,WHITE,BLACK,"按 键 码",16,0);	
															LCD_Clear_Rectangle(70+64+8,90,70+64+8+80,90+16,BLACK);
                               switch(key_value)
															 {
															   case 1: Show_Str(70+64+8,90, WHITE, BLACK, "0001", 16, 0);break; //00 低，0x31是液晶显示0break;
																 case 2: Show_Str(70+64+8,90, WHITE, BLACK, "0010", 16, 0);break; //00 低，0x31是液晶显示0break;
															   case 4: Show_Str(70+64+8,90, WHITE, BLACK, "0100", 16, 0);break; //00 低，0x31是液晶显示0break;
															   case 8: Show_Str(70+64+8,90, WHITE, BLACK, "1000", 16, 0);break; //00 低，0x31是液晶显示0break;
																 
																	default:break;
															 }
	 //-------------------------------------------------------------------------------------------------------------------//															
																LCD_Clear_Rectangle(70+64+8,110,70+64+8+80,110+16,BLACK);
																//显示周期
                            	
                              // Show_Str(70,110,WHITE,BLACK,"周    期",16,0);	
															 //Show_Str(70+32,110,WHITE,BLACK,"码",16,0);
															//Show_Str(70+64,110,WHITE,BLACK,":",16,0);
															 //Show_Str(70+64+8,110,WHITE,BLACK,"0X",16,0);
															 
															 //cycle=zhouqi;
															 cycle[0] = zhouqi / 16;
														   cycle[1] = zhouqi % 16;
															 ShowNum(70+64+8+16, 110, WHITE, BLACK, cycle[0], 16, 0);
															 ShowNum(70+64+8+16+8, 110, WHITE, BLACK, cycle[1], 16, 0);
 //-------------------------------------------------------------------------------------------------------------------//																
																
                                //显示电阻		

                              // Show_Str(70,130,WHITE,BLACK,"电    阻",16,0);	
															 Show_Str(70+64,130,WHITE,BLACK,":",16,0);
															 
															 	LCD_Clear_Rectangle(70+64+8,130,70+64+8+80,130+16,BLACK);
															 cycle[0] = zhouqi / 16;
														   cycle[1] = zhouqi % 16;
															 ShowNum(70+64+8+16, 130, WHITE, BLACK, cycle[0], 16, 0);
															 ShowNum(70+64+8+16+8, 130, WHITE, BLACK, cycle[1], 16, 0);
															 
															 
															 
															 
 //-------------------------------------------------------------------------------------------------------------------//	





}


