#include <stdlib.h>
#include <stdint.h>
#include "mt_common.h"
#include "cmt2300a_hal.h"
#include "cmt2300a_hal_rf315.h"
#include "cmt2300a_hal_rf330.h"
#include "cmt2300a_hal_rf433.h"

unsigned char rf_en = 0;
unsigned char da1527[2][4];
unsigned int short_k = 0;
unsigned char T19_S ;
unsigned char T19_L ;
unsigned char key_d;
unsigned char jmnx ;
unsigned char RF_data[31];


unsigned char ii = 0;

#define Delay_us(x) delay_us(x)

void RF315_IN(void) 
{
	unsigned char j = 0;
	unsigned char k = 0;
	unsigned char u,i;
    unsigned int head_k=0;
  	if (rf_en==0)
    {
        short_k = 0;
        while (RFIN_RF315 && j < 252)
        {
            Delay_us(5);
            short_k++;
        }
	  	while(!RFIN_RF315)
        {
		  	Delay_us(5);
		  	head_k++;
        }
      	if (((short_k*22)<head_k) && (head_k<(short_k*38)))
        {
            for(ii=0;ii<3;ii++)										//3??
            {
                for(k=0;k<8;k++)									//????8?
                {        	 	  	   
                    j = 0;
                    while(RFIN_RF315  && j<245) 
                    {
                        Delay_us(5);								//16us(6mhz:2~5)
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
                        Delay_us(5);
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
                            Delay_us(5);								//16us(6mhz:2~5)
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
                            Delay_us(5);
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
                Delay_us(5);
                j++;
                //j1++;
            }            			   
            head_k = 0;
            while(!RFIN_RF315 ) 						   //???????????
            {
                Delay_us(5);
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
                            Delay_us(5);								//16us(6mhz:2~5)
                            j++;
                        } 	
                        head_k = 0;
                        while(!RFIN_RF315  && j<255)	   		//?????
                        {
                            Delay_us(5);
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
                        Delay_us(5);								//16us(6mhz:2~5)
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
                        Delay_us(5);
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
                                Delay_us(5);								//16us(6mhz:2~5)
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
                                Delay_us(5);
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
            }

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
            else if((da1527[0][0]==da1527[1][0]) && (da1527[0][1]==da1527[1][1]) && (da1527[0][2]==da1527[1][2]))	//?????????????
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
                    for(i=0;i<4;i++)	  			//???? 2262?? ?????
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
