#include "parameter.h"
#include <stdlib.h>
#include "string.h"


extern CALIB_PARA  calib_para;

#define   user_para_len_MAX  0x20000 

float  	val_group[1400];
char		temp_string[50];

uint32_t var_NUM1= sizeof(CALIB_PARA)>>2;

uint32_t bd_cs_len = 0;

uint32_t read_cs_data(char *pbuf,uint8_t type)
{
	uint32_t 	px = 0;
	uint32_t 	val_i = 0;
	uint32_t 	head_num = 0;
	uint32_t 	str_head;
	if(str_compare(pbuf, "#STIM300",strlen("#STIM300")))
	return 1;
	while(pbuf[px] != '$' && px <user_para_len_MAX)
	{
		if(pbuf[px] == '#') head_num++;					
		if(head_num == 2)
		{
			head_num = 0;
			px += 3;
			str_head = px;
			while(pbuf[px] != '#' && pbuf[px] != '$' && px <user_para_len_MAX)
			{
				while(pbuf[px] != ' ' && pbuf[px] != '\r' && px <user_para_len_MAX) px++;				
				memcpy(temp_string,&pbuf[str_head],px-str_head);				
				temp_string[px-str_head] = '\0';				
				val_group[val_i++] = (float)atof(temp_string);
				while(pbuf[px] == ' '&& pbuf[px] != '$'&& px <user_para_len_MAX) px++;
				if(pbuf[px] == '\r')
				{	px+=2; str_head = px;}
				else if((pbuf[px] >='0' && pbuf[px] <='9') || pbuf[px] =='-')
				{str_head = px;}	
			}
			head_num++;
		}
		px++;			
	}
	
	float * p;
	if(type == 0x0C)
	{
		bd_cs_len = px;
		p	= (float *)&calib_para;
		for(uint32_t i = 0;i<(sizeof(CALIB_PARA)>>2);i++)
		{
			*p= val_group[i]; 
			p++;
		}	
	}
	else
	{
		return 1;
	}
}

uint8_t str_compare(char *s1, char *s2,uint32_t len)
{
	uint32_t i;
	for(i=0;i<len;i++)
	{
		if(s1[i]!=s2[i])
			break;
	}
	if(i != len)
		return 1;
	
	return 0;
	
}
