/**
* Copyright (c) Pervasive Displays Inc. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
**/

/***************************************************************************
 * The definition in this file is switched from Configuration under project.
 * It defines what functions can be supported in Driver_Handle_t structure.
 ***************************************************************************/

#ifndef COG_Drivers_List_H_
#define COG_Drivers_List_H_

//#include "conf_EPD.h"
#include "eTC_G2_Size_Aurora_Mb.h"



static const Driver_Handle_t Driver_Handle_list[]=
{


		{
			dr_eTC_G2_Aurora_Mb,
			(uint8_t []){EPD_144_BW,EPD_200_BW,EPD_271_BW,EPD_190_BW,EPD_260_BW,EPD_Empty},
		    eTC_G2Mb_IO_Config,
			eTC_G2Mb_power_on,
			eTC_G2Mb_initialize_driver,
			eTC_G2Mb_Display_from_Ram,
			eTC_G2Mb_Display_from_flash,
			eTC_G2Mb_Display_PU_from_Ram,
			eTC_G2Mb_Display_PU_From_Flash,
			eTC_G2Mb_power_off,
			NULL
		},


};

#endif
