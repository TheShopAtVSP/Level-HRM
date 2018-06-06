/*---------------------------------------------------------------------------*/
/*                                                                           */
/* FILE:    	Load Store Data.c                                  			 */
/* Data Dir:	C:\Act_Dis_Data_1                                            */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Include files                                                             */
/*---------------------------------------------------------------------------*/
#include <stdlib.h> 
#include <ansi_c.h>
#include <math.h>
#include <cvirte.h>
#include <userint.h>
#include <rs232.h>
#include <utility.h>
#include <formatio.h>
#include <string.h>
#include <toolbox.h>
#include "Activity Discriminator vars.h" 
#include "Activity Discriminator.h"



/*---------------------------------------------------------------------------*/
/* Module-globals                                                            */
/*---------------------------------------------------------------------------*/
static char		file_name[MAX_PATHNAME_LEN];
static int 		handle;

/*---------------------------------------------------------------------------*/
/* This function brings up a File Selection dialog and allows you to enter a */
/* file name with a .dat extension.                   		                 */
/*---------------------------------------------------------------------------*/
int CVICALLBACK Save_Data (int panel, int control, int event, void *callbackData,
                      int eventData1, int eventData2)
{
    int 	i;
	char 	index_fn[64], tmp_str[128];

    if (event == EVENT_COMMIT)
    {
		// Mux the data and store as ASCII in 6 col for easy review/edit.
/*		for(i=0; i<ser_input_size+64; i++)
		{
			 fio_array[i*6+0] = raw_acl_x[i];
			 fio_array[i*6+1] = raw_acl_y[i];
			 fio_array[i*6+2] = raw_acl_z[i];
			 fio_array[i*6+3] = raw_gyr_p[i];
			 fio_array[i*6+4] = raw_gyr_y[i];
			 fio_array[i*6+5] = raw_gyr_r[i];
			 
		}  */
		
		if (FileSelectPopupEx ("c:\\Act_Dis_Data_1", "*.dat", "*.dat",
		"Name of File to Save", VAL_OK_BUTTON, 1, 1, file_name) > 0)
    	{
            ArrayToFile (file_name, fio_array, VAL_DOUBLE, (ser_input_size + 64)*6, 6,
                         VAL_GROUPS_TOGETHER, VAL_GROUPS_AS_ROWS,
                         VAL_CONST_WIDTH, 16, VAL_ASCII, VAL_TRUNCATE);

			// store number of elements in a seperate file.
			sprintf(tmp_str, "%d", ser_input_size); 
			strcpy(index_fn, file_name);
			strcat(index_fn, ".nos");  // numb of samples
			handle = OpenFile (index_fn, VAL_READ_WRITE, VAL_TRUNCATE, VAL_ASCII);
			WriteFile (handle, tmp_str , strlen(tmp_str));
			CloseFile (handle);
		
			// store a copy into Last_One for fast load.
			ArrayToFile ("c:\\Act_Dis_Data_1\\Last_One_Data.dat", fio_array, VAL_DOUBLE, (ser_input_size + 64)*6, 6,
                         VAL_GROUPS_TOGETHER, VAL_GROUPS_AS_ROWS,
                         VAL_CONST_WIDTH, 16, VAL_ASCII, VAL_TRUNCATE);
			
			handle = OpenFile ("c:\\Act_Dis_Data_1\\Last_One_Index.dat.nos", VAL_READ_WRITE, VAL_TRUNCATE, VAL_ASCII);
			sprintf(tmp_str, "%d", ser_input_size);
			WriteFile (handle, tmp_str , strlen(tmp_str));
			CloseFile (handle);
    	}
    }
    return 0;
}

/*---------------------------------------------------------------------------*/
/* Read data from the file into our array, then plot it.                     */
/*---------------------------------------------------------------------------*/
int CVICALLBACK Load_Data (int panel, int control, int event, void *callbackData,
                      int eventData1, int eventData2)
{
	char 	nosfn[256];
	int 	i;
	char 	tmp_str[128];
	
    if (event == EVENT_COMMIT)
    {
    	if (FileSelectPopupEx ("c:\\Act_Dis_Data_1", "*.dat", "*.dat", "Name of File to Read", VAL_OK_BUTTON, 0, 1, file_name) > 0)
        {
			strcpy(nosfn, file_name);
			strcat(nosfn, ".nos");
			handle = OpenFile (nosfn, VAL_READ_ONLY, VAL_TRUNCATE, VAL_ASCII);
			ReadFile (handle, tmp_str , 10);
			CloseFile (handle);
			ser_input_size = atoi(tmp_str);
			look_back_starting_point = 0;
			//SetCtrlVal(mainpnl, MAINPNL_NRTS, ser_input_size);
			
			memset(fio_array, 0, sizeof(fio_array));
            FileToArray (file_name, fio_array, VAL_DOUBLE, (ser_input_size + 64)*6, 6,
                         VAL_GROUPS_TOGETHER, VAL_GROUPS_AS_ROWS, VAL_ASCII);
		
			// Demux the data.
			for(i=0; i<(ser_input_size + 64) * 6; i+=6)
			{
			 raw_acl_x[i/6] = fio_array[i+0];
			 raw_acl_y[i/6] = fio_array[i+1];
			 raw_acl_z[i/6] = fio_array[i+2];
			 raw_gyr_p[i/6] = fio_array[i+3];
			 raw_gyr_y[i/6] = fio_array[i+4];
			 raw_gyr_r[i/6] = fio_array[i+5];
            }
		}
		
		Apply_Raw_Data_Filtering();
		//Apply_Position_Integrator();
		//Apply_Peak_Detector();
		Update_Sig_Plots();
    }
	
    return 0;
}



int CVICALLBACK Load_Last_Data (int panel, int control, int event,
								void *callbackData, int eventData1, int eventData2)
{
	int 	i;
	char 	tmp_str[128];
	
	if (event == EVENT_COMMIT)
    {
		handle = OpenFile ("c:\\Act_Dis_Data_1\\Last_One_Index.dat.nos", VAL_READ_ONLY, VAL_TRUNCATE, VAL_ASCII);
		ReadFile (handle, tmp_str , 10);
		CloseFile (handle);
		ser_input_size = atoi(tmp_str);
		//SetCtrlVal(mainpnl, MAINPNL_NRTS, ser_input_size);
		
		memset(fio_array, 0, sizeof(fio_array));
        FileToArray ("c:\\Act_Dis_Data_1\\Last_One_Data.dat", fio_array, VAL_DOUBLE, (ser_input_size + 64)*6, 6,
                     VAL_GROUPS_TOGETHER, VAL_GROUPS_AS_ROWS, VAL_ASCII);
	
		// Demux the data.
		for(i=0; i<(ser_input_size + 64) * 6; i+=6)
		{
		 raw_acl_x[i/6] = fio_array[i+0];
		 raw_acl_y[i/6] = fio_array[i+1];
		 raw_acl_z[i/6] = fio_array[i+2];
		 raw_gyr_p[i/6] = fio_array[i+3];
		 raw_gyr_y[i/6] = fio_array[i+4];
		 raw_gyr_r[i/6] = fio_array[i+5];
        }
	
		Apply_Raw_Data_Filtering();
		//Apply_Position_Integrator();
		//Apply_Peak_Detector();
		Update_Sig_Plots();
    }
    return 0;
}



