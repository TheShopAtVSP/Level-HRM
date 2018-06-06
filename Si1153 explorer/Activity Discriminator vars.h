//----------------------------------------------------------------------------
// Activity_Discriminator_includes.h 
// WMM 5-3-09
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// Protos
//----------------------------------------------------------------------------
void GenerateGraphs(void); // Sig Generator.c
void  Set_Motor_Speed(void); //  Motor Control.c
void Uno_Scan_Loop(void); // Uno Serial Sensor Aqu.c
unsigned char Serial_Get_Byte(void); // Uno Serial Sensor Aqu.c
void Self_Steady(void);   // Self Steady.c
void Apply_Raw_Data_Filtering(void); // Raw Data Filters.c
void Apply_Position_Integrator(void); // Position Integrator.c
void Basic_Integrate(const double x[], ssize_t n, int dt, double xInit, double xFinal, double y[]); // Position Integrator.c
void Simpson_Integrate(const double x[], ssize_t n, int dt, double xInit, double xFinal, double y[]); // Position Integrator.c
void Apply_Peak_Detector(void); // Peak Detector.c
void Get_Gravity_Vector(void); // Gravity Vector.c
void Get_Manitude(int source_sel); // Magnitude.c
void Apply_Histogram_Mag(void); // Histogram.c
void Apply_Histogram_Acc(void); // Histogram.c
void Apply_Histogram_Gyro(void); // Histogram.c
void UpdateSG1(void); // Sig Graph.c
void UpdateSG2(void); // Sig Graph.c
void UpdateSG3(void); // Sig Graph.c
void UpdateSG4(void); // Sig Graph.c
void Update_PD_Plots(void); // Sig Graph.c
void Update_Sig_Plots(void); // Sig Graph.c
void Update_Cursors (void); // Sig Graph.c 
void InitOGLControl(void); // Gravity Vector OGL.c 
void DrawGVImage(int fastFlag); // Gravity Vector OGL.c								    
void InitOGLControl(void); // Gravity Vector OGL.c
void RenderGVImage(int fastFlag); // Gravity Vector OGL.c
void AX3dPlot(void); // AX3dPlot.c
void Determine_Activity(void); // Activity_Procedural.c


//----------------------------------------------------------------------------
// Defines
//----------------------------------------------------------------------------
#define HELP_MSG 		"This program: Activity Discriminator"
// #define RAW_DATA_SIZE 	2102	= 2048 + 64, 4160 = 4096 + 64
#define RAW_DATA_SIZE 	4160
#define DATA_DIR		"c:\Act_Dis_Data_1"
#define ON 1
#define OFF 0

//----------------------------------------------------------------------------
// Vars
//----------------------------------------------------------------------------
int mainpnl;				// handle for the main panel in the user interface
int OGLControlPanel;		// handle for the gravity vector window in the user interface

double	dbl_tmp[RAW_DATA_SIZE];
double 	acc_plot_index; 
double 	raw_acl_x[RAW_DATA_SIZE], raw_acl_y[RAW_DATA_SIZE], raw_acl_z[RAW_DATA_SIZE];
double 	raw_gyr_p[RAW_DATA_SIZE], raw_gyr_y[RAW_DATA_SIZE], raw_gyr_r[RAW_DATA_SIZE], raw_hart[RAW_DATA_SIZE], long_hart[2000];
double 	filt_acl_x[RAW_DATA_SIZE], filt_acl_y[RAW_DATA_SIZE], filt_acl_z[RAW_DATA_SIZE];
double 	filt_gyr_p[RAW_DATA_SIZE], filt_gyr_y[RAW_DATA_SIZE], filt_gyr_r[RAW_DATA_SIZE];
double 	intgtr_acl_x[RAW_DATA_SIZE], intgtr_acl_y[RAW_DATA_SIZE], intgtr_acl_z[RAW_DATA_SIZE];
double 	intgtr_gyr_p[RAW_DATA_SIZE], intgtr_gyr_y[RAW_DATA_SIZE], intgtr_gyr_r[RAW_DATA_SIZE];
double	acc_mag[RAW_DATA_SIZE], gyr_mag[RAW_DATA_SIZE], gyr_tmp[RAW_DATA_SIZE];
double 	*peak_position, *peak_ampl, *peak_deriv;
ssize_t pd_peak_cnt;
int		self_steady;
double 	ssax, ssay, ssaz; // Self Steady values
double 	ssgp, ssgy, ssgr;
int 	s1gx, s1gy, s1gz, s1gm, s2gx, s2gy, s2gz, s2gm;
int		s3gx, s3gy, s3gz, s3gm, s4gx, s4gy, s4gz, s4gm;
double	running_avg_acc_x, running_avg_acc_y, running_avg_acc_z;
double  running_avg_gyr_p, running_avg_gyr_y, running_avg_gyr_r; 
int 	ser_input_size;
int 	panel_handle,
    	config_handle,
	    ser_com_port,
	    bytes_read,
	    RS232Error,
	    com_status,
	    read_term_index,
	    read_term,
	    inqlen,         // Stores result from GetInQLen 
	    outqlen;        // Stores result from GetOutQLen 
short 	read_cnt;
double 	timeout, run_time;
char 	read_data[64],
	    tbox_read_data[64];
double 	fio_array[RAW_DATA_SIZE*6 + 64];     

double 	*hart_peak_pos, *hart_peak_amp, *hart_peak_deriv;
ssize_t hart_peak_cnt;
//
double 	wavedeconv[RAW_DATA_SIZE];
int 	autoGenerateGraphs;
int 	gv_firstSample, gv_acc_plot_index_old;
double	theta_x, theta_y;

double 	axdacgain, aydacgain, azdacgain, dacgain;
int		daconoff, invdacax, invdacay, invdacaz;

int		selfsteadyhasrun;
double  raw_hart_cir_buff[RAW_DATA_SIZE];
int		raw_hart_cir_buff_idx, data_stream_cir_buff_idx, sqra_on, sqrb_on;
double  aquisition_time, aquisition_start_time, phase2sf, phase2sfcof, aquisition_add_time, last_good_time;
double  ggprdavg[25], ggyrdavg[25];
int		ggrdavgindex, glasses_on, stitch_rear_edge_dect;
int		total_run_time, total_peaks, last_data_stream_cir_buff_idx, look_back_starting_point;
double 	delta_t,delta_f,frequency,ffreq,frequency_array[2048]; 

double	final_hrm_avg[64], final_hrm_avg_old, final_hrm_avg_second_diff;
int		final_hrm_avg_index, final_hrm_avg_valid_index, fo_avg_len, fo_start, fo_hit, fo_miss;

/*****************************************************************/
/*                                                               */
/*  Variable Descriptions                                        */
/*                                                               */
/*    sig1wavetype - variable indicating which type of wave    	 */
/*                     has been selected by the user             */
/*                     0 = sine wave (default)                   */
/*                     1 = square wave                           */
/*                     2 = triangle wave                         */
/*    sig1points - contains the number of points in the first  	 */
/*                   signal (default = 128)                      */
/*    sig1phase - contains the phase offset of the first       	 */
/*                  signal (default = 0)                         */
/*    wave1 - array containing the points of wave 1            	 */
/*    sig1noise - indicates whether noise is added to the      	 */
/*                  first signal prior to graphing and           */
/*                  calculating the convolve                     */
/*                  0 = disabled (default)                       */
/*                  1 = enabled                                  */
/*    sig1cycles - number of cycles in the first signal        	 */
/*                   (default = 5)                               */
/*    noisewave1 - array containing the noise that is added to 	 */
/*                   the first signal                            */
/*    i - loop variable                                        	 */
/*   sig1noiseamp - standard deviation of the Gaussian noise  	 */
/*                     added to the first signal                 */
/*   seed - seed passed to the Gaussian noise function        	 */
/*             (begins at 1 and changes throughout the program)  */
/*   sig2wavetype - signal type for the second wave           	 */
/*                     0 = sine wave (default)                   */
/*                     1 = square wave                           */
/*                     2 = triangular wave                       */
/*   sig2points - number of points in the second wave         	 */
/*                   (default = 32)                              */
/*   sig2phase - phase of the second signal (default = 0)     	 */
/*   wave2 - array containing the points of the second wave   	 */
/*   sig2noise - indicates whether noise will be added to the 	 */
/*                  second signal                                */
/*                  0 = disabled (default)                       */
/*                  1 = enabled                                  */
/*   sig2cycles - number of cycles in the second signal       	 */
/*                   (default = 1)                               */
/*   noisewave2 - array containing the Gaussian noise added   	 */
/*                   to the second signal                        */
/*   sig2noiseamp - standard deviation of the Gaussian noise  	 */
/*                     added to the second signal                */
/*   waveconv - array containing the convolve of the two      	 */
/*                 signals                                       */
/*                                                               */
/*   wavedeconv - array containing the deconvolve of the     	 */
/*                 convolve signal and signal one                */
/*                                                               */
/*****************************************************************/
