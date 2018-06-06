 //----------------------------------------------------------------------------
// Plot_routines 
// This is the threaded code for outout display.
// WMM 7-15-12
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// Includes
//----------------------------------------------------------------------------
#include <cviauto.h>
#include <analysis.h>
#include <ansi_c.h>
#include <utility.h>
#include <cvirte.h> 
#include <userint.h>
#include <math.h> 
#include "ii_raw_veiwer_includes.h"
#include "CII FCS Control Panel.h"   
#include "toolbox.h"
#include "3DGraphCtrl.h"	  
#include "ChartPanel with menu.h"
#include "PW_Menubar.h"

//----------------------------------------------------------------------------
// Vars
//----------------------------------------------------------------------------	
//#define SCALE 1
extern int color[9]; //={VAL_RED,VAL_BLUE,VAL_GREEN,VAL_MAGENTA,0xFF8710,VAL_YELLOW,VAL_WHITE,VAL_DK_GRAY};  
/*****************************************************************************/
/* Constants and Macros                                                      */
/*****************************************************************************/
//#define hrChk(f)            if (hr = (f), FAILED(hr)) goto Error; else
//#define PI 3.1415926535


/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/
//static CAObjHandle      gSurfaceGraph;
CAObjHandle      gSurfaceGraph;
CAObjHandle 	 plotsHandle = 0;
CAObjHandle 	 plotHandle = 0;
CAObjHandle      gAxis;
CAObjHandle      axes = 0;

int              gEditingLevels,
                 gEditingInterval,
                 gEditingAnchor,
                 gRefreshing;
int              gColorTable[] = { 0, 0x0000FFL, 0x00FF00L, 0xFF0000L }; 

VARIANT 		vxArray, vyArray, vzArray;

void Plot_Hist(int node)
{
	int 	range_log, range_lin, hit, overlay=0, draw_overlay, plot_what_tmp;
	int		i, j, x, y, paw, skip, step, err = 0;
	int 	gate_color, xaxisrhsv = 525000, xasmax, xasmin;
	char 	str_tmp[64];
	double 	si, p_max = 0, pxmin, pxmax, pymin = 0, pymax, y_position, crange, crange2;
	int 	used[2048], used_index, used_cnt, skip_j, cnevent, zindex, contor_index;
	double  gamma, contor_x[1024], contor_y[1024], agamma;																            
	double 	Dmax, Dmin, sumx, lscale, Pmean = 0, sy, hscale = 1.0;
	int    	gmax = 3000, nn;
	double 	ratio, max_ref;
	HRESULT     hr;
    VARIANT     zVt, tVt;
	CAObjHandle contours = 0;
 

// If no window or no new data exit.
	if((tree[node].window_is_alive == 0) || (tree[node].has_window == 0))
		return;

	// printf("PH - panel:%d  node=%d xp=%d, yp=%d\n", tree[node].panel_numb, node, tree[node].x_is_peak, tree[node].y_is_peak);  
	/// GetCtrlVal(tree[node].panel_numb, PANEL_PLOT_WHAT, &plot_what[node]);    
	if((!tree[node].has_window) || (!tree[node].redraw))	
		return;
	if((plot_what[node] & 0x03) == 1)  
		if(!cell_total_numb) 
			return;                
	if((plot_what[node] & 0x03) == 2)
		if(!ref_total_numb)	
			return;
	if((plot_what[node] & 0x03) == 3)
		if((!cell_total_numb) && (!ref_total_numb))	
			return;
		
// Window Inits
	tree[node].redraw = 0;
	xasmax = xaxisrhsv;
	xasmin = 0;
	skip = 100;  // Speed up if doing aquisition
	if(done == 1) 
		skip = 1;
	overlay = 0;
	plot_what_tmp = plot_what[node] & 0x03;
	if (aquire_events == 1)
		plot_what[node] = 1;	
	if((plot_what[node] & 0x03) == 3)
		overlay = 1;	
	
	SetCtrlAttribute (tree[node].panel_numb, PANEL_PCV_GRAPH, ATTR_REFRESH_GRAPH, 0);
	DeleteGraphPlot (tree[node].panel_numb, PANEL_PCV_GRAPH, -1, VAL_DELAYED_DRAW);  
	GetCtrlAttribute(tree[node].panel_numb, PANEL_PCV_GRAPH, ATTR_PLOT_AREA_WIDTH , &paw); 	// This is for the log plot bin numbers

	if((done) && (tree[node].plot_type <= 6))
		SetCtrlAttribute (tree[node].panel_numb, PANEL_MAKE_GATE, ATTR_DIMMED, 0); 		
	else
		SetCtrlAttribute (tree[node].panel_numb, PANEL_MAKE_GATE, ATTR_DIMMED, 1);
	
	if((tree[node].plot_type > 3) && (tree[node].plot_type != 7) && done)   
	{
		err = GetObjHandleFromActiveXCtrl(tree[node].panel_numb, PANEL_SURFACE_GRAPH, &gSurfaceGraph);
		if(err == 0)
		{
			err = CW3DGraphLib__DCWGraph3DGetPlots (gSurfaceGraph, NULL, &plotsHandle);
			if(err == 0)
				CW3DGraphLib_CWPlots3DItem (plotsHandle, NULL, CA_VariantInt(1), &plotHandle);
		}
		else
			return;
	}
				
for(draw_overlay = 0; draw_overlay <= overlay; draw_overlay++)
{
	if(alt_color <= 1) 
		gate_color = culor[(((node * 32) & 0xFF) + (((node * 32) & 0x100) >> 4)) & 0xFF]; 
	if(alt_color == 2)
	{
		if(tree[node].node_type == 1)   // parents are light green
			gate_color = VAL_GREEN;
		if(tree[node].node_type == 2)   // gates are light red
			gate_color = VAL_RED;
	}	
		
	if(overlay)
	{
		plot_what[node]	= 2 - draw_overlay;
		if(plot_what[node] == 2)
			gate_color = 0x00B0C0A0;
			//gate_color = VAL_GRAY;
	}
	
// Set Plot start and number of points.
	if(aquire_events == 1)
	{
		base_event = cell_total_numb - epu;
		nevent = epu - 1;
		if(base_event <= epu)
		{
			tree[node].redraw = 0;    
			return;
		}
	}
	else
	{
		base_event = 0;
		if((plot_what[node] & 0x03) == 1)
			nevent = cell_total_numb - 1;
		if((plot_what[node] & 0x03) == 2)
			nevent = ref_total_numb - 1;
	}
					 
// Mask if parent and do SDs
	Make_Node_Mask(node);
	Do_SD(node);
//	if(done)
//		Print_Cell_Stats(node); 
	
/// ------------------------------------------------- End of Inits ---------------------------------------------------------------------------------
// X Axis Linear Histogram Plots

	if(tree[node].plot_type == 1)
	{
		SetCtrlVal(tree[node].panel_numb, PANEL_LOG_LIN_Y, VAL_LINEAR);
		SetCtrlAttribute (tree[node].panel_numb, PANEL_LOG_LIN_Y, ATTR_DIMMED, 1);
		if(tree[node].x_is_log == 0)  
		{
			paw = RESOLUTION;
			if(tree[node].x_is_peak == 0)
				paw = paw / 8;

			step = 1;
			k=0;
		
			if(tree[node].x_is_log == 0)  // X Axis Linear Plots
			{
				Dmax = 525000;  //524288
				Dmin = -OFFSET;
			               
				SetAxisScalingMode (tree[node].panel_numb, PANEL_PCV_GRAPH, VAL_BOTTOM_XAXIS, VAL_MANUAL, Dmin, Dmax);
				SetCtrlAttribute(tree[node].panel_numb, PANEL_PCV_GRAPH, ATTR_XMAP_MODE, VAL_LINEAR);
				x_axis_scale_min_old[node] = x_axis_scale_min_curr[node] = 1;
				x_axis_scale_max_old[node] = x_axis_scale_max_curr[node] = 525000;  //printf("ph1\n");
				Draw_Xaxis_Labels(tree[node].panel_numb, 1, 525000, VAL_LINEAR);	 
				
				gmax = 256;
				for(i = 0; i <= gmax; i++) p2[node][i] = 0;				//p = zeros([gmax,1]);  //clear graphics histogram array
				p_max = 0;
			
				lscale = (gmax-1)/ (Dmax-Dmin);  					//Scaling factor for histogram array 

				for(nn = base_event; nn < (base_event + nevent); nn++) 
				{
				    if(tree[node].x_is_peak == 1) 					// Peak
						sy = peak[tree[node].x_chan_numb][nn];
					if(tree[node].x_is_peak == 0) 					// Area
						sy = area[tree[node].x_chan_numb][nn];
					if(tree[node].x_is_peak == 2) 					// Width
						sy = width[tree[node].x_chan_numb][nn];

					Pmean = floor((sy-Dmin) * lscale) + 1;
					if(Pmean < 0)
						Pmean = 0;
					p2[node][(int)Pmean]++;
					//if(p_max < p2[(int)Pmean]) 
					//	p_max = p2[(int)Pmean]; 
				}
			
				for (i = 0; i < gmax; i++)
				{
					xa[node][i] = i * Dmax/(gmax-1);
					if(p_max < p2[node][i]) 
							p_max = p2[node][i];
				}
			
				if((alt_color == 2) && (done == 1))
				{
					if(tree[node].node_type == 2)   // gates are filled light red 
						PlotXY (tree[node].panel_numb, PANEL_PCV_GRAPH, xa[node], p2[node], gmax, VAL_DOUBLE, VAL_DOUBLE,  VAL_BASE_ZERO_VERTICAL_BAR, VAL_SIMPLE_DOT, VAL_SOLID, 1, 0xf08080); 
					if(tree[node].node_type == 1)   // parents are filled light green 
						PlotXY (tree[node].panel_numb, PANEL_PCV_GRAPH, xa[node], p2[node], gmax, VAL_DOUBLE, VAL_DOUBLE,  VAL_BASE_ZERO_VERTICAL_BAR, VAL_SIMPLE_DOT, VAL_SOLID, 1, 0x80f080); 
				}
				PlotXY (tree[node].panel_numb, PANEL_PCV_GRAPH, xa[node], p2[node], gmax, VAL_DOUBLE, VAL_DOUBLE,  VAL_THIN_LINE, VAL_SIMPLE_DOT, VAL_SOLID, 1, gate_color);
				SetAxisScalingMode (tree[node].panel_numb, PANEL_PCV_GRAPH, VAL_LEFT_YAXIS, VAL_MANUAL, 0, ((p_max + 1.0) * 1.1));
				y_axis_scale_min_old[node] = y_axis_scale_min_curr[node] = 0;
				y_axis_scale_max_old[node] = y_axis_scale_max_curr[node] = ((p_max + 1.0) * 1.1);
				Draw_Yaxis_Labels(tree[node].panel_numb, 0, ((p_max + 1.0) * 1.1), 0); 
			}
		}
	
// X Axis Log Histogram Plots
		if(tree[node].x_is_log == 1)  
		{
			Dmax = 19*log10(2);
			Dmin = 0;
			
			SetCtrlAttribute(tree[node].panel_numb, PANEL_PCV_GRAPH, ATTR_XMAP_MODE, VAL_LINEAR);
			SetAxisScalingMode (tree[node].panel_numb, PANEL_PCV_GRAPH, VAL_BOTTOM_XAXIS, VAL_MANUAL, Dmin, 525000);
			x_axis_scale_min_old[node] = x_axis_scale_min_curr[node] = 1;
			x_axis_scale_max_old[node] = x_axis_scale_max_curr[node] = 525000;	 // printf("ph 2\n");
			Draw_Xaxis_Labels(tree[node].panel_numb, 1, 525000, VAL_LOG); 
	
			gmax = 256;
			for(i = 0; i <= gmax; i++) p2[node][1] = p2[node][i] = 0;		//p = zeros([gmax,1]);  //clear graphics histogram array		
			sumx = p_max = 0;
			lscale = (gmax-1)/ (Dmax-Dmin);  					//scaling factor for histogram array

			for(nn = base_event; nn < (base_event + nevent); nn++)  	 
			{
			    if(tree[node].x_is_peak == 1) 					// Peak
					sy = peak[tree[node].x_chan_numb][nn];
				if(tree[node].x_is_peak == 0) 					// Area
					sy = area[tree[node].x_chan_numb][nn];
				if(tree[node].x_is_peak == 2) 					// Width
					sy = width[tree[node].x_chan_numb][nn];

				if((sy > 1) && (sy < 1000000))
				{
			        Pmean = floor((log10(sy)-Dmin) * lscale) + 1;
					p2[node][(int)Pmean]++;
					//if(p_max < p2[(int)Pmean]) 
						//p_max = p2[(int)Pmean];
				}
			    else
			    	p2[node][1]++;									//p(1) = p(1)+1;

				p2[node][1] = p2[node][1] / 10;								//p(1) = p(1)/10;
			}
		
			for (i = 0; i < gmax; i++)
			{
				xa[node][i] = 91790.11841136133 * i * Dmax/(gmax-1);
				if(p_max < p2[node][i]) 
						p_max = p2[node][i];
			}
			
			if((alt_color == 2) && (done == 1))
			{
				if(tree[node].node_type == 2)   // gates are filled light red 
					PlotXY (tree[node].panel_numb, PANEL_PCV_GRAPH, xa[node], p2[node], gmax, VAL_DOUBLE, VAL_DOUBLE,  VAL_BASE_ZERO_VERTICAL_BAR, VAL_SIMPLE_DOT, VAL_SOLID, 1, 0xf08080); 
				if(tree[node].node_type == 1)   // parents are filled light green 
					PlotXY (tree[node].panel_numb, PANEL_PCV_GRAPH, xa[node], p2[node], gmax, VAL_DOUBLE, VAL_DOUBLE,  VAL_BASE_ZERO_VERTICAL_BAR, VAL_SIMPLE_DOT, VAL_SOLID, 1, 0x80f080); 
			}
	
			SetAxisScalingMode (tree[node].panel_numb, PANEL_PCV_GRAPH, VAL_LEFT_YAXIS, VAL_MANUAL, 0, ((p_max + 1.0) * 1.1));   
			PlotXY (tree[node].panel_numb, PANEL_PCV_GRAPH, xa[node], p2[node], gmax, VAL_DOUBLE, VAL_DOUBLE,  VAL_THIN_LINE, VAL_SIMPLE_DOT, VAL_SOLID, 1, gate_color);
			/// SetAxisScalingMode (tree[node].panel_numb, PANEL_PCV_GRAPH, VAL_LEFT_YAXIS, VAL_MANUAL, 0, ((p_max + 1.0) * 1.1));
			y_axis_scale_min_old[node] = y_axis_scale_min_curr[node] = 0;
			y_axis_scale_max_old[node] = y_axis_scale_max_curr[node] = ((p_max + 1.0) * 1.1);
			Draw_Yaxis_Labels(tree[node].panel_numb, 0, ((p_max + 1.0) * 1.1), VAL_LINEAR);
		}
	}			

/// ------------------------------------------------- End of Histograms ---------------------------------------------------------------------------------

// Dot Plots

	if((tree[node].plot_type >= 2) && (tree[node].plot_type != 7))            
	{
		SetCtrlAttribute (tree[node].panel_numb, PANEL_LOG_LIN_Y, ATTR_DIMMED, 0);
		SetCtrlAttribute(tree[node].panel_numb, PANEL_PCV_GRAPH, ATTR_XMAP_MODE, tree[node].x_is_log);
		SetCtrlAttribute(tree[node].panel_numb, PANEL_PCV_GRAPH, ATTR_YMAP_MODE, tree[node].y_is_log);
		SetAxisScalingMode (tree[node].panel_numb, PANEL_PCV_GRAPH, VAL_BOTTOM_XAXIS, VAL_MANUAL, tree[node].x_is_log, xaxisrhsv);
		SetAxisScalingMode (tree[node].panel_numb, PANEL_PCV_GRAPH, VAL_LEFT_YAXIS, VAL_MANUAL, tree[node].y_is_log, xaxisrhsv);
		
		x_axis_scale_min_old[node] = x_axis_scale_min_curr[node] = tree[node].x_is_log;
		x_axis_scale_max_old[node] = x_axis_scale_max_curr[node] = xaxisrhsv; //printf("dp3\n");
		Draw_Xaxis_Labels(tree[node].panel_numb, tree[node].x_is_log, xaxisrhsv, tree[node].x_is_log);   
		y_axis_scale_min_old[node] = y_axis_scale_min_curr[node] = tree[node].y_is_log;
		y_axis_scale_max_old[node] = y_axis_scale_max_curr[node] = xaxisrhsv;
		Draw_Yaxis_Labels(tree[node].panel_numb, tree[node].y_is_log, xaxisrhsv, tree[node].y_is_log);
		
		if(cell_total_numb != 0)
		{
			for(i = 0; i < nevent; i++)
			{
				if(tree[node].x_is_peak == 1)
					cell_peak_tmp[node][i] = peak[tree[node].x_chan_numb][i + base_event]; 
				if(tree[node].x_is_peak == 0)
					cell_peak_tmp[node][i] = area[tree[node].x_chan_numb][i + base_event];
				if(tree[node].x_is_peak == 2)
					cell_peak_tmp[node][i] = width[tree[node].x_chan_numb][i + base_event];
			
				if(tree[node].y_is_peak == 1)
					cell_area_tmp[node][i] = peak[tree[node].y_chan_numb][i + base_event]; 
				if(tree[node].y_is_peak == 0)
					cell_area_tmp[node][i] = area[tree[node].y_chan_numb][i + base_event];
				if(tree[node].y_is_peak == 2)
					cell_area_tmp[node][i] = width[tree[node].y_chan_numb][i + base_event];
			}
		}
		
		if((tree[node].plot_type == 2) || !done)
		{
			PlotXY (tree[node].panel_numb, PANEL_PCV_GRAPH, cell_peak_tmp[node], cell_area_tmp[node], \
					nevent, VAL_DOUBLE, VAL_DOUBLE,  VAL_SCATTER, VAL_SIMPLE_DOT, VAL_SOLID, 1, gate_color);
		}
	
		// 3D plot 
		if( (tree[node].plot_type == 6) && done )    
		{
			CW3DGraphLib__DCWGraph3DGetAxes(gSurfaceGraph, NULL, &axes);
			CW3DGraphLib_CWAxes3DItem(axes, NULL, CA_VariantInt(1), &gAxis);
			CW3DGraphLib_CWAxis3DSetLog(gAxis, NULL, tree[node].x_is_log);
			strcpy(str_tmp, chan_label[tree[node].x_chan_numb]);
			if(tree[node].x_is_peak == 0)
				strcat(str_tmp, "-A"); 
			if(tree[node].x_is_peak == 1)
				strcat(str_tmp, "-H");
			if(tree[node].x_is_peak == 2)
				strcat(str_tmp, "-W");
			CW3DGraphLib_CWAxis3DSetCaption (gAxis, NULL, str_tmp);

			CW3DGraphLib_CWAxes3DItem(axes, NULL, CA_VariantInt(2), &gAxis);
			CW3DGraphLib_CWAxis3DSetLog(gAxis, NULL, tree[node].y_is_log);
			strcpy(str_tmp, chan_label[tree[node].y_chan_numb]);
			if(tree[node].y_is_peak == 0)
				strcat(str_tmp, "-A"); 
			if(tree[node].y_is_peak == 1)
				strcat(str_tmp, "-H");
			if(tree[node].y_is_peak == 2)
				strcat(str_tmp, "-W");
			CW3DGraphLib_CWAxis3DSetCaption (gAxis, NULL, str_tmp);
			
			CW3DGraphLib_CWAxes3DItem(axes, NULL, CA_VariantInt(3), &gAxis);
			CW3DGraphLib_CWAxis3DSetLog(gAxis, NULL, tree[node].z_is_log);
			if(tree[node].z_chan_numb == -1)
			{
				strcpy(str_tmp, "Zero");	
			}
			else
			{
				strcpy(str_tmp, chan_label[tree[node].z_chan_numb]);
				if(tree[node].z_is_peak == 0)
					strcat(str_tmp, "-A"); 
				if(tree[node].z_is_peak == 1)
					strcat(str_tmp, "-H");
				if(tree[node].z_is_peak == 2)
					strcat(str_tmp, "-W");
			}
			CW3DGraphLib_CWAxis3DSetCaption (gAxis, NULL, str_tmp);
																		 
			CA_VariantSet1DArray (&vxArray, CAVT_DOUBLE, nevent, &cell_peak_tmp[node]);
			CA_VariantSet1DArray (&vyArray, CAVT_DOUBLE, nevent, &cell_area_tmp[node]);
			

			if(tree[node].z_chan_numb == -1)
			{
				for(i = 0; i < nevent; i++)     
					xa[node][i] = 0;
				CA_VariantSet1DArray (&vzArray, CAVT_DOUBLE, nevent, &xa[node]);
			}
			else
			{
				if(tree[node].z_is_peak == 0x01)
					CA_VariantSet1DArray (&vzArray, CAVT_DOUBLE, nevent, &peak[tree[node].z_chan_numb]);
				if(tree[node].z_is_peak == 0x00) 
					CA_VariantSet1DArray (&vzArray, CAVT_DOUBLE, nevent, &area[tree[node].z_chan_numb]);
				if(tree[node].z_is_peak == 0x02) 
					CA_VariantSet1DArray (&vzArray, CAVT_DOUBLE, nevent, &width[tree[node].z_chan_numb]);
			}
			//CW3DGraphLib__DCWGraph3DClearData (gSurfaceGraph, NULL);
			CW3DGraphLib_CWPlot3DPlot3DCurve(plotHandle, NULL, vxArray, vyArray, vzArray, CA_DEFAULT_VAL);
			
			CA_VariantClear (&vxArray);
			CA_VariantClear (&vyArray);
			CA_VariantClear (&vzArray);
		}
		
		//-------------------------------------------------------------------------------
		/// ==========  Color Density, Heat Map and Contour Map Dot Plots =============
		//-------------------------------------------------------------------------------     
		if( ((tree[node].plot_type == 3) || (tree[node].plot_type == 4) || (tree[node].plot_type == 5)) && done)
		{
			// Inits:
			crange = 2500;  // linear axis range seed value
			gamma = 1;
			used_cnt = 0;
			for(i = 0; i < 2048; i++)
				used[i] = 0;
			
			// Since the density is shown by color we do not have to show all the dots. This will speed it up. Might change it to step size later...
			/// cnevent = 100;  /// 
			cnevent = nevent;
			if(cnevent >= 1600)
				cnevent = 1600;   
				//step = (int)((float)cnevent / (2000.0));
			//else
				step = 1;
			
			// First sort the X axis data -- This is for x=p or x=a   PS. cell_peak_tmp[node] is re-used to save memory 
			if(tree[node].x_is_peak == 1)
				Sort (peak[tree[node].x_chan_numb], cnevent, ANALYSIS_SORT_ASCENDING, cell_peak_tmp[node] );
			if(tree[node].x_is_peak == 0)
				Sort (area[tree[node].x_chan_numb], cnevent, ANALYSIS_SORT_ASCENDING, cell_peak_tmp[node] );
				
			/// Now link the Y axis data to the sorted x axis data. This presumes that no two peaks are alike...
			// Now link the Y axis data to the sorted x axis data. Use used[n] to make sure that there are no repeated assignments
			for(i = 0; i < cnevent - step; i = i + step)
			{   
				if(node_mask[node][i])
				{
					for(j = 0; j < cnevent; j ++)
					{
						if(node_mask[node][j])
						{
							if(tree[node].x_is_peak == 1)
							{
								if(peak[tree[node].x_chan_numb][j + base_event] == cell_peak_tmp[node][i])
								{
									skip_j = 0;
									for(used_index = 0; used_index <= used_cnt; used_index++)
									{
										if(j == used[used_index])
										{
											skip_j = 1;	
										}
									}
									if(!skip_j)
									{
										if(tree[node].y_is_peak == 1)
											cell_area_tmp[node][i] = peak[tree[node].y_chan_numb][j + base_event];
										if(tree[node].y_is_peak == 0)
											cell_area_tmp[node][i] = area[tree[node].y_chan_numb][j + base_event];
										used[used_cnt] = j;
										used_cnt++;
										j = cnevent; // Array 2 is linked to Array 1 so out we go...
									} 
								}
							}
					
							if(tree[node].x_is_peak == 0)
							{
								if(area[tree[node].x_chan_numb][j + base_event] == cell_peak_tmp[node][i])
								{
									skip_j = 0;
									for(used_index = 0; used_index <= used_cnt; used_index++)
									{
										if(j == used[used_index])
										{
											skip_j = 1;	
										}
									}
									if(!skip_j)
									{
										if(tree[node].y_is_peak == 1)
											cell_area_tmp[node][i] = peak[tree[node].y_chan_numb][j + base_event];
										if(tree[node].y_is_peak == 0)
											cell_area_tmp[node][i] = area[tree[node].y_chan_numb][j + base_event];
										used[used_cnt] = j;
										used_cnt++;
										j = cnevent; // Array 2 is linked to Array 1 so out we go...
									} 
								}
							}
						}
					}
				}
			}
			//sprintf(tmpstr, "CDP Link done.\nlin mean = %f, crange=%f\n",gamma, crange); ///  
			//SetCtrlVal (panel, PANEL_TEXTBOX, tmpstr); /// 
			//PlotXY (tree[node].panel_numb, PANEL_PCV_GRAPH, cell_peak_tmp[node], cell_area_tmp[node], cnevent, VAL_DOUBLE, VAL_DOUBLE,  VAL_SCATTER, VAL_SIMPLE_DOT, VAL_SOLID, 1, VAL_DK_GRAY); /// 
			
			// Now take the sorted and associated arrays and generate a 2d array with a 
			// z value that is relitive to the number of surounding dots.  x=p, y=a
			zindex = 0;
			if(tree[node].x_is_log == 0)  
			{
				GetCtrlVal(tree[node].panel_numb, PANEL_GAMMA, &range_lin);
				step = 1; //cnevent/10; // get all of the data 
				j = 0;
				while(((int)gamma != range_lin) && (j < 40))
				{
					for(i = 0; i < cnevent  - step; i = i + step)
					{		
						z[zindex] = 0;
						x=0;
						hit = 1;
						while( (hit) && (x < 50))
						{
							x++;
							if( ((cell_peak_tmp[node][i] < (cell_peak_tmp[node][i+x] + crange))  && \
								(cell_peak_tmp[node][i] > (cell_peak_tmp[node][i+x]  - crange))) )
							{	
								if(	((cell_area_tmp[node][i] < (cell_area_tmp[node][i+x] + crange))  && \
									(cell_area_tmp[node][i] > (cell_area_tmp[node][i+x]  - crange))) )
								{
									z[zindex]++;
								}
							}
							else
								hit = 0;
						}
						zindex++;
						j++;
					}
					Mean (z, zindex, &gamma );
					crange = crange*((float)(range_lin+1)/(gamma+1));
					if(crange <= 10.0)
						break;
					zindex = 0;
					//sprintf(tmpstr, "lin mean = %f, crange=%f\n",gamma, crange); ///  
					//SetCtrlVal (panel, PANEL_TEXTBOX, tmpstr); /// 
				}
			}
		
			if(tree[node].x_is_log == 1)  
			{
				GetCtrlVal(tree[node].panel_numb, PANEL_GAMMA, &range_log);
				range_lin = 2;
				step = 1; //cnevent/10; // get all of the data
				for(i = 0; i < cnevent  - step; i = i + step)
				{ 		
					z[i] = 0;
					x=0;
					hit = 1;
					
					crange2 = ( (cell_peak_tmp[node][i]) / (double)(51 - range_log) );
					while( (hit) && (x < 50) )
					{
						x++;
						if( ((cell_peak_tmp[node][i] < (cell_peak_tmp[node][i+x] + crange2))  && \
							(cell_peak_tmp[node][i] > (cell_peak_tmp[node][i+x]  - crange2))) )
						{	
							if(	((cell_area_tmp[node][i] < (cell_area_tmp[node][i+x] + crange2))  && \
								(cell_area_tmp[node][i] > (cell_area_tmp[node][i+x]  - crange2))) )
							{
								z[i]++;
							}
						}
						else
							hit = 0;
					}
				}
			}
			
			QScale1D (z, cnevent, z, &q_factor);
			sprintf(tmpstr, "q_factor = %f\n",q_factor); ///  
			SetCtrlVal (panel, PANEL_TEXTBOX, tmpstr); /// 
			
			// Now see how many "white" points there are and make that at least 20% of the points.	   Agamma
			// after this is done once, let the agamma control multiply this factor.
			Sort (z, cnevent, ANALYSIS_SORT_ASCENDING, z_tmp );  
			agamma = z_tmp[(int)((float)cnevent * 0.8)];
			for(i = 0; i < cnevent-1; i++)
			{
				z[i] = z[i] / agamma;
				if((z[i]* 100) < 1)
					z[i] = 0.01;
				if(z[i] > 1.0)  // limit z[i] Agamma
					z[i] = 1.0;
			}
			
			if( ((tree[node].plot_type == 4) || (tree[node].plot_type == 5)) ) // Clear zData[n][n]
			{
				for (i = 0; i < G3D_POINTS; ++i) 
					for (j = 0; j < G3D_POINTS; ++j) 
						zData[i][j] = 0;
			}
			
			// now plot x,y with z as the color
			step = 1;
			for(i = 0; i < cnevent  - step; i = i + step)
			{
				if( ((tree[node].plot_type == 4) || (tree[node].plot_type == 5)) ) // Smooth or contour plots
				{
					if(cell_peak_tmp[node][i] >= 550001)
						cell_peak_tmp[node][i] = 550000; 
					if(cell_area_tmp[node][i] >= 500001)
						cell_area_tmp[node][i] = 550000;
					if(cell_peak_tmp[node][i] <= 1)
						cell_peak_tmp[node][i] = 1; 
					if(cell_area_tmp[node][i] <= 1)
						cell_area_tmp[node][i] = 1;
					
					// x =  (int)(cell_peak_tmp[node][i]/3465);  x_axis_scale_max_curr[node] x_axis_scale_min_curr[node]
					// y =  (int)(cell_area_tmp[node][i]/4500); 
					// x =  (int)(cell_peak_tmp[node][i]*( 550000/(x_axis_scale_max_curr[node] - x_axis_scale_min_curr[node])) + x_axis_scale_min_curr[node]);
					// y =  (int)(cell_area_tmp[node][i]*( 550000/(y_axis_scale_max_curr[node] - y_axis_scale_min_curr[node])) + y_axis_scale_min_curr[node]);
					x =  (int)cell_peak_tmp[node][i];
					y =  (int)cell_area_tmp[node][i];   
					x = x / 5500;
					y = y / 5500; //5250
					
					if( x > 100) x = 100;
					if( y > 100) y = 100;
				
					if((tree[node].x_is_log == 1) && (tree[node].y_is_log == 1)) 
					    zData[(int)(log10(cell_peak_tmp[node][i]) * (15))][(int)(log10(cell_area_tmp[node][i]) * (15))] = z[i];
					if((tree[node].x_is_log == 0) && (tree[node].y_is_log == 0)) 
						zData[x][y] = z[i];
					if((tree[node].x_is_log == 0) && (tree[node].y_is_log == 1)) 
					    zData[x][(int)(log10(cell_area_tmp[node][i]) * (15))] = z[i];  /// 17.482
					if((tree[node].x_is_log == 1) && (tree[node].y_is_log == 0)) 
					    zData[(int)(log10(cell_peak_tmp[node][i]) * (15))][y] = z[i];
				}
				
				if(tree[node].plot_type == 3)  // Density Plot
				{
					PlotPoint (tree[node].panel_numb, PANEL_PCV_GRAPH, cell_peak_tmp[node][i], cell_area_tmp[node][i], VAL_SMALL_SOLID_SQUARE, colors[(int)(z[i]*15.99)]);
				}
					
			}
			
			if( ((tree[node].plot_type == 4) || (tree[node].plot_type == 5)) ) // Smooth or Contour plots
			{
				//CW3DGraphLib__DCWGraph3DClearData (gSurfaceGraph, NULL);
				for (i = 0; i < G3D_POINTS; ++i)
		        	t[i] = ( (i * 1000) / (G3D_POINTS - 1.0) ) * 550;
				CA_VariantSet1DArray(&tVt, CAVT_DOUBLE, G3D_POINTS, t);
            	CA_VariantSet2DArray(&zVt, CAVT_DOUBLE, G3D_POINTS, G3D_POINTS, zData);
				
				CW3DGraphLib__DCWGraph3DPlot3DSurface(gSurfaceGraph, NULL, tVt, tVt, zVt, CA_DEFAULT_VAL);
	
				if(tree[node].plot_type == 4)
   					CW3DGraphLib_CWPlot3DSetStyle (plotHandle, NULL, CW3DGraphLibConst_cwSurface);
                if(tree[node].plot_type == 5)
                    CW3DGraphLib_CWPlot3DSetStyle (plotHandle, NULL, CW3DGraphLibConst_cwContourLine);
				
				CW3DGraphLib__DCWGraph3DPlot3DSurface(gSurfaceGraph, NULL, tVt, tVt, zVt, CA_DEFAULT_VAL);   
			}
		}
	}
/// ------------------------------------------------- end of dot plots ---------------------------------------------------------------------------------
	
// Event Time Plots
	if(tree[node].plot_type == 7)            
	{
		if(cell_total_numb != 0)
		{
			SetCtrlAttribute (tree[node].panel_numb, PANEL_LOG_LIN_Y, ATTR_DIMMED, 0);
			SetCtrlAttribute(tree[node].panel_numb, PANEL_PCV_GRAPH, ATTR_XMAP_MODE, 0);
			SetCtrlAttribute(tree[node].panel_numb, PANEL_PCV_GRAPH, ATTR_YMAP_MODE, tree[node].y_is_log);
			SetAxisScalingMode (tree[node].panel_numb, PANEL_PCV_GRAPH, VAL_BOTTOM_XAXIS, VAL_MANUAL, 0, cell_timer[cell_total_numb - 1]);
			SetAxisScalingMode (tree[node].panel_numb, PANEL_PCV_GRAPH, VAL_LEFT_YAXIS, VAL_MANUAL, tree[node].y_is_log, xaxisrhsv);
		
			x_axis_scale_min_old[node] = x_axis_scale_min_curr[node] = 0;
			x_axis_scale_max_old[node] = x_axis_scale_max_curr[node] = (int)cell_timer[cell_total_numb - 1]; 
			Draw_Xaxis_Labels(tree[node].panel_numb, 0, (int)cell_timer[cell_total_numb - 1], 0);   
			y_axis_scale_min_old[node] = y_axis_scale_min_curr[node] = tree[node].y_is_log;
			y_axis_scale_max_old[node] = y_axis_scale_max_curr[node] = xaxisrhsv;
			Draw_Yaxis_Labels(tree[node].panel_numb, tree[node].y_is_log, xaxisrhsv, tree[node].y_is_log);
		
			for(i = 0; i < nevent; i++)
			{
				cell_peak_tmp[node][i] = cell_timer[i + base_event];   
				
				if(tree[node].y_is_peak == 1)
					cell_area_tmp[node][i] = peak[tree[node].y_chan_numb][i + base_event]; 
				if(tree[node].y_is_peak == 0)
					cell_area_tmp[node][i] = area[tree[node].y_chan_numb][i + base_event];
				if(tree[node].y_is_peak == 2)
					cell_area_tmp[node][i] = width[tree[node].y_chan_numb][i + base_event];
			}
		}

			PlotXY (tree[node].panel_numb, PANEL_PCV_GRAPH, cell_peak_tmp[node], cell_area_tmp[node], \
					nevent, VAL_DOUBLE, VAL_DOUBLE,  VAL_SCATTER, VAL_SIMPLE_DOT, VAL_SOLID, 1, gate_color);
	}
/// ------------------------------------------------- end of Event Timer plots ---------------------------------------------------------------------------------     
	
// Redraw Gate lines in Parent window
	Redraw_Gate_Lines( node, aquire_events );

// Print window stats
	//GetCtrlVal(tree[node].panel_numb, PANEL_STATS, &show_stats);
	if(show_stats[node])
	{
		GetAxisScalingMode(tree[node].panel_numb, PANEL_PCV_GRAPH, VAL_BOTTOM_XAXIS, NULL, &pxmin, &pxmax);
		GetAxisScalingMode(tree[node].panel_numb, PANEL_PCV_GRAPH, VAL_LEFT_YAXIS, NULL, &pymin, &pymax);

		if(tree[node].node_mask_size != 0)
		{	 
			SetCtrlAttribute (tree[node].panel_numb, PANEL_PCV_GRAPH, ATTR_SHIFT_TEXT_PLOTS, 1);   
			pxmax = 4; p_max = pymax;
			//if(draw_overlay)
				//p_max = pymax * 0.9;
			if(tree[node].y_is_peak == 1) 
			{   //printf("yip1 %f, %f  ", pxmax-1, p_max);     
				sprintf(tmpstr, "H Events:%d   Avg%6.2f  CV%6.2f  SD%6.2f   ", tree[node].node_mask_size, peak_avg[tree[node].x_chan_numb], pcv[tree[node].x_chan_numb], fabs(psd));
				if(alt_color && !draw_overlay)
					PlotText (tree[node].panel_numb, PANEL_PCV_GRAPH, pxmax-1, p_max, tmpstr, VAL_MESSAGE_BOX_META_FONT, VAL_OFFWHITE, VAL_TRANSPARENT);
				if(alt_color && draw_overlay)
					PlotText (tree[node].panel_numb, PANEL_PCV_GRAPH, pxmax-1, p_max, tmpstr, VAL_MESSAGE_BOX_META_FONT, VAL_DK_GRAY, VAL_TRANSPARENT);
				if(!alt_color)
					PlotText (tree[node].panel_numb, PANEL_PCV_GRAPH, pxmax-1, p_max, tmpstr, VAL_MESSAGE_BOX_META_FONT, VAL_DK_GRAY, VAL_TRANSPARENT);
			}
			if(tree[node].y_is_peak == 0)
			{   //printf("yip0 %f, %f  ", pxmax-1, p_max);     
				sprintf(tmpstr, "A Events:%d   Avg%6.2f  CV%6.2f  SD%6.2f   ", tree[node].node_mask_size, area_avg[tree[node].x_chan_numb], acv[tree[node].x_chan_numb], fabs(asd));
				if(alt_color && !draw_overlay)
					PlotText (tree[node].panel_numb, PANEL_PCV_GRAPH, pxmax-1, p_max, tmpstr, VAL_MESSAGE_BOX_META_FONT, VAL_OFFWHITE, VAL_TRANSPARENT);
				if(alt_color && draw_overlay)
					PlotText (tree[node].panel_numb, PANEL_PCV_GRAPH, pxmax-1, p_max, tmpstr, VAL_MESSAGE_BOX_META_FONT, VAL_DK_GRAY, VAL_TRANSPARENT);
				if(!alt_color)
					PlotText (tree[node].panel_numb, PANEL_PCV_GRAPH, pxmax-1, p_max, tmpstr, VAL_MESSAGE_BOX_META_FONT, VAL_DK_GRAY, VAL_TRANSPARENT);
			}	
			if(tree[node].y_chan_numb == -1)
				y_position = pymax;
			//if(draw_overlay)
				//y_position = pymax * 0.9;
			else
				y_position = pymin;
			if(tree[node].x_is_peak == 1) 
			{   //printf("xip1 %f, %f  ", pxmax-1, p_max);
				sprintf(tmpstr, "H Events:%d   Avg%6.2f  CV%6.2f  SD%6.2f   ", tree[node].node_mask_size, peak_avg[tree[node].x_chan_numb], pcv[tree[node].x_chan_numb], fabs(psd));
				if(alt_color && !draw_overlay)  
					PlotText (tree[node].panel_numb, PANEL_PCV_GRAPH, pxmax-1, y_position, tmpstr, VAL_MESSAGE_BOX_META_FONT, VAL_OFFWHITE, VAL_TRANSPARENT);
				if(alt_color && draw_overlay)  
					PlotText (tree[node].panel_numb, PANEL_PCV_GRAPH, pxmax-1, y_position, tmpstr, VAL_MESSAGE_BOX_META_FONT, VAL_DK_GRAY, VAL_TRANSPARENT);
				if(!alt_color)
					PlotText (tree[node].panel_numb, PANEL_PCV_GRAPH, pxmax-1, y_position, tmpstr, VAL_MESSAGE_BOX_META_FONT, VAL_DK_GRAY, VAL_TRANSPARENT);
			}
			if(tree[node].x_is_peak == 0)
			{   //printf("xip0 %f, %f  ", pxmax-1, p_max);
				sprintf(tmpstr, "A Events:%d   Avg%6.2f  CV%6.2f  SD%6.2f   ", tree[node].node_mask_size, area_avg[tree[node].x_chan_numb], acv[tree[node].x_chan_numb], fabs(asd));
				if(alt_color && !draw_overlay)  
					PlotText (tree[node].panel_numb, PANEL_PCV_GRAPH, pxmax-1, y_position, tmpstr, VAL_MESSAGE_BOX_META_FONT, VAL_OFFWHITE, VAL_TRANSPARENT);
				if(alt_color && draw_overlay)  
					PlotText (tree[node].panel_numb, PANEL_PCV_GRAPH, pxmax-1, y_position, tmpstr, VAL_MESSAGE_BOX_META_FONT, VAL_DK_GRAY, VAL_TRANSPARENT);
				if(!alt_color)
					PlotText (tree[node].panel_numb, PANEL_PCV_GRAPH, pxmax-1, y_position, tmpstr, VAL_MESSAGE_BOX_META_FONT, VAL_DK_GRAY, VAL_TRANSPARENT);
			}
		}
	}
}
	/// GetCtrlVal(tree[node].panel_numb, PANEL_PLOT_WHAT, &plot_what[node]);
	SetCtrlAttribute (tree[node].panel_numb, PANEL_PCV_GRAPH, ATTR_REFRESH_GRAPH, 1);
	plot_what[node] = plot_what_tmp;
	if( ((tree[node].plot_type == 3) || (tree[node].plot_type == 4) || (tree[node].plot_type == 5) || (tree[node].plot_type == 6)) && done )
		Color_Dot_Plot (tree[node].panel_numb, 99, EVENT_COMMIT, NULL, 99, 99);
} 

/// ------------------------------------------------- end of Plot_Hist() ---------------------------------------------------------------------------------

	
//==========================================	
/// Support Functions:
//==========================================

void Refresh_Plots(int plot_node)
{
	int  i, j, tmp_node, node;
	char tmpstr[64], gtree[32][8];  

	for(node = 0; node < 32; node++)
	{
		if((node == plot_node) || (plot_node == -1)) 
		{
			if(tree[node].node_type != 0)
			{
				if((tree[node].window_is_alive == 1) && (tree[node].has_window == 1))
				{
					if(alt_color != alt_color_old[node])
					{
						if(alt_color == 1)
							background_color = VAL_BLACK;															 
						if(alt_color == 0)
							background_color = 0x00D2D3AD;   
						if(alt_color == 2)
							background_color = VAL_WHITE; 
						if((tree[node].plot_type < 4) || (tree[node].plot_type == 7))
							SetCtrlAttribute (tree[node].panel_numb, PANEL_PCV_GRAPH, ATTR_PLOT_BGCOLOR, background_color);
						//Make_culor_Array(node);
						alt_color_old[node] = alt_color;
					}
					if(done)
					{
						Make_culor_Array(node);     
						x_scale_drawn = 0;
	/// printf("Refresh_Plots node  %d\n", node);
						//tree[node].redraw = 1;
					
						if(ref_data_loaded)
						{
							SetCtrlAttribute (tree[node].panel_numb, PANEL_PLOT_WHAT, ATTR_DIMMED, 0); 
							/// GetCtrlVal(tree[node].panel_numb, PANEL_PLOT_WHAT, &plot_what[node]);
							if((plot_what[node] & 0x03) == 1)
								SetCtrlAttribute (tree[node].panel_numb, PANEL_PLOT_WHAT, ATTR_FRAME_COLOR, 0x006699CC); 
							if((plot_what[node] & 0x03) == 2)
								SetCtrlAttribute (tree[node].panel_numb, PANEL_PLOT_WHAT, ATTR_FRAME_COLOR, 0x00CC9966);  
							if((plot_what[node] & 0x03) == 3)
								SetCtrlAttribute (tree[node].panel_numb, PANEL_PLOT_WHAT, ATTR_FRAME_COLOR, 0x0099CC66);   
						}
					
						/// Re_Title
/// mutigate		
						i = 1; j = 0;
						tmp_node = node;
						strcpy(title_buffer[node], "");
						while( i == 1)
						{
							if( (tree[node].node_type == 1) )
							{
								sprintf(gtree[j], "P%d ", tree[node].node_parent); 
									i = 0;
							}
						
							if( (tree[node].node_type == 2) )
							{
								sprintf(gtree[j], "G%d ", tree[node].node_gate_numb[tree[node].node_parent]); 
							}
						
							node = tree[node].node_parent; 
							j++;
						}
						node = tmp_node;       
						for(i = j - 1; i >= 0; i--)
						{
							strcat(title_buffer[node], gtree[i]);   	
						}
					
						if(tree[node].y_is_peak == -1)
						{
					    	if(tree[node].x_is_peak == 1)
								sprintf (tmpstr, " H Hist Chan %d", tree[node].x_chan_numb);
							else
								sprintf (tmpstr, " A Hist Chan %d", tree[node].x_chan_numb);
							strcat(title_buffer[node], tmpstr);
							i = 0;
						}
					
	//					SetPanelAttribute (tree[node].panel_numb, ATTR_TITLE, title_buffer[node]);   This has to be done within the thread. 
						draw_title[node] = 1;
					///	Plot_Hist(node);
					}
					tree[node].redraw = 1;
				}
			}
		}
	}
}

int CVICALLBACK Show_Stats (int ppanel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			GetCtrlVal(ppanel, PANEL_STATS, &show_stats[Node_from_Panel(ppanel)]);
			Refresh_Plots(Node_from_Panel(ppanel));
		break;
	}
	return 0;
}

int CVICALLBACK Plot_Selcector (int ps_panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		int pt, node;
		
		case EVENT_COMMIT:
			
			node = Node_from_Panel(ps_panel);
			GetCtrlVal(ps_panel, PANEL_PLOT_TYPE, &tree[node].plot_type); 
			
			if(tree[node].plot_type == 1)
			{
				if(tree[node].y_chan_numb != -1)
				{
					y_axis_tmp[node] = tree[node].y_chan_numb;
					y_is_peak_tmp[node] = tree[node].y_is_peak;
					y_log_tmp[node] =  tree[node].y_is_log;
					SetCtrlVal(ps_panel, PANEL_LOG_LIN_Y, VAL_LINEAR);
					SetCtrlVal(ps_panel, PANEL_Y_AXIS_CHAN, -1);
				}
			}
			
			if(tree[node].plot_type != 1)
			{
				if(tree[node].y_chan_numb == -1)
				{
					SetCtrlVal(ps_panel, PANEL_LOG_LIN_Y, y_log_tmp[node]);
					SetCtrlVal(ps_panel, PANEL_Y_AXIS_CHAN, y_axis_tmp[node]*4 + y_is_peak_tmp[node]);    
				}
			}
			Plot_Selcect (ps_panel);   /// reverse, move up, whatch menus
			Set_Axis_Chan_Funct (ps_panel, 0);
	///		Plot_Selcect (panel);   /// reverse, move up, whatch menus
			break;
	}
	return 0;
}

// plot selector
void Plot_Selcect (int p_s_panel)
{
	int hight, node, menubar, perr = 0;
	int h3d, l3d, t3d, w3d;
	double ch3d, cl3d, ct3d, cw3d;
	double oh3d, ol3d, ot3d, ow3d; 

	node = Node_from_Panel(p_s_panel);            
	
	// Adjust plot size for 3D graph and menus/buttons
	if(use_menus)
	{
		ch3d = 551.0; cl3d = 78.0; ct3d = 41.0; cw3d = 551.0;
		oh3d = 595.0; ol3d = 2.0; ot3d = 41.0; ow3d = 618.0;
	}
	else
	{
		ch3d = 551.0; cl3d = 68.0; ct3d = 41.0; cw3d = 551.0;
		oh3d = 582.0; ol3d = 2.0; ot3d = 41.0; ow3d = 618.0;
	}
	
	if((tree[node].plot_type == 6) && (resized_3d[node] == 0))
	{
		GetCtrlAttribute (p_s_panel, PANEL_SURFACE_GRAPH, ATTR_HEIGHT, &h3d);
		GetCtrlAttribute (p_s_panel, PANEL_SURFACE_GRAPH, ATTR_LEFT, &l3d);
		GetCtrlAttribute (p_s_panel, PANEL_SURFACE_GRAPH, ATTR_TOP, &t3d);
		GetCtrlAttribute (p_s_panel, PANEL_SURFACE_GRAPH, ATTR_WIDTH, &w3d);
		SetCtrlAttribute (p_s_panel, PANEL_SURFACE_GRAPH, ATTR_HEIGHT, (int)(oh3d*(double)h3d/ch3d) );
		SetCtrlAttribute (p_s_panel, PANEL_SURFACE_GRAPH, ATTR_LEFT, (int)(ol3d*(double)l3d/cl3d));
		SetCtrlAttribute (p_s_panel, PANEL_SURFACE_GRAPH, ATTR_TOP, (int)(ot3d*(double)t3d/ct3d));
		SetCtrlAttribute (p_s_panel, PANEL_SURFACE_GRAPH, ATTR_WIDTH, (int)(ow3d*(double)w3d/cw3d));
		resized_3d[node] = 1;
	}
	if((tree[node].plot_type != 6) && (resized_3d[node] == 1))
	{
		GetCtrlAttribute (p_s_panel, PANEL_SURFACE_GRAPH, ATTR_HEIGHT, &h3d);
		GetCtrlAttribute (p_s_panel, PANEL_SURFACE_GRAPH, ATTR_LEFT, &l3d);
		GetCtrlAttribute (p_s_panel, PANEL_SURFACE_GRAPH, ATTR_TOP, &t3d);
		GetCtrlAttribute (p_s_panel, PANEL_SURFACE_GRAPH, ATTR_WIDTH, &w3d);
		SetCtrlAttribute (p_s_panel, PANEL_SURFACE_GRAPH, ATTR_HEIGHT, (int)((ch3d+0.5)*(double)h3d/oh3d) );
		SetCtrlAttribute (p_s_panel, PANEL_SURFACE_GRAPH, ATTR_LEFT, (int)((cl3d+0.5)*(double)l3d/ol3d));
		SetCtrlAttribute (p_s_panel, PANEL_SURFACE_GRAPH, ATTR_TOP, (int)((ct3d+0.5)*(double)t3d/ot3d));
		SetCtrlAttribute (p_s_panel, PANEL_SURFACE_GRAPH, ATTR_WIDTH, (int)((cw3d+0.5)*(double)w3d/ow3d));
		resized_3d[node] = 0;   
	}
	
	// init CW display for 3d, heat and contour maps
	if((tree[node].plot_type > 3) && (tree[node].plot_type != 7))   
	{
		GetObjHandleFromActiveXCtrl(p_s_panel, PANEL_SURFACE_GRAPH, &gSurfaceGraph);  
		CW3DGraphLib__DCWGraph3DGetPlots (gSurfaceGraph, NULL, &plotsHandle);
		CW3DGraphLib_CWPlots3DItem (plotsHandle, NULL, CA_VariantInt(1), &plotHandle);
	}
	
	// 3d init
	if(tree[node].plot_type == 6)
	{
		SetCtrlAttribute (p_s_panel, PANEL_PCV_GRAPH, ATTR_VISIBLE, 0);
		
		CW3DGraphLib_CWPlot3DSetStyle (plotHandle, NULL, CW3DGraphLibConst_cwPoint);
		
		CW3DGraphLib__DCWGraph3DSetViewDistance (gSurfaceGraph, NULL, 0.9);
		CW3DGraphLib__DCWGraph3DSetViewLatitude (gSurfaceGraph, NULL, 55);
		CW3DGraphLib__DCWGraph3DSetViewLongitude (gSurfaceGraph, NULL,80);
		CW3DGraphLib__DCWGraph3DSetGridXY (gSurfaceGraph, NULL, VTRUE);
		CW3DGraphLib__DCWGraph3DSetGridXZ (gSurfaceGraph, NULL, VTRUE);
		CW3DGraphLib__DCWGraph3DSetGridYZ (gSurfaceGraph, NULL, VTRUE);
		
		perr += CW3DGraphLib__DCWGraph3DGetAxes(gSurfaceGraph, NULL, &axes);
		perr += CW3DGraphLib_CWAxes3DItem(axes, NULL, CA_VariantInt(1), &gAxis);
		CW3DGraphLib_CWAxis3DSetAutoScale (gAxis, NULL, VTRUE);
		perr += CW3DGraphLib_CWAxes3DItem(axes, NULL, CA_VariantInt(2), &gAxis);
		CW3DGraphLib_CWAxis3DSetAutoScale (gAxis, NULL, VTRUE);
		
		SetCtrlAttribute (tree[node].panel_numb, PANEL_XAXIS_LABEL, ATTR_VISIBLE, 0);
		SetCtrlAttribute (tree[node].panel_numb, PANEL_YAXIS_LABEL, ATTR_VISIBLE, 0);
	}
	else
		SetCtrlAttribute (p_s_panel, PANEL_PCV_GRAPH, ATTR_VISIBLE, 1);   
	
	// heat and contour map init
	if((tree[node].plot_type == 4) || (tree[node].plot_type == 5))
	{
		SetCtrlAttribute (tree[node].panel_numb, PANEL_XAXIS_LABEL, ATTR_VISIBLE, 1);
		SetCtrlAttribute (tree[node].panel_numb, PANEL_YAXIS_LABEL, ATTR_VISIBLE, 1);
   
		if(tree[node].plot_type == 4)
			 perr += CW3DGraphLib_CWPlot3DSetStyle (plotHandle, NULL, CW3DGraphLibConst_cwSurface);
        if(tree[node].plot_type == 5)
             perr += CW3DGraphLib_CWPlot3DSetStyle (plotHandle, NULL, CW3DGraphLibConst_cwContourLine);
		
		perr += CW3DGraphLib__DCWGraph3DSetViewDistance (gSurfaceGraph, NULL, 0.4);
		perr += CW3DGraphLib__DCWGraph3DSetViewLatitude (gSurfaceGraph, NULL, 0);
		perr += CW3DGraphLib__DCWGraph3DSetViewLongitude (gSurfaceGraph, NULL, 90);
		perr += CW3DGraphLib__DCWGraph3DSetGridXY (gSurfaceGraph, NULL, VFALSE);
		perr += CW3DGraphLib__DCWGraph3DSetGridXZ (gSurfaceGraph, NULL, VFALSE); 
		perr += CW3DGraphLib__DCWGraph3DSetGridYZ (gSurfaceGraph, NULL, VFALSE); 
		
		perr += CW3DGraphLib__DCWGraph3DGetAxes(gSurfaceGraph, NULL, &axes);
		perr += CW3DGraphLib_CWAxes3DItem(axes, NULL, CA_VariantInt(1), &gAxis);
		perr += CW3DGraphLib_CWAxis3DSetLog(gAxis, NULL, 0);
		CW3DGraphLib_CWAxis3DSetAutoScale (gAxis, NULL, VFALSE);
		//CW3DGraphLib_CWAxis3DSetMinimum (gAxis, NULL, CA_VariantInt(x_axis_scale_min_curr[node])); 
		//CW3DGraphLib_CWAxis3DSetMaximum (gAxis, NULL, CA_VariantInt(x_axis_scale_max_curr[node]));
		CW3DGraphLib_CWAxis3DSetMinimum (gAxis, NULL, CA_VariantInt(tree[node].x_is_log)); 
		CW3DGraphLib_CWAxis3DSetMaximum (gAxis, NULL, CA_VariantInt(525000));
		perr += CW3DGraphLib_CWAxes3DItem(axes, NULL, CA_VariantInt(2), &gAxis);
		perr += CW3DGraphLib_CWAxis3DSetLog(gAxis, NULL, 0);
		CW3DGraphLib_CWAxis3DSetAutoScale (gAxis, NULL, VFALSE);
		//CW3DGraphLib_CWAxis3DSetMinimum (gAxis, NULL, CA_VariantInt(y_axis_scale_min_curr[node]));
		//CW3DGraphLib_CWAxis3DSetMaximum (gAxis, NULL, CA_VariantInt(y_axis_scale_max_curr[node]));
		CW3DGraphLib_CWAxis3DSetMinimum (gAxis, NULL, CA_VariantInt(tree[node].y_is_log));
		CW3DGraphLib_CWAxis3DSetMaximum (gAxis, NULL, CA_VariantInt(525000));
		perr += CW3DGraphLib_CWAxes3DItem(axes, NULL, CA_VariantInt(3), &gAxis);
		perr += CW3DGraphLib_CWAxis3DSetLog(gAxis, NULL, 0);
		//printf(" %f %f \n", y_axis_scale_min_curr[node], y_axis_scale_max_curr[node]); 
	}
	
	// show/hide graphics layer 1 for plot type
	if((tree[node].plot_type >= 4) && (tree[node].plot_type != 7))	
		SetCtrlAttribute (p_s_panel, PANEL_PCV_GRAPH, ATTR_PLOT_BGCOLOR, VAL_TRANSPARENT);
	else
		SetCtrlAttribute (p_s_panel, PANEL_PCV_GRAPH, ATTR_PLOT_BGCOLOR, background_color);

	// if button window handle drawers and histogram stuff
	if(!use_menus)
	{ 	
		if( ((tree[node].plot_type <= 2) || (tree[node].plot_type == 7)) && (gamma_drawer_open[node] == 1) )
		{
			SetPanelAttribute (p_s_panel, ATTR_SCALE_CONTENTS_ON_RESIZE,  0);
			GetPanelAttribute (p_s_panel, ATTR_HEIGHT,  &hight);
			SetPanelAttribute (p_s_panel, ATTR_HEIGHT,  (int)((double)hight * (286.0 / 315.0)));
			gamma_drawer_open[node] = 0;
		}
		if( ((tree[node].plot_type >= 3) && (tree[node].plot_type != 7)) && (gamma_drawer_open[node] == 0) ) 
		{
			SetPanelAttribute (p_s_panel, ATTR_SCALE_CONTENTS_ON_RESIZE,  0);
			GetPanelAttribute (p_s_panel, ATTR_HEIGHT,  &hight);
			SetPanelAttribute (p_s_panel, ATTR_HEIGHT,  (int)((double)hight * (315.0 / 285.0)));
			gamma_drawer_open[node] = 1;   
		}
		SetPanelAttribute (p_s_panel, ATTR_SCALE_CONTENTS_ON_RESIZE,  1);
	
		if(tree[node].plot_type == 7)
		{
			SetCtrlVal(p_s_panel, PANEL_X_AXIS_CHAN, 32);	
			SetCtrlAttribute (p_s_panel, PANEL_X_AXIS_CHAN, ATTR_DIMMED, 1); 
		}
		else
		{
			SetCtrlVal(p_s_panel, PANEL_X_AXIS_CHAN, tree[node].x_chan_numb*4 + tree[node].x_is_peak);	
			SetCtrlAttribute (p_s_panel, PANEL_X_AXIS_CHAN, ATTR_DIMMED, 0);
		}
	
		if(tree[node].plot_type >= 6)
		{															    
			SetCtrlAttribute (p_s_panel, PANEL_MAKE_GATE, ATTR_DIMMED, 1); 
		}
		else if (done)
		{
			SetCtrlAttribute (p_s_panel, PANEL_MAKE_GATE, ATTR_DIMMED, 0);	
		}

		if(tree[node].plot_type == 7)
			SetCtrlAttribute (p_s_panel, PANEL_LOG_LIN_X, ATTR_DIMMED, 1);
		else
			SetCtrlAttribute (p_s_panel, PANEL_LOG_LIN_X, ATTR_DIMMED, 0);	
	}
	
	// if menu window handle menus and histogram stuff 
	if(use_menus)
	{
		menubar = GetPanelMenuBar(p_s_panel);
		if(tree[node].plot_type == 1)
		{
			SetMenuBarAttribute (menubar, PWMENUBAR_GATE_LASSO, ATTR_DIMMED, 1);
			SetMenuBarAttribute (menubar, PWMENUBAR_GATE_G_HIST, ATTR_DIMMED, 0);
		}
		if((tree[node].plot_type > 1) && (tree[node].plot_type != 7))
		{
			SetMenuBarAttribute (menubar, PWMENUBAR_GATE_G_HIST, ATTR_DIMMED, 1);
			if(tree[node].plot_type == 6)
				SetMenuBarAttribute (menubar, PWMENUBAR_GATE_LASSO, ATTR_DIMMED, 1);
			else
				SetMenuBarAttribute (menubar, PWMENUBAR_GATE_LASSO, ATTR_DIMMED, 0);
		}
	
		if(tree[node].plot_type == 7)
		{
			for(i = PWMENUBAR_XCHAN_FSC_X; i < PWMENUBAR_XCHAN_TIME_X; i++)
			{
				SetMenuBarAttribute (menubar ,i , ATTR_CHECKED, 0);
				SetMenuBarAttribute (menubar ,i , ATTR_DIMMED, 1);
			}
			SetMenuBarAttribute (menubar ,PWMENUBAR_XCHAN_TIME_X , ATTR_CHECKED, 1);
			for(i = PWMENUBAR_XCHAN_HEIGHT_X; i <= PWMENUBAR_XCHAN_WIDTH_X; i++)
			{
				SetMenuBarAttribute (menubar ,i , ATTR_CHECKED, 0);
				SetMenuBarAttribute (menubar ,i , ATTR_DIMMED, 1);  
			}
			SetMenuBarAttribute (menubar ,PWMENUBAR_XCHAN_LINEAR_X , ATTR_CHECKED, 1);  
			SetMenuBarAttribute (menubar ,PWMENUBAR_XCHAN_LINEAR_X , ATTR_DIMMED, 1);   
			SetMenuBarAttribute (menubar ,PWMENUBAR_XCHAN_LOG_X , ATTR_CHECKED, 0);  
			SetMenuBarAttribute (menubar ,PWMENUBAR_XCHAN_LOG_X , ATTR_DIMMED, 1);    
		}
		else
		{
			for(i = PWMENUBAR_XCHAN_FSC_X; i < PWMENUBAR_XCHAN_TIME_X; i++)
				SetMenuBarAttribute (menubar ,i , ATTR_DIMMED, 0);
			for(i = PWMENUBAR_XCHAN_HEIGHT_X; i <= PWMENUBAR_XCHAN_WIDTH_X; i++)
				SetMenuBarAttribute (menubar ,i , ATTR_DIMMED, 0);  
			SetMenuBarAttribute (menubar ,PWMENUBAR_XCHAN_LINEAR_X , ATTR_DIMMED, 0);   
			SetMenuBarAttribute (menubar ,PWMENUBAR_XCHAN_LOG_X , ATTR_DIMMED, 0);    
			Update_Menu_Bar_X(node);
		}
	
		if(tree[node].plot_type >= 6)
		{
			SetMenuBarAttribute (menubar, PWMENUBAR_GATE_LASSO, ATTR_DIMMED, 1);
			SetMenuBarAttribute (menubar, PWMENUBAR_GATE_G_HIST, ATTR_DIMMED, 1);
		}
		else if(done)
		{
			if(tree[node].plot_type >= 2)
				SetMenuBarAttribute (menubar, PWMENUBAR_GATE_LASSO, ATTR_DIMMED, 0);
			else
				SetMenuBarAttribute (menubar, PWMENUBAR_GATE_G_HIST, ATTR_DIMMED, 0);	
		}
	}	
	return;
}


	
int CVICALLBACK Plot_What (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			
			GetCtrlVal(panel, PANEL_PLOT_WHAT, &plot_what[Node_from_Panel(panel)]);
			Refresh_Plots(Node_from_Panel(panel)); 
			break;
	}
	return 0;
}

