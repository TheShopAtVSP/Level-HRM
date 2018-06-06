//----------------------------------------------------------------------------
// Gravity Vector OGL.c
//
//  initialization and drawing routines.  
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// Includes
//----------------------------------------------------------------------------
#include <windows.h>
#include <GL\gl.h>
#include <GL\glu.h>
#include <ansi_c.h>
#include <cvirte.h>
#include <userint.h>
#include "cviogl.h"
#include "Activity Discriminator vars.h" 
#include "Activity Discriminator.h"

//----------------------------------------------------------------------------
// Define default values
//----------------------------------------------------------------------------
#define DFLT_VIEW_LATITUDE      75.0
#define DFLT_VIEW_LONGITUDE     45.0
#define DFLT_VIEWPOINT_X        0.0
#define DFLT_VIEWPOINT_Y        0.0
#define DFLT_VIEWPOINT_Z        0.0
#define DFLT_VIEW_DISTANCE      8.5
#define DFLT_LIGHT_LATITUDE     30.0
#define TRUE                    1
#define FALSE                   0

//----------------------------------------------------------------------------
// Variables
//----------------------------------------------------------------------------

// Define vars used for view
double viewLongitude =          DFLT_VIEW_LONGITUDE;
double viewLatitude =           DFLT_VIEW_LATITUDE;
double viewDistance =           DFLT_VIEW_DISTANCE;
double viewPointX =             DFLT_VIEWPOINT_X;
double viewPointY =             DFLT_VIEWPOINT_Y;
double viewPointZ =             DFLT_VIEWPOINT_Z;

GLUquadricObj   *object;

//----------------------------------------------------------------------------
//  InitOGLControl():  Initializes the OGL control and sets the rendering
//  properties appropriate to the image

void InitOGLControl(void)
{
    // Setup lighting for system
    OGLSetCtrlAttribute(mainpnl,OGLControlPanel,OGLATTR_LIGHTING_ENABLE, 1);
    OGLSetCtrlAttribute(mainpnl,OGLControlPanel,OGLATTR_LIGHT_SELECT,    1);
    OGLSetCtrlAttribute(mainpnl,OGLControlPanel,OGLATTR_LIGHT_ENABLE,    1);
    OGLSetCtrlAttribute(mainpnl,OGLControlPanel,OGLATTR_LIGHT_DISTANCE,  2.0);
    OGLSetCtrlAttribute (mainpnl, OGLControlPanel,OGLATTR_LIGHT_LATITUDE, DFLT_LIGHT_LATITUDE);

    // Setup viewing position for system
    OGLSetCtrlAttribute(mainpnl,OGLControlPanel,OGLATTR_PROJECTION_TYPE, OGLVAL_PERSPECTIVE);
    OGLSetCtrlAttribute (mainpnl, OGLControlPanel,OGLATTR_VIEW_DIRECTION, OGLVAL_USER_DEFINED);
    OGLSetCtrlAttribute (mainpnl, OGLControlPanel,OGLATTR_VIEW_LATITUDE, DFLT_VIEW_LATITUDE);
    OGLSetCtrlAttribute (mainpnl, OGLControlPanel,OGLATTR_VIEW_LONGITUDE, DFLT_VIEW_LONGITUDE);
    OGLSetCtrlAttribute (mainpnl, OGLControlPanel, OGLATTR_VIEW_CENTERX,DFLT_VIEWPOINT_X);
    OGLSetCtrlAttribute (mainpnl, OGLControlPanel, OGLATTR_VIEW_CENTERY,DFLT_VIEWPOINT_Y);
    OGLSetCtrlAttribute (mainpnl, OGLControlPanel, OGLATTR_VIEW_CENTERZ,DFLT_VIEWPOINT_Z);
    OGLSetCtrlAttribute(mainpnl,OGLControlPanel,OGLATTR_VIEW_DISTANCE,DFLT_VIEW_DISTANCE);

    // Disable 3D plotting feature of the OGL instrument driver; use only lighting properties
    // and coordinate system
    OGLSetCtrlAttribute (mainpnl,OGLControlPanel, OGLATTR_PLOTTING_ENABLE, 0);
}

//----------------------------------------------------------------------------
//  RenderArmImage():  Renders the arm image to the OGL control.
//----------------------------------------------------------------------------
void RenderGVImage(int fastFlag)
{

    GLfloat specularLight0[]    ={1.0f, 1.0f, 1.0f, 1.0f};
	GLfloat specularLight1[]    ={1.0f, 0.0f, 0.0f, 1.0f};
	GLfloat specularLight2[]    ={1.0f, 1.0f, 0.0f, 1.0f};
	GLfloat emissionLight0[]    ={0.1f, 0.1f, 0.1f, 0.1f};
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();

            glEnable(GL_DEPTH_TEST);
            glShadeModel(GL_SMOOTH);

            glEnable(GL_COLOR_MATERIAL);
            glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specularLight0);
			glMaterialfv(GL_FRONT_LEFT, GL_SPECULAR, specularLight2);
			glMaterialfv(GL_FRONT_RIGHT, GL_SPECULAR, specularLight1);
			glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emissionLight0);
            glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 128); 
			glMateriali(GL_FRONT_AND_BACK, GL_EMISSION, 128); 

            DrawGVImage(fastFlag);

        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
    glFlush();
}

//----------------------------------------------------------------------------
//  DrawArmImage():  Draws the OGL robotic arm image in its entirety
//----------------------------------------------------------------------------
void DrawGVImage(int fastFlag)
{

    // Create a new quadric object; we'll refer to this structure when using
    // the GLU routines to draw higher-level primitives
    object = gluNewQuadric();
    // Draw in "Line" mode for speed
    if (fastFlag)
        gluQuadricDrawStyle(object,GLU_LINE);

    // Draw the Axis pointer piece by piece
    glPushMatrix();
		glColor3f(1.0f, 1.0f, 1.0f);
		gluSphere(object, 0.3, 30, 30);
		
	///	glRotatef(theta_x,0.0,1.0,0.0);   //x 
		glRotatef(theta_y,0.0,1.0,0.0);  //z
		
		glColor3f(0.1f, 0.1f, 1.0f); //b
		glRotatef(0.0,0.0,0.0,1.0);  //z+
		gluCylinder(object, 0.1, 0.1, 1.0, 10, 10);
		glTranslatef(0.0, 0.0, 1.0);
		gluCylinder(object, 0.2, 0.01, 0.3, 10, 10);
		glTranslatef(0.0, 0.0, -1.0);
		
		glColor3f(0.1f, 1.0f, 0.1f); //g
		glRotatef(90.0,1.0,0.0,0.0);   //x
		gluCylinder(object, 0.1, 0.1, 1.0, 10, 10);
		glTranslatef(0.0, 0.0, 1.0);
		gluCylinder(object, 0.2, 0.01, 0.3, 10, 10);
		glTranslatef(0.0, 0.0, -1.0);
		
		glColor3f(1.0f, 0.1f, 0.1f); //r
		glRotatef(-90.0,0.0,1.0,0.0);//y
		gluCylinder(object, 0.1, 0.1, 1.0, 10, 10);
		glTranslatef(0.0, 0.0, 1.0);
		gluCylinder(object, 0.2, 0.01, 0.3, 10, 10);
		glTranslatef(0.0, 0.0, -1.0);
	glPopMatrix();
	
	glPushMatrix();
		glColor3f(1.0f, 1.0f, 0.1f); //y
		glRotatef(180.0,0.0,0.0,0.0);//z-
		gluCylinder(object, 0.1, 0.1, 1.0, 10, 10);
		glTranslatef(0.0, 0.0, 1.0);
		gluCylinder(object, 0.2, 0.01, 0.3, 10, 10);
	glPopMatrix();

    gluDeleteQuadric(object);
	
    return;
}

//----------------------------------------------------------------------------
//  OGLCallback():  Required by CVI for image refreshes and paints
//----------------------------------------------------------------------------
int CVICALLBACK OGLCallback (int panel, int control, int event,
        void *callbackData, int eD1, int eventData2)
{
	switch (event) {
        case OGLEVENT_REFRESH:

            // Render the arm image when REFRESH event is received
            RenderGVImage(eD1);
            break;
    }
    return 0;
}

