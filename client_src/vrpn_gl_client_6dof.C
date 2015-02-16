#include "vrpn_Tracker.h"
#include "vrpn_Configure.h"
#include "quat.h"

#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#include <stdio.h>

// This sample uses GLUT to render a Tracker's pose as a pair of 3D spheres along with a grid on the XZ plane.

// Global variables

vrpn_Tracker_Remote *tkr;
q_xyz_quat_type *tposquat;
GLUquadricObj *qobj1;
GLUquadricObj *qobj2;

// Helper functions

void VRPN_CALLBACK handle_tracker(void *userdata, const vrpn_TRACKERCB t)
{
  q_xyz_quat_type *pq = (q_xyz_quat_type *) userdata;

  pq->xyz[Q_X] = t.pos[Q_X];
  pq->xyz[Q_Y] = t.pos[Q_Y];
  pq->xyz[Q_Z] = t.pos[Q_Z];
  pq->quat[Q_X]= t.quat[Q_X];
  pq->quat[Q_Y] = t.quat[Q_Y];
  pq->quat[Q_Z] = t.quat[Q_Z];
  pq->quat[Q_W] = t.quat[Q_W];
}

void init_graphics()
{
  // Set up OpenGL.

  glShadeModel(GL_SMOOTH);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0, 1.0, 0.01, 10.0);
  gluLookAt( 0.0,  0.1, -5.0,     // eye pos
             0.0,  0.0,  0.0,     // look at this point
             0.0,  1.0,  0.0);    // up direction 

  glClearColor(0.0, 0.0, 0.0, 0.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glEnable(GL_DEPTH_TEST);

  // Set up some lighting and materials.

  glEnable(GL_LIGHTING);
  glEnable(GL_LEQUAL);

  GLfloat lpos[] = { 0.0,  0.0, -100.0,  1.0};
  GLfloat lamb[] = { 0.05, 0.05,  0.05, 1.0};
  GLfloat ldif[] = { 0.5,  0.5,   0.5,  1.0};
  GLfloat lspc[] = { 0.5,  0.5,   0.5,  1.0};

  glLightfv(GL_LIGHT0, GL_POSITION, lpos);
  glLightfv(GL_LIGHT0, GL_AMBIENT,  lamb);
  glLightfv(GL_LIGHT0, GL_DIFFUSE,  ldif);
  glLightfv(GL_LIGHT0, GL_SPECULAR, lspc);
  glLightf(GL_LIGHT0, GL_SHININESS, 400.0);
  glEnable(GL_LIGHT0);

  GLfloat mdif[] = {0.9,  0.0,  0.2, 1.0};
  GLfloat mspc[] = {0.2, 0.2, 0.2, 1.0};
  glMaterialfv(GL_FRONT, GL_SPECULAR, mspc);
  glMaterialfv(GL_FRONT, GL_DIFFUSE,  mdif);
  glEnable(GL_COLOR_MATERIAL);
}

void draw_axes()
{
     // Draw Coordinate Axes

     glDisable(GL_LIGHTING);

     glBegin(GL_LINES);

     glColor3f(0.0, 1.0, 0.0);

     for (float i=-10; i<10; i += 0.50) {
       glVertex3f( -10.0,     0.0,        i);
       glVertex3f(  10.0,     0.0,        i);
       glVertex3f(     i,     0.0,    -10.0);
       glVertex3f(     i,     0.0,     10.0);
     }

     glEnd();
} 

void draw_tracker(q_xyz_quat_type* pq)
{
  // Convert from Tracker coordinates to our scene coordinates.

  float x = -10.0*(pq->xyz[Q_X]);
  float y =  10.0*(pq->xyz[Q_Y]);
  float z = -10.0*(pq->xyz[Q_Z]);
  double q[4] = {
      -pq->quat[Q_X],
       pq->quat[Q_Y],
      -pq->quat[Q_Z],
       pq->quat[Q_W]
  };

  // Convert the orientation from quaternion to OpenGL matrix.

  double rotation[16];
  q_to_ogl_matrix(rotation,q);

  // First draw line to object.

  glColor3f(1.0, 1.0, 1.0);
  glBegin(GL_LINES);
  glVertex3f(0.0,    0.0,    0.0   );
  glVertex3f(x, y, z); 
  glEnd();

  // Draw the tracker itself as two spheres.

  glEnable(GL_LIGHTING);
  glColor3f(1.0, 0.0, 0.2);
  glPushMatrix();
    glTranslatef(x, y, z);
    glMultMatrixd(rotation);
    glTranslatef(-1.5,  0.0,  0.0);
    gluSphere(qobj1, 0.2, 32, 32);
    glTranslatef(3.0,0.0,0.0);
    gluSphere(qobj2, 0.2, 32, 32);
  glPopMatrix();
}

void on_display()
{
    // Render the scene.

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    draw_axes();
    draw_tracker(tposquat);

    glutSwapBuffers();
}

void on_idle()
{
    // Let the tracker do its thing.

    tkr->mainloop();
    glutPostRedisplay();
}

// Main entry point

int main(int argc, char **argv)
{
  // Parse command line.

  if (argc < 2) {
    printf("%s: Invalid parms\n", argv[0]);
    printf("usage: \n");
    printf("  %s Tracker0@host\n", argv[0]);

    return -1;
  }

  char * server = argv[1];
  printf("Opening: %s ...\n",  server);

  // Initialize global variables.

  tkr = new vrpn_Tracker_Remote(server);

  tposquat = new q_xyz_quat_type;
  tkr->register_change_handler(tposquat, handle_tracker);

  qobj1 = gluNewQuadric();
  qobj2 = gluNewQuadric();  

  // Initialize GLUT and create window.

  glutInitWindowSize(600, 600) ;
  glutInit(&argc, argv) ;
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH) ;
  glutCreateWindow("VRPN GL Client Example");
  glutIdleFunc(on_idle);
  glutDisplayFunc(on_display);

  init_graphics();

  /* Enter the GLUT main loop */
  glutMainLoop() ;
}
