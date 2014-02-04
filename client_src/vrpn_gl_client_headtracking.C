#include "vrpn_Tracker.h"
#include "vrpn_Configure.h"
#include "quat.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <stdio.h>

// This sample uses GLUT to render a grid on the XZ plane. The camera's pose and projection follow a Tracker.

// Global variables

vrpn_Tracker_Remote *tkr;
q_xyz_quat_type *tposquat;

// Helper functions

void compute_frustum(qogl_matrix_type frustum, double left, double right, double top, double bottom, double zNear, double zFar)
{
    if (frustum == NULL)
        return;

//    2 zNear
//  ------------       0              A              0
//  right - left
//
//                  2 zNear
//      0         ------------        B              0
//                top - bottom
//
//      0              0              C              D
//
//      0              0              -1             0
//
//                   A =  (right + left) / (right - left)
//                   B =  (top + bottom) / (top - bottom)
//                   C = -(zFar + zNear) / (zFar - zNear)
//                   D = -(2 zFar zNear) / (zFar - zNear)

    double A = (right + left) / (right - left);
    double B = (top + bottom) / (top - bottom);
    double C = -(zFar + zNear) / (zFar - zNear);
    double D = -(2.0 * zFar * zNear) / (zFar - zNear);
    double E = (2.0 * zNear) / (right - left);
    double F = (2 * zNear) / (top - bottom);

    double result[] = {
          E, 0.0,    A, 0.0,
        0.0,   F,    B, 0.0,
        0.0, 0.0,    C,   D,
        0.0, 0.0, -1.0, 0.0};

    memcpy(frustum, result, sizeof(result));
}

void compute_projection(qogl_matrix_type proj, const q_xyz_quat_type* camera_pose)
{
    if (camera_pose == NULL)
        return;

    //FIXME
    double left, right, top, bottom, zNear, zFar;
    compute_frustum(proj, left, right, top, bottom, zNear, zFar);
}

void update_perspective(const q_xyz_quat_type* camera_pose)
{
    if (camera_pose == NULL)
        return;

    // Compute the current projection matrix.

    qogl_matrix_type proj;
    compute_projection(proj, camera_pose);

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd(proj);

    // Compute the current view matrix.

    q_xyz_quat_type inv_camera_pose;
    q_xyz_quat_invert(&inv_camera_pose, camera_pose);

    qogl_matrix_type view;
    q_xyz_quat_to_ogl_matrix(view, &inv_camera_pose);

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd(view);
}

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

void on_idle()
{
    // Let the tracker do its thing.

    tkr->mainloop();
    
    glutPostRedisplay();
}

void on_display()
{
    // Render the scene.

    update_perspective(tposquat);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    draw_axes();

    glutSwapBuffers();
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
