#include "vrpn_Tracker.h"
#include "vrpn_Configure.h"
#include "quat.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <stdio.h>

const int LEFT_EYE = 0;
const int RIGHT_EYE = 1;

// This sample uses GLUT to render a grid on the XZ plane. The camera's pose and projection follow a Tracker.

// Global variables

double pupillary_distance = 0.06;
double z_near = 0.01;
double z_far = 100.0;
q_xyz_quat_type viewport_pose = { {0, 0, 0}, {0, 0, 0, 1} };
double viewport_width = 4.2188;
double viewport_height = 4.2188;

vrpn_Tracker_Remote *tkr;
q_xyz_quat_type *tposquat;

// Helper functions

void activate_target(int eye)
{
    //FIXME: Handle side-by-side, top-bottom, and quad-buffer stereo.
}

void compute_frustum(qogl_matrix_type frustum, double left, double right, double top, double bottom, double zNear, double zFar)
{
    if (frustum == NULL)
        return;

    double A = (right + left) / (right - left);
    double B = (top + bottom) / (top - bottom);
    double C = -(zFar + zNear) / (zFar - zNear);
    double D = -(2.0 * zFar * zNear) / (zFar - zNear);
    double E = (2.0 * zNear) / (right - left);
    double F = (2 * zNear) / (top - bottom);

    double result[] = {
        E, 0,  A, 0,
        0, F,  B, 0,
        0, 0,  C, D,
        0, 0, -1, 0};

    memcpy(frustum, result, sizeof(result));
}

void compute_projection(qogl_matrix_type proj, const q_vec_type eye_pos)
{
    //FIXME: Account for PD offset.
    double left = -0.5 * viewport_width;
    double right = 0.5 * viewport_width;
    double top = 0.5 * viewport_height;
    double bottom = -0.5 * viewport_height;
    compute_frustum(proj, left, right, top, bottom, z_near, z_far);
}

void update_perspective(int eye)
{
    if (eye != LEFT_EYE && eye != RIGHT_EYE)
        return;

    q_vec_type eye_pos;

    q_vec_type pd_offset = {(eye == LEFT_EYE) ? -pupillary_distance : pupillary_distance, 0.0, 0.0};
    q_xform(pd_offset, tposquat->quat, pd_offset);
    q_vec_add(eye_pos, tposquat->xyz, pd_offset);

    // Compute the current projection matrix.

    qogl_matrix_type proj;
    compute_projection(proj, eye_pos);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //FIXME: glLoadMatrixd(proj);    
    gluPerspective(45.0, 1.0, 0.01, 10.0);

    // Compute the current view matrix.

    q_vec_type up = {0, 1, 0};
    q_type viewport_inv_rot;
    q_invert(viewport_inv_rot, viewport_pose.quat);
    q_xform(up, viewport_inv_rot, up);
    q_xform(up, tposquat->quat, up);

    q_vec_type look_at;
    q_vec_subtract(look_at, viewport_pose.xyz, tposquat->xyz);
    q_vec_add(look_at, eye_pos, look_at);

    //FIXME
    //glMatrixMode(GL_MODELVIEW);
    //gluLookAt(eye_pos[Q_X], eye_pos[Q_Y], eye_pos[Q_Z],
    //    look_at[Q_X], look_at[Q_Y], look_at[Q_Z],
    //    up[Q_X], up[Q_Y], up[Q_Z]);
    gluLookAt( eye_pos[Q_X], eye_pos[Q_Y] + 0.1, eye_pos[Q_Z] - 5.0, // eye position
               0.0,  0.0,  0.0,     // look at this point
               0.0,  1.0,  0.0);    // up direction
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
    // Draw from the left eye's perspective.

    activate_target(LEFT_EYE);

    update_perspective(LEFT_EYE);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    draw_axes();

    // Draw from the right eye's perspective.

    //activate_target(RIGHT_EYE);

    //update_perspective(RIGHT_EYE);

    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //draw_axes();

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

  glutInitWindowSize(600, 600);
  glutInit(&argc, argv) ;
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH) ;
  glutCreateWindow("VRPN GL Client Example");
  glutIdleFunc(on_idle);
  glutDisplayFunc(on_display);

  init_graphics();

  /* Enter the GLUT main loop */
  glutMainLoop() ;
}
