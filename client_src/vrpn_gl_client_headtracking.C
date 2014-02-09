#include "vrpn_Tracker.h"
#include "vrpn_Configure.h"
#include "quat.h"

#include <GL/gl.h>
#include <GL/glut.h>
#include <math.h>
#include <stdio.h>

typedef enum {
    CENTER_EYE = -1,
    LEFT_EYE = 0,
    RIGHT_EYE = 1
} eye_t;

typedef enum {
    NO_STEREO = -1,
    QUAD_BUFFER = 0,
    LEFT_RIGHT = 1,
    TOP_BOTTOM = 2
    //TODO: Support interlaced, etc.
} stereo_mode_t;

// This sample uses GLUT to render a grid on the XZ plane. The camera's pose and projection follow a Tracker.

// Global variables

stereo_mode_t stereo_mode = NO_STEREO;
double pupillary_distance = 0.06;
double z_near = 0.01;
double z_far = 100.0;
q_xyz_quat_type viewport_pose = { {0, 0, 0}, {0, 0, 0, 1} };
int buffer_width = 800;
int buffer_height = 600;
double dpi = 1000;
double viewport_width = buffer_width / dpi;
double viewport_height = buffer_height / dpi;

vrpn_Tracker_Remote* tracker;
q_xyz_quat_type sensor_pose = { {0, 0.1, 5}, {0, 0, 0, 1} };
q_xyz_quat_type tracker_pose = { {0, 0, 0}, {0, 0, 0, 1} };

// Helper functions

void activate_target(eye_t eye)
{
    // Select the right buffer and set the viewport position and size.

    if (stereo_mode == QUAD_BUFFER)
    {
        int buffer_id = (eye == RIGHT_EYE) ? GL_BACK_RIGHT : GL_BACK_LEFT;
        glDrawBuffer(buffer_id);

        glViewport(0, 0, buffer_width, buffer_height);        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }
    else if (stereo_mode == LEFT_RIGHT)
    {
        int x = (eye == RIGHT_EYE) ? 0.5 * buffer_width : 0;
        glViewport(x, 0, 0.5 * buffer_width, buffer_height);

        if (eye == LEFT_EYE)
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }
    else if (stereo_mode == TOP_BOTTOM)
    {
        int y = (eye == LEFT_EYE) ? 0.5 * buffer_height : 0;
        glViewport(0, y, buffer_width, 0.5 * buffer_height);

        if (eye == LEFT_EYE)
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }
    else
    {
        glViewport(0, 0, buffer_width, buffer_height);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }
}

void compute_frustum(qogl_matrix_type frustum, double left, double right, double top, double bottom, double zNear, double zFar)
{
    if (frustum == NULL)
        return;

    double x_2n = zNear + zNear;
    double x_2nf = 2 * zNear * zFar;

    double p_fn = zFar + zNear;
    double m_nf = zNear - zFar;

    double p_rl = right + left;
    double m_rl = right - left;
    double p_tb = top + bottom;
    double m_tb = top - bottom;

    double result[] = {
        x_2n/m_rl, 0,         0,          0,
        0,         x_2n/m_tb, 0,          0,
        p_rl/m_rl, p_tb/m_tb, p_fn/m_nf,  -1,
        0,         0,         x_2nf/m_nf, 0};

    memcpy(frustum, result, sizeof(result));
}

void compute_perspective(qogl_matrix_type proj, double fovY, double aspect, double zNear, double zFar)
{
    double fH = tan( fovY / 360 * Q_PI ) * zNear;
    double fW = fH * aspect;
    compute_frustum(proj, -fW, fW, -fH, fH, zNear, zFar);
}

void compute_look_at(qogl_matrix_type view, const q_vec_type eye_pos,
                     const q_vec_type look_at_pos, const q_vec_type up_dir)
{
   q_vec_type forward;
   q_vec_subtract(forward, look_at_pos, eye_pos);
   q_vec_normalize(forward, forward);

   // Right = forward x up
   q_vec_type right;
   q_vec_cross_product(right, forward, up_dir);
   q_vec_normalize(right, right);

   // Recompute up as: up = side x forward so they're all orthogonal.
   q_vec_type up;
   q_vec_cross_product(up, right, forward);
   
   qogl_matrix_type result = {
       right[0],   up[0],      -forward[0], 0,
       right[1],   up[1],      -forward[1], 0,
       right[2],   up[2],      -forward[2], 0,
       eye_pos[0], eye_pos[1], eye_pos[2],  1
   };

   for (int i = 0; i < 16; ++i)
       view[i] = result[i];
}

void compute_projection(qogl_matrix_type proj, const q_vec_type eye_pos)
{
    // Find the display in camera space. Note its rotation matches the camera's.

    q_vec_type local_display_pos;
    q_vec_subtract(local_display_pos, viewport_pose.xyz, eye_pos);
    q_type inv_viewport_rot;
    q_invert(inv_viewport_rot, viewport_pose.quat);
    q_xform(local_display_pos, inv_viewport_rot, local_display_pos);

    double near_plane = z_near / local_display_pos[Q_Z];

    // Find the frustum bounds in camera space.

    //TODO: Adjust for side-by-side and top-bottom stereo?
    double left    = near_plane * (local_display_pos[Q_X] - 0.5 * viewport_width);
    double right   = near_plane * (local_display_pos[Q_X] + 0.5 * viewport_width);
    double bottom  = near_plane * (local_display_pos[Q_Y] - 0.5 * viewport_height);
    double top     = near_plane * (local_display_pos[Q_Y] + 0.5 * viewport_height);

    compute_frustum(proj, left, right, top, bottom, z_near, z_far);
}

void compute_view(qogl_matrix_type view, const q_vec_type eye_pos)
{
    // Make the camera match the display's rotation.

    q_vec_type up = {0, 1, 0};
    q_xform(up, viewport_pose.quat, up);

    q_vec_type look_at = {0, 0, -1};
    q_xform(look_at, viewport_pose.quat, look_at);
    q_vec_add(look_at, eye_pos, look_at);

    compute_look_at(view, eye_pos, look_at, up);
}

void update_perspective(eye_t eye)
{
    // Find the world-space position of the eye we're rendering from.

    double pd_offset = (eye == CENTER_EYE) ? 0.0
                                           : (eye == LEFT_EYE) ? -0.5 * pupillary_distance
                                                               : 0.5 * pupillary_distance;

    q_vec_type eye_pos = {pd_offset, 0.0, 0.0};
    q_xform(eye_pos, tracker_pose.quat, eye_pos);
    q_vec_add(eye_pos, tracker_pose.xyz, eye_pos);

    // Compute the current projection matrix.

    glMatrixMode(GL_PROJECTION);

    qogl_matrix_type proj;
    compute_projection(proj, eye_pos);
    glLoadMatrixd(proj);

    // Compute the current view matrix.

    glMatrixMode(GL_MODELVIEW);

    qogl_matrix_type view;
    compute_view(view, eye_pos);
    glLoadMatrixd(view);
}

void VRPN_CALLBACK handle_tracker(void *userdata, const vrpn_TRACKERCB t)
{
  q_xyz_quat_type* pq = (q_xyz_quat_type*) userdata;

  // Transform the local tracker pose by the sensor pose to get a world pose.

  q_vec_type pos;
  q_xform(pos, sensor_pose.quat, t.pos);
  q_vec_add(pq->xyz, sensor_pose.quat, pos);

  q_mult(pq->quat, sensor_pose.quat, t.quat);
}

void init_graphics()
{
  // Set up OpenGL.

  glShadeModel(GL_SMOOTH);
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

void render_eye(eye_t eye)
{
    activate_target(eye);

    update_perspective(eye);

    draw_axes();
}

void on_idle()
{
    // Let the tracker do its thing.

    tracker->mainloop();
    
    glutPostRedisplay();
}

void on_display()
{
    // Draw from the left eye's perspective.

    if (stereo_mode == NO_STEREO) {
        render_eye(CENTER_EYE);
    } else {
        render_eye(LEFT_EYE);
        render_eye(RIGHT_EYE);
    }

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

  tracker = new vrpn_Tracker_Remote(server);

  tracker->register_change_handler(&tracker_pose, handle_tracker);

  // Initialize GLUT and create window.

  glutInitWindowSize(buffer_width, buffer_height);
  glutInit(&argc, argv);
  int display_mode = GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH;
  if (stereo_mode == QUAD_BUFFER)
      display_mode |= GLUT_STEREO;
  glutInitDisplayMode(display_mode) ;
  glutCreateWindow("VRPN GL Client Example");
  glutIdleFunc(on_idle);
  glutDisplayFunc(on_display);

  init_graphics();

  /* Enter the GLUT main loop */
  glutMainLoop() ;
}
