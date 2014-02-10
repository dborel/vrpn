#include "vrpn_Tracker.h"
#include "vrpn_Configure.h"
#include "quat.h"

#include <GL/gl.h>
#include <GL/glut.h>
#include <math.h>
#include <stdio.h>

// This sample uses GLUT to render a grid on the XZ plane. The camera's pose and projection follow a Tracker.
// All pose data is in OpenGL's "world space": after the model transform, but before the viewing transform.

/// Specifies an eye in the stereo camera rig.
typedef enum {
    /// Halfway between the left and right eyes. Used for monoscopic rendering.
    CENTER_EYE = -1,

    /// The left eye. -0.5 * pupillary_distance along the x axis, looking in the -z direction.
    LEFT_EYE = 0,

    /// The right eye. 0.5 * pupillary_distance along the x axis, looking in the -z direction.
    RIGHT_EYE = 1
} eye_t;

/// Specifies a method for displaying stereo images. Different graphics accept stereo via different methods.
typedef enum {
    /// Monoscopic or "one-eyed" rendering.
    NO_STEREO = -1,

    /// Each eye is rendered at full resolution, to its own back-buffer.
    QUAD_BUFFER = 0,

    /// Both eyes are rendered to the same back-buffer at half-resolution. The left eye gets the left half of the buffer.
    LEFT_RIGHT = 1,

    /// Both eyes are rendered to the same back-buffer at half-resolution. The left eye gets the top half of the buffer.
    TOP_BOTTOM = 2

    //TODO: Support interlaced, NVIDIA 3D Vision, HD3D, etc.
} stereo_mode_t;

// Global variables

/// Controls the stereo frame encoding method. Choose a method appropriate for your GPU and monitor.
stereo_mode_t stereo_mode = NO_STEREO;

/// The distance (in meters) between the user's eyes.
double pupillary_distance = 0.06;

/// The distance from either of the user's eyes to the near-clipping plane.
double z_near = 0.01;

/// The distance from either of the user's eyes to the far-clipping plane.
double z_far = 100.0;

/// The display's position and rotation in world space.
q_xyz_quat_type viewport_pose = { {0, 0, 0}, {0, 0, 0, 1} };

/// The horizontal resolution of the 3D viewport.
int buffer_width = 800;

/// The vertical resolution of the 3D viewport.
int buffer_height = 600;

/// The number of screen pixels per meter. Determines the virtual size of the viewport.
double pixel_density = 1000;

/// The 3D viewport's horizontal size in meters.
double viewport_width = buffer_width / pixel_density;

/// The 3D viewport's vertical size in meters.
double viewport_height = buffer_height / pixel_density;

/// Tracks the user's head.
vrpn_Tracker_Remote* tracker;

/// The position and rotation of the sensor tracking the user's head in world space.
q_xyz_quat_type sensor_pose = { {0, 0.1, 5}, {0, 0, 0, 1} };

/// The current position and rotation of the user's head in world space.
q_xyz_quat_type tracker_pose = { {0, 0, 0}, {0, 0, 0, 1} };

// Helper functions

/// Prepares the system to render a frame for the given eye.
/// Performs any necessary buffer activation and clearing.
void begin_frame(eye_t eye)
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

/// Presents a frame that has been rendered for the given eye.
void end_frame(eye_t eye)
{
    if (eye != LEFT_EYE || stereo_mode == QUAD_BUFFER)
        glutSwapBuffers();
}

/// Computes a projection matrix for the given perspective projection.
/// Supports off-axis (asymmetric) frusta such as for stereo or head-tracked rendering.
/// Generates the same matrix you would get from calling glFrustum(...).
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

/// Computes a view matrix for the given camera position and rotation.
/// Generates the same matrix you would get from calling gluLookAt(...).
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

/// Computes a physically-correct projection matrix for the given eye position and the current viewport pose.
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

/// Computes a viewing matrix (inverse camera pose) for the given eye position and the current viewport pose.
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

/// Applies correct viewing and projection matrices for the given eye position and the current viewport pose.
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

/// A callback that computes the current head pose when new Tracker data arrives.
void VRPN_CALLBACK handle_tracker(void *userdata, const vrpn_TRACKERCB t)
{
  q_xyz_quat_type* pq = (q_xyz_quat_type*) userdata;

  // Transform the local tracker pose by the sensor pose to get a world pose.

  q_vec_type pos;
  q_xform(pos, sensor_pose.quat, t.pos);
  q_vec_add(pq->xyz, sensor_pose.quat, pos);

  q_mult(pq->quat, sensor_pose.quat, t.quat);
}

/// Configures OpenGL for rendering.
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

/// Renders all 3D geometry in the scene.
void draw_scene()
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

/// Sets up and renders a perspective-correct image for the given eye.
void render_eye(eye_t eye)
{
    begin_frame(eye);

    update_perspective(eye);

    draw_scene();

    end_frame(eye);
}

/// A callback that updates the left and right (or center) images when GLUT requests a screen redraw.
void on_display()
{
    // Draw from the left eye's perspective.

    if (stereo_mode == NO_STEREO) {
        render_eye(CENTER_EYE);
    } else {
        render_eye(LEFT_EYE);
        render_eye(RIGHT_EYE);
    }
}

/// A callback that checks for new Tracker data and initiates a screen redraw when GLUT is idle.
void on_idle()
{
    // Let the tracker do its thing.

    tracker->mainloop();

    glutPostRedisplay();
}

/// The main entry point.
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
