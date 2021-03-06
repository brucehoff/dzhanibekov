/*

Adopted from:

http://homepages.paradise.n­et.nz/tmcgavin/box.c

*/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include <GL/freeglut_std.h>
#include <rksuite.h>

#include <unistd.h>

#ifdef __GCC__
#define NORETURN __attribute__ ((noreturn))
#else
#define NORETURN
#endif

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

#ifndef M_PI
#define M_PI 3.1415926535897932384626
#endif

/** replaces 'nag' library **/
static RKSUITE rksuite;

static NORETURN void error (char *msg, ...)
{
  va_list argptr;

  va_start (argptr, msg);
  vfprintf (stderr, msg, argptr);
  va_end (argptr);
  fprintf (stderr, "\n");
  exit (EXIT_FAILURE);
}

/**********************************************************************/
#define X 0
#define Y 1
#define Z 2

typedef GLdouble vec2[2];
typedef GLdouble vec3[3];
typedef GLdouble vec6[6];

#define NEQ 6

/**********************************************************************/
static int winWidth, winHeight;
static GLdouble zNear = 1.0, zFar = 10.0, scale = 0.6;
static GLuint displayListBase;
static vec3 box, I;
static vec6 y;
static GLdouble mass, t, work[NEQ*32];
static struct timeval tv0;

/**********************************************************************/
#ifdef SAVEFILES

static void save_snapshot (char *filename, int width, int height)
{
  int iy;
  unsigned char *pixel_data;
  FILE *f;
  char command[255];

  if ((pixel_data = (unsigned char*)malloc(3 * width * height)) == NULL)
    error ("Out of memory in save_snapshot");
  glReadPixels (0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixel_data);
  sprintf (command, "pnmtopng -transparent=rgb:ff/ff/ff -force > %s", filename);
  if ((f = popen (command, "w")) == NULL) {
    perror ("popen");
    error ("popen(\"%s\", \"w\") failed in save_snapshot()", filename);
  }
  fprintf (f, "P6 %d %d 255\n", width, height);
  for (iy = height - 1; iy >= 0; --iy)
    fwrite (&pixel_data[3 * width * iy], 3, width, f);
  pclose (f);
  free (pixel_data);
}

#endif

/**********************************************************************/
static void display (void)
{
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glDisable (GL_LIGHTING);
  glEnable (GL_DEPTH_TEST);
      glEnable(GL_TEXTURE_2D);
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity ();
  glTranslated (0.0, 0.0, -3.0);

  /* draw axis of rotation */
  glBegin (GL_LINES);
  glColor3d (0.0, 0.0, 0.0);
  glVertex3d (10.0 * y[0], 10.0 * y[1], 10.0 * y[2]);
  glVertex3d (-10.0 * y[0], -10.0 * y[1], -10.0 * y[2]);
  glEnd();

  glRotated (y[3] * (180.0 / M_PI), 0.0, 0.0, 1.0); /* phi around Z */
  glRotated (y[4] * (180.0 / M_PI), 1.0, 0.0, 0.0); /* theta around X */
  glRotated (y[5] * (180.0 / M_PI), 0.0, 0.0, 1.0); /* psi around Z */

  glScaled (scale, scale, scale);
  glCallList (displayListBase + 1); /* draw wire frame and axes */

  glScaled (box[X], box[Y], box[Z]);
  glCallList (displayListBase + 0); /* draw box */

  glutSwapBuffers();

#ifdef SAVEFILES
  if (t > 2300.0 && t < 2500.0) {
    char filename[255];
    sprintf (filename, "/frames/x_%f.png", t);
    save_snapshot (filename, winWidth, winHeight);
  }
#endif

}

/**********************************************************************/
static void reshape (int width, int height)
{
  winWidth  = width;
  winHeight = height;

  glViewport (0, 0, width, height);
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();
  /* eye is at (0.0, 0.0, 0.0)
     look at radius 1.0 sphere centred at (0.0, 0.0, -3.0) */
  gluPerspective ((180.0 / M_PI) * 2.0 * asin(1.0 / 3.0),
		  (GLdouble)width / (GLdouble)height, zNear, zFar);
}

/**********************************************************************/
static void compile_display_lists (void)
{
  glNewList (displayListBase, GL_COMPILE);
  // OK, let's start drawing our planer quads.
  glBegin (GL_QUADS);

  // Bottom Face.  Red
  //glNormal3d ( 0.0, -1.0, 0.0); // Needed for lighting
  glColor3d (1.0, 0.0, 0.0); // Basic polygon color
  glVertex3d (-1.0, -1.0, -1.0);
  glVertex3d ( 1.0, -1.0, -1.0);
  glVertex3d ( 1.0, -1.0,  1.0);
  glVertex3d (-1.0, -1.0,  1.0);

  // Top face; White
  //glNormal3d ( 0.0, 1.0, 0.0);
  glColor3d (0.8, 0.8, 0.8);
  glVertex3d (-1.0,  1.0, -1.0);
  glVertex3d (-1.0,  1.0,  1.0);
  glVertex3d ( 1.0,  1.0,  1.0);
  glVertex3d ( 1.0,  1.0, -1.0);

  // Far face.  Green
  //glNormal3d ( 0.0, 0.0,-1.0);
  glColor3d (0.0, 1.0, 0.0);
  glVertex3d (-1.0, -1.0, -1.0);
  glVertex3d (-1.0,  1.0, -1.0);
  glVertex3d ( 1.0,  1.0, -1.0);
  glVertex3d ( 1.0, -1.0, -1.0);

  // Right face.  Blue
  //glNormal3d ( 1.0, 0.0, 0.0);
  glColor3d (0.0, 0.0, 1.0);
  glVertex3d ( 1.0, -1.0, -1.0);
  glVertex3d ( 1.0,  1.0, -1.0);
  glVertex3d ( 1.0,  1.0,  1.0);
  glVertex3d ( 1.0, -1.0,  1.0);

  // Front face; Yellow
  //glNormal3d ( 0.0, 0.0, 1.0);
  glColor3d ( 1.0, 1.0, 0.0);
  glVertex3d (-1.0, -1.0,  1.0);
  glVertex3d ( 1.0, -1.0,  1.0);
  glVertex3d ( 1.0,  1.0,  1.0);
  glVertex3d (-1.0,  1.0,  1.0);

  // Left Face; Magenta
  //glNormal3d (-1.0, 0.0, 0.0);
  glColor3d (1.0, 0.0, 1.0);
  glVertex3d (-1.0, -1.0, -1.0);
  glVertex3d (-1.0, -1.0,  1.0);
  glVertex3d (-1.0,  1.0,  1.0);
  glVertex3d (-1.0,  1.0, -1.0);

  // All polygons have been drawn.
  glEnd();


  glEndList();


  /* displayList for wire frame and axes */
  glNewList (displayListBase + 1, GL_COMPILE);
  glColor3d (0.5, 0.5, 0.5);
  glutWireCube (2.0);
  glBegin (GL_LINES);
  glColor3d (1.0, 0.0, 0.0);
  glVertex3d (0.0, 0.0, 0.0);
  glVertex3d (2.0, 0.0, 0.0);
  glColor3d (0.0, 1.0, 0.0);
  glVertex3d (0.0, 0.0, 0.0);
  glVertex3d (0.0, 2.0, 0.0);
  glColor3d (0.0, 0.0, 1.0);
  glVertex3d (0.0, 0.0, 0.0);
  glVertex3d (0.0, 0.0, 2.0);
  glEnd ();
  glEndList();
}

/**********************************************************************/


/**********************************************************************/
static void f (double t, double *y, double *yp)
{
/*
        [0]  [1]  [2]  [3]   [4]     [5]
   y    wx   wy   wz   phi   theta   psi
   yp   w'x  w'y  w'z  phi'  theta'  psi'
*/
  yp[0] = -(I[Z] - I[Y]) * y[1] * y[2] / I[X];
  yp[1] = -(I[X] - I[Z]) * y[2] * y[0] / I[Y];
  yp[2] = -(I[Y] - I[X]) * y[0] * y[1] / I[Z];
  yp[3] = (y[0] * sin(y[5]) + y[1] * cos(y[5])) / sin(y[4]);
  yp[4] = y[0] * cos(y[5]) - y[1] * sin(y[5]);
  yp[5] = y[2] - cos(y[4]) * yp[3];
}

/**********************************************************************/
static void idle (void)
{
  GLdouble twant, dt, c1, c2;
  int ifail;
  vec6 dydt;
  struct timeval tv1, dtv;
  static vec6 ymax;

#ifdef SAVEFILES
  dt = 0.1;
#else
  gettimeofday (&tv1, NULL);
  timersub (&tv1, &tv0, &dtv);
  dt = 1.0 * 0.000001 * dtv.tv_usec;
  tv0 = tv1;
#endif

  twant = t + dt;
  do {
    ifail = 1;
    rksuite.ut(&f, twant, t, y, dydt, ymax, ifail);
  } while (ifail!=1 && ifail != 6 && ifail != 911);
  /*
   *
   * uflag - OUT status, (1 - success, 2 - inefficient usage, 3 - work intensive,
        4 - problem probably stiff, 5 - too much accuracy requested, 6 - global error
        assessment unreliable beyond this point, 911 - catastrophic failure reported on
        stdout..
   */
  if (ifail == 6 || ifail==911)
    error ("ut() failed, ifail = %d", ifail);

  c1 = I[X] * y[0] * y[0] + I[Y] * y[1] * y[1] + I[Z] * y[2] * y[2];
  c2 = I[X] * I[X] * y[0] * y[0] +
    I[Y] * I[Y] * y[1] * y[1] +
    I[Z] * I[Z] * y[2] * y[2];

  printf ("%f (%17.15f %17.15f %17.15f) (%f %f %f) %f %f\n", t, y[0], y[1], y[2], y[3], y[4], y[5], c1, c2);
  usleep(100000);
  display ();
}

/**********************************************************************/

/**********************************************************************/
int main (int argc, char *argv[])
{
  int win, i;
  const GLfloat whiteLight[] = {1.0, 1.0, 1.0, 1.0};
  const GLfloat lightPosition1[] = {-5.0, 5.0, 1.0, 0.0};
  GLdouble tend, tol, hstart;
  int ifail, neq, method, erras, lenwrk;
  vec6 thres;
  char task;

  box[X] = 1.0;
  box[Y] = 0.5;
  box[Z] = 0.25;
  mass = 8 * box[X] * box[Y] * box[Z];
  I[X] = mass * (box[Y] * box[Y] + box[Z] * box[Z]) / 3.0;
  I[Y] = mass * (box[X] * box[X] + box[Z] * box[Z]) / 3.0;
  I[Z] = mass * (box[X] * box[X] + box[Y] * box[Y]) / 3.0;
/*
        [0]  [1]  [2]  [3]   [4]     [5]
   y    wx   wy   wz   phi   theta   psi
   yp   w'x  w'y  w'z  phi'  theta'  psi'
*/
/*
  y[0] = 0.0001;
  y[1] = 1.0;
  y[2] = 0.0001;
  y[3] = 0.0;
  y[4] = 0.00000001;
  y[5] = 0.0;
*/
  y[0] = 0.001;
  y[1] = 1.0;
  y[2] = 0.0001;
  y[3] = 0.0;
  y[4] = 0.00000001;
  y[5] = 0.0;

  t = 0.0;

  neq = NEQ;
  tend = 1000000.0;
  tol = 1.0e-8;
  for (i = 0; i < NEQ; i++)
    thres[i] = 1.0e-12;
  method = 3;
  task = 'U';
  erras = FALSE;
  hstart = 0.0;
  lenwrk = NEQ * 32;
  ifail = 0;
  rksuite.setup(neq, t, y, tend, tol, thres, method, &task, (bool)erras, hstart, (bool)ifail);

#ifdef SAVEFILES
  tv0.tv_sec = 0;
  tv0.tv_usec = 0;
#else
  gettimeofday (&tv0, NULL);
#endif

  glutInit (&argc, argv);

  /* open rgb, double-buffered OpenGL window with depth-buffer */
  winWidth = 1080;
  winHeight = 1080;
  glutInitWindowSize (winWidth, winHeight);
  glutInitDisplayMode (GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
  win = glutCreateWindow (argv[0]);

  /* declare glut callbacks */
  glutDisplayFunc (&display);
  glutReshapeFunc (&reshape);
  glutIdleFunc (&idle);

  glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);

  /* set up lighting */
  glClearColor (1.0, 1.0, 1.0, 0.0);
  glShadeModel (GL_SMOOTH);
  glLightfv (GL_LIGHT0, GL_DIFFUSE, whiteLight);
  glLightfv (GL_LIGHT0, GL_POSITION, lightPosition1);
  glEnable (GL_LIGHT0);
  glEnable (GL_COLOR_MATERIAL);
  glEnable (GL_CULL_FACE);

  /* compile displayLists */
  displayListBase = glGenLists (2);
  compile_display_lists ();

  /* handle glut events forever */
  glutMainLoop();

  return EXIT_SUCCESS;
}

/**********************************************************************/
