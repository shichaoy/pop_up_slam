/**
 * @file Viewer.cpp
 * @brief 3D visualization.
 * @author Michael Kaess
 * @version $Id: Viewer.cpp 6335 2012-03-22 23:13:52Z kaess $
 *
 * Copyright (C) 2009-2013 Massachusetts Institute of Technology.
 * Michael Kaess, Hordur Johannsson, David Rosen,
 * Nicholas Carlevaris-Bianco and John. J. Leonard
 *
 * This file is part of iSAM.
 *
 * iSAM is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 *
 * iSAM is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with iSAM.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <vector>
#include <map>

#if defined(__APPLE__) && defined(__MACH__)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#include "SDL.h"
#include "SDL_thread.h"

#include "isam/isam.h"

#include "Collections.h"
#include "Viewer.h"

using namespace std;
using namespace isam;
using namespace Eigen;

// current window size, with fixed initial size
GLsizei width = 800;
GLsizei height = 600;

// set to true whenever new drawing primitives were added
// multi-thread access variable, use mutex
bool redisplay_requested = false;
// set to true if processing thread should exit
// multi-thread access variable, use mutex
bool exit_request = false;

// redraw only if something changed (only used by main thread)
bool redraw = false;

// background color (gray scale 0 to 1)
float bg_color = 0.0f;

Pose3d eye_;

GLuint gl_list;

SDL_Thread* thread;
SDL_mutex* mutex;

/**
 * Redraw the scene into a GLList.
 */
void populateGLList() {
  glNewList(gl_list, GL_COMPILE_AND_EXECUTE);
  for (collections_t::iterator it = collections.begin(); it != collections.end(); it++) {
    it->second->draw();
  }
  glEndList();
}

/**
 * Draw everything, using the GLList if no change in the actual content.
 */
void drawGL() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(bg_color, bg_color, bg_color, 1.0f);
  glLoadIdentity();

  /* Camera rotation */
  glTranslated(-eye_.x(), -eye_.y(), -eye_.z());
  glRotated(rad_to_deg(eye_.roll()),  1.0f, 0.0f, 0.0f);
  glRotated(rad_to_deg(eye_.pitch()), 0.0f, 1.0f, 0.0f);
  glRotated(rad_to_deg(eye_.yaw()),   0.0f, 0.0f, 1.0f);

  GLfloat light_position0[] = {1.0, 1.0, 1.0, 0.0};
  glLightfv(GL_LIGHT0, GL_POSITION, light_position0);

  // redraw (re-populate the GL list) only if something changed
  SDL_LockMutex(mutex);
  if (redisplay_requested) {
    redisplay_requested = false;
    populateGLList();
  }
  SDL_UnlockMutex(mutex);

  // draws the list
  glCallList(gl_list);
}

/**
 * Initialize OpenGL (only done once at beginning.
 */
void initGL() {
  glShadeModel(GL_SMOOTH);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_COLOR_MATERIAL);
  //  glEnable(GL_CULL_FACE);
}

/**
 * Close graphics window and tell process thread to quit, then clean up.
 */
void quit() {
  SDL_QuitSubSystem(SDL_INIT_VIDEO);
  SDL_LockMutex(mutex);
  exit_request = true;
  SDL_UnlockMutex(mutex);
  SDL_WaitThread(thread, NULL);
  SDL_DestroyMutex(mutex);
  SDL_Quit();
}

/**
 * Resize the viewport, at creation, or after window was resized by user.
 */
void resize(GLsizei w, GLsizei h, int videoFlags) {
  if (w<10) {
    w = 10;
  }
  if (h<10) {
    h = 10;
  }
  width = w;
  height = h;

  if (SDL_SetVideoMode(w, h, 0, videoFlags) == NULL) {
    require(false, "Could not open GL window");
  }

  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0, (float)w/(float)h, 0.1, 10000.0);
  glMatrixMode(GL_MODELVIEW);
}

/**
 * Provide scale factor to make translation and scaling more sensitive
 * if further away from origin to allow for fast navigation.
 */
double factor(bool use_sqrt) {
  double distance = eye_.trans().vector().norm();
  if (use_sqrt) {
    return 1. + sqrt(distance) / 2.;
  } else {
    return 1. + distance / 20.;

  }
}

/**
 * Incrementally change rotation of camera/scene.
 */
void rotate(int dx, int dy) {
  eye_.set_roll (eye_.roll()  + deg_to_rad(dy));
  eye_.set_pitch(eye_.pitch() + deg_to_rad(dx));
  redraw = true;
}

void rotate2(int dx, int dy) {
  eye_.set_roll(eye_.roll() + deg_to_rad(dy));
  eye_.set_yaw (eye_.yaw()  + deg_to_rad(dx));
  redraw = true;
}

/**
 * Incrementally change translation of camera/scene.
 */
void translate(int dx, int dy) {
  eye_.set_x(eye_.x() - dx * 0.03*factor(false));
  eye_.set_y(eye_.y() + dy * 0.03*factor(false));
  redraw = true;
}

/**
 * Incrementally change scale of scene.
 */
void scale(int dx, int dy) {
  eye_.set_z(eye_.z() + dy * 0.1*factor(true));
  redraw = true;
}

/**
 * Reset viewpoint to default starting position.
 */
void reset() {
  eye_ = Pose3d(0.0f, 0.0f, 100.0f, 0.0f, 0.0f, 0.0f);
  redraw = true;
}

/**
 * Switch background between black and white (useful for screen shots).
 */
void toggle_color() {
  bg_color = 1.0f - bg_color;
}

/**
 * Process key press.
 */
void keyPress(SDL_keysym *keysym) {
  switch(keysym->sym) {
  case SDLK_ESCAPE:
  case SDLK_q:
    quit();
    break;
  case SDLK_r:
    reset();
    break;
  case SDLK_c:
    toggle_color();
    break;
  default:
    break;
  }
}

/**
 * Process mouse wheel for scaling function.
 */
void processMouseWheel(int button) {
  if(button == SDL_BUTTON_WHEELUP) {
    scale(0, -2.*factor(true));
  } else if(button == SDL_BUTTON_WHEELDOWN) {
    scale(0, 2.*factor(true));
  }
}

/**
 * Process mouse motion, allowing navigation with various
 * mouse key and modifier key combinations.
 */
void mouseMotion(int x, int y) {
  static int x_pos = 0;
  static int y_pos = 0;

  // ignore if mouse outside window - avoids strange behavior on Mac
  if (x==0 || y==0 || x==width-1 || y==height-1) return;

  int dx = x - x_pos;
  int dy = y - y_pos;
  SDLMod modState = SDL_GetModState();
  Uint8 mouseState = SDL_GetMouseState(NULL, NULL);

  if (mouseState & SDL_BUTTON(SDL_BUTTON_LEFT)) {
    if (modState & KMOD_SHIFT) {
      scale(dx, dy);
    } else if (modState & KMOD_CTRL) {
      translate(dx, dy);
    } else if (modState & KMOD_ALT) {
      rotate2(dx, dy);
    } else {
      rotate(dx, dy);
    }
  } else if (mouseState & SDL_BUTTON(SDL_BUTTON_MIDDLE)) {
    translate(dx, dy);
  } else if (mouseState & SDL_BUTTON(SDL_BUTTON_RIGHT)) {
    scale(dx, dy);
  }

  x_pos = x;
  y_pos = y;
}

/**
 * Setup SDL, only called once at startup.
 */
int setupSDL() {
  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    require(false, "Could not initialize SDL");
  }

  atexit(quit);

  // create a GL window
  int videoFlags = SDL_OPENGL|SDL_RESIZABLE|SDL_DOUBLEBUF;
  const SDL_VideoInfo* videoInfo = SDL_GetVideoInfo();
  if (videoInfo->hw_available) {
    videoFlags |= SDL_HWSURFACE;
  } else {
    videoFlags |= SDL_SWSURFACE;
  }
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

  SDL_WM_SetCaption("iSAM Viewer", "opengl");

  return videoFlags;
}

int main_loop(void* unused) {
  int videoFlags = setupSDL();
  resize(width, height, videoFlags);
  initGL();

  // allocate space for an OpenGL list
  gl_list = glGenLists(1);

  // SDL even loop
  bool done = false;
  bool isActive = true;
  while(!done) {
    // save CPU cycles: only redraw scene if change in viewpoint etc
    redraw = false;
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      switch (event.type) {
      case SDL_VIDEORESIZE:
        resize(event.resize.w, event.resize.h, videoFlags);
        initGL();
        break;
      case SDL_KEYDOWN:
        keyPress(&event.key.keysym);
        break;
      case SDL_MOUSEBUTTONDOWN:
        processMouseWheel(event.button.button);
        break;
      case SDL_MOUSEMOTION:
        mouseMotion(event.button.x, event.button.y);
        break;
      case SDL_ACTIVEEVENT:
        if ((event.active.gain==0) && (event.active.state & SDL_APPACTIVE)) {
          isActive = false;
        } else {
          isActive = true;
          redraw = true;
        }
        break;
      case SDL_QUIT:
        done = true;
        break;
      }
    }

    // also redraw if scene itself changed;
    // note that redisplay_requested is reset in populateGLList
    SDL_LockMutex(mutex);
    if (redisplay_requested) {
      redraw = true;
    }
    SDL_UnlockMutex(mutex);

    if (isActive && redraw) {
      drawGL();
      SDL_GL_SwapBuffers();
    }

    SDL_Delay(20); // don't eat up all CPU cycles, restrict to 50Hz

  }

  quit();

  return 0;
}

void Viewer::init(int (*process)(void*)) {
  reset();

  mutex = SDL_CreateMutex();

  thread = SDL_CreateThread(process, NULL);
  main_loop(NULL);
}

void Viewer::set_nodes(const vector<Pose3d, Eigen::aligned_allocator<isam::Pose3d> >& nodes, int id, const string& name, int type) {
  SDL_LockMutex(mutex);
  ObjCollection* collection = new ObjCollection(id, name, type, nodes);
  collections_t::iterator it = collections.find(id);
  if (it!=collections.end()) {
    // if previously created, deallocate and remove
    Collection* c = it->second;
    collections.erase(id);
    delete c;
  }
  collections.insert(make_pair(id, collection));
  redisplay_requested = true;
  SDL_UnlockMutex(mutex);
}

void Viewer::set_links(const vector<pair<int,int> >& links, int id, const string& name, int col1, int col2) {
  SDL_LockMutex(mutex);
  LinkCollection* collection = new LinkCollection(id, name, links, col1, col2);
  collections_t::iterator it = collections.find(id);
  if (it!=collections.end()) {
    // if previously created, deallocate and remove
    Collection* c = it->second;
    collections.erase(id);
    delete c;
  }
  collections.insert(make_pair(id, collection));
  redisplay_requested = true;
  SDL_UnlockMutex(mutex);
}

void Viewer::set_covariances(const list<MatrixXd>& covariances, int id, char* name, int col, bool is_3d) const {
  SDL_LockMutex(mutex);
  CovCollection* collection = new CovCollection(id, name, covariances, col, is_3d);
  collections_t::iterator it = collections.find(id);
  if (it!=collections.end()) {
    // if previously created, deallocate and remove
    Collection* c = it->second;
    collections.erase(id);
    delete c;
  }
  collections.insert(make_pair(id, collection));
  redisplay_requested = true;
  SDL_UnlockMutex(mutex);
}

bool Viewer::exit_requested() {
  SDL_LockMutex(mutex);
  bool ret = exit_request;
  SDL_UnlockMutex(mutex);
  return ret;
}
