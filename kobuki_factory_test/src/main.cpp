/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/kobuki_factory_test/main_window.hpp"

#include <X11/Xlib.h>

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

  // Required to show an OpenCV simultaneously
  XInitThreads();

  /*********************
  ** Qt
  **********************/
  QApplication app(argc, argv);
  kobuki_factory_test::MainWindow w(argc,argv);
  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();

  return result;
}
