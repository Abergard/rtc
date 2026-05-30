#include <QApplication>
#include <QSurfaceFormat>

#include "main_window.hpp"

int main(int argc, char** argv)
{
  QSurfaceFormat format;
  format.setVersion(2, 1);
  format.setProfile(QSurfaceFormat::CompatibilityProfile);
  QSurfaceFormat::setDefaultFormat(format);

  QApplication app(argc, argv);
  rtc_gui::MainWindow window;
  window.show();
  return app.exec();
}
