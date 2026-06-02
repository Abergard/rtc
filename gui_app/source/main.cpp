#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QSurfaceFormat>

#include "main_window.hpp"

int main(int argc, char** argv)
{
  QSurfaceFormat format;
  format.setVersion(2, 1);
  format.setProfile(QSurfaceFormat::CompatibilityProfile);
  QSurfaceFormat::setDefaultFormat(format);

  QApplication app(argc, argv);
  QCommandLineParser parser;
  parser.setApplicationDescription("RTC GUI scene viewer and renderer");
  parser.addHelpOption();
  const QCommandLineOption renderOption{
      "render",
      "Load the given BRS/scene file and start rendering when the app opens.",
      "brs file"};
  parser.addOption(renderOption);
  parser.process(app);

  rtc_gui::MainWindow window;
  window.show();
  if (parser.isSet(renderOption))
    window.openSceneAndRender(parser.value(renderOption));

  return app.exec();
}
