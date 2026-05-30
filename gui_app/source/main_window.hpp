#pragma once

#include <QMainWindow>
#include <QString>
#include <QTimer>

#include <future>

#include "bitmap.hpp"
#include "scene_view.hpp"

namespace rtc_gui
{
class MainWindow final : public QMainWindow
{
 public:
  explicit MainWindow(QWidget* parent = nullptr);

 private:
  void loadScene();
  void startRender();
  void pollRender();
  void flipSelectedTriangle();
  void saveScene();

  SceneView* view_{};
  QString loadedScenePath_{};
  QTimer renderPoll_{this};
  std::future<rtc::bitmap> renderFuture_{};
};
}  // namespace rtc_gui
