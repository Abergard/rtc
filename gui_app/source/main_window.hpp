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
  void fixVisibleTriangleNormals();
  void editSelectedMaterial();
  void addLight();
  void editSelectedLight();
  void removeSelectedLight();
  void saveScene();

  SceneView* view_{};
  QString loadedScenePath_{};
  bool loadedSceneIsBrs_{false};
  QTimer renderPoll_{this};
  std::future<rtc::screen_surface> renderFuture_{};
};
}  // namespace rtc_gui
