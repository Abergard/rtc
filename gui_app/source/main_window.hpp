#pragma once

#include <QMainWindow>
#include <QProgressBar>
#include <QString>
#include <QTimer>

#include <future>
#include <memory>

#include "bitmap.hpp"
#include "rt_service.hpp"
#include "scene_view.hpp"

namespace rtc_gui
{
class MainWindow final : public QMainWindow
{
 public:
  explicit MainWindow(QWidget* parent = nullptr);
  auto openSceneAndRender(const QString& path) -> bool;

 private:
  void closeEvent(QCloseEvent* event) override;
  void loadScene();
  auto loadSceneFromPath(const QString& path) -> bool;
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
  QProgressBar* renderProgressBar_{};
  std::shared_ptr<rtc::rt_render_progress> renderProgress_{};
};
}  // namespace rtc_gui
