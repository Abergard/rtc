#pragma once

#include <QImage>
#include <QOpenGLWidget>
#include <QPoint>
#include <QSize>

#include <memory>
#include <optional>

#include "camera.hpp"
#include "geometry.hpp"
#include "scene_model.hpp"

namespace rtc_gui
{
class SceneView final : public QOpenGLWidget
{
 public:
  explicit SceneView(QWidget* parent = nullptr);

  void setScene(std::shared_ptr<rtc::scene_model> scene);
  void showRenderedImage(QImage image);
  void showOpenGlPreview();
  void resetCamera();
  void setWireframe(bool enabled);
  void setShowNormals(bool enabled);
  void setBackfaceCulling(bool enabled);

  [[nodiscard]] auto hasScene() const noexcept -> bool;
  [[nodiscard]] auto scene() const noexcept -> std::shared_ptr<rtc::scene_model>;
  [[nodiscard]] auto selectedTriangle() const noexcept -> std::optional<std::size_t>;
  [[nodiscard]] auto cameraForRender(const QSize& renderSize) const -> rtc::camera;
  auto flipSelectedTriangle() -> bool;

 protected:
  void initializeGL() override;
  void resizeGL(int width, int height) override;
  void paintGL() override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;

 private:
  struct CameraBasis
  {
    Vec3 eye{};
    Vec3 forward{};
    Vec3 right{};
    Vec3 up{};
  };

  void fitCameraToScene();
  void fitRadiusToScene();
  [[nodiscard]] auto eyePosition() const -> Vec3;
  [[nodiscard]] auto cameraBasis() const -> CameraBasis;
  [[nodiscard]] auto rayFromViewport(const QPoint& pos) const -> Ray;
  [[nodiscard]] auto pickTriangle(const QPoint& pos) const -> std::optional<std::size_t>;

  void setupProjection();
  void setupCamera();
  void drawScene();
  void drawSelectedTriangle();
  void drawNormals();

  std::shared_ptr<rtc::scene_model> scene_{};
  QImage renderedImage_{};
  QPoint lastMouse_{};
  Vec3 center_{};
  float radius_{10.0F};
  float distance_{25.0F};
  float yaw_{-35.0F};
  float pitch_{22.0F};
  float fovYDegrees_{45.0F};
  bool wireframe_{false};
  bool showNormals_{false};
  bool backfaceCulling_{true};
  std::optional<std::size_t> selectedTriangle_{};
};
}  // namespace rtc_gui
