#include "scene_view.hpp"

#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

#include <GL/gl.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "color.hpp"
#include "math_vector.hpp"
#include "triangle3d.hpp"

namespace rtc_gui
{
namespace
{
auto material_color(const rtc::scene_model& scene, const std::size_t triangle_index) -> rtc::color
{
  if (triangle_index < scene.material_id.size())
  {
    const auto material_index = scene.material_id[triangle_index];
    if (material_index < scene.materials.size())
      return scene.materials[material_index].material_color;
  }

  return rtc::color{180.0F, 180.0F, 180.0F};
}

auto color_component(float value) -> float
{
  if (value > 1.0F)
    value /= 255.0F;

  return std::clamp(value, 0.08F, 1.0F);
}

auto triangle_normal(const rtc::scene_model& scene, const rtc::triangle3d& triangle) -> Vec3
{
  const auto a = to_vec3(scene.points[triangle.vertex_a()]);
  const auto b = to_vec3(scene.points[triangle.vertex_b()]);
  const auto c = to_vec3(scene.points[triangle.vertex_c()]);
  return normalized(cross(b - a, c - a));
}

auto valid_triangle(const rtc::scene_model& scene, const rtc::triangle3d& triangle) -> bool
{
  return triangle.vertex_a() < scene.points.size() && triangle.vertex_b() < scene.points.size() &&
         triangle.vertex_c() < scene.points.size();
}

void flip_triangle(rtc::scene_model& scene, const std::size_t triangleIndex)
{
  const auto& triangle = scene.triangles[triangleIndex];
  scene.triangles[triangleIndex] = rtc::triangle3d{triangle.vertex_a(), triangle.vertex_c(), triangle.vertex_b()};
}
}  // namespace

SceneView::SceneView(QWidget* parent) : QOpenGLWidget(parent)
{
  setFocusPolicy(Qt::StrongFocus);
  setMinimumSize(640, 420);
}

void SceneView::setScene(std::shared_ptr<rtc::scene_model> scene)
{
  scene_ = std::move(scene);
  renderedImage_ = {};
  selectedTriangle_ = std::nullopt;
  fitCameraToScene();
  update();
}

void SceneView::showRenderedImage(QImage image)
{
  renderedImage_ = std::move(image);
  update();
}

void SceneView::showOpenGlPreview()
{
  renderedImage_ = {};
  update();
}

auto SceneView::hasScene() const noexcept -> bool { return static_cast<bool>(scene_); }

auto SceneView::scene() const noexcept -> std::shared_ptr<rtc::scene_model> { return scene_; }

auto SceneView::selectedTriangle() const noexcept -> std::optional<std::size_t> { return selectedTriangle_; }

void SceneView::resetCamera()
{
  fitCameraToScene();
  renderedImage_ = {};
  update();
}

void SceneView::setWireframe(const bool enabled)
{
  wireframe_ = enabled;
  update();
}

void SceneView::setShowNormals(const bool enabled)
{
  showNormals_ = enabled;
  update();
}

void SceneView::setBackfaceCulling(const bool enabled)
{
  backfaceCulling_ = enabled;
  update();
}

auto SceneView::flipSelectedTriangle() -> bool
{
  if (!scene_ || !selectedTriangle_ || *selectedTriangle_ >= scene_->triangles.size())
    return false;

  flip_triangle(*scene_, *selectedTriangle_);
  scene_->normals.assign(scene_->triangles.size(), rtc::math_vector{});
  renderedImage_ = {};
  update();
  return true;
}

auto SceneView::fixVisibleTriangleNormals(const int sampleStep) -> VisibleNormalFixResult
{
  VisibleNormalFixResult result{};
  if (!scene_ || scene_->triangles.empty())
    return result;

  const auto step = std::max(1, sampleStep);
  std::vector<bool> visible(scene_->triangles.size(), false);
  std::vector<float> visibleDot(scene_->triangles.size(), 0.0F);

  const auto markVisible = [&](const Ray& ray) {
    if (const auto triangle = frontVisibleTriangle(ray))
    {
      visible[*triangle] = true;
      if (valid_triangle(*scene_, scene_->triangles[*triangle]))
        visibleDot[*triangle] = dot(triangle_normal(*scene_, scene_->triangles[*triangle]), ray.direction);
    }
  };

  for (int y = 0; y < height(); y += step)
  {
    for (int x = 0; x < width(); x += step)
    {
      markVisible(rayFromViewport(QPoint{x, y}));
    }
  }

  for (const QPoint edgePoint : {QPoint{std::max(0, width() - 1), std::max(0, height() - 1)},
                                 QPoint{std::max(0, width() - 1), 0},
                                 QPoint{0, std::max(0, height() - 1)}})
  {
    markVisible(rayFromViewport(edgePoint));
  }

  for (std::size_t i = 0; i < visible.size(); ++i)
  {
    if (!visible[i] || !valid_triangle(*scene_, scene_->triangles[i]))
      continue;

    ++result.visibleTriangles;
    if (visibleDot[i] > 0.0F)
    {
      flip_triangle(*scene_, i);
      ++result.flippedTriangles;
    }
  }

  if (result.flippedTriangles != 0)
    scene_->normals.assign(scene_->triangles.size(), rtc::math_vector{});

  renderedImage_ = {};
  update();
  return result;
}

auto SceneView::cameraForRender(const QSize& renderSize) const -> rtc::camera
{
  const auto basis = cameraBasis();

  const auto planeDistance = std::max(1.0F, distance_ * 0.45F);
  const auto aspect = std::max(1.0F, static_cast<float>(renderSize.width())) /
                      std::max(1.0F, static_cast<float>(renderSize.height()));
  const auto halfHeight = std::tan(fovYDegrees_ * pi / 360.0F) * planeDistance;
  const auto halfWidth = halfHeight * aspect;
  const auto planeCenter = basis.eye + basis.forward * planeDistance;

  rtc::camera camera{};
  camera.view_point = to_point(basis.eye);
  camera.screen.resolution.x = static_cast<std::uint32_t>(std::max(1, renderSize.width()));
  camera.screen.resolution.y = static_cast<std::uint32_t>(std::max(1, renderSize.height()));
  camera.screen.surface.upper_left_corner = to_point(planeCenter + basis.up * halfHeight - basis.right * halfWidth);
  camera.screen.surface.upper_right_corner = to_point(planeCenter + basis.up * halfHeight + basis.right * halfWidth);
  camera.screen.surface.lower_left_corner = to_point(planeCenter - basis.up * halfHeight - basis.right * halfWidth);
  return camera;
}

void SceneView::initializeGL()
{
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  glClearColor(0.05F, 0.06F, 0.07F, 1.0F);
}

void SceneView::resizeGL(int width, int height)
{
  glViewport(0, 0, width, std::max(1, height));
}

void SceneView::paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (!renderedImage_.isNull())
  {
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    QPainter painter(this);
    painter.setCompositionMode(QPainter::CompositionMode_Source);
    painter.drawImage(rect(), renderedImage_);
    painter.end();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    return;
  }

  setupProjection();
  setupCamera();
  drawScene();
}

void SceneView::mousePressEvent(QMouseEvent* event)
{
  lastMouse_ = event->pos();
  setFocus();

  if (event->button() == Qt::LeftButton && (event->modifiers() & Qt::ShiftModifier))
  {
    selectedTriangle_ = pickTriangle(event->pos());
    renderedImage_ = {};
    update();
  }
}

void SceneView::mouseMoveEvent(QMouseEvent* event)
{
  const auto delta = event->pos() - lastMouse_;
  lastMouse_ = event->pos();
  renderedImage_ = {};

  if ((event->buttons() & Qt::LeftButton) && !(event->modifiers() & Qt::ShiftModifier))
  {
    yaw_ += delta.x() * 0.45F;
    pitch_ = std::clamp(pitch_ + delta.y() * 0.45F, -89.0F, 89.0F);
  }
  else if (event->buttons() & Qt::MiddleButton || event->buttons() & Qt::RightButton)
  {
    const auto basis = cameraBasis();
    const auto scale = distance_ * 0.0018F;
    center_ = center_ - basis.right * (delta.x() * scale) + basis.up * (delta.y() * scale);
  }

  update();
}

void SceneView::wheelEvent(QWheelEvent* event)
{
  renderedImage_ = {};
  const auto steps = event->angleDelta().y() / 120.0F;
  distance_ *= std::pow(0.86F, steps);
  distance_ = std::clamp(distance_, radius_ * 0.03F, radius_ * 40.0F);
  update();
}

void SceneView::keyPressEvent(QKeyEvent* event)
{
  const auto basis = cameraBasis();
  const auto step = distance_ * 0.06F;

  renderedImage_ = {};
  switch (event->key())
  {
    case Qt::Key_W:
      center_ = center_ + basis.forward * step;
      break;
    case Qt::Key_S:
      center_ = center_ - basis.forward * step;
      break;
    case Qt::Key_A:
      center_ = center_ - basis.right * step;
      break;
    case Qt::Key_D:
      center_ = center_ + basis.right * step;
      break;
    case Qt::Key_Q:
      center_ = center_ - basis.up * step;
      break;
    case Qt::Key_E:
      center_ = center_ + basis.up * step;
      break;
    default:
      QOpenGLWidget::keyPressEvent(event);
      return;
  }

  update();
}

void SceneView::fitCameraToScene()
{
  if (!scene_)
  {
    center_ = {};
    radius_ = 10.0F;
    distance_ = 25.0F;
    return;
  }

  fitRadiusToScene();

  const auto& camera = scene_->optical_system;
  const auto upperLeft = to_vec3(camera.screen.surface.upper_left_corner);
  const auto upperRight = to_vec3(camera.screen.surface.upper_right_corner);
  const auto lowerLeft = to_vec3(camera.screen.surface.lower_left_corner);
  const auto eye = to_vec3(camera.view_point);
  const auto screenCenter = upperLeft + (upperRight - upperLeft) * 0.5F + (lowerLeft - upperLeft) * 0.5F;
  const auto view = eye - screenCenter;

  if (length(view) > 0.0001F)
  {
    center_ = screenCenter;
    distance_ = length(view);
    yaw_ = std::atan2(view.x, view.z) * 180.0F / pi;
    pitch_ = std::asin(std::clamp(view.y / distance_, -1.0F, 1.0F)) * 180.0F / pi;
    const auto screenHeight = length(lowerLeft - upperLeft);
    fovYDegrees_ = 2.0F * std::atan((screenHeight * 0.5F) / distance_) * 180.0F / pi;
    return;
  }

  center_ = {};
  distance_ = radius_ * 2.8F;
  yaw_ = -35.0F;
  pitch_ = 22.0F;
}

void SceneView::fitRadiusToScene()
{
  if (!scene_ || scene_->points.empty())
  {
    radius_ = 10.0F;
    return;
  }

  Vec3 minPoint{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
  Vec3 maxPoint{std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};

  for (const auto& point : scene_->points)
  {
    const auto p = to_vec3(point);
    minPoint.x = std::min(minPoint.x, p.x);
    minPoint.y = std::min(minPoint.y, p.y);
    minPoint.z = std::min(minPoint.z, p.z);
    maxPoint.x = std::max(maxPoint.x, p.x);
    maxPoint.y = std::max(maxPoint.y, p.y);
    maxPoint.z = std::max(maxPoint.z, p.z);
  }

  radius_ = std::max(1.0F, length(maxPoint - minPoint) * 0.5F);
}

auto SceneView::eyePosition() const -> Vec3
{
  const auto yaw = yaw_ * pi / 180.0F;
  const auto pitch = pitch_ * pi / 180.0F;

  return {
      center_.x + distance_ * std::cos(pitch) * std::sin(yaw),
      center_.y + distance_ * std::sin(pitch),
      center_.z + distance_ * std::cos(pitch) * std::cos(yaw),
  };
}

auto SceneView::cameraBasis() const -> CameraBasis
{
  CameraBasis basis{};
  basis.eye = eyePosition();
  basis.forward = normalized(center_ - basis.eye);
  basis.right = normalized(cross(basis.forward, Vec3{0.0F, 1.0F, 0.0F}));

  if (length(basis.right) < 0.0001F)
    basis.right = {1.0F, 0.0F, 0.0F};

  basis.up = normalized(cross(basis.right, basis.forward));
  return basis;
}

auto SceneView::rayFromViewport(const QPoint& pos) const -> Ray
{
  const auto basis = cameraBasis();
  const auto planeDistance = std::max(1.0F, distance_ * 0.45F);
  const auto aspect = std::max(1.0F, static_cast<float>(width())) / std::max(1.0F, static_cast<float>(height()));
  const auto halfHeight = std::tan(fovYDegrees_ * pi / 360.0F) * planeDistance;
  const auto halfWidth = halfHeight * aspect;
  const auto planeCenter = basis.eye + basis.forward * planeDistance;
  const auto upperLeft = planeCenter + basis.up * halfHeight - basis.right * halfWidth;
  const auto upperRight = planeCenter + basis.up * halfHeight + basis.right * halfWidth;
  const auto lowerLeft = planeCenter - basis.up * halfHeight - basis.right * halfWidth;

  const auto u = std::clamp(pos.x() / std::max(1.0F, static_cast<float>(width() - 1)), 0.0F, 1.0F);
  const auto v = std::clamp(pos.y() / std::max(1.0F, static_cast<float>(height() - 1)), 0.0F, 1.0F);
  const auto point = upperLeft + (upperRight - upperLeft) * u + (lowerLeft - upperLeft) * v;
  return {basis.eye, normalized(point - basis.eye)};
}

auto SceneView::pickTriangle(const QPoint& pos) const -> std::optional<std::size_t>
{
  return frontVisibleTriangle(rayFromViewport(pos));
}

auto SceneView::frontVisibleTriangle(const Ray& ray) const -> std::optional<std::size_t>
{
  if (!scene_)
    return std::nullopt;

  std::optional<std::size_t> result{};
  auto nearest = std::numeric_limits<float>::max();
  for (std::size_t i = 0; i < scene_->triangles.size(); ++i)
  {
    const auto& triangle = scene_->triangles[i];
    if (!valid_triangle(*scene_, triangle))
      continue;

    const auto hit = intersect_triangle(ray,
                                        to_vec3(scene_->points[triangle.vertex_a()]),
                                        to_vec3(scene_->points[triangle.vertex_b()]),
                                        to_vec3(scene_->points[triangle.vertex_c()]));
    if (hit && *hit < nearest)
    {
      nearest = *hit;
      result = i;
    }
  }

  return result;
}

void SceneView::setupProjection()
{
  const auto aspect = std::max(1, width()) / static_cast<float>(std::max(1, height()));
  const auto nearPlane = std::max(0.01F, distance_ - radius_ * 4.0F);
  const auto farPlane = distance_ + radius_ * 8.0F;
  const auto top = std::tan(fovYDegrees_ * pi / 360.0F) * nearPlane;
  const auto right = top * aspect;

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glFrustum(-right, right, -top, top, nearPlane, farPlane);
}

void SceneView::setupCamera()
{
  const auto basis = cameraBasis();

  glMatrixMode(GL_MODELVIEW);
  const GLfloat viewMatrix[] = {
      basis.right.x,
      basis.up.x,
      -basis.forward.x,
      0.0F,
      basis.right.y,
      basis.up.y,
      -basis.forward.y,
      0.0F,
      basis.right.z,
      basis.up.z,
      -basis.forward.z,
      0.0F,
      -dot(basis.right, basis.eye),
      -dot(basis.up, basis.eye),
      dot(basis.forward, basis.eye),
      1.0F,
  };
  glLoadMatrixf(viewMatrix);

  GLfloat ambient[] = {0.22F, 0.22F, 0.24F, 1.0F};
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);

  if (scene_ && !scene_->lights.empty())
  {
    const auto light = scene_->lights.front().position;
    GLfloat pos[] = {light.x(), light.y(), light.z(), 1.0F};
    glLightfv(GL_LIGHT0, GL_POSITION, pos);
  }
  else
  {
    GLfloat pos[] = {center_.x + radius_, center_.y + radius_, center_.z + radius_, 1.0F};
    glLightfv(GL_LIGHT0, GL_POSITION, pos);
  }
}

void SceneView::drawScene()
{
  if (!scene_)
    return;

  if (backfaceCulling_)
    glEnable(GL_CULL_FACE);
  else
    glDisable(GL_CULL_FACE);

  glPolygonMode(GL_FRONT_AND_BACK, wireframe_ ? GL_LINE : GL_FILL);

  glBegin(GL_TRIANGLES);
  for (std::size_t i = 0; i < scene_->triangles.size(); ++i)
  {
    const auto& triangle = scene_->triangles[i];
    if (triangle.vertex_a() >= scene_->points.size() || triangle.vertex_b() >= scene_->points.size() ||
        triangle.vertex_c() >= scene_->points.size())
      continue;

    const auto a = scene_->points[triangle.vertex_a()];
    const auto b = scene_->points[triangle.vertex_b()];
    const auto c = scene_->points[triangle.vertex_c()];
    const auto normal = rtc::normalize(rtc::cross(b - a, c - a));
    const auto color = material_color(*scene_, i);

    glColor3f(color_component(color.red()), color_component(color.green()), color_component(color.blue()));
    glNormal3f(normal.x(), normal.y(), normal.z());
    glVertex3f(a.x(), a.y(), a.z());
    glVertex3f(b.x(), b.y(), b.z());
    glVertex3f(c.x(), c.y(), c.z());
  }
  glEnd();

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  drawSelectedTriangle();
  drawNormals();

  glDisable(GL_LIGHTING);
  glPointSize(8.0F);
  glBegin(GL_POINTS);
  for (const auto& light : scene_->lights)
  {
    glColor3f(1.0F, 0.95F, 0.55F);
    glVertex3f(light.position.x(), light.position.y(), light.position.z());
  }
  glEnd();
  glEnable(GL_LIGHTING);
}

void SceneView::drawSelectedTriangle()
{
  if (!scene_ || !selectedTriangle_ || *selectedTriangle_ >= scene_->triangles.size())
    return;

  const auto& triangle = scene_->triangles[*selectedTriangle_];
  if (triangle.vertex_a() >= scene_->points.size() || triangle.vertex_b() >= scene_->points.size() ||
      triangle.vertex_c() >= scene_->points.size())
    return;

  const auto a = scene_->points[triangle.vertex_a()];
  const auto b = scene_->points[triangle.vertex_b()];
  const auto c = scene_->points[triangle.vertex_c()];

  glDisable(GL_LIGHTING);
  glDisable(GL_CULL_FACE);
  glLineWidth(3.0F);
  glColor3f(1.0F, 0.9F, 0.1F);
  glBegin(GL_LINE_LOOP);
  glVertex3f(a.x(), a.y(), a.z());
  glVertex3f(b.x(), b.y(), b.z());
  glVertex3f(c.x(), c.y(), c.z());
  glEnd();
  glLineWidth(1.0F);
  glEnable(GL_LIGHTING);
}

void SceneView::drawNormals()
{
  if (!scene_ || !showNormals_)
    return;

  const auto normalLength = std::max(0.04F, radius_ * 0.035F);

  glDisable(GL_LIGHTING);
  glDisable(GL_CULL_FACE);
  glColor3f(0.15F, 0.85F, 1.0F);
  glBegin(GL_LINES);
  for (std::size_t i = 0; i < scene_->triangles.size(); ++i)
  {
    const auto& triangle = scene_->triangles[i];
    if (triangle.vertex_a() >= scene_->points.size() || triangle.vertex_b() >= scene_->points.size() ||
        triangle.vertex_c() >= scene_->points.size())
      continue;

    const auto a = to_vec3(scene_->points[triangle.vertex_a()]);
    const auto b = to_vec3(scene_->points[triangle.vertex_b()]);
    const auto c = to_vec3(scene_->points[triangle.vertex_c()]);
    const auto center = (a + b + c) / 3.0F;
    const auto normal = normalized(cross(b - a, c - a));
    const auto end = center + normal * normalLength;

    glVertex3f(center.x, center.y, center.z);
    glVertex3f(end.x, end.y, end.z);
  }
  glEnd();
  glEnable(GL_LIGHTING);
}
}  // namespace rtc_gui
