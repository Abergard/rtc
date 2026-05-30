#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QColor>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QImage>
#include <QKeyEvent>
#include <QLabel>
#include <QMainWindow>
#include <QMouseEvent>
#include <QOpenGLWidget>
#include <QPainter>
#include <QPushButton>
#include <QStatusBar>
#include <QSurfaceFormat>
#include <QTimer>
#include <QToolBar>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <GL/gl.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <future>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "bitmap.hpp"
#include "brs.hpp"
#include "math_vector.hpp"
#include "render_engine.hpp"
#include "scene_model.hpp"

namespace
{
constexpr float pi = 3.14159265358979323846F;

struct Vec3
{
  float x{};
  float y{};
  float z{};
};

auto operator+(const Vec3& a, const Vec3& b) -> Vec3 { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
auto operator-(const Vec3& a, const Vec3& b) -> Vec3 { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
auto operator*(const Vec3& a, const float s) -> Vec3 { return {a.x * s, a.y * s, a.z * s}; }
auto operator/(const Vec3& a, const float s) -> Vec3 { return {a.x / s, a.y / s, a.z / s}; }

auto dot(const Vec3& a, const Vec3& b) -> float { return a.x * b.x + a.y * b.y + a.z * b.z; }

auto cross(const Vec3& a, const Vec3& b) -> Vec3
{
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

auto length(const Vec3& v) -> float { return std::sqrt(dot(v, v)); }

auto normalized(const Vec3& v) -> Vec3
{
  const auto len = length(v);
  return len > 0.00001F ? v / len : Vec3{0.0F, 0.0F, 1.0F};
}

auto to_vec3(const rtc::math_point& p) -> Vec3 { return {p.x(), p.y(), p.z()}; }
auto to_point(const Vec3& v) -> rtc::math_point { return {v.x, v.y, v.z}; }

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

auto bitmap_to_image(const rtc::bitmap& bitmap) -> QImage
{
  QImage image(static_cast<int>(bitmap.width()), static_cast<int>(bitmap.height()), QImage::Format_RGB32);

  for (int y = 0; y < image.height(); ++y)
  {
    for (int x = 0; x < image.width(); ++x)
    {
      const auto& c = bitmap(static_cast<std::uint16_t>(x), static_cast<std::uint16_t>(y));
      image.setPixel(x, y, qRgb(c.red, c.green, c.blue));
    }
  }

  return image;
}

auto non_black_pixels(const QImage& image) -> std::size_t
{
  std::size_t result{};

  for (int y = 0; y < image.height(); ++y)
  {
    for (int x = 0; x < image.width(); ++x)
    {
      const auto pixel = image.pixel(x, y);
      if (qRed(pixel) != 0 || qGreen(pixel) != 0 || qBlue(pixel) != 0)
        ++result;
    }
  }

  return result;
}

}  // namespace

class SceneView final : public QOpenGLWidget
{
 public:
  explicit SceneView(QWidget* parent = nullptr) : QOpenGLWidget(parent)
  {
    setFocusPolicy(Qt::StrongFocus);
    setMinimumSize(640, 420);
  }

  void setScene(std::shared_ptr<rtc::scene_model> scene)
  {
    scene_ = std::move(scene);
    renderedImage_ = {};
    fitCameraToScene();
    update();
  }

  void showRenderedImage(QImage image)
  {
    renderedImage_ = std::move(image);
    update();
  }

  void showOpenGlPreview()
  {
    renderedImage_ = {};
    update();
  }

  auto hasScene() const noexcept -> bool { return static_cast<bool>(scene_); }
  auto scene() const noexcept -> std::shared_ptr<rtc::scene_model> { return scene_; }

  void resetCamera()
  {
    fitCameraToScene();
    renderedImage_ = {};
    update();
  }

  void setWireframe(const bool enabled)
  {
    wireframe_ = enabled;
    update();
  }

  auto cameraForRender(const QSize& renderSize) const -> rtc::camera
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

 protected:
  void initializeGL() override
  {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glClearColor(0.05F, 0.06F, 0.07F, 1.0F);
  }

  void resizeGL(int width, int height) override
  {
    glViewport(0, 0, width, std::max(1, height));
  }

  void paintGL() override
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

      glEnable(GL_CULL_FACE);
      glEnable(GL_DEPTH_TEST);
      glEnable(GL_LIGHTING);
      return;
    }

    setupProjection();
    setupCamera();
    drawScene();
  }

  void mousePressEvent(QMouseEvent* event) override
  {
    lastMouse_ = event->pos();
    setFocus();
  }

  void mouseMoveEvent(QMouseEvent* event) override
  {
    const auto delta = event->pos() - lastMouse_;
    lastMouse_ = event->pos();
    renderedImage_ = {};

    if (event->buttons() & Qt::LeftButton)
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

  void wheelEvent(QWheelEvent* event) override
  {
    renderedImage_ = {};
    const auto steps = event->angleDelta().y() / 120.0F;
    distance_ *= std::pow(0.86F, steps);
    distance_ = std::clamp(distance_, radius_ * 0.03F, radius_ * 40.0F);
    update();
  }

  void keyPressEvent(QKeyEvent* event) override
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

 private:
  void fitCameraToScene()
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

  void fitRadiusToScene()
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

  auto eyePosition() const -> Vec3
  {
    const auto yaw = yaw_ * pi / 180.0F;
    const auto pitch = pitch_ * pi / 180.0F;

    return {
        center_.x + distance_ * std::cos(pitch) * std::sin(yaw),
        center_.y + distance_ * std::sin(pitch),
        center_.z + distance_ * std::cos(pitch) * std::cos(yaw),
    };
  }

  struct CameraBasis
  {
    Vec3 eye{};
    Vec3 forward{};
    Vec3 right{};
    Vec3 up{};
  };

  auto cameraBasis() const -> CameraBasis
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

  void setupProjection()
  {
    const auto aspect = std::max(1, width()) / static_cast<float>(std::max(1, height()));
    const auto nearPlane = std::max(0.01F, distance_ - radius_ * 4.0F);
    const auto farPlane = distance_ + radius_ * 8.0F;
    const auto top = std::tan(45.0F * pi / 360.0F) * nearPlane;
    const auto right = top * aspect;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-right, right, -top, top, nearPlane, farPlane);
  }

  void setupCamera()
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

  void drawScene()
  {
    if (!scene_)
      return;

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
};

class MainWindow final : public QMainWindow
{
 public:
  MainWindow()
  {
    auto* central = new QWidget(this);
    auto* layout = new QVBoxLayout(central);
    layout->setContentsMargins(0, 0, 0, 0);
    view_ = new SceneView(central);
    layout->addWidget(view_);
    setCentralWidget(central);

    auto* toolbar = addToolBar("Scene");
    toolbar->setMovable(false);
    auto* loadAction = toolbar->addAction("Load BRS");
    auto* renderAction = toolbar->addAction("Render");
    auto* resetAction = toolbar->addAction("Reset Camera");
    auto* previewAction = toolbar->addAction("OpenGL Preview");
    auto* wireframe = new QCheckBox("Wireframe", this);
    toolbar->addWidget(wireframe);

    statusBar()->showMessage("Load a BRS XML scene to begin.");

    connect(loadAction, &QAction::triggered, this, [this] { loadScene(); });
    connect(renderAction, &QAction::triggered, this, [this] { startRender(); });
    connect(resetAction, &QAction::triggered, view_, [this] { view_->resetCamera(); });
    connect(previewAction, &QAction::triggered, view_, [this] { view_->showOpenGlPreview(); });
    connect(wireframe, &QCheckBox::toggled, view_, &SceneView::setWireframe);

    renderPoll_.setInterval(100);
    connect(&renderPoll_, &QTimer::timeout, this, [this] { pollRender(); });

    resize(1100, 760);
  }

 private:
  void loadScene()
  {
    const auto path = QFileDialog::getOpenFileName(
        this,
        "Load BRS scene",
        QString::fromStdString(std::string{"assets/brs_xml"}),
        "BRS XML scenes (*.xml);;All files (*)");

    if (path.isEmpty())
      return;

    try
    {
      auto scene = std::make_shared<rtc::brs>(path.toStdString());
      view_->setScene(scene);
      setWindowTitle("RTC GUI - " + QFileInfo(path).fileName());
      statusBar()->showMessage(QString("Loaded %1 triangles, %2 lights")
                                   .arg(static_cast<qulonglong>(scene->triangles.size()))
                                   .arg(static_cast<qulonglong>(scene->lights.size())));
    }
    catch (const std::exception& e)
    {
      statusBar()->showMessage(QString("Load failed: %1").arg(e.what()));
    }
  }

  void startRender()
  {
    if (!view_->hasScene())
    {
      statusBar()->showMessage("Load a scene before rendering.");
      return;
    }

    if (renderFuture_.valid())
    {
      statusBar()->showMessage("Render already running.");
      return;
    }

    const auto source = view_->scene();
    auto renderScene = std::make_shared<rtc::scene_model>(*source);
    const QSize renderSize{std::clamp(view_->width(), 64, 1600), std::clamp(view_->height(), 64, 1200)};
    renderScene->optical_system = view_->cameraForRender(renderSize);

    statusBar()->showMessage(QString("Rendering %1 x %2 with rt_service...")
                                 .arg(renderSize.width())
                                 .arg(renderSize.height()));

    renderFuture_ = std::async(std::launch::async, [renderScene] {
      rtc::render_engine<> engine{renderScene};
      return engine.bitmap();
    });
    renderPoll_.start();
  }

  void pollRender()
  {
    if (!renderFuture_.valid())
    {
      renderPoll_.stop();
      return;
    }

    if (renderFuture_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      return;

    renderPoll_.stop();
    try
    {
      auto bitmap = renderFuture_.get();
      auto image = bitmap_to_image(bitmap);
      const auto nonBlack = non_black_pixels(image);
      view_->showRenderedImage(std::move(image));
      statusBar()->showMessage(QString("Rendered %1 x %2 image, %3 non-black pixels")
                                   .arg(static_cast<qulonglong>(bitmap.width()))
                                   .arg(static_cast<qulonglong>(bitmap.height()))
                                   .arg(static_cast<qulonglong>(nonBlack)));
    }
    catch (const std::exception& e)
    {
      statusBar()->showMessage(QString("Render failed: %1").arg(e.what()));
    }
  }

  SceneView* view_{};
  QTimer renderPoll_{this};
  std::future<rtc::bitmap> renderFuture_{};
};

int main(int argc, char** argv)
{
  QSurfaceFormat format;
  format.setVersion(2, 1);
  format.setProfile(QSurfaceFormat::CompatibilityProfile);
  QSurfaceFormat::setDefaultFormat(format);

  QApplication app(argc, argv);
  MainWindow window;
  window.show();
  return app.exec();
}
