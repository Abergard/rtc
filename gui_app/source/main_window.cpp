#include "main_window.hpp"

#include <QAction>
#include <QCheckBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QStatusBar>
#include <QToolBar>
#include <QVBoxLayout>

#include <algorithm>
#include <memory>

#include "brs.hpp"
#include "image_conversion.hpp"
#include "render_engine.hpp"

namespace rtc_gui
{
MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent)
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
  auto* flipSelectedAction = toolbar->addAction("Flip Selected");
  auto* wireframe = new QCheckBox("Wireframe", this);
  auto* normals = new QCheckBox("Normals", this);
  auto* backfaceCulling = new QCheckBox("Cull backfaces", this);
  backfaceCulling->setChecked(true);
  toolbar->addWidget(wireframe);
  toolbar->addWidget(normals);
  toolbar->addWidget(backfaceCulling);

  statusBar()->showMessage("Load a BRS XML scene to begin.");

  connect(loadAction, &QAction::triggered, this, [this] { loadScene(); });
  connect(renderAction, &QAction::triggered, this, [this] { startRender(); });
  connect(resetAction, &QAction::triggered, view_, [this] { view_->resetCamera(); });
  connect(previewAction, &QAction::triggered, view_, [this] { view_->showOpenGlPreview(); });
  connect(flipSelectedAction, &QAction::triggered, this, [this] { flipSelectedTriangle(); });
  connect(wireframe, &QCheckBox::toggled, view_, &SceneView::setWireframe);
  connect(normals, &QCheckBox::toggled, view_, &SceneView::setShowNormals);
  connect(backfaceCulling, &QCheckBox::toggled, view_, &SceneView::setBackfaceCulling);

  renderPoll_.setInterval(100);
  connect(&renderPoll_, &QTimer::timeout, this, [this] { pollRender(); });

  resize(1100, 760);
}

void MainWindow::loadScene()
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

void MainWindow::startRender()
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

void MainWindow::pollRender()
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

void MainWindow::flipSelectedTriangle()
{
  if (view_->flipSelectedTriangle())
  {
    const auto selected = view_->selectedTriangle();
    statusBar()->showMessage(QString("Flipped triangle %1").arg(static_cast<qulonglong>(*selected)));
  }
  else
  {
    statusBar()->showMessage("Select a triangle first with Shift + left click.");
  }
}
}  // namespace rtc_gui
