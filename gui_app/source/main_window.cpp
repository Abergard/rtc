#include "main_window.hpp"

#include <QAction>
#include <QCheckBox>
#include <QCloseEvent>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QLabel>
#include <QProgressBar>
#include <QSpinBox>
#include <QStatusBar>
#include <QToolBar>
#include <QVBoxLayout>

#include <boost/property_tree/xml_parser.hpp>

#include <algorithm>
#include <atomic>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "brs.hpp"
#include "collada_format.hpp"
#include "image_conversion.hpp"
#include "light.hpp"
#include "render_engine.hpp"

namespace rtc_gui
{
namespace
{
auto bool_text(const bool value) -> const char*
{
  return value ? "true" : "false";
}

template <typename T>
void put_material_param(boost::property_tree::ptree& attributeNode, const std::string& name, const T& value)
{
  for (auto& node : attributeNode)
  {
    if (node.first == "p" && node.second.get<std::string>("<xmlattr>.name", {}) == name)
    {
      node.second.data() = std::to_string(value);
      return;
    }
  }
}

void put_material_param_text(boost::property_tree::ptree& attributeNode, const std::string& name, const std::string& value)
{
  for (auto& node : attributeNode)
  {
    if (node.first == "p" && node.second.get<std::string>("<xmlattr>.name", {}) == name)
    {
      node.second.data() = value;
      return;
    }
  }
}

auto make_color_node(const rtc::color& color) -> boost::property_tree::ptree
{
  boost::property_tree::ptree colorNode;
  colorNode.put("<xmlattr>.model", "RGB");
  colorNode.put("<xmlattr>.min", "0.0");
  colorNode.put("<xmlattr>.max", "1.0");
  colorNode.put("r", color.red());
  colorNode.put("g", color.green());
  colorNode.put("b", color.blue());
  return colorNode;
}

auto make_light_node(const rtc::light& light, const std::size_t lightIndex) -> boost::property_tree::ptree
{
  boost::property_tree::ptree lightNode;
  lightNode.put("<xmlattr>.name", QString("L%1").arg(lightIndex + 1, 6, 10, QChar('0')).toStdString());
  lightNode.put("<xmlattr>.type", "spherical");
  lightNode.put("position.<xmlattr>.x", light.position.x());
  lightNode.put("position.<xmlattr>.y", light.position.y());
  lightNode.put("position.<xmlattr>.z", light.position.z());
  lightNode.add_child("color", make_color_node(light.light_color));
  lightNode.put("radius", light.radius);
  lightNode.put("sampling", light.sampling);
  return lightNode;
}
}  // namespace

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
  auto* saveAction = toolbar->addAction("Save BRS");
  auto* renderAction = toolbar->addAction("Render");
  auto* resetAction = toolbar->addAction("Reset Camera");
  auto* previewAction = toolbar->addAction("OpenGL Preview");
  auto* flipSelectedAction = toolbar->addAction("Flip Selected");
  auto* fixVisibleNormalsAction = toolbar->addAction("Fix Visible Normals");
  auto* editMaterialAction = toolbar->addAction("Edit Material");
  auto* addLightAction = toolbar->addAction("Add Light");
  auto* editLightAction = toolbar->addAction("Edit Light");
  auto* removeLightAction = toolbar->addAction("Remove Light");
  auto* wireframe = new QCheckBox("Wireframe", this);
  auto* normals = new QCheckBox("Normals", this);
  auto* backfaceCulling = new QCheckBox("Cull backfaces", this);
  backfaceCulling->setChecked(true);
  toolbar->addWidget(wireframe);
  toolbar->addWidget(normals);
  toolbar->addWidget(backfaceCulling);

  statusBar()->showMessage("Load a BRS XML scene to begin.");

  connect(loadAction, &QAction::triggered, this, [this] { loadScene(); });
  connect(saveAction, &QAction::triggered, this, [this] { saveScene(); });
  connect(renderAction, &QAction::triggered, this, [this] { startRender(); });
  connect(resetAction, &QAction::triggered, view_, [this] { view_->resetCamera(); });
  connect(previewAction, &QAction::triggered, view_, [this] { view_->showOpenGlPreview(); });
  connect(flipSelectedAction, &QAction::triggered, this, [this] { flipSelectedTriangle(); });
  connect(fixVisibleNormalsAction, &QAction::triggered, this, [this] { fixVisibleTriangleNormals(); });
  connect(editMaterialAction, &QAction::triggered, this, [this] { editSelectedMaterial(); });
  connect(addLightAction, &QAction::triggered, this, [this] { addLight(); });
  connect(editLightAction, &QAction::triggered, this, [this] { editSelectedLight(); });
  connect(removeLightAction, &QAction::triggered, this, [this] { removeSelectedLight(); });
  connect(wireframe, &QCheckBox::toggled, view_, &SceneView::setWireframe);
  connect(normals, &QCheckBox::toggled, view_, &SceneView::setShowNormals);
  connect(backfaceCulling, &QCheckBox::toggled, view_, &SceneView::setBackfaceCulling);

  renderProgressBar_ = new QProgressBar(this);
  renderProgressBar_->setRange(0, 100);
  renderProgressBar_->setValue(0);
  renderProgressBar_->setTextVisible(true);
  renderProgressBar_->setVisible(false);
  statusBar()->addPermanentWidget(renderProgressBar_, 1);

  renderPoll_.setInterval(100);
  connect(&renderPoll_, &QTimer::timeout, this, [this] { pollRender(); });

  resize(1100, 760);
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  if (renderProgress_)
    renderProgress_->request_cancel();

  renderPoll_.stop();
  QMainWindow::closeEvent(event);
}

void MainWindow::loadScene()
{
  const auto path = QFileDialog::getOpenFileName(
      this,
      "Load scene",
      QString::fromStdString(std::string{"assets"}),
      "Scene files (*.xml *.dae *.blend);;BRS XML scenes (*.xml);;Blender files (*.blend);;Collada files (*.dae);;All files (*)");

  if (path.isEmpty())
    return;

  try
  {
    const QFileInfo fileInfo{path};
    const auto suffix = fileInfo.suffix().toLower();
    std::shared_ptr<rtc::scene_model> scene;

    if (suffix == "xml")
    {
      scene = std::make_shared<rtc::brs>(path.toStdString());
      loadedSceneIsBrs_ = true;
    }
    else if (suffix == "dae" || suffix == "blend")
    {
      scene = std::make_shared<rtc::collada_format>(path.toStdString());
      loadedSceneIsBrs_ = false;
    }
    else
    {
      throw std::runtime_error{"Unsupported scene file extension."};
    }

    view_->setScene(scene);
    loadedScenePath_ = path;
    setWindowTitle("RTC GUI - " + fileInfo.fileName());
    statusBar()->showMessage(QString("Loaded %1 triangles, %2 lights%3")
                                 .arg(static_cast<qulonglong>(scene->triangles.size()))
                                 .arg(static_cast<qulonglong>(scene->lights.size()))
                                 .arg(loadedSceneIsBrs_ ? "" : " (view/edit only; Save BRS is disabled)"));
  }
  catch (const std::exception& e)
  {
    statusBar()->showMessage(QString("Load failed: %1").arg(e.what()));
  }
}

void MainWindow::saveScene()
{
  if (loadedScenePath_.isEmpty() || !view_->hasScene())
  {
    statusBar()->showMessage("Load a BRS file before saving.");
    return;
  }

  if (!loadedSceneIsBrs_)
  {
    statusBar()->showMessage("Saving is supported only for loaded BRS XML files.");
    return;
  }

  try
  {
    using boost::property_tree::ptree;

    const auto scene = view_->scene();
    ptree tree;
    read_xml(loadedScenePath_.toStdString(), tree);
    auto& trianglesNode = tree.get_child("model.triangles");

    std::size_t triangleIndex{};
    for (auto& node : trianglesNode)
    {
      if (node.first != "triangle")
        continue;

      if (triangleIndex >= scene->triangles.size())
        throw std::runtime_error{"XML contains fewer in-memory triangles than expected."};

      const auto& triangle = scene->triangles[triangleIndex++];
      node.second.put("<xmlattr>.v1", triangle.vertex_a());
      node.second.put("<xmlattr>.v2", triangle.vertex_b());
      node.second.put("<xmlattr>.v3", triangle.vertex_c());
    }

    if (triangleIndex != scene->triangles.size())
      throw std::runtime_error{"XML triangle count does not match the loaded scene."};

    auto& attributesNode = tree.get_child("model.attributes");
    std::size_t materialIndex{};
    for (auto& node : attributesNode)
    {
      if (node.first != "attribute")
        continue;

      if (materialIndex >= scene->materials.size())
        throw std::runtime_error{"XML contains fewer material attributes than expected."};

      const auto& material = scene->materials[materialIndex++];
      put_material_param(node.second, "kd", material.kd);
      put_material_param(node.second, "ks", material.ks);
      put_material_param(node.second, "kts", material.kts);
      put_material_param(node.second, "ktd", material.ktd);
      put_material_param(node.second, "ka", material.ka);
      put_material_param(node.second, "kf", material.kf);
      put_material_param(node.second, "gs", material.gs);
      put_material_param(node.second, "gm", material.gm);
      put_material_param(node.second, "selfLuminance", material.selfLuminance);
      put_material_param(node.second, "heatLam", material.heatLam);
      put_material_param(node.second, "HeatLam", material.heatLam);
      put_material_param(node.second, "ft", material.ft);
      put_material_param(node.second, "eta", material.eta);
      put_material_param_text(node.second, "oneSheet", bool_text(material.oneSheet));
      put_material_param_text(node.second, "reflection", bool_text(material.reflection));
      put_material_param_text(node.second, "indirect", bool_text(material.indirect));
      put_material_param_text(node.second, "mirror", bool_text(material.mirror));
      put_material_param_text(node.second, "shadowcast", bool_text(material.shadowcast));
      put_material_param_text(node.second, "shadowfall", bool_text(material.shadowfall));

      auto& colorNode = node.second.get_child("color");
      colorNode.put("r", material.material_color.red());
      colorNode.put("g", material.material_color.green());
      colorNode.put("b", material.material_color.blue());
    }

    if (materialIndex != scene->materials.size())
      throw std::runtime_error{"XML material count does not match the loaded scene."};

    auto& lightsNode = tree.get_child("model.lights");
    lightsNode.put("<xmlattr>.size", scene->lights.size());
    boost::property_tree::ptree rewrittenLights;

    for (const auto& node : lightsNode)
    {
      if (node.first == "ambient")
      {
        rewrittenLights.push_back(node);
        break;
      }
    }

    for (std::size_t i = 0; i < scene->lights.size(); ++i)
      rewrittenLights.push_back({"light", make_light_node(scene->lights[i], i)});

    lightsNode.clear();
    lightsNode.put("<xmlattr>.size", scene->lights.size());
    for (const auto& node : rewrittenLights)
      lightsNode.push_back(node);

    boost::property_tree::xml_writer_settings<std::string> settings{' ', 2};
    write_xml(loadedScenePath_.toStdString(), tree, std::locale{}, settings);
    statusBar()->showMessage(
        QString("Saved triangle winding, materials, and lights to %1").arg(QFileInfo(loadedScenePath_).fileName()));
  }
  catch (const std::exception& e)
  {
    statusBar()->showMessage(QString("Save failed: %1").arg(e.what()));
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

  renderProgress_ = std::make_shared<rtc::rt_render_progress>();
  renderProgress_->reset(0);
  renderProgressBar_->setRange(0, 100);
  renderProgressBar_->setValue(0);
  renderProgressBar_->setVisible(true);

  renderFuture_ = std::async(std::launch::async, [renderScene, progress = renderProgress_] {
    rtc::render_engine<> engine{renderScene, progress};
    return engine.bitmap();
  });
  renderPoll_.start();
}

void MainWindow::pollRender()
{
  if (!renderFuture_.valid())
  {
    renderPoll_.stop();
    renderProgressBar_->setVisible(false);
    return;
  }

  if (renderProgress_)
  {
    const auto total = renderProgress_->total_tiles.load(std::memory_order_relaxed);
    const auto completed = renderProgress_->completed_tiles.load(std::memory_order_relaxed);
    const auto pixels = renderProgress_->processed_pixels.load(std::memory_order_relaxed);
    const auto tileTimeUs = renderProgress_->tile_time_us.load(std::memory_order_relaxed);

    if (total > 0)
    {
      const auto percent = static_cast<int>((100 * completed) / total);
      const auto avgTileMs = completed > 0 ? static_cast<double>(tileTimeUs) / (1000.0 * completed) : 0.0;
      const auto avgPixelUs = pixels > 0 ? static_cast<double>(tileTimeUs) / pixels : 0.0;
      renderProgressBar_->setValue(std::clamp(percent, 0, 100));
      renderProgressBar_->setFormat(QString("%1 / %2 tiles, %3 ms/tile, %4 us/pixel")
                                        .arg(completed)
                                        .arg(total)
                                        .arg(avgTileMs, 0, 'f', 2)
                                        .arg(avgPixelUs, 0, 'f', 2));
    }
    else
    {
      renderProgressBar_->setValue(0);
      renderProgressBar_->setFormat("Preparing render...");
    }
  }

  if (renderFuture_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    return;

  renderPoll_.stop();
  try
  {
    auto bitmap = renderFuture_.get();
    renderProgressBar_->setValue(100);
    renderProgressBar_->setVisible(false);
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
    renderProgressBar_->setVisible(false);
    statusBar()->showMessage(QString("Render failed: %1").arg(e.what()));
  }

  renderProgress_.reset();
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

void MainWindow::fixVisibleTriangleNormals()
{
  if (!view_->hasScene())
  {
    statusBar()->showMessage("Load a scene before fixing normals.");
    return;
  }

  const auto result = view_->fixVisibleTriangleNormals();
  statusBar()->showMessage(QString("Visible triangles: %1, flipped normals: %2. Use Save BRS to persist.")
                               .arg(static_cast<qulonglong>(result.visibleTriangles))
                               .arg(static_cast<qulonglong>(result.flippedTriangles)));
}

void MainWindow::editSelectedMaterial()
{
  const auto materialIndex = view_->selectedMaterial();
  if (!materialIndex)
  {
    statusBar()->showMessage("Select a triangle first with Shift + left click.");
    return;
  }

  auto scene = view_->scene();
  auto& material = scene->materials[*materialIndex];

  QDialog dialog(this);
  dialog.setWindowTitle(QString("Material %1").arg(static_cast<qulonglong>(*materialIndex)));

  auto* layout = new QFormLayout(&dialog);
  layout->addRow("Material index", new QLabel(QString::number(*materialIndex), &dialog));

  const auto addFloat = [&](const QString& label, const float value) {
    auto* input = new QDoubleSpinBox(&dialog);
    input->setDecimals(4);
    input->setRange(-100000.0, 100000.0);
    input->setSingleStep(0.1);
    input->setValue(value);
    layout->addRow(label, input);
    return input;
  };

  const auto addColor = [&](const QString& label, const float value) {
    auto* input = new QSpinBox(&dialog);
    input->setRange(0, 255);
    input->setValue(static_cast<int>(std::clamp(value, 0.0F, 255.0F)));
    layout->addRow(label, input);
    return input;
  };

  const auto addBool = [&](const QString& label, const bool value) {
    auto* input = new QCheckBox(&dialog);
    input->setChecked(value);
    layout->addRow(label, input);
    return input;
  };

  auto* red = addColor("Red", material.material_color.red());
  auto* green = addColor("Green", material.material_color.green());
  auto* blue = addColor("Blue", material.material_color.blue());
  auto* kd = addFloat("kd", material.kd);
  auto* ks = addFloat("ks", material.ks);
  auto* kts = addFloat("kts", material.kts);
  auto* ktd = addFloat("ktd", material.ktd);
  auto* ka = addFloat("ka", material.ka);
  auto* kf = addFloat("kf", material.kf);
  auto* gs = addFloat("gs", material.gs);
  auto* gm = addFloat("gm", material.gm);
  auto* selfLuminance = addFloat("selfLuminance", material.selfLuminance);
  auto* heatLam = addFloat("heatLam", material.heatLam);
  auto* ft = addFloat("ft", material.ft);
  auto* eta = addFloat("eta", material.eta);
  auto* oneSheet = addBool("oneSheet", material.oneSheet);
  auto* reflection = addBool("reflection", material.reflection);
  auto* indirect = addBool("indirect", material.indirect);
  auto* mirror = addBool("mirror", material.mirror);
  auto* shadowcast = addBool("shadowcast", material.shadowcast);
  auto* shadowfall = addBool("shadowfall", material.shadowfall);

  auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
  layout->addRow(buttons);
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

  if (dialog.exec() != QDialog::Accepted)
    return;

  material.material_color.red() = static_cast<float>(red->value());
  material.material_color.green() = static_cast<float>(green->value());
  material.material_color.blue() = static_cast<float>(blue->value());
  material.kd = static_cast<float>(kd->value());
  material.ks = static_cast<float>(ks->value());
  material.kts = static_cast<float>(kts->value());
  material.ktd = static_cast<float>(ktd->value());
  material.ka = static_cast<float>(ka->value());
  material.kf = static_cast<float>(kf->value());
  material.gs = static_cast<float>(gs->value());
  material.gm = static_cast<float>(gm->value());
  material.selfLuminance = static_cast<float>(selfLuminance->value());
  material.heatLam = static_cast<float>(heatLam->value());
  material.ft = static_cast<float>(ft->value());
  material.eta = static_cast<float>(eta->value());
  material.oneSheet = oneSheet->isChecked();
  material.reflection = reflection->isChecked();
  material.indirect = indirect->isChecked();
  material.mirror = mirror->isChecked();
  material.shadowcast = shadowcast->isChecked();
  material.shadowfall = shadowfall->isChecked();

  view_->showOpenGlPreview();
  statusBar()->showMessage(QString("Edited material %1. Use Save BRS to persist.")
                               .arg(static_cast<qulonglong>(*materialIndex)));
}

void MainWindow::addLight()
{
  if (!view_->hasScene())
  {
    statusBar()->showMessage("Load a scene before adding a light.");
    return;
  }

  const auto index = view_->addLightAtCameraTarget();
  statusBar()->showMessage(QString("Added light %1 at camera target. Use Edit Light to tune it.")
                               .arg(static_cast<qulonglong>(index)));
}

void MainWindow::editSelectedLight()
{
  const auto selectedLight = view_->selectedLight();
  if (!selectedLight)
  {
    statusBar()->showMessage("Select a light first with Ctrl + left click.");
    return;
  }

  auto scene = view_->scene();
  if (*selectedLight >= scene->lights.size())
  {
    statusBar()->showMessage("Selected light is no longer available.");
    return;
  }

  auto& light = scene->lights[*selectedLight];
  QDialog dialog(this);
  dialog.setWindowTitle(QString("Light %1").arg(static_cast<qulonglong>(*selectedLight)));
  auto* layout = new QFormLayout(&dialog);

  const auto addFloat = [&](const QString& label, const float value) {
    auto* input = new QDoubleSpinBox(&dialog);
    input->setDecimals(4);
    input->setRange(-100000.0, 100000.0);
    input->setSingleStep(0.1);
    input->setValue(value);
    layout->addRow(label, input);
    return input;
  };

  const auto addPositiveFloat = [&](const QString& label, const float value) {
    auto* input = addFloat(label, value);
    input->setRange(0.0, 100000.0);
    return input;
  };

  auto* x = addFloat("Position X", light.position.x());
  auto* y = addFloat("Position Y", light.position.y());
  auto* z = addFloat("Position Z", light.position.z());
  auto* red = addPositiveFloat("Red", light.light_color.red());
  auto* green = addPositiveFloat("Green", light.light_color.green());
  auto* blue = addPositiveFloat("Blue", light.light_color.blue());
  auto* radius = addPositiveFloat("Radius", light.radius);
  auto* sampling = new QSpinBox(&dialog);
  sampling->setRange(1, 100000);
  sampling->setValue(static_cast<int>(std::max<std::uint32_t>(1, light.sampling)));
  layout->addRow("Sampling", sampling);

  auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
  layout->addRow(buttons);
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

  if (dialog.exec() != QDialog::Accepted)
    return;

  light.position.x() = static_cast<float>(x->value());
  light.position.y() = static_cast<float>(y->value());
  light.position.z() = static_cast<float>(z->value());
  light.light_color.red() = static_cast<float>(red->value());
  light.light_color.green() = static_cast<float>(green->value());
  light.light_color.blue() = static_cast<float>(blue->value());
  light.radius = static_cast<float>(radius->value());
  light.sampling = static_cast<std::uint32_t>(sampling->value());

  view_->showOpenGlPreview();
  statusBar()->showMessage(QString("Edited light %1. Use Save BRS to persist.")
                               .arg(static_cast<qulonglong>(*selectedLight)));
}

void MainWindow::removeSelectedLight()
{
  if (view_->removeSelectedLight())
  {
    statusBar()->showMessage("Removed selected light. Use Save BRS to persist.");
  }
  else
  {
    statusBar()->showMessage("Select a light first with Ctrl + left click.");
  }
}
}  // namespace rtc_gui
