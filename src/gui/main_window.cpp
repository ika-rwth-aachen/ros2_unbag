// MIT License
//
// Copyright (c) 2026 Institute for Automotive Engineering (ika), RWTH Aachen University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "main_window.hpp"

#include "bridge_client.hpp"
#include "processor_chain_widget.hpp"

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDialog>
#include <QDir>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFileSystemModel>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QLabel>
#include <QLineEdit>
#include <QMap>
#include <QMessageBox>
#include <QMovie>
#include <QBrush>
#include <QColor>
#include <QFont>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <QScreen>
#include <QShowEvent>
#include <QSlider>
#include <QSortFilterProxyModel>
#include <QSplitter>
#include <QStackedWidget>
#include <QStatusBar>
#include <QStyle>
#include <QPixmap>
#include <QResizeEvent>
#include <QTextStream>
#include <QTimer>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>

namespace {

/**
 * @brief Keep ROS bag files visible but unselectable in the bag folder picker.
 */
class BagFolderProxyModel : public QSortFilterProxyModel {
public:
  using QSortFilterProxyModel::QSortFilterProxyModel;

  /**
   * @brief Return item flags with selection disabled for bag database files.
   * @param index Proxy-model item whose flags should be queried.
   * @return Base item flags, with enabled and selectable flags cleared for bags.
   */
  Qt::ItemFlags flags(const QModelIndex &index) const override {
    Qt::ItemFlags itemFlags = QSortFilterProxyModel::flags(index);
    const QModelIndex sourceIndex = mapToSource(index);
    const QAbstractItemModel *source = sourceModel();
    if (!sourceIndex.isValid() || source == nullptr) {
      return itemFlags;
    }

    const QFileSystemModel *fileSystemModel = qobject_cast<const QFileSystemModel *>(source);
    if (fileSystemModel == nullptr) {
      return itemFlags;
    }

    // Keep bag files visible in the dialog, but prevent selecting them so the
    // picker behaves like the previous GUI folder chooser.
    if (!fileSystemModel->isDir(sourceIndex)) {
      const QString suffix = QFileInfo(fileSystemModel->filePath(sourceIndex)).suffix().toLower();
      if (suffix == QStringLiteral("db3") || suffix == QStringLiteral("mcap")) {
        itemFlags &= ~Qt::ItemIsEnabled;
        itemFlags &= ~Qt::ItemIsSelectable;
      }
    }

    return itemFlags;
  }
};

/**
 * @brief Extract the format name before an optional mode suffix.
 * @param format Export format, optionally followed by an at-sign and mode.
 * @return Format name without its mode suffix.
 */
QString splitFormatBase(const QString &format) {
  const int idx = format.lastIndexOf('@');
  if (idx < 0) {
    return format;
  }
  return format.left(idx);
}

/**
 * @brief Extract the mode suffix from an export format.
 * @param format Export format, optionally followed by an at-sign and mode.
 * @return Mode suffix, or an empty string when the format has no suffix.
 */
QString splitFormatMode(const QString &format) {
  const int idx = format.lastIndexOf('@');
  if (idx < 0) {
    return {};
  }
  return format.mid(idx + 1);
}

/**
 * @brief Shorten the middle of a topic path to fit the target label.
 * @param path Full topic path to display.
 * @param label Target label providing the font and available width; may be null.
 * @return Elided path, or the original path when no label is supplied.
 */
QString elideTopicPath(const QString &path, const QLabel *label) {
  if (label == nullptr) {
    return path;
  }

  // Match the old GUI behavior by preserving both ends of long paths.
  const QFontMetrics metrics = label->fontMetrics();
  return metrics.elidedText(path, Qt::ElideMiddle, qMax(40, label->width()));
}

/**
 * @brief Apply the selected-topic background to all columns of a tree item.
 * @param item Topic tree item to update; null is ignored.
 * @param checked Whether the topic is selected for export.
 */
void applyCheckedStyle(QTreeWidgetItem *item, bool checked) {
  if (item == nullptr) {
    return;
  }
  const QBrush checkedBrush(QColor(QStringLiteral("#e9fcdc")));
  const QBrush defaultBrush;
  for (int column = 0; column < item->columnCount(); ++column) {
    item->setBackground(column, checked ? checkedBrush : defaultBrush);
  }
}

}  // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      bridge_(new BridgeClient(this)),
      middleScrollArea_(nullptr),
      loadingMovie_(nullptr),
      placeholderPixmap_(new QPixmap(QStringLiteral(":/assets/title.png"))),
      feedbackTimer_(nullptr) {
  const QScreen *screen = QGuiApplication::primaryScreen();
  if (screen != nullptr) {
    const QRect available = screen->availableGeometry();
    resize(int(available.width() * 0.8), int(available.height() * 0.8));
  } else {
    resize(1200, 800);
  }
  setMinimumSize(900, 600);
  setWindowTitle(QStringLiteral("ros2 unbag"));

  buildUi();
  applyStyleSheet();
  showTopicPlaceholder();
  setExportRunning(false);

  connect(bridge_, &BridgeClient::exportProgress, this, &MainWindow::handleExportProgress);
  connect(bridge_, &BridgeClient::exportCompleted, this, &MainWindow::handleExportCompleted);
  connect(bridge_, &BridgeClient::exportError, this, &MainWindow::handleExportError);
}

void MainWindow::buildUi() {
  auto *centralWidget = new QWidget(this);
  setCentralWidget(centralWidget);

  auto *rootLayout = new QVBoxLayout(centralWidget);
  rootLayout->setContentsMargins(0, 0, 0, 0);
  rootLayout->setSpacing(8);

  auto *topBar = new QWidget(this);
  topBar->setObjectName(QStringLiteral("topBar"));
  topBar->setFixedHeight(60);
  auto *topLayout = new QHBoxLayout(topBar);
  topLayout->setContentsMargins(12, 0, 12, 0);
  topLayout->setSpacing(10);

  auto *iconLabel = new QLabel(topBar);
  iconLabel->setPixmap(QIcon(QStringLiteral(":/assets/badge.svg")).pixmap(50, 50));
  topLayout->addWidget(iconLabel);

  auto *titleLabel = new QLabel(QStringLiteral("ros2 unbag"), topBar);
  titleLabel->setObjectName(QStringLiteral("titleLabel"));
  topLayout->addWidget(titleLabel);
  topLayout->addStretch();

  loadBagButton_ = new QPushButton(QStringLiteral("Load Bag"), topBar);
  loadBagButton_->setObjectName(QStringLiteral("headerLoadButton"));
  topLayout->addWidget(loadBagButton_);
  rootLayout->addWidget(topBar);

  auto *columnsContainer = new QWidget(this);
  auto *columnsLayout = new QHBoxLayout(columnsContainer);
  columnsLayout->setContentsMargins(10, 10, 10, 10);
  columnsLayout->setSpacing(0);
  rootLayout->addWidget(columnsContainer);

  auto *splitter = new QSplitter(Qt::Horizontal, columnsContainer);
  splitter->setChildrenCollapsible(false);
  splitter->setHandleWidth(6);
  columnsLayout->addWidget(splitter);

  auto *leftContainer = new QWidget(splitter);
  leftContainer->setObjectName(QStringLiteral("leftContainer"));
  auto *leftLayout = new QVBoxLayout(leftContainer);
  leftLayout->setContentsMargins(8, 8, 8, 8);
  leftLayout->setSpacing(10);

  auto *leftHeader = new QLabel(QStringLiteral("Bag"), leftContainer);
  leftHeader->setProperty("class", QStringLiteral("sectionHeader"));
  leftHeader->setObjectName(QStringLiteral("sectionHeader"));
  leftLayout->addWidget(leftHeader);

  auto *bagGroup = new QGroupBox(QStringLiteral("Bag Source"), leftContainer);
  auto *bagLayout = new QVBoxLayout(bagGroup);
  auto *bagRow = new QHBoxLayout();
  bagNameLabel_ = new QLabel(QStringLiteral("No bag loaded"), bagGroup);
  bagNameLabel_->setWordWrap(true);
  bagRow->addWidget(bagNameLabel_, 1);
  loadBagSecondaryButton_ = new QPushButton(QStringLiteral("Load Bag"), bagGroup);
  bagRow->addWidget(loadBagSecondaryButton_);
  bagLayout->addLayout(bagRow);
  leftLayout->addWidget(bagGroup);

  topicFilterEdit_ = new QLineEdit(leftContainer);
  topicFilterEdit_->setPlaceholderText(QStringLiteral("Filter topics..."));
  leftLayout->addWidget(topicFilterEdit_);

  topicTree_ = new QTreeWidget(leftContainer);
  topicTree_->setHeaderLabels({QStringLiteral("Topic"), QStringLiteral("# Msgs")});
  topicTree_->setColumnCount(2);
  topicTree_->setSelectionMode(QAbstractItemView::SingleSelection);
  topicTree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
  topicTree_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  topicTree_->header()->setStretchLastSection(false);
  topicTree_->setRootIsDecorated(false);
  leftLayout->addWidget(topicTree_, 1);

  auto *buttonRow = new QHBoxLayout();
  selectAllButton_ = new QPushButton(QStringLiteral("All"), leftContainer);
  selectNoneButton_ = new QPushButton(QStringLiteral("None"), leftContainer);
  buttonRow->addWidget(selectAllButton_);
  buttonRow->addWidget(selectNoneButton_);
  leftLayout->addLayout(buttonRow);

  auto *configRow = new QHBoxLayout();
  loadConfigButton_ = new QPushButton(QStringLiteral("Load Config"), leftContainer);
  saveConfigButton_ = new QPushButton(QStringLiteral("Save Config"), leftContainer);
  configRow->addWidget(loadConfigButton_);
  configRow->addWidget(saveConfigButton_);
  leftLayout->addLayout(configRow);

  auto *middleContainer = new QWidget(splitter);
  auto *middleLayout = new QVBoxLayout(middleContainer);
  middleLayout->setContentsMargins(0, 0, 0, 0);
  middleLayout->setSpacing(0);

  auto *scrollArea = new QScrollArea(middleContainer);
  scrollArea->setWidgetResizable(true);
  scrollArea->setFrameShape(QFrame::NoFrame);
  middleLayout->addWidget(scrollArea);
  middleScrollArea_ = scrollArea;
  middleScrollArea_->viewport()->installEventFilter(this);

  middleStack_ = new QStackedWidget(scrollArea);
  scrollArea->setWidget(middleStack_);

  auto *loadingPage = new QWidget(middleStack_);
  loadingPage->setObjectName(QStringLiteral("loadingPage"));
  auto *loadingLayout = new QVBoxLayout(loadingPage);
  loadingLayout->setAlignment(Qt::AlignCenter);
  loadingLabel_ = new QLabel(loadingPage);
  loadingLabel_->setAlignment(Qt::AlignCenter);
  loadingLabel_->setMinimumHeight(240);
  loadingMovie_ = new QMovie(QStringLiteral(":/assets/loading.gif"));
  loadingLabel_->setMovie(loadingMovie_);
  loadingPercentLabel_ = new QLabel(QString(), loadingPage);
  loadingPercentLabel_->setAlignment(Qt::AlignCenter);
  loadingLayout->addWidget(loadingLabel_, 0, Qt::AlignCenter);
  loadingLayout->addWidget(loadingPercentLabel_, 0, Qt::AlignCenter);
  middleStack_->addWidget(loadingPage);

  settingsPage_ = new QWidget(middleStack_);
  settingsPage_->setObjectName(QStringLiteral("settingsPage"));
  auto *settingsLayout = new QVBoxLayout(settingsPage_);
  settingsLayout->setAlignment(Qt::AlignTop);

  auto *headerRow = new QHBoxLayout();
  auto *headerBlock = new QVBoxLayout();
  auto *settingsHeader = new QLabel(QStringLiteral("Export Settings"), settingsPage_);
  settingsHeader->setProperty("class", QStringLiteral("sectionHeader"));
  topicLabel_ = new QLabel(QStringLiteral("No Topic Selected"), settingsPage_);
  topicLabel_->setProperty("class", QStringLiteral("topicHeader"));
  headerBlock->addWidget(settingsHeader);
  headerBlock->addWidget(topicLabel_);
  headerRow->addLayout(headerBlock);
  headerRow->addStretch();

  exportBadgeButton_ = new QPushButton(QStringLiteral("•"), settingsPage_);
  exportBadgeButton_->setObjectName(QStringLiteral("exportBadgeButton"));
  exportBadgeButton_->setFixedSize(40, 40);
  headerRow->addWidget(exportBadgeButton_);
  settingsLayout->addLayout(headerRow);

  topicFormWidget_ = new QWidget(settingsPage_);
  auto *formLayout = new QFormLayout(topicFormWidget_);

  auto *formatRow = new QWidget(topicFormWidget_);
  auto *formatLayout = new QHBoxLayout(formatRow);
  formatLayout->setContentsMargins(0, 0, 0, 0);
  formatLayout->setSpacing(8);
  formatCombo_ = new QComboBox(formatRow);
  formatCombo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  formatLayout->addWidget(formatCombo_, 1);
  auto *modeContainer = new QWidget(formatRow);
  modeContainer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  auto *modeLayout = new QHBoxLayout(modeContainer);
  modeLayout->setContentsMargins(0, 0, 0, 0);
  modeLayout->setSpacing(4);
  modeLabel_ = new QLabel(QStringLiteral("Mode"), modeContainer);
  modeLabel_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
  modeCombo_ = new QComboBox(modeContainer);
  modeCombo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  modeCombo_->setMinimumContentsLength(12);
  modeLayout->addWidget(modeLabel_);
  modeLayout->addWidget(modeCombo_);
  modeLayout->setStretch(1, 1);
  formatLayout->addWidget(modeContainer, 1);
  formLayout->addRow(QStringLiteral("Format"), formatRow);

  auto *pathRow = new QWidget(topicFormWidget_);
  auto *pathLayout = new QHBoxLayout(pathRow);
  pathLayout->setContentsMargins(0, 0, 0, 0);
  pathEdit_ = new QLineEdit(pathRow);
  auto *browseButton = new QPushButton(QStringLiteral("Browse"), pathRow);
  pathLayout->addWidget(pathEdit_, 1);
  pathLayout->addWidget(browseButton);
  formLayout->addRow(QStringLiteral("Output Directory"), pathRow);

  subdirEdit_ = new QLineEdit(topicFormWidget_);
  formLayout->addRow(QStringLiteral("Subdirectory"), subdirEdit_);

  namingEdit_ = new QLineEdit(topicFormWidget_);
  formLayout->addRow(QStringLiteral("Naming"), namingEdit_);

  processorChainWidget_ = new ProcessorChainWidget(topicFormWidget_);
  formLayout->addRow(QStringLiteral("Processors"), processorChainWidget_);

  settingsLayout->addWidget(topicFormWidget_);

  placeholderLabel_ = new QLabel(settingsPage_);
  placeholderLabel_->setAlignment(Qt::AlignCenter);
  placeholderLabel_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Expanding);
  placeholderLabel_->setScaledContents(false);
  settingsLayout->addWidget(placeholderLabel_, 0, Qt::AlignCenter);

  placeholderHintLabel_ = new QLabel(QStringLiteral("Please select topic to configure export"), settingsPage_);
  placeholderHintLabel_->setObjectName(QStringLiteral("placeholderHintLabel"));
  placeholderHintLabel_->setAlignment(Qt::AlignCenter);
  settingsLayout->addWidget(placeholderHintLabel_, 0, Qt::AlignCenter);

  settingsLayout->addStretch();
  helpTextLabel_ = new QLabel(
      QStringLiteral("Placeholders:\n"
                     "  %name (topic name)\n"
                     "  %index (msg idx)\n"
                     "  %timestamp (msg timestamp in nanoseconds)\n"
                     "  %master_timestamp (master topic timestamp when resampling)\n"
                     "  %Y-%m-%d_%H-%M-%S (timestamp)"),
      settingsPage_);
  helpTextLabel_->setObjectName(QStringLiteral("helpTextLabel"));
  settingsLayout->addWidget(helpTextLabel_);

  middleStack_->addWidget(settingsPage_);

  auto *globalWrapper = new QWidget(splitter);
  globalWrapper->setObjectName(QStringLiteral("globalContainer"));
  globalWrapper->setFixedWidth(300);
  auto *globalLayout = new QVBoxLayout(globalWrapper);

  settingsGroup_ = new QGroupBox(QStringLiteral("Global Settings"), globalWrapper);
  auto *settingsForm = new QFormLayout(settingsGroup_);
  cpuSlider_ = new QSlider(Qt::Horizontal, settingsGroup_);
  cpuSlider_->setRange(0, 100);
  cpuSlider_->setValue(80);
  cpuSlider_->setSingleStep(10);
  cpuSlider_->setPageStep(10);
  cpuSlider_->setTickInterval(10);
  cpuSlider_->setTickPosition(QSlider::TicksBelow);
  cpuSpin_ = new QDoubleSpinBox(settingsGroup_);
  cpuSpin_->setRange(0.0, 100.0);
  cpuSpin_->setSingleStep(1.0);
  cpuSpin_->setDecimals(1);
  cpuSpin_->setValue(80.0);
  auto *cpuRowWidget = new QWidget(settingsGroup_);
  auto *cpuRow = new QHBoxLayout(cpuRowWidget);
  cpuRow->setContentsMargins(0, 0, 0, 0);
  cpuRow->addWidget(cpuSlider_);
  cpuRow->addWidget(cpuSpin_);
  auto *cpuLabel = new QLabel(QStringLiteral("CPU Usage %"), settingsGroup_);
  cpuLabel->setToolTip(QStringLiteral("Set the maximum CPU percentage allowed for export workers."));
  settingsForm->addRow(cpuLabel, cpuRowWidget);

  assocCombo_ = new QComboBox(settingsGroup_);
  assocCombo_->addItems({QStringLiteral("no resampling"), QStringLiteral("last"), QStringLiteral("nearest")});
  auto *assocLabel = new QLabel(QStringLiteral("Association"), settingsGroup_);
  assocLabel->setToolTip(QStringLiteral("Choose how to align messages to a master timeline, or disable resampling."));
  settingsForm->addRow(assocLabel, assocCombo_);

  epsEdit_ = new QLineEdit(settingsGroup_);
  epsEdit_->setEnabled(false);
  epsEdit_->setPlaceholderText(QStringLiteral("e.g. 0.5"));
  auto *epsLabel = new QLabel(QStringLiteral("Discard Eps (s)"), settingsGroup_);
  epsLabel->setToolTip(QStringLiteral("Discard messages with timestamp offsets larger than this value in seconds."));
  settingsForm->addRow(epsLabel, epsEdit_);

  masterCombo_ = new QComboBox(settingsGroup_);
  masterCombo_->setEnabled(false);
  auto *masterLabel = new QLabel(QStringLiteral("Master Topic"), settingsGroup_);
  masterLabel->setToolTip(QStringLiteral("Select the master topic used as the timing reference when resampling."));
  settingsForm->addRow(masterLabel, masterCombo_);
  globalLayout->addWidget(settingsGroup_);

  baseGroup_ = new QGroupBox(QStringLiteral("Base Directory"), globalWrapper);
  baseGroup_->setToolTip(QStringLiteral("Changes the base directory for all topic exports at once."));
  auto *baseLayout = new QVBoxLayout(baseGroup_);
  auto *baseRow = new QHBoxLayout();
  baseDirLabel_ = new QLabel(QDir::currentPath(), baseGroup_);
  baseDirLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  baseRow->addWidget(baseDirLabel_, 1);
  baseDirButton_ = new QPushButton(QStringLiteral("Change"), baseGroup_);
  baseRow->addWidget(baseDirButton_);
  baseLayout->addLayout(baseRow);
  auto *baseDescription = new QLabel(QStringLiteral("Changes the base directory for all topic exports at once."), baseGroup_);
  baseDescription->setWordWrap(true);
  baseLayout->addWidget(baseDescription);
  globalLayout->addWidget(baseGroup_);

  summaryGroup_ = new QGroupBox(QStringLiteral("Summary"), globalWrapper);
  summaryGroup_->setToolTip(QStringLiteral("Shows how many topics are selected for export out of the total loaded."));
  auto *summaryLayout = new QVBoxLayout(summaryGroup_);
  summaryLabel_ = new QLabel(QStringLiteral("No bag loaded."), summaryGroup_);
  summaryLayout->addWidget(summaryLabel_);
  globalLayout->addWidget(summaryGroup_);

  globalLayout->addStretch();

  feedbackBanner_ = new QFrame(globalWrapper);
  feedbackBanner_->setObjectName(QStringLiteral("feedbackBanner"));
  feedbackBanner_->setProperty("feedbackType", QStringLiteral("success"));
  auto *feedbackLayout = new QHBoxLayout(feedbackBanner_);
  feedbackLabel_ = new QLabel(feedbackBanner_);
  feedbackLayout->addWidget(feedbackLabel_);
  feedbackLayout->addStretch();
  feedbackCloseButton_ = new QToolButton(feedbackBanner_);
  feedbackCloseButton_->setObjectName(QStringLiteral("feedbackCloseButton"));
  feedbackCloseButton_->setText(QStringLiteral("x"));
  feedbackLayout->addWidget(feedbackCloseButton_);
  globalLayout->addWidget(feedbackBanner_);
  feedbackTimer_ = new QTimer(this);
  feedbackTimer_->setSingleShot(true);

  exportButton_ = new QPushButton(QStringLiteral("Unbag"), globalWrapper);
  exportButton_->setObjectName(QStringLiteral("exportButton"));
  exportButton_->setMinimumHeight(56);
  exportButton_->setEnabled(false);
  exportButton_->setToolTip(QStringLiteral("Select at least one topic to unbag."));
  globalLayout->addWidget(exportButton_);

  cancelButton_ = new QPushButton(QStringLiteral("Cancel Export"), globalWrapper);
  cancelButton_->setObjectName(QStringLiteral("cancelButton"));
  cancelButton_->setMinimumHeight(56);
  globalLayout->addWidget(cancelButton_);

  splitter->setSizes({350, 650, 300});

  statusBar_ = new QStatusBar(this);
  setStatusBar(statusBar_);
  statusProgress_ = new QProgressBar(this);
  statusProgress_->setVisible(false);
  statusProgress_->setTextVisible(false);
  statusBar_->addPermanentWidget(statusProgress_, 1);
  statusBar_->showMessage(QStringLiteral("Ready"));

  baseDirectory_ = QDir::currentPath();
  feedbackBanner_->setVisible(false);
  cancelButton_->setVisible(false);
  middleStack_->setCurrentIndex(1);
  updateBaseDirLabel();
  updatePlaceholderPixmap();
  updateStatusProgressWidth();
  QTimer::singleShot(0, this, [this]() { updatePlaceholderPixmap(); });
  QTimer::singleShot(10, this, [this]() { updatePlaceholderPixmap(); });

  connect(loadBagButton_, &QPushButton::clicked, this, &MainWindow::loadBag);
  connect(loadBagSecondaryButton_, &QPushButton::clicked, this, &MainWindow::loadBag);
  connect(topicTree_, &QTreeWidget::itemClicked, this, &MainWindow::handleTopicClicked);
  connect(topicTree_, &QTreeWidget::itemChanged, this, &MainWindow::handleTopicChanged);
  connect(formatCombo_, &QComboBox::currentTextChanged, this, [this]() { handleFormatChanged(); });
  connect(modeCombo_, &QComboBox::currentTextChanged, this, [this]() { handleModeChanged(); });
  connect(baseDirButton_, &QPushButton::clicked, this, &MainWindow::handleBaseDirectoryChange);
  connect(loadConfigButton_, &QPushButton::clicked, this, &MainWindow::loadConfigFile);
  connect(saveConfigButton_, &QPushButton::clicked, this, &MainWindow::saveConfigFile);
  connect(exportButton_, &QPushButton::clicked, this, &MainWindow::startExport);
  connect(cancelButton_, &QPushButton::clicked, this, &MainWindow::cancelExport);
  connect(exportBadgeButton_, &QPushButton::clicked, this, [this]() {
    if (currentTopic_.isEmpty()) {
      return;
    }
    setTopicChecked(currentTopic_, !isTopicChecked(currentTopic_));
  });
  connect(feedbackCloseButton_, &QToolButton::clicked, this, &MainWindow::clearFeedback);
  connect(feedbackTimer_, &QTimer::timeout, this, &MainWindow::clearFeedback);
  connect(topicFilterEdit_, &QLineEdit::textChanged, this, &MainWindow::applyProxyFilter);
  connect(selectAllButton_, &QPushButton::clicked, this, [this]() {
    for (int i = 0; i < topicTree_->topLevelItemCount(); ++i) {
      QTreeWidgetItem *parent = topicTree_->topLevelItem(i);
      for (int j = 0; j < parent->childCount(); ++j) {
        setTopicChecked(parent->child(j)->data(0, Qt::UserRole).toString(), true, true);
      }
    }
    updateSummary();
  });
  connect(selectNoneButton_, &QPushButton::clicked, this, [this]() {
    for (int i = 0; i < topicTree_->topLevelItemCount(); ++i) {
      QTreeWidgetItem *parent = topicTree_->topLevelItem(i);
      for (int j = 0; j < parent->childCount(); ++j) {
        setTopicChecked(parent->child(j)->data(0, Qt::UserRole).toString(), false, true);
      }
    }
    updateSummary();
  });
  connect(browseButton, &QPushButton::clicked, this, [this]() {
    const QString directory = QFileDialog::getExistingDirectory(this, QStringLiteral("Select Directory"), pathEdit_->text());
    if (!directory.isEmpty()) {
      pathEdit_->setText(directory);
      updateValidationStyles();
    }
  });
  connect(cpuSlider_, &QSlider::valueChanged, this, [this](int value) {
    const int snapped = qRound(value / 10.0) * 10;
    if (snapped != value) {
      cpuSlider_->blockSignals(true);
      cpuSlider_->setValue(snapped);
      cpuSlider_->blockSignals(false);
    }
    cpuSpin_->blockSignals(true);
    cpuSpin_->setValue(double(snapped));
    cpuSpin_->blockSignals(false);
  });
  connect(cpuSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double value) {
    cpuSlider_->blockSignals(true);
    cpuSlider_->setValue(int(qRound(value)));
    cpuSlider_->blockSignals(false);
  });
  connect(assocCombo_, &QComboBox::currentTextChanged, this, [this](const QString &text) {
    const bool enabled = text != QStringLiteral("no resampling");
    epsEdit_->setEnabled(enabled);
    if (text == QStringLiteral("nearest") && epsEdit_->text().trimmed().isEmpty()) {
      epsEdit_->setText(QStringLiteral("0.5"));
    }
    updateMasterTopics();
  });
  connect(pathEdit_, &QLineEdit::textChanged, this, [this](const QString &) { updateValidationStyles(); });
  connect(namingEdit_, &QLineEdit::textChanged, this, [this](const QString &) { updateValidationStyles(); });
  connect(pathEdit_, &QLineEdit::editingFinished, this, &MainWindow::saveCurrentTopicConfig);
  connect(subdirEdit_, &QLineEdit::editingFinished, this, &MainWindow::saveCurrentTopicConfig);
  connect(namingEdit_, &QLineEdit::editingFinished, this, &MainWindow::saveCurrentTopicConfig);
  connect(processorChainWidget_, &ProcessorChainWidget::changed, this, &MainWindow::saveCurrentTopicConfig);
  updateUiAvailability();
}

void MainWindow::applyStyleSheet() {
  QFile file(QStringLiteral(":/styles/gui.qss"));
  if (file.open(QIODevice::ReadOnly)) {
    qApp->setStyleSheet(QString::fromUtf8(file.readAll()));
  }
}

bool MainWindow::eventFilter(QObject *watched, QEvent *event) {
  if (middleScrollArea_ != nullptr && watched == middleScrollArea_->viewport() && event->type() == QEvent::Resize &&
      placeholderPixmap_ != nullptr && !placeholderPixmap_->isNull() && placeholderLabel_->isVisible()) {
    QTimer::singleShot(10, this, [this]() { updatePlaceholderPixmap(); });
  }
  return QMainWindow::eventFilter(watched, event);
}

void MainWindow::showEvent(QShowEvent *event) {
  QMainWindow::showEvent(event);
  if (placeholderPixmap_ != nullptr && !placeholderPixmap_->isNull()) {
    QTimer::singleShot(0, this, [this]() { updatePlaceholderPixmap(); });
  }
}

void MainWindow::resizeEvent(QResizeEvent *event) {
  QMainWindow::resizeEvent(event);
  updateBaseDirLabel();
  if (placeholderPixmap_ != nullptr && !placeholderPixmap_->isNull() && placeholderLabel_->isVisible()) {
    QTimer::singleShot(10, this, [this]() { updatePlaceholderPixmap(); });
  }
  updateStatusProgressWidth();
}

void MainWindow::loadBag() {
  QFileDialog dialog(this, QStringLiteral("Open Bag Folder"), baseDirectory_);
  dialog.setOption(QFileDialog::DontUseNativeDialog, true);
  dialog.setFileMode(QFileDialog::Directory);
  dialog.setOption(QFileDialog::ShowDirsOnly, false);
  dialog.setNameFilter(QStringLiteral("ROS bag files (*.db3 *.mcap)"));
  dialog.setProxyModel(new BagFolderProxyModel(&dialog));

  if (dialog.findChild<QWidget *>(QStringLiteral("fileTypeLabel")) != nullptr) {
    dialog.findChild<QWidget *>(QStringLiteral("fileTypeLabel"))->hide();
  }
  if (dialog.findChild<QWidget *>(QStringLiteral("fileTypeCombo")) != nullptr) {
    dialog.findChild<QWidget *>(QStringLiteral("fileTypeCombo"))->hide();
  }

  if (!dialog.exec()) {
    return;
  }

  const QStringList selected = dialog.selectedFiles();
  if (selected.isEmpty()) {
    return;
  }

  bagLoading_ = true;
  updateUiAvailability();
  showStatusProgress(QStringLiteral("Loading bag..."), true);
  QCoreApplication::processEvents();

  QString error;
  QJsonObject response = bridge_->call(
      QStringLiteral("inspect_bag"),
      QJsonObject{
          {QStringLiteral("bag_path"), selected.first()},
          {QStringLiteral("base_dir"), selected.first()},
      },
      &error);

  if (!error.isEmpty()) {
    bagLoading_ = false;
    hideStatusProgress(QStringLiteral("Failed to load bag"));
    QMessageBox::critical(this, QStringLiteral("Invalid Bag Path"), error);
    return;
  }

  setBagData(response);
  bagLoading_ = false;
  hideStatusProgress(QStringLiteral("Loaded %1").arg(QFileInfo(bagPath_).fileName()));
  updateUiAvailability();
}

void MainWindow::handleTopicClicked(QTreeWidgetItem *item, int) {
  if (item == nullptr || item->parent() == nullptr) {
    return;
  }
  loadTopicIntoEditor(item->data(0, Qt::UserRole).toString());
}

void MainWindow::handleTopicChanged(QTreeWidgetItem *item, int column) {
  if (updatingTree_ || item == nullptr || item->parent() == nullptr || column != 0) {
    return;
  }
  applyCheckedStyle(item, item->checkState(0) == Qt::Checked);
  updateSummary();
  if (currentTopic_ == item->data(0, Qt::UserRole).toString()) {
    updateBadgeState(item->checkState(0) == Qt::Checked);
  }
}

void MainWindow::handleFormatChanged() {
  if (currentTopic_.isEmpty()) {
    return;
  }
  const QString selectedMode = modeCombo_->currentData().toString();
  refreshModeControls(currentTopic_, formatCombo_->currentText(), selectedMode);
  if (modeCombo_->currentData().toString() == QStringLiteral("single_file")) {
    if (namingEdit_->text().trimmed().isEmpty() || namingEdit_->text() == QStringLiteral("%name_%index")) {
      namingEdit_->setText(QStringLiteral("%name"));
    }
  } else if (namingEdit_->text() == QStringLiteral("%name")) {
    namingEdit_->setText(QStringLiteral("%name_%index"));
  }
  saveCurrentTopicConfig();
}

void MainWindow::handleModeChanged() {
  if (currentTopic_.isEmpty()) {
    return;
  }
  if (modeCombo_->currentData().toString() == QStringLiteral("single_file")) {
    if (namingEdit_->text().contains(QStringLiteral("%index"))) {
      namingEdit_->setText(QStringLiteral("%name"));
    }
  } else if (namingEdit_->text() == QStringLiteral("%name")) {
    namingEdit_->setText(QStringLiteral("%name_%index"));
  }
  saveCurrentTopicConfig();
}

void MainWindow::handleBaseDirectoryChange() {
  const QString directory = QFileDialog::getExistingDirectory(this, QStringLiteral("Select Base Directory"), baseDirectory_);
  if (directory.isEmpty()) {
    return;
  }

  baseDirectory_ = directory;
  updateBaseDirLabel();
  for (auto it = topicConfigs_.begin(); it != topicConfigs_.end(); ++it) {
    QJsonObject cfg = it.value().toObject();
    cfg.insert(QStringLiteral("path"), baseDirectory_);
    it.value() = cfg;
  }
  if (!currentTopic_.isEmpty()) {
    pathEdit_->setText(baseDirectory_);
  }
}

void MainWindow::saveConfigFile() {
  if (bagPath_.isEmpty()) {
    return;
  }

  saveCurrentTopicConfig();
  const QString filePath = QFileDialog::getSaveFileName(
      this, QStringLiteral("Save Config"), QDir::current().filePath(QStringLiteral("config.json")), QStringLiteral("JSON (*.json)"));
  if (filePath.isEmpty()) {
    return;
  }

  QJsonObject fullConfig = topicConfigs_;
  QJsonObject globalConfig;
  globalConfig.insert(QStringLiteral("cpu_percentage"), cpuSpin_->value());
  if (assocCombo_->currentText() != QStringLiteral("no resampling")) {
    QJsonObject resample;
    resample.insert(QStringLiteral("association"), assocCombo_->currentText());
    resample.insert(QStringLiteral("master_topic"), masterCombo_->currentText());
    if (!epsEdit_->text().trimmed().isEmpty()) {
      resample.insert(QStringLiteral("discard_eps"), epsEdit_->text().toDouble());
    } else {
      resample.insert(QStringLiteral("discard_eps"), QJsonValue::Null);
    }
    globalConfig.insert(QStringLiteral("resample_config"), resample);
  }
  fullConfig.insert(QStringLiteral("__global__"), globalConfig);

  QFile file(filePath);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    QMessageBox::critical(this, QStringLiteral("Error"), QStringLiteral("Failed to save config file."));
    return;
  }
  file.write(QJsonDocument(fullConfig).toJson(QJsonDocument::Indented));
  statusBar_->showMessage(QStringLiteral("Saved config to %1").arg(filePath));
}

void MainWindow::loadConfigFile() {
  if (bagPath_.isEmpty()) {
    QMessageBox::warning(this, QStringLiteral("Load Bag First"), QStringLiteral("Please load a bag file before loading a config."));
    return;
  }

  const QString filePath =
      QFileDialog::getOpenFileName(this, QStringLiteral("Load Config"), QDir::currentPath(), QStringLiteral("JSON (*.json)"));
  if (filePath.isEmpty()) {
    return;
  }

  QFile file(filePath);
  if (!file.open(QIODevice::ReadOnly)) {
    QMessageBox::critical(this, QStringLiteral("Error"), QStringLiteral("Failed to read config file."));
    return;
  }

  QJsonParseError parseError;
  const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
  if (parseError.error != QJsonParseError::NoError || !document.isObject()) {
    QMessageBox::critical(this, QStringLiteral("Error"), QStringLiteral("Failed to parse config JSON."));
    return;
  }

  QJsonObject config = document.object();
  const QJsonObject globalConfig = config.take(QStringLiteral("__global__")).toObject();
  topicConfigs_ = config;
  const QString requestedMasterTopic =
      globalConfig.value(QStringLiteral("resample_config")).toObject().value(QStringLiteral("master_topic")).toString();

  if (globalConfig.contains(QStringLiteral("cpu_percentage"))) {
    cpuSpin_->setValue(globalConfig.value(QStringLiteral("cpu_percentage")).toDouble(80.0));
  }
  if (globalConfig.contains(QStringLiteral("resample_config"))) {
    const QJsonObject resample = globalConfig.value(QStringLiteral("resample_config")).toObject();
    const QString assoc = resample.value(QStringLiteral("association")).toString(QStringLiteral("no resampling"));
    const int assocIndex = assocCombo_->findText(assoc);
    assocCombo_->setCurrentIndex(assocIndex >= 0 ? assocIndex : 0);
    if (resample.contains(QStringLiteral("discard_eps")) && !resample.value(QStringLiteral("discard_eps")).isNull()) {
      epsEdit_->setText(QString::number(resample.value(QStringLiteral("discard_eps")).toDouble()));
    } else {
      epsEdit_->clear();
    }
  } else {
    assocCombo_->setCurrentIndex(0);
    epsEdit_->clear();
  }

  QStringList missingTopics;
  updatingTree_ = true;
  for (int i = 0; i < topicTree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem *parent = topicTree_->topLevelItem(i);
    for (int j = 0; j < parent->childCount(); ++j) {
      QTreeWidgetItem *child = parent->child(j);
      const QString topicName = child->data(0, Qt::UserRole).toString();
      const bool checked = topicConfigs_.contains(topicName);
      child->setCheckState(0, checked ? Qt::Checked : Qt::Unchecked);
      applyCheckedStyle(child, checked);
    }
  }
  updatingTree_ = false;

  for (auto it = topicConfigs_.begin(); it != topicConfigs_.end(); ++it) {
    if (topicMeta(it.key()).isEmpty()) {
      missingTopics << it.key();
    }
  }

  updateSummary();
  if (!requestedMasterTopic.isEmpty()) {
    const int masterIndex = masterCombo_->findText(requestedMasterTopic);
    if (masterIndex >= 0) {
      masterCombo_->setCurrentIndex(masterIndex);
    }
  }
  if (!currentTopic_.isEmpty()) {
    loadTopicIntoEditor(currentTopic_);
  }
  if (!missingTopics.isEmpty()) {
    QMessageBox::warning(
        this,
        QStringLiteral("Missing Topics"),
        QStringLiteral("The following topics from the config are not in the loaded bag:\n%1").arg(missingTopics.join('\n')));
  }
}

void MainWindow::startExport() {
  if (bagPath_.isEmpty()) {
    return;
  }

  saveCurrentTopicConfig();
  clearFeedback();

  QString error;
  const QJsonObject payload = selectedPayload();
  const QJsonObject response = bridge_->call(QStringLiteral("validate_export_config"), payload, &error);
  if (!error.isEmpty()) {
    QMessageBox::critical(this, QStringLiteral("Configuration Error"), error);
    return;
  }

  const QJsonObject normalizedTopics = response.value(QStringLiteral("topic_configs")).toObject();
  for (auto it = normalizedTopics.begin(); it != normalizedTopics.end(); ++it) {
    topicConfigs_.insert(it.key(), it.value());
  }

  setExportRunning(true);
  showStatusProgress(QStringLiteral("Exporting..."), false);
  bridge_->startExport(payload);
  updateUiAvailability();
}

void MainWindow::cancelExport() {
  if (!exportRunning_) {
    return;
  }
  statusBar_->showMessage(QStringLiteral("Cancelling export..."));
  cancelButton_->setEnabled(false);
  bridge_->cancelExport();
}

void MainWindow::handleExportProgress(int, int, int percent) {
  statusProgress_->setValue(percent);
  loadingPercentLabel_->setText(QStringLiteral("%1%").arg(percent));
  statusBar_->showMessage(QStringLiteral("Exporting... (%1%)").arg(percent));
}

void MainWindow::handleExportCompleted() {
  setExportRunning(false);
  hideStatusProgress(QStringLiteral("Export complete"));
  setFeedback(QStringLiteral("Export complete."), QStringLiteral("success"));
}

void MainWindow::handleExportError(const QString &message) {
  setExportRunning(false);
  if (message.contains(QStringLiteral("Export aborted by user"))) {
    hideStatusProgress(QStringLiteral("Export canceled"));
    setFeedback(QStringLiteral("Export canceled."), QStringLiteral("cancel"));
    return;
  }

  hideStatusProgress(QStringLiteral("Export failed"));
  QMessageBox::critical(this, QStringLiteral("Export Error"), message);
}

void MainWindow::setBagData(const QJsonObject &bagData) {
  bagData_ = bagData;
  bagPath_ = bagData_.value(QStringLiteral("bag_path")).toString();
  baseDirectory_ = bagPath_;
  updateBaseDirLabel();
  bagNameLabel_->setText(QFileInfo(bagPath_).fileName());

  topicTree_->clear();
  topicConfigs_ = QJsonObject();
  currentTopic_.clear();

  const QJsonArray topics = bagData_.value(QStringLiteral("topics")).toArray();
  QMap<QString, QList<QJsonObject>> groupedTopics;
  for (const QJsonValue &value : topics) {
    const QJsonObject topic = value.toObject();
    groupedTopics[topic.value(QStringLiteral("type")).toString()].append(topic);
  }

  updatingTree_ = true;
  QFont headerFont(topicTree_->font());
  headerFont.setBold(true);
  const QBrush headerBrush(QColor(QStringLiteral("#f1f3f7")));
  for (auto it = groupedTopics.begin(); it != groupedTopics.end(); ++it) {
    auto *parent = new QTreeWidgetItem({it.key(), QString()});
    parent->setFirstColumnSpanned(true);
    parent->setFlags(Qt::ItemIsEnabled);
    parent->setFont(0, headerFont);
    parent->setBackground(0, headerBrush);
    parent->setBackground(1, headerBrush);
    topicTree_->addTopLevelItem(parent);
    for (const QJsonObject &topic : it.value()) {
      auto *child = new QTreeWidgetItem({topic.value(QStringLiteral("name")).toString(),
                                         QString::number(topic.value(QStringLiteral("count")).toInt())});
      child->setData(0, Qt::UserRole, topic.value(QStringLiteral("name")).toString());
      child->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsUserCheckable);
      child->setCheckState(0, Qt::Unchecked);
      child->setTextAlignment(1, Qt::AlignRight | Qt::AlignVCenter);
      parent->addChild(child);
    }
  }
  updatingTree_ = false;

  topicTree_->expandAll();
  topicTree_->resizeColumnToContents(1);
  updateSummary();
  setExportRunning(false);
  showTopicPlaceholder();
  updateUiAvailability();
}

void MainWindow::setExportRunning(bool running) {
  exportRunning_ = running;
  exportButton_->setVisible(!running);
  cancelButton_->setVisible(running);
  cancelButton_->setEnabled(false);
  updateUiAvailability();
}

void MainWindow::updateUiAvailability() {
  const bool hasBag = !bagPath_.isEmpty();
  int selectedCount = 0;
  for (int i = 0; i < topicTree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem *parent = topicTree_->topLevelItem(i);
    for (int j = 0; j < parent->childCount(); ++j) {
      if (parent->child(j)->checkState(0) == Qt::Checked) {
        ++selectedCount;
      }
    }
  }

  loadBagButton_->setEnabled(!exportRunning_ && !bagLoading_);
  loadBagSecondaryButton_->setEnabled(!exportRunning_ && !bagLoading_);
  topicTree_->setEnabled(hasBag && !exportRunning_ && !bagLoading_);
  topicFilterEdit_->setEnabled(hasBag && !exportRunning_ && !bagLoading_);
  selectAllButton_->setEnabled(hasBag && !exportRunning_ && !bagLoading_);
  selectNoneButton_->setEnabled(hasBag && !exportRunning_ && !bagLoading_);
  topicFormWidget_->setEnabled(hasBag && !exportRunning_ && !bagLoading_);
  loadConfigButton_->setEnabled(hasBag && !exportRunning_ && !bagLoading_);
  saveConfigButton_->setEnabled(hasBag && !exportRunning_ && !bagLoading_);

  settingsGroup_->setEnabled(hasBag && !exportRunning_ && !bagLoading_);
  baseGroup_->setEnabled(hasBag && !exportRunning_ && !bagLoading_);
  summaryGroup_->setEnabled(hasBag);
  feedbackCloseButton_->setEnabled(!exportRunning_);
  baseDirButton_->setEnabled(hasBag && !exportRunning_ && !bagLoading_);

  exportButton_->setEnabled(hasBag && selectedCount > 0 && !exportRunning_);
  exportButton_->setToolTip(selectedCount > 0 ? QStringLiteral("Start unbagging")
                                              : QStringLiteral("Select at least one topic to unbag."));
  cancelButton_->setEnabled(exportRunning_ && bridge_->isExportRunning());
}

void MainWindow::showTopicPlaceholder() {
  topicLabel_->setVisible(false);
  exportBadgeButton_->setVisible(false);
  topicFormWidget_->setVisible(false);
  placeholderLabel_->setVisible(true);
  placeholderHintLabel_->setVisible(true);
  helpTextLabel_->setVisible(false);
  QTimer::singleShot(0, this, [this]() { updatePlaceholderPixmap(); });
  QTimer::singleShot(10, this, [this]() { updatePlaceholderPixmap(); });
}

void MainWindow::showTopicEditor() {
  topicLabel_->setVisible(true);
  exportBadgeButton_->setVisible(true);
  topicFormWidget_->setVisible(true);
  placeholderLabel_->setVisible(false);
  placeholderHintLabel_->setVisible(false);
  helpTextLabel_->setVisible(true);
}

void MainWindow::updateSummary() {
  int selectedCount = 0;
  QStringList selectedTopics;
  int totalCount = 0;

  for (int i = 0; i < topicTree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem *parent = topicTree_->topLevelItem(i);
    for (int j = 0; j < parent->childCount(); ++j) {
      QTreeWidgetItem *child = parent->child(j);
      ++totalCount;
      if (child->checkState(0) == Qt::Checked) {
        ++selectedCount;
        selectedTopics << child->data(0, Qt::UserRole).toString();
      }
    }
  }

  if (bagPath_.isEmpty()) {
    summaryLabel_->setText(QStringLiteral("No bag loaded."));
  } else {
    summaryLabel_->setText(QStringLiteral("Selected Topics: %1\nTotal Topics: %2").arg(selectedCount).arg(totalCount));
  }

  updateMasterTopics();
  updateUiAvailability();

  if (!currentTopic_.isEmpty()) {
    updateBadgeState(isTopicChecked(currentTopic_));
  }
}

void MainWindow::updateMasterTopics() {
  const QString previous = masterCombo_->currentText();
  masterCombo_->blockSignals(true);
  masterCombo_->clear();
  for (int i = 0; i < topicTree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem *parent = topicTree_->topLevelItem(i);
    for (int j = 0; j < parent->childCount(); ++j) {
      QTreeWidgetItem *child = parent->child(j);
      if (child->checkState(0) == Qt::Checked) {
        masterCombo_->addItem(child->data(0, Qt::UserRole).toString());
      }
    }
  }
  const int restoreIndex = masterCombo_->findText(previous);
  if (restoreIndex >= 0) {
    masterCombo_->setCurrentIndex(restoreIndex);
  }
  masterCombo_->setEnabled(assocCombo_->currentText() != QStringLiteral("no resampling") && masterCombo_->count() > 0);
  masterCombo_->blockSignals(false);
}

void MainWindow::setFeedback(const QString &message, const QString &type) {
  feedbackBanner_->setProperty("feedbackType", type);
  feedbackLabel_->setText(message);
  feedbackBanner_->style()->unpolish(feedbackBanner_);
  feedbackBanner_->style()->polish(feedbackBanner_);
  feedbackBanner_->setVisible(true);
  feedbackTimer_->stop();
  feedbackTimer_->start(3500);
}

void MainWindow::clearFeedback() {
  feedbackTimer_->stop();
  feedbackBanner_->setVisible(false);
}

void MainWindow::updateBaseDirLabel() {
  baseDirLabel_->setToolTip(baseDirectory_);
  baseDirLabel_->setText(elideTopicPath(baseDirectory_, baseDirLabel_));
}

void MainWindow::updatePlaceholderPixmap() {
  if (placeholderPixmap_ == nullptr || placeholderPixmap_->isNull() || !placeholderLabel_->isVisible()) {
    return;
  }

  int availableWidth = 400;
  if (middleScrollArea_ != nullptr) {
    availableWidth = middleScrollArea_->viewport()->width();
    if (middleScrollArea_->verticalScrollBar()->isVisible()) {
      availableWidth -= middleScrollArea_->verticalScrollBar()->width();
    }
  } else if (middleStack_ != nullptr && middleStack_->width() > 0) {
    availableWidth = middleStack_->width();
  } else if (placeholderLabel_->width() > 0) {
    availableWidth = placeholderLabel_->width();
  }
  availableWidth = qMax(200, availableWidth - 20);
  placeholderLabel_->setPixmap(placeholderPixmap_->scaledToWidth(availableWidth, Qt::SmoothTransformation));
}

void MainWindow::updateStatusProgressWidth() {
  if (statusBar_ == nullptr || statusProgress_ == nullptr) {
    return;
  }
  statusProgress_->setFixedWidth(qMax(120, int(statusBar_->width() * 0.8)));
}

void MainWindow::showStatusProgress(const QString &message, bool indeterminate) {
  statusBar_->showMessage(message);
  if (indeterminate) {
    statusProgress_->setRange(0, 0);
  } else {
    statusProgress_->setRange(0, 100);
    statusProgress_->setValue(0);
  }
  statusProgress_->setVisible(true);
  middleStack_->setCurrentIndex(0);
  loadingPercentLabel_->setText(QString());
  if (loadingMovie_ != nullptr) {
    loadingMovie_->start();
  }
  cancelButton_->setEnabled(false);
}

void MainWindow::hideStatusProgress(const QString &message) {
  if (!message.isEmpty()) {
    statusBar_->showMessage(message);
  }
  statusProgress_->setVisible(false);
  middleStack_->setCurrentIndex(1);
  loadingPercentLabel_->setText(QString());
  if (loadingMovie_ != nullptr) {
    loadingMovie_->stop();
  }
  updateUiAvailability();
}

void MainWindow::saveCurrentTopicConfig() {
  if (currentTopic_.isEmpty()) {
    return;
  }
  topicConfigs_.insert(currentTopic_, currentEditorConfig());
}

void MainWindow::loadTopicIntoEditor(const QString &topicName) {
  saveCurrentTopicConfig();
  currentTopic_ = topicName;
  const QJsonObject config = effectiveTopicConfig(topicName);
  applyEditorConfig(topicName, config);
}

QJsonObject MainWindow::effectiveTopicConfig(const QString &topicName) const {
  if (topicConfigs_.contains(topicName)) {
    return topicConfigs_.value(topicName).toObject();
  }
  return topicMeta(topicName).value(QStringLiteral("default_config")).toObject();
}

QJsonObject MainWindow::currentEditorConfig() const {
  const QString formatName = formatCombo_->currentData().toString();
  QString storedFormat = formatName;
  if (modeCombo_->isVisible() && modeCombo_->count() > 1 &&
      modeCombo_->currentData().toString() == QStringLiteral("single_file")) {
    storedFormat += QStringLiteral("@") + modeCombo_->currentData().toString();
  }

  QString subfolder = subdirEdit_->text().trimmed();
  while (subfolder.startsWith('/')) {
    subfolder.remove(0, 1);
  }
  while (subfolder.endsWith('/')) {
    subfolder.chop(1);
  }

  QJsonObject config;
  config.insert(QStringLiteral("format"), storedFormat);
  config.insert(QStringLiteral("path"), pathEdit_->text().trimmed());
  config.insert(QStringLiteral("subfolder"), subfolder);
  config.insert(QStringLiteral("naming"), namingEdit_->text().trimmed());
  config.insert(QStringLiteral("processors"), processorChainWidget_->chain());
  return config;
}

void MainWindow::applyEditorConfig(const QString &topicName, const QJsonObject &config) {
  const QJsonObject meta = topicMeta(topicName);
  if (meta.isEmpty()) {
    showTopicPlaceholder();
    return;
  }

  showTopicEditor();

  topicLabel_->setText(topicName);
  formatCombo_->blockSignals(true);
  formatCombo_->clear();
  for (const QJsonValue &value : meta.value(QStringLiteral("formats")).toArray()) {
    const QJsonObject fmt = value.toObject();
    formatCombo_->addItem(fmt.value(QStringLiteral("name")).toString(), fmt.value(QStringLiteral("name")).toString());
  }

  const QString storedFormat = config.value(QStringLiteral("format")).toString();
  const QString baseFormat = splitFormatBase(storedFormat);
  const QString selectedMode = splitFormatMode(storedFormat);
  const int formatIndex = formatCombo_->findData(baseFormat);
  formatCombo_->setCurrentIndex(formatIndex >= 0 ? formatIndex : 0);
  formatCombo_->blockSignals(false);

  refreshModeControls(topicName, formatCombo_->currentData().toString(), selectedMode);

  pathEdit_->setText(config.value(QStringLiteral("path")).toString(baseDirectory_));
  subdirEdit_->setText(config.value(QStringLiteral("subfolder")).toString(QStringLiteral("%name")));
  namingEdit_->setText(config.value(QStringLiteral("naming")).toString(QStringLiteral("%name")));

  processorChainWidget_->setProcessorDefinitions(meta.value(QStringLiteral("processors")).toArray());
  processorChainWidget_->setChain(config.value(QStringLiteral("processors")).toArray());
  updateBadgeState(isTopicChecked(topicName));
  updateValidationStyles();
}

void MainWindow::refreshModeControls(const QString &topicName, const QString &formatName, const QString &selectedMode) {
  const QJsonObject meta = topicMeta(topicName);
  QStringList modes;
  for (const QJsonValue &value : meta.value(QStringLiteral("formats")).toArray()) {
    const QJsonObject fmt = value.toObject();
    if (fmt.value(QStringLiteral("name")).toString() == formatName) {
      for (const QJsonValue &modeValue : fmt.value(QStringLiteral("modes")).toArray()) {
        modes << modeValue.toString();
      }
      break;
    }
  }
  if (modes.isEmpty()) {
    modes << QStringLiteral("multi_file");
  }

  modeCombo_->blockSignals(true);
  modeCombo_->clear();
  if (modes.size() > 1) {
    for (const QString &mode : modes) {
      modeCombo_->addItem(mode == QStringLiteral("single_file") ? QStringLiteral("Single file") : QStringLiteral("Multi file"), mode);
    }
    int index = modeCombo_->findData(selectedMode);
    if (index < 0) {
      index = modeCombo_->findData(defaultModeForFormat(meta, formatName));
    }
    modeCombo_->setCurrentIndex(index >= 0 ? index : 0);
    modeCombo_->setVisible(true);
    modeLabel_->setVisible(true);
  } else {
    modeCombo_->addItem(modes.first() == QStringLiteral("single_file") ? QStringLiteral("Single file") : QStringLiteral("Multi file"), modes.first());
    modeCombo_->setCurrentIndex(0);
    modeCombo_->setVisible(false);
    modeLabel_->setVisible(false);
  }
  modeCombo_->blockSignals(false);
}

QString MainWindow::defaultModeForFormat(const QJsonObject &topicMeta, const QString &formatName) const {
  for (const QJsonValue &value : topicMeta.value(QStringLiteral("formats")).toArray()) {
    const QJsonObject fmt = value.toObject();
    if (fmt.value(QStringLiteral("name")).toString() != formatName) {
      continue;
    }
    const QJsonArray modes = fmt.value(QStringLiteral("modes")).toArray();
    for (const QJsonValue &mode : modes) {
      if (mode.toString() == QStringLiteral("multi_file")) {
        return QStringLiteral("multi_file");
      }
    }
    return modes.isEmpty() ? QStringLiteral("multi_file") : modes.first().toString();
  }
  return QStringLiteral("multi_file");
}

QJsonObject MainWindow::topicMeta(const QString &topicName) const {
  for (const QJsonValue &value : bagData_.value(QStringLiteral("topics")).toArray()) {
    const QJsonObject topic = value.toObject();
    if (topic.value(QStringLiteral("name")).toString() == topicName) {
      return topic;
    }
  }
  return {};
}

QJsonObject MainWindow::selectedPayload() const {
  QJsonArray selectedTopics;
  for (int i = 0; i < topicTree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem *parent = topicTree_->topLevelItem(i);
    for (int j = 0; j < parent->childCount(); ++j) {
      QTreeWidgetItem *child = parent->child(j);
      if (child->checkState(0) == Qt::Checked) {
        selectedTopics.append(child->data(0, Qt::UserRole).toString());
      }
    }
  }

  QJsonObject globalConfig;
  globalConfig.insert(QStringLiteral("cpu_percentage"), cpuSpin_->value());
  if (assocCombo_->currentText() != QStringLiteral("no resampling")) {
    QJsonObject resample;
    resample.insert(QStringLiteral("association"), assocCombo_->currentText());
    resample.insert(QStringLiteral("master_topic"), masterCombo_->currentText());
    if (!epsEdit_->text().trimmed().isEmpty()) {
      resample.insert(QStringLiteral("discard_eps"), epsEdit_->text().toDouble());
    } else {
      resample.insert(QStringLiteral("discard_eps"), QJsonValue::Null);
    }
    globalConfig.insert(QStringLiteral("resample_config"), resample);
  }

  return QJsonObject{
      {QStringLiteral("bag_path"), bagPath_},
      {QStringLiteral("base_dir"), baseDirectory_},
      {QStringLiteral("topic_configs"), topicConfigs_},
      {QStringLiteral("global_config"), globalConfig},
      {QStringLiteral("selected_topics"), selectedTopics},
  };
}

void MainWindow::selectTopicItem(const QString &topicName) {
  for (int i = 0; i < topicTree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem *parent = topicTree_->topLevelItem(i);
    for (int j = 0; j < parent->childCount(); ++j) {
      QTreeWidgetItem *child = parent->child(j);
      if (child->data(0, Qt::UserRole).toString() == topicName) {
        topicTree_->setCurrentItem(child);
        return;
      }
    }
  }
}

bool MainWindow::isTopicChecked(const QString &topicName) const {
  for (int i = 0; i < topicTree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem *parent = topicTree_->topLevelItem(i);
    for (int j = 0; j < parent->childCount(); ++j) {
      QTreeWidgetItem *child = parent->child(j);
      if (child->data(0, Qt::UserRole).toString() == topicName) {
        return child->checkState(0) == Qt::Checked;
      }
    }
  }
  return false;
}

void MainWindow::setTopicChecked(const QString &topicName, bool checked, bool blockSignals) {
  for (int i = 0; i < topicTree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem *parent = topicTree_->topLevelItem(i);
    for (int j = 0; j < parent->childCount(); ++j) {
      QTreeWidgetItem *child = parent->child(j);
      if (child->data(0, Qt::UserRole).toString() == topicName) {
        const bool previous = updatingTree_;
        updatingTree_ = previous || blockSignals;
        child->setCheckState(0, checked ? Qt::Checked : Qt::Unchecked);
        applyCheckedStyle(child, checked);
        updatingTree_ = previous;
        updateSummary();
        return;
      }
    }
  }
}

void MainWindow::updateBadgeState(bool checked) {
  exportBadgeButton_->setProperty("checkedState", checked ? QStringLiteral("true") : QStringLiteral("false"));
  exportBadgeButton_->setText(checked ? QStringLiteral("✓") : QStringLiteral("•"));
  exportBadgeButton_->style()->unpolish(exportBadgeButton_);
  exportBadgeButton_->style()->polish(exportBadgeButton_);
}

void MainWindow::updateValidationStyles() {
  pathEdit_->setProperty("error", pathEdit_->text().trimmed().isEmpty());
  namingEdit_->setProperty("error", namingEdit_->text().trimmed().isEmpty());
  pathEdit_->style()->unpolish(pathEdit_);
  pathEdit_->style()->polish(pathEdit_);
  namingEdit_->style()->unpolish(namingEdit_);
  namingEdit_->style()->polish(namingEdit_);
}

void MainWindow::applyProxyFilter(const QString &text) {
  const QString needle = text.trimmed().toLower();
  for (int i = 0; i < topicTree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem *parent = topicTree_->topLevelItem(i);
    bool hasVisibleChild = false;
    for (int j = 0; j < parent->childCount(); ++j) {
      QTreeWidgetItem *child = parent->child(j);
      const QString topicName = child->data(0, Qt::UserRole).toString().toLower();
      const QString typeName = parent->text(0).toLower();
      const bool matches = needle.isEmpty() || topicName.contains(needle) || typeName.contains(needle);
      child->setHidden(!matches);
      hasVisibleChild = hasVisibleChild || matches;
    }
    parent->setHidden(!hasVisibleChild);
  }
}
