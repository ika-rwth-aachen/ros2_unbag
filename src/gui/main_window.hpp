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

#pragma once

#include <QJsonArray>
#include <QJsonObject>
#include <QMainWindow>

class BridgeClient;
class ProcessorChainWidget;
class QEvent;
class QLabel;
class QLineEdit;
class QObject;
class QShowEvent;
class QTreeWidget;
class QTreeWidgetItem;
class QComboBox;
class QPushButton;
class QStackedWidget;
class QScrollArea;
class QMovie;
class QProgressBar;
class QSlider;
class QDoubleSpinBox;
class QStatusBar;
class QFrame;
class QToolButton;
class QWidget;
class QGroupBox;
class QTimer;
class QPixmap;

/**
 * @brief Main application window coordinating bag inspection, config editing, and export.
 *
 * Hosts the three-column export workflow:
 * - left: bag loading, topic selection, config load/save
 * - middle: per-topic export settings
 * - right: global settings, summary, export/cancel controls
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  /**
   * @brief Construct the main window and initialize the GUI.
   *
   * @param parent Optional Qt parent widget.
   */
  explicit MainWindow(QWidget *parent = nullptr);

private slots:
  /**
   * @brief Open a bag-selection dialog and inspect the selected bag through the bridge.
   */
  void loadBag();
  /**
   * @brief Load the clicked topic into the middle-column editor.
   * @param item Topic tree item whose state changed.
   * @param column Tree column associated with the event.
   */
  void handleTopicClicked(QTreeWidgetItem *item, int column);
  /**
   * @brief Keep selection state, summary, and per-topic config in sync with tree changes.
   * @param item Topic tree item whose state changed.
   * @param column Tree column associated with the event.
   */
  void handleTopicChanged(QTreeWidgetItem *item, int column);
  /**
   * @brief Refresh mode controls and defaults after a format change.
   */
  void handleFormatChanged();
  /**
   * @brief Adjust naming defaults after a mode change.
   */
  void handleModeChanged();
  /**
   * @brief Change the shared base output directory for all topic configs.
   */
  void handleBaseDirectoryChange();
  /**
   * @brief Save the current GUI configuration as JSON.
   */
  void saveConfigFile();
  /**
   * @brief Load a JSON config file into the current bag context.
   */
  void loadConfigFile();
  /**
   * @brief Validate the current selection and start the export worker process.
   */
  void startExport();
  /**
   * @brief Cancel the currently running export.
   */
  void cancelExport();

  /**
   * @brief Update progress visuals from a bridge progress event.
   * @param current Number of completed export tasks.
   * @param total Total number of export tasks.
   * @param percent Export completion percentage.
   */
  void handleExportProgress(int current, int total, int percent);
  /**
   * @brief Finalize UI state after a successful export.
   */
  void handleExportCompleted();
  /**
   * @brief Finalize UI state after an export error or cancellation.
   * @param message Message to display to the user.
   */
  void handleExportError(const QString &message);

private:
  /**
   * @brief Build the complete widget tree and wire signal/slot connections.
   */
  void buildUi();
  /**
   * @brief Apply the packaged Qt stylesheet.
   */
  void applyStyleSheet();
  /**
   * @brief Track scroll viewport resizes used for placeholder image scaling.
   * @param watched Object receiving the event.
   * @param event Qt event being handled.
   * @return Whether Qt should stop further processing of the event.
   */
  bool eventFilter(QObject *watched, QEvent *event) override;
  /**
   * @brief Trigger post-show layout-dependent UI adjustments.
   * @param event Qt event being handled.
   */
  void showEvent(QShowEvent *event) override;
  /**
   * @brief Keep responsive UI details sized correctly during window resize.
   * @param event Qt event being handled.
   */
  void resizeEvent(QResizeEvent *event) override;
  /**
   * @brief Replace the current bag metadata and rebuild topic UI state.
   * @param bagData Inspected bag metadata returned by the Python bridge.
   */
  void setBagData(const QJsonObject &bagData);
  /**
   * @brief Toggle export-running UI state.
   * @param running Whether an export is currently running.
   */
  void setExportRunning(bool running);
  /**
   * @brief Enable or disable controls based on bag/loading/export state.
   */
  void updateUiAvailability();
  /**
   * @brief Show the empty topic-settings placeholder view.
   */
  void showTopicPlaceholder();
  /**
   * @brief Show the per-topic settings editor view.
   */
  void showTopicEditor();
  /**
   * @brief Recompute selected-topic summary and master-topic choices.
   */
  void updateSummary();
  /**
   * @brief Rebuild the resampling master-topic combo from selected topics.
   */
  void updateMasterTopics();
  /**
   * @brief Show a transient feedback banner.
   * @param message Message to display to the user.
   * @param type Feedback category used for styling.
   */
  void setFeedback(const QString &message, const QString &type);
  /**
   * @brief Hide the feedback banner.
   */
  void clearFeedback();
  /**
   * @brief Refresh the elided base-directory label text.
   */
  void updateBaseDirLabel();
  /**
   * @brief Scale the title placeholder image to the available middle-column width.
   */
  void updatePlaceholderPixmap();
  /**
   * @brief Keep the status-bar progress widget sized relative to window width.
   */
  void updateStatusProgressWidth();
  /**
   * @brief Switch the UI into a loading/progress state.
   * @param message Message to display to the user.
   * @param indeterminate Whether to show progress without a known completion percentage.
   */
  void showStatusProgress(const QString &message, bool indeterminate);
  /**
   * @brief Leave the loading/progress state and restore the settings page.
   * @param message Message to display to the user.
   */
  void hideStatusProgress(const QString &message = QString());
  /**
   * @brief Persist the editor contents into the current topic config map.
   */
  void saveCurrentTopicConfig();
  /**
   * @brief Load a topic config into the editor widgets.
   * @param topicName Topic whose metadata or configuration is being accessed.
   */
  void loadTopicIntoEditor(const QString &topicName);
  /**
   * @brief Return the effective config for a topic, including defaults.
   * @param topicName Topic whose metadata or configuration is being accessed.
   * @return Saved topic configuration, or generated defaults when absent.
   */
  QJsonObject effectiveTopicConfig(const QString &topicName) const;
  /**
   * @brief Read the current editor widgets into a topic config object.
   * @return Topic configuration assembled from the editor widgets.
   */
  QJsonObject currentEditorConfig() const;
  /**
   * @brief Apply a topic config object to the editor widgets.
   * @param topicName Topic whose metadata or configuration is being accessed.
   * @param config Saved configuration to apply to the editor.
   */
  void applyEditorConfig(const QString &topicName, const QJsonObject &config);
  /**
   * @brief Refresh mode choices for a given topic format.
   * @param topicName Topic whose metadata or configuration is being accessed.
   * @param formatName Selected export format name.
   * @param selectedMode Mode to select when it is available.
   */
  void refreshModeControls(const QString &topicName, const QString &formatName, const QString &selectedMode);
  /**
   * @brief Return the default mode name for a topic format.
   * @param topicMeta Bag metadata describing the topic and its supported formats.
   * @param formatName Selected export format name.
   * @return Default export mode for the selected topic format.
   */
  QString defaultModeForFormat(const QJsonObject &topicMeta, const QString &formatName) const;
  /**
   * @brief Look up bag metadata for one topic.
   * @param topicName Topic whose metadata or configuration is being accessed.
   * @return Metadata for the requested topic, or an empty object when absent.
   */
  QJsonObject topicMeta(const QString &topicName) const;
  /**
   * @brief Build the normalized bridge payload from current UI state.
   * @return Normalized export request for the selected topics.
   */
  QJsonObject selectedPayload() const;
  /**
   * @brief Select the topic item matching the given topic name.
   * @param topicName Topic whose metadata or configuration is being accessed.
   */
  void selectTopicItem(const QString &topicName);
  /**
   * @brief Return whether a topic is currently marked for export.
   * @param topicName Topic whose metadata or configuration is being accessed.
   * @return True when the topic is checked for export.
   */
  bool isTopicChecked(const QString &topicName) const;
  /**
   * @brief Set a topic export checkbox while optionally suppressing signal reactions.
   * @param topicName Topic whose metadata or configuration is being accessed.
   * @param checked Whether the topic is selected for export.
   * @param blockSignals Whether to suppress tree signals while updating the checkbox.
   */
  void setTopicChecked(const QString &topicName, bool checked, bool blockSignals = false);
  /**
   * @brief Update the export badge styling for the current topic.
   * @param checked Whether the topic is selected for export.
   */
  void updateBadgeState(bool checked);
  /**
   * @brief Mark invalid editor fields using widget properties/styles.
   */
  void updateValidationStyles();
  /**
   * @brief Filter the topic tree by the given text.
   * @param text Text used to filter the topic tree.
   */
  void applyProxyFilter(const QString &text);

  BridgeClient *bridge_;
  QJsonObject bagData_;
  QJsonObject topicConfigs_;
  QString currentTopic_;
  QString bagPath_;
  QString baseDirectory_;
  bool updatingTree_{false};
  bool bagLoading_{false};
  bool exportRunning_{false};

  QTreeWidget *topicTree_;
  QLineEdit *topicFilterEdit_;
  QPushButton *selectAllButton_;
  QPushButton *selectNoneButton_;
  QPushButton *loadBagButton_;
  QPushButton *loadBagSecondaryButton_;
  QLabel *bagNameLabel_;
  QPushButton *loadConfigButton_;
  QPushButton *saveConfigButton_;

  QScrollArea *middleScrollArea_;
  QStackedWidget *middleStack_;
  QLabel *loadingLabel_;
  QLabel *loadingPercentLabel_;
  QMovie *loadingMovie_;
  QWidget *settingsPage_;
  QLabel *topicLabel_;
  QPushButton *exportBadgeButton_;
  QWidget *topicFormWidget_;
  QComboBox *formatCombo_;
  QComboBox *modeCombo_;
  QLabel *modeLabel_;
  QLineEdit *pathEdit_;
  QLineEdit *subdirEdit_;
  QLineEdit *namingEdit_;
  ProcessorChainWidget *processorChainWidget_;
  QLabel *placeholderLabel_;
  QLabel *placeholderHintLabel_;
  QLabel *helpTextLabel_;
  QPixmap *placeholderPixmap_;

  QGroupBox *settingsGroup_;
  QGroupBox *baseGroup_;
  QGroupBox *summaryGroup_;
  QSlider *cpuSlider_;
  QDoubleSpinBox *cpuSpin_;
  QComboBox *assocCombo_;
  QLineEdit *epsEdit_;
  QComboBox *masterCombo_;
  QLabel *baseDirLabel_;
  QPushButton *baseDirButton_;
  QLabel *summaryLabel_;
  QFrame *feedbackBanner_;
  QLabel *feedbackLabel_;
  QToolButton *feedbackCloseButton_;
  QTimer *feedbackTimer_;
  QPushButton *exportButton_;
  QPushButton *cancelButton_;

  QStatusBar *statusBar_;
  QProgressBar *statusProgress_;
};
