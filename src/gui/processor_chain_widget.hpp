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

#include <QFrame>
#include <QJsonArray>
#include <QJsonObject>
#include <QWidget>

class QComboBox;
class QFormLayout;
class QLabel;
class QPushButton;
class QToolButton;
class QVBoxLayout;

class ProcessorChainWidget;

/**
 * @brief One editable processor entry inside a processor chain.
 *
 * Stores the selected processor plus any argument inputs for that processor.
 */
class ProcessorEntryWidget : public QFrame {
  Q_OBJECT

public:
  /**
   * @brief Construct one processor entry widget.
   *
   * @param chainWidget Owning processor chain widget.
   */
  explicit ProcessorEntryWidget(ProcessorChainWidget *chainWidget);

  /**
   * @brief Update the displayed 1-based index and move-button state.
   * @param index One-based position of this processor entry.
   * @param total Total number of processor entries.
   */
  void setIndex(int index, int total);
  /**
   * @brief Apply a saved processor config to the entry UI.
   * @param config Saved configuration to apply to the editor.
   */
  void applyConfig(const QJsonObject &config);
  /**
   * @brief Select the first available processor definition.
   */
  void selectDefault();
  /**
   * @brief Serialize the current entry UI state into a config object.
   * @return Selected processor name and argument values.
   */
  QJsonObject config() const;

private slots:
  /**
   * @brief Rebuild argument inputs after the selected processor changes.
   * @param processorName Name of the selected processor.
   */
  void onProcessorChanged(const QString &processorName);

private:
  /**
   * @brief Remove all dynamic argument input rows.
   */
  void clearArgs();
  /**
   * @brief Look up the processor definition for one processor name.
   * @param name Processor name to look up.
   * @return Definition matching the processor name, or an empty object.
   */
  QJsonObject processorDefinition(const QString &name) const;

  ProcessorChainWidget *chainWidget_;
  QLabel *indexLabel_;
  QComboBox *combo_;
  QToolButton *upButton_;
  QToolButton *downButton_;
  QToolButton *removeButton_;
  QFormLayout *argsLayout_;
  QList<QPair<QString, class QLineEdit *>> argInputs_;
};

/**
 * @brief Editor widget for ordered per-topic processor chains.
 */
class ProcessorChainWidget : public QWidget {
  Q_OBJECT

public:
  /**
   * @brief Construct the processor chain editor widget.
   *
   * @param parent Optional Qt parent widget.
   */
  explicit ProcessorChainWidget(QWidget *parent = nullptr);

  /**
   * @brief Replace the available processor definitions for the current topic type.
   * @param definitions Available processor definitions for the topic.
   */
  void setProcessorDefinitions(const QJsonArray &definitions);
  /**
   * @brief Replace the current chain entries from saved config.
   * @param configs Ordered processor configurations to display.
   */
  void setChain(const QJsonArray &configs);
  /**
   * @brief Serialize the current chain to JSON config.
   * @return Processor configurations in their current execution order.
   */
  QJsonArray chain() const;
  /**
   * @brief Return available processor names for the current topic type.
   * @return Names of the available processors for the topic.
   */
  QStringList availableProcessorNames() const;
  /**
   * @brief Look up one processor definition by name.
   * @param name Processor name to look up.
   * @return Definition matching the processor name, or an empty object.
   */
  QJsonObject processorDefinition(const QString &name) const;

signals:
  /**
   * @brief Emitted whenever the chain contents or order changes.
   */
  void changed();

private slots:
  /**
   * @brief Append a new empty processor entry.
   */
  void addEntry();

private:
  /**
   * @brief Append a new processor entry using an optional preset config.
   * @param preset Initial processor configuration for the new entry.
   */
  void addEntry(const QJsonObject &preset);
  /**
   * @brief Remove one processor entry from the chain.
   * @param entry Processor entry to remove or move.
   */
  void removeEntry(ProcessorEntryWidget *entry);
  /**
   * @brief Move one processor entry up or down by one position.
   * @param entry Processor entry to remove or move.
   * @param delta Position change, normally minus one or plus one.
   */
  void moveEntry(ProcessorEntryWidget *entry, int delta);
  /**
   * @brief Refresh entry indices and move-button states.
   */
  void reindexEntries();
  /**
   * @brief Show or hide the empty-state hint.
   */
  void updateEmptyHint();

  friend class ProcessorEntryWidget;

  QJsonArray processorDefinitions_;
  QList<ProcessorEntryWidget *> entries_;
  QVBoxLayout *chainLayout_;
  QLabel *emptyHint_;
  QPushButton *addButton_;
};
