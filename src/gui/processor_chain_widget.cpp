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

#include "processor_chain_widget.hpp"

#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QJsonValue>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QStyle>
#include <QToolButton>
#include <QVBoxLayout>

ProcessorEntryWidget::ProcessorEntryWidget(ProcessorChainWidget *chainWidget)
    : QFrame(chainWidget), chainWidget_(chainWidget) {
  setFrameShape(QFrame::StyledPanel);
  setFrameShadow(QFrame::Raised);

  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(4);

  auto *header = new QHBoxLayout();
  header->setSpacing(6);

  indexLabel_ = new QLabel(QStringLiteral("1."), this);
  indexLabel_->setFixedWidth(24);
  header->addWidget(indexLabel_);

  combo_ = new QComboBox(this);
  combo_->addItems(chainWidget_->availableProcessorNames());
  header->addWidget(combo_, 1);

  upButton_ = new QToolButton(this);
  upButton_->setIcon(style()->standardIcon(QStyle::SP_ArrowUp));
  upButton_->setAutoRaise(true);
  header->addWidget(upButton_);

  downButton_ = new QToolButton(this);
  downButton_->setIcon(style()->standardIcon(QStyle::SP_ArrowDown));
  downButton_->setAutoRaise(true);
  header->addWidget(downButton_);

  removeButton_ = new QToolButton(this);
  removeButton_->setIcon(style()->standardIcon(QStyle::SP_DialogCloseButton));
  removeButton_->setAutoRaise(true);
  header->addWidget(removeButton_);

  layout->addLayout(header);

  argsLayout_ = new QFormLayout();
  argsLayout_->setContentsMargins(0, 4, 0, 0);
  layout->addLayout(argsLayout_);

  connect(combo_, &QComboBox::currentTextChanged, this, &ProcessorEntryWidget::onProcessorChanged);
  connect(upButton_, &QToolButton::clicked, chainWidget_, [this]() { chainWidget_->moveEntry(this, -1); });
  connect(downButton_, &QToolButton::clicked, chainWidget_, [this]() { chainWidget_->moveEntry(this, 1); });
  connect(removeButton_, &QToolButton::clicked, chainWidget_, [this]() { chainWidget_->removeEntry(this); });

  onProcessorChanged(combo_->currentText());
}

void ProcessorEntryWidget::setIndex(int index, int total) {
  indexLabel_->setText(QStringLiteral("%1.").arg(index));
  upButton_->setEnabled(index > 1);
  downButton_->setEnabled(index < total);
}

void ProcessorEntryWidget::applyConfig(const QJsonObject &config) {
  const QString name = config.value(QStringLiteral("name")).toString();
  if (!name.isEmpty()) {
    const int existingIndex = combo_->findText(name);
    if (existingIndex >= 0) {
      combo_->setCurrentIndex(existingIndex);
    }
  }

  const QJsonObject args = config.value(QStringLiteral("args")).toObject();
  for (const auto &pair : argInputs_) {
    const QString key = pair.first;
    if (args.contains(key)) {
      pair.second->setText(args.value(key).toVariant().toString());
    }
  }
}

void ProcessorEntryWidget::selectDefault() {
  if (combo_->count() > 0) {
    combo_->setCurrentIndex(0);
  }
}

QJsonObject ProcessorEntryWidget::config() const {
  QJsonObject args;
  for (const auto &pair : argInputs_) {
    const QString value = pair.second->text().trimmed();
    if (!value.isEmpty()) {
      args.insert(pair.first, value);
    }
  }

  QJsonObject config;
  config.insert(QStringLiteral("name"), combo_->currentText());
  config.insert(QStringLiteral("args"), args);
  return config;
}

void ProcessorEntryWidget::onProcessorChanged(const QString &processorName) {
  // Rebuild the argument form from the selected processor definition so each
  // topic type exposes only the relevant configurable fields.
  clearArgs();

  const QJsonObject definition = processorDefinition(processorName);
  const QJsonArray args = definition.value(QStringLiteral("args")).toArray();
  for (const QJsonValue &value : args) {
    const QJsonObject arg = value.toObject();
    const QString argName = arg.value(QStringLiteral("name")).toString();

    auto *label = new QLabel(this);
    label->setText(arg.value(QStringLiteral("required")).toBool()
                       ? argName
                       : QStringLiteral("%1 (optional)").arg(argName));

    auto *edit = new QLineEdit(this);
    QStringList placeholderParts;
    const QString doc = arg.value(QStringLiteral("doc")).toString();
    if (!doc.isEmpty()) {
      placeholderParts << doc;
    }
    if (!arg.value(QStringLiteral("default")).isNull()) {
      placeholderParts << QStringLiteral("default: %1").arg(arg.value(QStringLiteral("default")).toVariant().toString());
    }
    const QString annotation = arg.value(QStringLiteral("annotation")).toString();
    if (!annotation.isEmpty()) {
      placeholderParts << QStringLiteral("Type: %1").arg(annotation);
    }
    edit->setPlaceholderText(placeholderParts.join(QStringLiteral(" - ")));

    argsLayout_->addRow(label, edit);
    argInputs_.append({argName, edit});

    connect(edit, &QLineEdit::textChanged, chainWidget_, &ProcessorChainWidget::changed);
  }

  emit chainWidget_->changed();
}

void ProcessorEntryWidget::clearArgs() {
  while (argsLayout_->rowCount() > 0) {
    argsLayout_->removeRow(0);
  }
  argInputs_.clear();
}

QJsonObject ProcessorEntryWidget::processorDefinition(const QString &name) const {
  return chainWidget_->processorDefinition(name);
}

ProcessorChainWidget::ProcessorChainWidget(QWidget *parent) : QWidget(parent) {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(6);

  chainLayout_ = new QVBoxLayout();
  chainLayout_->setSpacing(8);
  layout->addLayout(chainLayout_);

  emptyHint_ = new QLabel(QStringLiteral("No processors configured."), this);
  layout->addWidget(emptyHint_);

  auto *addRow = new QHBoxLayout();
  addRow->addStretch();
  addButton_ = new QPushButton(QStringLiteral("Add Processor"), this);
  addRow->addWidget(addButton_);
  layout->addLayout(addRow);

  connect(addButton_, &QPushButton::clicked, this, [this]() { addEntry(); });
  updateEmptyHint();
}

void ProcessorChainWidget::setProcessorDefinitions(const QJsonArray &definitions) {
  processorDefinitions_ = definitions;
  const bool hasDefinitions = !processorDefinitions_.isEmpty();
  addButton_->setEnabled(hasDefinitions);
  emptyHint_->setText(hasDefinitions ? QStringLiteral("No processors configured.")
                                     : QStringLiteral("No processors available."));
  if (!hasDefinitions) {
    setChain(QJsonArray{});
  }
}

void ProcessorChainWidget::setChain(const QJsonArray &configs) {
  // Recreate entries from scratch to keep widget order and signal wiring in
  // sync with the serialized processor chain.
  for (ProcessorEntryWidget *entry : entries_) {
    chainLayout_->removeWidget(entry);
    entry->deleteLater();
  }
  entries_.clear();

  for (const QJsonValue &value : configs) {
    addEntry(value.toObject());
  }
  updateEmptyHint();
  emit changed();
}

QJsonArray ProcessorChainWidget::chain() const {
  QJsonArray array;
  for (ProcessorEntryWidget *entry : entries_) {
    array.append(entry->config());
  }
  return array;
}

QStringList ProcessorChainWidget::availableProcessorNames() const {
  QStringList names;
  for (const QJsonValue &value : processorDefinitions_) {
    names << value.toObject().value(QStringLiteral("name")).toString();
  }
  return names;
}

QJsonObject ProcessorChainWidget::processorDefinition(const QString &name) const {
  for (const QJsonValue &value : processorDefinitions_) {
    const QJsonObject object = value.toObject();
    if (object.value(QStringLiteral("name")).toString() == name) {
      return object;
    }
  }
  return {};
}

void ProcessorChainWidget::addEntry() {
  addEntry(QJsonObject{});
}

void ProcessorChainWidget::addEntry(const QJsonObject &preset) {
  auto *entry = new ProcessorEntryWidget(this);
  entries_.append(entry);
  chainLayout_->addWidget(entry);
  reindexEntries();

  if (!preset.isEmpty()) {
    entry->applyConfig(preset);
  } else {
    entry->selectDefault();
  }

  updateEmptyHint();
  emit changed();
}

void ProcessorChainWidget::removeEntry(ProcessorEntryWidget *entry) {
  entries_.removeOne(entry);
  chainLayout_->removeWidget(entry);
  entry->deleteLater();
  reindexEntries();
  updateEmptyHint();
  emit changed();
}

void ProcessorChainWidget::moveEntry(ProcessorEntryWidget *entry, int delta) {
  const int index = entries_.indexOf(entry);
  const int newIndex = index + delta;
  if (index < 0 || newIndex < 0 || newIndex >= entries_.size()) {
    return;
  }

  entries_.move(index, newIndex);
  chainLayout_->removeWidget(entry);
  chainLayout_->insertWidget(newIndex, entry);
  reindexEntries();
  emit changed();
}

void ProcessorChainWidget::reindexEntries() {
  const int total = entries_.size();
  for (int index = 0; index < entries_.size(); ++index) {
    entries_[index]->setIndex(index + 1, total);
  }
}

void ProcessorChainWidget::updateEmptyHint() {
  emptyHint_->setVisible(entries_.isEmpty());
}
