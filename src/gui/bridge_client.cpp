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

#include "bridge_client.hpp"

#include <QJsonDocument>
#include <QJsonParseError>
#include <QProcess>

namespace {

/**
 * @brief Parse one JSON object returned by the Python bridge.
 * @param data UTF-8 JSON response bytes.
 * @param error Optional output for a parse or non-object response error.
 * @return Parsed object on success, or an empty object on failure.
 */
QJsonObject parseJsonObject(const QByteArray &data, QString *error) {
  QJsonParseError parseError;
  const QJsonDocument document = QJsonDocument::fromJson(data, &parseError);
  if (parseError.error != QJsonParseError::NoError) {
    if (error != nullptr) {
      *error = QStringLiteral("Bridge returned invalid JSON: %1").arg(parseError.errorString());
    }
    return {};
  }
  if (!document.isObject()) {
    if (error != nullptr) {
      *error = QStringLiteral("Bridge returned JSON that was not an object.");
    }
    return {};
  }
  return document.object();
}

}  // namespace

BridgeClient::BridgeClient(QObject *parent) : QObject(parent) {}

QJsonObject BridgeClient::call(const QString &command, const QJsonObject &payload, QString *error) const {
  // One-shot bridge commands use a short-lived blocking process and return a
  // single JSON envelope.
  QProcess process;
  process.start(pythonExecutable(), bridgeArguments(command));
  if (!process.waitForStarted()) {
    if (error != nullptr) {
      *error = QStringLiteral("Failed to start Python bridge process.");
    }
    return {};
  }

  process.write(QJsonDocument(payload).toJson(QJsonDocument::Compact));
  process.closeWriteChannel();

  if (!process.waitForFinished(-1)) {
    if (error != nullptr) {
      *error = QStringLiteral("Python bridge process timed out.");
    }
    return {};
  }

  const QJsonObject envelope = parseJsonObject(process.readAllStandardOutput(), error);
  if (envelope.isEmpty()) {
    if (error != nullptr && error->isEmpty()) {
      *error = QString::fromUtf8(process.readAllStandardError());
    }
    return {};
  }

  if (!envelope.value(QStringLiteral("ok")).toBool()) {
    if (error != nullptr) {
      *error = envelope.value(QStringLiteral("error")).toString();
    }
    return {};
  }

  const QJsonValue dataValue = envelope.value(QStringLiteral("data"));
  if (!dataValue.isObject()) {
    if (error != nullptr) {
      *error = QStringLiteral("Bridge response did not contain an object payload.");
    }
    return {};
  }

  return dataValue.toObject();
}

void BridgeClient::startExport(const QJsonObject &payload) {
  // Long-running exports use a dedicated process so progress events can stream
  // back as NDJSON without blocking the GUI thread.
  cancelExport();

  exportProcess_ = new QProcess(this);
  exportStdoutBuffer_.clear();
  exportSawTerminalEvent_ = false;
  cancelRequested_ = false;

  connect(exportProcess_, &QProcess::readyReadStandardOutput, this, &BridgeClient::handleExportStdout);
  connect(exportProcess_,
          QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
          this,
          [this](int exitCode, QProcess::ExitStatus) { handleExportFinished(exitCode); });

  exportProcess_->start(pythonExecutable(), bridgeArguments(QStringLiteral("run_export")));
  if (!exportProcess_->waitForStarted()) {
    emit exportError(QStringLiteral("Failed to start export bridge process."));
    cleanupExportProcess();
    return;
  }

  exportProcess_->write(QJsonDocument(payload).toJson(QJsonDocument::Compact));
  exportProcess_->closeWriteChannel();
}

void BridgeClient::cancelExport() {
  if (exportProcess_ == nullptr) {
    return;
  }

  // Export cancellation is defined as terminating the dedicated run_export
  // bridge process. The Python bridge handles SIGTERM/SIGINT and aborts the
  // active exporter from there, so no separate cancel RPC exists.
  cancelRequested_ = true;
  exportProcess_->terminate();
  if (!exportProcess_->waitForFinished(2000)) {
    exportProcess_->kill();
    exportProcess_->waitForFinished(2000);
  }

  if (exportProcess_ != nullptr) {
    handleExportFinished(exportProcess_->exitCode());
  }
}

bool BridgeClient::isExportRunning() const {
  return exportProcess_ != nullptr;
}

void BridgeClient::handleExportStdout() {
  if (exportProcess_ == nullptr) {
    return;
  }

  // Export output is NDJSON, so buffer partial reads until a full line arrives.
  exportStdoutBuffer_.append(exportProcess_->readAllStandardOutput());
  while (true) {
    const int newlineIndex = exportStdoutBuffer_.indexOf('\n');
    if (newlineIndex < 0) {
      break;
    }

    const QByteArray line = exportStdoutBuffer_.left(newlineIndex).trimmed();
    exportStdoutBuffer_.remove(0, newlineIndex + 1);
    if (!line.isEmpty()) {
      processExportLine(line);
    }
  }
}

void BridgeClient::handleExportFinished(int exitCode) {
  if (exportProcess_ == nullptr) {
    return;
  }

  if (!exportStdoutBuffer_.trimmed().isEmpty()) {
    processExportLine(exportStdoutBuffer_.trimmed());
    exportStdoutBuffer_.clear();
  }

  const QString stderrOutput = QString::fromUtf8(exportProcess_->readAllStandardError()).trimmed();
  const bool cancelRequested = cancelRequested_;
  const bool sawTerminalEvent = exportSawTerminalEvent_;
  cleanupExportProcess();

  if (cancelRequested) {
    if (!sawTerminalEvent) {
      emit exportError(QStringLiteral("Export aborted by user"));
    }
    return;
  }

  if (!sawTerminalEvent) {
    if (!stderrOutput.isEmpty()) {
      emit exportError(stderrOutput);
    } else if (exitCode != 0) {
      emit exportError(QStringLiteral("Export process failed."));
    }
  }
}

QString BridgeClient::pythonExecutable() const {
  const QByteArray env = qgetenv("ROS2_UNBAG_PYTHON");
  if (!env.isEmpty()) {
    return QString::fromUtf8(env);
  }
  return QStringLiteral("python3");
}

QStringList BridgeClient::bridgeArguments(const QString &command) const {
  return {QStringLiteral("-m"), QStringLiteral("ros2_unbag.bridge"), command};
}

void BridgeClient::processExportLine(const QByteArray &line) {
  QString error;
  const QJsonObject event = parseJsonObject(line, &error);
  if (event.isEmpty()) {
    emit exportError(error);
    exportSawTerminalEvent_ = true;
    return;
  }

  const QString type = event.value(QStringLiteral("type")).toString();
  if (type == QStringLiteral("progress")) {
    emit exportProgress(
        event.value(QStringLiteral("current")).toInt(),
        event.value(QStringLiteral("total")).toInt(),
        event.value(QStringLiteral("percent")).toInt());
    return;
  }

  if (type == QStringLiteral("completed")) {
    exportSawTerminalEvent_ = true;
    emit exportCompleted();
    return;
  }

  if (type == QStringLiteral("error")) {
    exportSawTerminalEvent_ = true;
    emit exportError(event.value(QStringLiteral("message")).toString());
  }
}

void BridgeClient::cleanupExportProcess() {
  if (exportProcess_ != nullptr) {
    exportProcess_->deleteLater();
    exportProcess_ = nullptr;
  }
  exportStdoutBuffer_.clear();
  exportSawTerminalEvent_ = false;
  cancelRequested_ = false;
}
