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

#include <QObject>
#include <QJsonObject>

class QProcess;

/**
 * @brief Small Qt wrapper around the Python bridge process.
 *
 * Uses blocking request/response calls for one-shot commands and a dedicated
 * long-running process for export progress events.
 */
class BridgeClient : public QObject {
  Q_OBJECT

public:
  /**
   * @brief Construct a bridge client instance.
   *
   * @param parent Optional Qt parent object.
   */
  explicit BridgeClient(QObject *parent = nullptr);

  /**
   * @brief Execute a one-shot bridge command and return its JSON object payload.
   *
   * @param command Bridge subcommand name.
   * @param payload JSON request object written to stdin.
   * @param error Optional output string for user-displayable errors.
   * @return Parsed JSON object payload on success, or an empty object on error.
   */
  QJsonObject call(const QString &command, const QJsonObject &payload, QString *error = nullptr) const;

  /**
   * @brief Start a dedicated bridge process for a long-running export.
   * @param payload JSON request payload sent to the Python bridge.
   */
  void startExport(const QJsonObject &payload);
  /**
   * @brief Cancel the currently running export process.
   */
  void cancelExport();
  /**
   * @brief Return whether an export bridge process is currently active.
   * @return True while an export bridge process is active.
   */
  bool isExportRunning() const;

signals:
  /**
   * @brief Emitted when the bridge reports export progress.
   * @param current Number of completed export tasks.
   * @param total Total number of export tasks.
   * @param percent Export completion percentage.
   */
  void exportProgress(int current, int total, int percent);
  /**
   * @brief Emitted when the bridge reports export completion.
   */
  void exportCompleted();
  /**
   * @brief Emitted when the bridge reports an export error.
   * @param message Message to display to the user.
   */
  void exportError(const QString &message);

private slots:
  /**
   * @brief Consume buffered NDJSON export output from the bridge process.
   */
  void handleExportStdout();
  /**
   * @brief Finalize export state after the bridge process exits.
   * @param exitCode Exit code reported by the export bridge process.
   */
  void handleExportFinished(int exitCode);

private:
  /**
   * @brief Return the Python executable used to launch the bridge module.
   * @return Python executable used to launch the bridge.
   */
  QString pythonExecutable() const;
  /**
   * @brief Build the argument vector for one bridge subcommand.
   * @param command Python bridge subcommand name.
   * @return Arguments selecting the bridge module and subcommand.
   */
  QStringList bridgeArguments(const QString &command) const;
  /**
   * @brief Parse and dispatch a single NDJSON export event line.
   * @param line One UTF-8 NDJSON export event.
   */
  void processExportLine(const QByteArray &line);
  /**
   * @brief Dispose the finished export process and clear client-side state.
   */
  void cleanupExportProcess();

  mutable QProcess *exportProcess_{nullptr};
  mutable QByteArray exportStdoutBuffer_;
  mutable bool exportSawTerminalEvent_{false};
  mutable bool cancelRequested_{false};
};
