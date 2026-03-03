# MIT License

# Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Routine Args Widget Module.

Provides RoutineArgsWidget, a dynamic form widget that auto-generates input
controls for the extra keyword arguments declared by an ExportRoutine function.

The widget mirrors the processor argument pattern: it queries ExportRoutine.get_args()
for the current message type and format, then builds a labelled row for each parameter
using an appropriate input control (combo box for known-choice parameters, spin boxes
for numeric types, checkbox for bool, and a line edit for everything else).

Usage in TopicSettingsWidget:
    self.routine_args_widget = RoutineArgsWidget(topic_type, fmt)
    self.routine_args_widget.args_changed.connect(self._emit_change)
    args_dict = self.routine_args_widget.get_args()   # -> dict[str, Any]
    self.routine_args_widget.set_args({"colormap": "turbo", "width": 1920})
"""

import inspect
from typing import Any, Dict, Optional

from PySide6 import QtCore, QtWidgets

from ros2_unbag.core.routines.base import ExportRoutine

__all__ = ["RoutineArgsWidget"]

# Known enumerated choices for specific parameter names (independent of message type)
_PARAM_CHOICES: Dict[str, list] = {
    "colormap": [
        "viridis", "jet", "plasma", "inferno", "turbo",
        "magma", "rainbow", "hot", "coolwarm", "hsv",
    ],
    "projection": ["topdown", "front", "side", "matplotlib3d"],
    "bg_color":   ["black", "white"],
}


class RoutineArgsWidget(QtWidgets.QWidget):
    """
    Auto-generated form widget for per-format routine keyword arguments.

    Introspects the registered ExportRoutine for the given message type and
    format, then creates an appropriate input control for each extra parameter
    beyond the four fixed positional ones (msg, path, fmt, metadata).

    Supported parameter types:
    - Parameters whose name matches a key in ``_PARAM_CHOICES`` → QComboBox
    - ``bool`` annotation → QCheckBox
    - ``int`` annotation → QSpinBox
    - ``float`` or ``Optional[float]`` annotation → QDoubleSpinBox
      (with an "auto" checkbox when the default is None)
    - Everything else → QLineEdit

    Signals:
        args_changed (): Emitted whenever any input value changes.
    """

    args_changed = QtCore.Signal()

    def __init__(self, topic_type: str, fmt: str, parent=None):
        """
        Initialise the widget and build input controls for the given routine.

        Args:
            topic_type (str): ROS2 message type string.
            fmt (str): Export format string.
            parent: Optional Qt parent widget.

        Returns:
            None
        """
        super().__init__(parent)
        self.topic_type = topic_type
        self.fmt = fmt
        self._inputs: Dict[str, QtWidgets.QWidget] = {}  # param_name -> primary input widget
        self._auto_checks: Dict[str, QtWidgets.QCheckBox] = {}  # for Optional[float] params

        self._form = QtWidgets.QFormLayout(self)
        self._form.setContentsMargins(0, 0, 0, 0)
        self._form.setFieldGrowthPolicy(QtWidgets.QFormLayout.ExpandingFieldsGrow)
        self._form.setSpacing(4)

        self._build()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_args(self) -> Dict[str, Any]:
        """
        Collect current values from all input controls.

        Returns:
            dict: Mapping of parameter name to its current value.  Optional
                  float parameters with "auto" checked are returned as None.
        """
        result = {}
        for name, widget in self._inputs.items():
            # Optional float with auto checkbox
            if name in self._auto_checks and self._auto_checks[name].isChecked():
                result[name] = None
                continue

            if isinstance(widget, QtWidgets.QCheckBox):
                result[name] = widget.isChecked()
            elif isinstance(widget, QtWidgets.QSpinBox):
                result[name] = widget.value()
            elif isinstance(widget, QtWidgets.QDoubleSpinBox):
                result[name] = widget.value()
            elif isinstance(widget, QtWidgets.QComboBox):
                result[name] = widget.currentText()
            else:
                text = widget.text().strip()
                result[name] = text if text else None
        return result

    def set_args(self, args: Optional[Dict[str, Any]]) -> None:
        """
        Populate input controls from a dictionary of argument values.

        Unknown keys are silently ignored.  None values set the "auto"
        checkbox (if available) or leave the field at its default.

        Args:
            args (dict or None): Argument name → value mapping to apply.

        Returns:
            None
        """
        if not args:
            return
        for name, value in args.items():
            if name not in self._inputs:
                continue
            widget = self._inputs[name]

            if value is None:
                if name in self._auto_checks:
                    self._auto_checks[name].setChecked(True)
                    self._inputs[name].setEnabled(False)
                continue

            # Clear auto checkbox if a concrete value is provided
            if name in self._auto_checks:
                self._auto_checks[name].setChecked(False)
                widget.setEnabled(True)

            if isinstance(widget, QtWidgets.QCheckBox):
                widget.setChecked(bool(value))
            elif isinstance(widget, QtWidgets.QSpinBox):
                widget.setValue(int(value))
            elif isinstance(widget, QtWidgets.QDoubleSpinBox):
                widget.setValue(float(value))
            elif isinstance(widget, QtWidgets.QComboBox):
                idx = widget.findText(str(value))
                if idx >= 0:
                    widget.setCurrentIndex(idx)
                else:
                    widget.setCurrentText(str(value))
            else:
                widget.setText(str(value))

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _build(self) -> None:
        """Build one form row per extra parameter reported by ExportRoutine.get_args()."""
        args = ExportRoutine.get_args(self.topic_type, self.fmt)
        if not args:
            no_args_label = QtWidgets.QLabel("No additional options.")
            no_args_label.setStyleSheet("color: #6b7280; font-style: italic;")
            self._form.addRow(no_args_label)
            return

        for name, (param, doc) in args.items():
            row_widget, auto_check = self._make_input(name, param, doc)
            self._inputs[name] = row_widget
            if auto_check is not None:
                self._auto_checks[name] = auto_check

            label_text = name
            if param.default is inspect.Parameter.empty:
                label_text += " *"  # mark required parameters

            label = QtWidgets.QLabel(label_text)
            if doc:
                label.setToolTip(doc)
                row_widget.setToolTip(doc)

            if auto_check is not None:
                # Wrap spin box + "auto" checkbox in a horizontal layout
                container = QtWidgets.QWidget()
                h = QtWidgets.QHBoxLayout(container)
                h.setContentsMargins(0, 0, 0, 0)
                h.setSpacing(6)
                h.addWidget(row_widget, 1)
                h.addWidget(auto_check)
                self._form.addRow(label, container)
            else:
                self._form.addRow(label, row_widget)

    def _make_input(
        self, name: str, param: inspect.Parameter, doc: str
    ):
        """
        Create the appropriate input widget for a single parameter.

        Args:
            name (str): Parameter name.
            param (inspect.Parameter): Inspect parameter object.
            doc (str): Documentation string for the parameter.

        Returns:
            tuple: (primary_widget, auto_checkbox_or_None)
        """
        annotation = param.annotation
        default = param.default if param.default is not inspect.Parameter.empty else None

        # --- Known-choices combo box ---
        if name in _PARAM_CHOICES:
            combo = QtWidgets.QComboBox()
            combo.addItems(_PARAM_CHOICES[name])
            if default is not None:
                idx = combo.findText(str(default))
                if idx >= 0:
                    combo.setCurrentIndex(idx)
            combo.currentTextChanged.connect(self.args_changed)
            return combo, None

        # Resolve Optional[X] → X
        inner, is_optional = _unwrap_optional(annotation)

        # --- bool ---
        if inner is bool:
            cb = QtWidgets.QCheckBox()
            cb.setChecked(bool(default) if default is not None else False)
            cb.stateChanged.connect(self.args_changed)
            return cb, None

        # --- int ---
        if inner is int:
            sb = QtWidgets.QSpinBox()
            sb.setRange(1, 99999)
            sb.setSingleStep(1)
            if default is not None:
                sb.setValue(int(default))
            sb.valueChanged.connect(self.args_changed)
            return sb, None

        # --- float (and Optional[float]) ---
        if inner is float:
            dsb = QtWidgets.QDoubleSpinBox()
            dsb.setRange(-1e9, 1e9)
            dsb.setDecimals(3)
            dsb.setSingleStep(1.0)
            auto_check = None
            if is_optional or default is None:
                dsb.setValue(0.0)
                dsb.setEnabled(False)
                auto_check = QtWidgets.QCheckBox("auto")
                auto_check.setChecked(True)
                auto_check.stateChanged.connect(
                    lambda state, w=dsb: w.setEnabled(state == 0)
                )
                auto_check.stateChanged.connect(lambda _: self.args_changed.emit())
            else:
                dsb.setValue(float(default))
            dsb.valueChanged.connect(self.args_changed)
            return dsb, auto_check

        # --- fallback: QLineEdit ---
        le = QtWidgets.QLineEdit()
        placeholder_parts = []
        if doc:
            placeholder_parts.append(doc)
        if default is not None:
            placeholder_parts.append(f"default: {default}")
        if annotation is not inspect.Parameter.empty:
            type_name = getattr(annotation, "__name__", str(annotation))
            placeholder_parts.append(f"type: {type_name}")
        le.setPlaceholderText(" — ".join(placeholder_parts))
        if default is not None:
            le.setText(str(default))
        le.textChanged.connect(self.args_changed)
        return le, None


# ---------------------------------------------------------------------------
# Utility
# ---------------------------------------------------------------------------

def _unwrap_optional(annotation):
    """
    Detect Optional[X] (i.e. Union[X, None]) and return (inner_type, True).
    For a plain type return (annotation, False).
    For inspect.Parameter.empty return (None, False).

    Args:
        annotation: Type annotation from an inspect.Parameter.

    Returns:
        tuple: (inner_type_or_None, is_optional: bool)
    """
    import typing

    if annotation is inspect.Parameter.empty:
        return None, False

    origin = getattr(annotation, "__origin__", None)
    args = getattr(annotation, "__args__", ())

    # typing.Optional[X] == Union[X, None]
    if origin is typing.Union and len(args) == 2 and type(None) in args:
        inner = next(a for a in args if a is not type(None))
        return inner, True

    return annotation, False
