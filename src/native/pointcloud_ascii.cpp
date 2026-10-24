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

#include "bindings.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <new>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

/** @brief Validated layout of one field in a packed little-endian point. */
struct Field {
    int datatype;
    std::size_t count;
    std::size_t offset;
};

/** @brief Byte widths indexed by the sensor_msgs/PointField datatype constants. */
constexpr std::size_t widths[] = {0, 1, 1, 2, 2, 4, 4, 4, 8};

/**
 * @brief Read an unaligned little-endian scalar without aliasing violations.
 * @tparam T Scalar type to decode.
 * @param data Source buffer containing at least sizeof(T) bytes.
 * @return Decoded scalar in host byte order.
 */
template<typename T>
T read_scalar(const unsigned char* data) {
    T value;
    const std::uint16_t endian = 1;
    if (*reinterpret_cast<const unsigned char*>(&endian) == 1) {
        std::memcpy(&value, data, sizeof(T));
    } else {
        unsigned char bytes[sizeof(T)];
        std::reverse_copy(data, data + sizeof(T), bytes);
        std::memcpy(&value, bytes, sizeof(T));
    }
    return value;
}

/**
 * @brief Append an integer using locale-independent decimal notation.
 * @tparam T Integer type to format.
 * @param output Destination text buffer.
 * @param value Integer value to append.
 */
template<typename T>
void append_integer(std::string& output, T value) {
    char buffer[32];
    const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value);
    output.append(buffer, result.ptr);
}

/**
 * @brief Append a float with Python's shortest-double notation and thresholds.
 *
 * FLOAT32 inputs are promoted to double before formatting, matching NumPy's
 * scalar.item() and struct.unpack(). Signed zero retains its sign; all NaNs
 * use "nan". Python switches to scientific notation below 1e-4 or at 1e16.
 *
 * @param output Destination text buffer.
 * @param value Double-precision value to append.
 */
void append_float(std::string& output, double value) {
    if (std::isnan(value)) {
        output += "nan";
        return;
    }
    if (std::isinf(value)) {
        output += std::signbit(value) ? "-inf" : "inf";
        return;
    }
    char buffer[64];
    const auto result = std::to_chars(
        buffer, buffer + sizeof(buffer), value, std::chars_format::scientific);
    if (result.ec != std::errc{}) throw std::runtime_error("Float formatting failed");
    const char* start = buffer + (buffer[0] == '-' ? 1 : 0);
    const char* exponent_start = std::find(start, static_cast<const char*>(result.ptr), 'e');
    int exponent = 0;
    std::from_chars(exponent_start + 2, static_cast<const char*>(result.ptr), exponent);
    if (exponent_start[1] == '-') exponent = -exponent;
    if (exponent < -4 || exponent >= 16) {
        output.append(buffer, result.ptr);
        return;
    }
    if (buffer[0] == '-') output.push_back('-');
    char digits[24];
    std::size_t length = 0;
    for (const char* p = start; p < exponent_start; ++p) {
        if (*p != '.') digits[length++] = *p;
    }
    const int position = exponent + 1;
    if (position <= 0) {
        output += "0.";
        output.append(static_cast<std::size_t>(-position), '0');
        output.append(digits, length);
    } else if (static_cast<std::size_t>(position) >= length) {
        output.append(digits, length);
        output.append(static_cast<std::size_t>(position) - length, '0');
        output += ".0";
    } else {
        output.append(digits, static_cast<std::size_t>(position));
        output.push_back('.');
        output.append(digits + position, length - position);
    }
}

/**
 * @brief Check floating fields for NaN when XYZ export requests row filtering.
 * @param point Packed point storage.
 * @param fields Validated field descriptions.
 * @return True if any floating component is NaN.
 */
bool has_nan(const unsigned char* point, const std::vector<Field>& fields) {
    for (const auto& field : fields) {
        for (std::size_t i = 0; i < field.count; ++i) {
            const auto* data = point + field.offset + i * widths[field.datatype];
            if ((field.datatype == 7 && std::isnan(read_scalar<float>(data))) ||
                (field.datatype == 8 && std::isnan(read_scalar<double>(data)))) return true;
        }
    }
    return false;
}

/**
 * @brief Append packed point rows without using Python objects or Python APIs.
 * @param data Immutable packed little-endian point buffer.
 * @param count Number of points.
 * @param step Packed bytes per point.
 * @param fields Validated field descriptions in output order.
 * @param skip_nan Whether to omit rows containing NaN coordinates.
 * @param output Destination text buffer.
 */
void format_rows(const unsigned char* data, std::size_t count, std::size_t step,
                 const std::vector<Field>& fields, bool skip_nan, std::string& output) {
    for (std::size_t point_index = 0; point_index < count; ++point_index) {
        const auto* point = data + point_index * step;
        if (skip_nan && has_nan(point, fields)) continue;
        bool first = true;
        for (const auto& field : fields) {
            for (std::size_t i = 0; i < field.count; ++i) {
                if (!first) output.push_back(' ');
                first = false;
                const auto* value = point + field.offset + i * widths[field.datatype];
                switch (field.datatype) {
                    case 1: append_integer(output, read_scalar<std::int8_t>(value)); break;
                    case 2: append_integer(output, read_scalar<std::uint8_t>(value)); break;
                    case 3: append_integer(output, read_scalar<std::int16_t>(value)); break;
                    case 4: append_integer(output, read_scalar<std::uint16_t>(value)); break;
                    case 5: append_integer(output, read_scalar<std::int32_t>(value)); break;
                    case 6: append_integer(output, read_scalar<std::uint32_t>(value)); break;
                    case 7: append_float(output, read_scalar<float>(value)); break;
                    case 8: append_float(output, read_scalar<double>(value)); break;
                }
            }
        }
        output.push_back('\n');
    }
}

}  // namespace

/**
 * @brief Format packed point fields as ASCII with Python-compatible decimals.
 *
 * Accepts immutable bytes, a tuple of (PointField datatype, component count)
 * tuples, a point count, and an optional flag for omitting NaN rows. Validates
 * all lengths and field types before releasing the GIL for the formatting loop.
 *
 * @param args Python positional argument tuple.
 * @return New reference to ASCII bytes, or nullptr with a Python exception set.
 */
PyObject* format_pointcloud_ascii(PyObject*, PyObject* args) {
    PyObject* input = nullptr;
    PyObject* schema = nullptr;
    Py_ssize_t point_count = 0;
    int skip_nan = 0;
    if (!PyArg_ParseTuple(args, "SO!n|p:format_pointcloud_ascii",
                          &input, &PyTuple_Type, &schema, &point_count, &skip_nan)) return nullptr;
    if (point_count < 0) {
        PyErr_SetString(PyExc_ValueError, "Point count must be nonnegative");
        return nullptr;
    }
    try {
        std::vector<Field> fields;
        fields.reserve(static_cast<std::size_t>(PyTuple_GET_SIZE(schema)));
        Py_ssize_t step = 0;
        for (Py_ssize_t i = 0; i < PyTuple_GET_SIZE(schema); ++i) {
            int datatype;
            Py_ssize_t count;
            PyObject* entry = PyTuple_GET_ITEM(schema, i);
            if (!PyTuple_Check(entry)) {
                PyErr_SetString(PyExc_TypeError, "Each field must be a (datatype, count) tuple");
                return nullptr;
            }
            if (!PyArg_ParseTuple(entry, "in:field", &datatype, &count))
                return nullptr;
            if (datatype < 1 || datatype > 8 || count < 1) {
                PyErr_SetString(PyExc_ValueError, "Unsupported field datatype or component count");
                return nullptr;
            }
            if (count > (PY_SSIZE_T_MAX - step) / static_cast<Py_ssize_t>(widths[datatype])) {
                PyErr_SetString(PyExc_OverflowError, "Packed point size exceeds the buffer size limit");
                return nullptr;
            }
            fields.push_back({datatype, static_cast<std::size_t>(count), static_cast<std::size_t>(step)});
            step += count * static_cast<Py_ssize_t>(widths[datatype]);
        }
        const Py_ssize_t size = PyBytes_GET_SIZE(input);
        if ((point_count && step > size / point_count) || step * point_count != size) {
            PyErr_SetString(PyExc_ValueError, "Packed data length does not match the point layout");
            return nullptr;
        }
        const auto* data = reinterpret_cast<const unsigned char*>(PyBytes_AS_STRING(input));
        std::string output;
        int failure = 0;
        Py_BEGIN_ALLOW_THREADS
        try {
            output.reserve(static_cast<std::size_t>(size));
            format_rows(data, static_cast<std::size_t>(point_count), static_cast<std::size_t>(step),
                        fields, skip_nan != 0, output);
        } catch (const std::bad_alloc&) {
            failure = 1;
        } catch (const std::length_error&) {
            failure = 2;
        } catch (...) {
            failure = 3;
        }
        Py_END_ALLOW_THREADS
        if (failure == 1) return PyErr_NoMemory();
        if (failure == 2 || output.size() > static_cast<std::size_t>(PY_SSIZE_T_MAX)) {
            PyErr_SetString(PyExc_OverflowError, "ASCII output exceeds the buffer size limit");
            return nullptr;
        }
        if (failure) {
            PyErr_SetString(PyExc_RuntimeError, "Point cloud ASCII formatting failed");
            return nullptr;
        }
        return PyBytes_FromStringAndSize(output.data(), static_cast<Py_ssize_t>(output.size()));
    } catch (const std::bad_alloc&) {
        return PyErr_NoMemory();
    } catch (const std::length_error&) {
        PyErr_SetString(PyExc_OverflowError, "Field layout exceeds the buffer size limit");
        return nullptr;
    }
}
