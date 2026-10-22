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

#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace {

/**
 * @brief Compress a byte buffer using the original Python implementation's LZF algorithm.
 *
 * Preserves match selection and the resulting byte stream, including inputs
 * whose compressed representation is larger than the source. All match
 * references point into the input, so overlapping matches need no output reads.
 *
 * @param data Immutable source buffer containing at least size bytes.
 * @param size Number of source bytes, bounded by PY_SSIZE_T_MAX.
 * @param output Destination with capacity of at least size + ceil(size / 32).
 * @return Number of compressed bytes written to output.
 */
std::size_t compress(const unsigned char* data, std::size_t size, unsigned char* output) {
    std::array<Py_ssize_t, 1 << 14> table;
    table.fill(-1);
    std::size_t i = 0, written = 0, literal_start = 0, literals = 0;
    auto flush_literals = [&]() {
        if (literals) {
            output[written++] = static_cast<unsigned char>(literals - 1);
            std::memcpy(output + written, data + literal_start, literals);
            written += literals;
            literals = 0;
        }
    };

    while (i < size) {
        if (size - i >= 3) {
            const std::uint32_t value = (std::uint32_t(data[i]) << 16)
                                     | (std::uint32_t(data[i + 1]) << 8) | data[i + 2];
            // Widen before multiplying to match Python's unbounded integers.
            const auto slot = ((std::uint64_t(value) * 57321) >> 9) & ((1 << 14) - 1);
            const auto ref = table[slot];
            table[slot] = static_cast<Py_ssize_t>(i);
            if (ref >= 0) {
                const auto reference = static_cast<std::size_t>(ref);
                const auto offset = i - reference - 1;
                if (offset < (1 << 13) && std::memcmp(data + reference, data + i, 3) == 0) {
                    flush_literals();
                    std::size_t match = 3;
                    while (match < 264 && match < size - i && data[reference + match] == data[i + match])
                        ++match;
                    const auto length = match - 2;
                    output[written++] = static_cast<unsigned char>(
                        ((length < 7 ? length : 7) << 5) | (offset >> 8));
                    if (length >= 7) output[written++] = static_cast<unsigned char>(length - 7);
                    output[written++] = static_cast<unsigned char>(offset & 255);
                    i += match;
                    continue;
                }
            }
        }
        if (!literals) literal_start = i;
        ++literals;
        ++i;
        if (literals == 32) flush_literals();
    }
    flush_literals();
    return written;
}

/**
 * @brief Expose LZF compression as a Python function accepting immutable bytes.
 *
 * The input is retained by args and the output is private while the GIL is
 * released. Python API calls and allocation occur only while holding the GIL.
 *
 * @param args Python argument tuple containing one bytes object.
 * @return New reference to compressed bytes, or nullptr with a Python exception
 *         set for invalid arguments, allocation failure, or size overflow.
 */
PyObject* lzf_compress(PyObject*, PyObject* args) {
    PyObject* input = nullptr;
    // Immutable bytes remain safe to read while the GIL is released. The call
    // arguments retain the input, and output storage is private to this call.
    if (!PyArg_ParseTuple(args, "S:lzf_compress", &input)) return nullptr;
    const Py_ssize_t size = PyBytes_GET_SIZE(input);
    if (!size) return PyBytes_FromStringAndSize("", 0);
    const Py_ssize_t overhead = size / 32 + (size % 32 != 0);
    if (size > PY_SSIZE_T_MAX - overhead) {
        PyErr_SetString(PyExc_OverflowError, "LZF output exceeds the Python bytes size limit");
        return nullptr;
    }
    PyObject* output = PyBytes_FromStringAndSize(nullptr, size + overhead);
    if (!output) return nullptr;
    const auto* source = reinterpret_cast<const unsigned char*>(PyBytes_AS_STRING(input));
    auto* destination = reinterpret_cast<unsigned char*>(PyBytes_AS_STRING(output));
    std::size_t written;
    Py_BEGIN_ALLOW_THREADS
    written = compress(source, static_cast<std::size_t>(size), destination);
    Py_END_ALLOW_THREADS

    PyObject* result = PyBytes_FromStringAndSize(
        PyBytes_AS_STRING(output), static_cast<Py_ssize_t>(written));
    Py_DECREF(output);
    return result;
}

/** @brief Python-callable functions exported by the compression module. */
PyMethodDef methods[] = {
    {"lzf_compress", lzf_compress, METH_VARARGS,
     "Compress bytes to the same LZF stream as the Python PCD compressor."},
    {nullptr, nullptr, 0, nullptr},
};

/** @brief Definition of the stateless native PCD compression module. */
PyModuleDef module = {
    PyModuleDef_HEAD_INIT, "_native", "Native PCD compression accelerator.",
    -1, methods, nullptr, nullptr, nullptr, nullptr,
};

}  // namespace

/**
 * @brief Initialize the ros2_unbag._native extension module.
 * @return New reference to the module, or nullptr with a Python exception set.
 */
PyMODINIT_FUNC PyInit__native() {
    return PyModule_Create(&module);
}
