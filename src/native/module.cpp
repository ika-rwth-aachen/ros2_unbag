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

namespace {

/** @brief Python-callable native point cloud helpers. */
PyMethodDef methods[] = {
    {"lzf_compress", lzf_compress, METH_VARARGS,
     "Compress bytes to the same LZF stream as the Python PCD compressor."},
    {"format_pointcloud_ascii", format_pointcloud_ascii, METH_VARARGS,
     "Format packed point fields as Python-compatible ASCII rows."},
    {nullptr, nullptr, 0, nullptr},
};

/** @brief Definition of the stateless native point cloud module. */
PyModuleDef module = {
    PyModuleDef_HEAD_INIT, "_native", "Native point cloud helpers.",
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
