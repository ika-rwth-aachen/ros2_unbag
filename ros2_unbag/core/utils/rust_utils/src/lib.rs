/*
MIT License

Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


use pyo3::prelude::*;
use pyo3::types::{PyAny, PyDict, PyList, PyBytes};
use pyo3::exceptions::PyValueError;
use serde_yaml::{Value as YamlValue, Mapping, Number, to_string};
use byteorder::{LittleEndian, WriteBytesExt};


/// Serialize a Python dictionary to a YAML string.
///
/// Args:
///     dict (dict): A Python dictionary to serialize.
///
/// Returns:
///     str: The serialized YAML string.
///
/// Raises:
///     ValueError: If the serialization fails or if the input is not valid.
#[pyfunction]
fn serialize_yaml(dict: &PyDict) -> PyResult<String> {
    let value = convert_pyany_to_yaml_value(dict)?;
    to_string(&value)
        .map_err(|e| PyErr::new::<PyValueError, _>(e.to_string()))
}


/// Recursively convert a Python object (PyAny) into a serde_yaml::Value.
///
/// Supported types:
/// - dict → Mapping
/// - list → Sequence
/// - bool, int → Number
/// - float → String (to maintain YAML compatibility)
/// - str → String
/// - other → Null
///
/// Args:
///     obj (&PyAny): The Python object to convert.
///
/// Returns:
///     YamlValue: The corresponding YAML-compatible value.
fn convert_pyany_to_yaml_value(obj: &PyAny) -> PyResult<YamlValue> {
    if obj.is_instance_of::<PyDict>() {
        let dict = obj.downcast::<PyDict>()?;
        let mut map = Mapping::new();
        for (k, v) in dict.iter() {
            let key = YamlValue::String(k.str()?.to_str()?.to_string());
            let value = convert_pyany_to_yaml_value(v)?;
            map.insert(key, value);
        }
        Ok(YamlValue::Mapping(map))
    } else if obj.is_instance_of::<PyList>() {
        let list = obj.downcast::<PyList>()?;
        let mut vec = Vec::new();
        for item in list.iter() {
            vec.push(convert_pyany_to_yaml_value(item)?);
        }
        Ok(YamlValue::Sequence(vec))
    } else if let Ok(val) = obj.extract::<bool>() {
        Ok(YamlValue::Bool(val))
    } else if let Ok(val) = obj.extract::<i64>() {
        Ok(YamlValue::Number(Number::from(val)))
    } else if let Ok(val) = obj.extract::<f64>() {
        // Serialize float as string to preserve compatibility with YAML
        Ok(YamlValue::String(val.to_string()))
    } else if let Ok(val) = obj.str() {
        Ok(YamlValue::String(val.to_str()?.to_string()))
    } else {
        Ok(YamlValue::Null)
    }
}


/// Pack point cloud data into a binary format.
///
/// Args:
///     data (bytes): The raw point cloud data as bytes.
///     offsets (list of int): List of offsets for each field in the point cloud.
///     fmts (list of str): List of format characters for each field (e.g., "f", "B", etc.).
///     point_step (int): The size of a single point in bytes.
///
/// Returns:
///     bytes: The packed point cloud data as bytes.
///
/// Raises:
///     ValueError: If the input data is not bytes-like or if offsets and formats do not match.
#[pyfunction]
fn pack_pointcloud_data<'py>(
    py: Python<'py>,
    data: &PyAny,              // Python bytes or bytearray
    offsets: Vec<usize>,       // per-field offset
    fmts: Vec<String>,         // per-field format char ("f", "B", etc.)
    point_step: usize          // size of a point
) -> PyResult<&'py PyBytes> {
    let raw = data
        .extract::<&[u8]>()
        .map_err(|_| PyValueError::new_err("Expected bytes-like object for 'data'"))?;

    if offsets.len() != fmts.len() {
        return Err(PyValueError::new_err("offsets and fmts must have the same length"));
    }

    let num_points = raw.len() / point_step;
    let mut out = Vec::with_capacity(raw.len()); // output buffer

    for i in 0..num_points {
        let base = i * point_step;
        for (j, fmt) in fmts.iter().enumerate() {
            let off = base + offsets[j];
            match fmt.as_str() {
                "B" => out.write_u8(raw[off])?,
                "H" => {
                    let val = u16::from_le_bytes(raw[off..off+2].try_into().unwrap());
                    out.write_u16::<LittleEndian>(val)?;
                },
                "I" => {
                    let val = u32::from_le_bytes(raw[off..off+4].try_into().unwrap());
                    out.write_u32::<LittleEndian>(val)?;
                },
                "b" => out.write_i8(raw[off] as i8)?,
                "h" => {
                    let val = i16::from_le_bytes(raw[off..off+2].try_into().unwrap());
                    out.write_i16::<LittleEndian>(val)?;
                },
                "i" => {
                    let val = i32::from_le_bytes(raw[off..off+4].try_into().unwrap());
                    out.write_i32::<LittleEndian>(val)?;
                },
                "f" => {
                    let val = f32::from_le_bytes(raw[off..off+4].try_into().unwrap());
                    out.write_f32::<LittleEndian>(val)?;
                },
                _ => return Err(PyValueError::new_err(format!("Unsupported fmt: {}", fmt))),
            }
        }
    }

    Ok(PyBytes::new(py, &out))
}


/// Python module definition for `rust_utils`.
///
/// This module exposes the `serialize_yaml` function to Python.
#[pymodule]
fn rust_utils(_py: Python<'_>, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(serialize_yaml, m)?)?;
    m.add_function(wrap_pyfunction!(pack_pointcloud_data, m)?)?;
    Ok(())
}
