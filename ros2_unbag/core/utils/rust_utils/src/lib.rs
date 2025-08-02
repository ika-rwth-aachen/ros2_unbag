use pyo3::prelude::*;
use pyo3::types::{PyAny, PyDict, PyList};
use pyo3::exceptions::PyValueError;
use serde_yaml::{Value as YamlValue, Mapping, Number, to_string};

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

/// Python module definition for `rust_utils`.
///
/// This module exposes the `serialize_yaml` function to Python.
#[pymodule]
fn rust_utils(_py: Python<'_>, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(serialize_yaml, m)?)?;
    Ok(())
}
