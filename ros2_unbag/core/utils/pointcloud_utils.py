# MIT License

# Copyright (c) 2026 Institute for Automotive Engineering (ika), RWTH Aachen University

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

import struct

import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

try:
    from ros2_unbag._native import lzf_compress as _native_lzf_compress
except ImportError:
    _native_lzf_compress = None


_POINT_FIELD_DTYPES = {
    PointField.INT8: ("i1", 1, "I", "b"),
    PointField.UINT8: ("u1", 1, "U", "B"),
    PointField.INT16: ("i2", 2, "I", "h"),
    PointField.UINT16: ("u2", 2, "U", "H"),
    PointField.INT32: ("i4", 4, "I", "i"),
    PointField.UINT32: ("u4", 4, "U", "I"),
    PointField.FLOAT32: ("f4", 4, "F", "f"),
    PointField.FLOAT64: ("f8", 8, "F", "d"),
}


def quaternion_matrix(quaternion):
    """
    Compute a 4×4 transformation matrix from a quaternion [x, y, z, w].

    Args:
        quaternion: Sequence of 4 floats [x, y, z, w].

    Returns:
        numpy.ndarray: 4x4 transformation matrix.
    """
    x, y, z, w = quaternion
    N = x * x + y * y + z * z + w * w
    if N < np.finfo(float).eps:
        return np.eye(4)
    s = 2.0 / N
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s

    M = np.eye(4)
    M[0, 0] = 1 - (yy + zz)
    M[0, 1] = xy - wz
    M[0, 2] = xz + wy
    M[1, 0] = xy + wz
    M[1, 1] = 1 - (xx + zz)
    M[1, 2] = yz - wx
    M[2, 0] = xz - wy
    M[2, 1] = yz + wx
    M[2, 2] = 1 - (xx + yy)
    return M


def apply_pointcloud_transform(msg, translation, rotation):
    """
    Apply a rigid-body transform to all points in a PointCloud2 message.

    Args:
        msg: PointCloud2 message instance.
        translation: Iterable of 3 floats [x, y, z].
        rotation: Iterable of 4 floats [x, y, z, w] quaternion.

    Returns:
        PointCloud2: Transformed PointCloud2 message.

    Raises:
        ValueError: If coordinate fields are missing, are not scalar FLOAT32 or
            FLOAT64 values, or inputs are malformed.
    """
    translation = np.asarray(translation, dtype=float).reshape(-1)
    rotation = np.asarray(rotation, dtype=float).reshape(-1)
    if translation.size != 3:
        raise ValueError("Translation must have exactly three elements [x, y, z]")
    if rotation.size != 4:
        raise ValueError("Rotation must have exactly four elements [x, y, z, w]")

    transform_matrix = quaternion_matrix(rotation)
    transform_matrix[0:3, 3] = translation

    coordinates = {field.name: field for field in msg.fields if field.name in ('x', 'y', 'z')}
    if not all(name in coordinates for name in ('x', 'y', 'z')):
        raise ValueError("PointCloud2 message does not contain x, y, z fields")

    for field in coordinates.values():
        if field.datatype not in (PointField.FLOAT32, PointField.FLOAT64) or _field_count(field) != 1:
            raise ValueError(f"Coordinate field '{field.name}' must be scalar FLOAT32 or FLOAT64")
        if field.offset < 0 or field.offset + _field_byte_width(field) > msg.point_step:
            raise ValueError(f"Coordinate field '{field.name}' exceeds point_step")

    data = bytearray(msg.data)  # mutable copy
    if msg.width and msg.height:
        endian = ">" if msg.is_bigendian else "<"
        points = np.ones((msg.height, msg.width, 4), dtype=np.float64)
        views = []
        for axis, name in enumerate(('x', 'y', 'z')):
            field = coordinates[name]
            view = np.ndarray(
                shape=(msg.height, msg.width),
                dtype=_field_dtype(field, endian=endian),
                buffer=data,
                offset=field.offset,
                strides=(msg.row_step, msg.point_step),
            )
            views.append(view)
            points[..., axis] = view

        # Batch the original 4x4-by-vector operation. A matrix-by-matrix product
        # or expanded scalar formula can round differently, even in FLOAT32 output.
        transformed = (transform_matrix @ points[..., np.newaxis])[..., 0]
        for axis, view in enumerate(views):
            view[...] = transformed[..., axis]

    transformed_msg = PointCloud2()
    transformed_msg.header = msg.header
    transformed_msg.height = msg.height
    transformed_msg.width = msg.width
    transformed_msg.fields = msg.fields
    transformed_msg.is_bigendian = msg.is_bigendian
    transformed_msg.point_step = msg.point_step
    transformed_msg.row_step = msg.row_step
    transformed_msg.is_dense = msg.is_dense
    transformed_msg.data = bytes(data)

    return transformed_msg


def _field_count(field) -> int:
    """
    Normalize the declared element count for one PointCloud2 field.

    Args:
        field: PointField-like object.

    Returns:
        int: Field element count, defaulting to ``1``.
    """
    return field.count if getattr(field, "count", 0) > 0 else 1


def _field_dtype(field, *, endian: str) -> str:
    """
    Build a numpy dtype string for one PointCloud2 field.

    Args:
        field: PointField-like object.
        endian: Byte order prefix, ``"<"`` or ``">"``.

    Returns:
        str: Numpy dtype string.

    Raises:
        ValueError: If the PointField datatype is unsupported.
    """
    info = _POINT_FIELD_DTYPES.get(field.datatype)
    if info is None:
        raise ValueError(f"Unsupported PointField datatype: {field.datatype}")
    return endian + info[0]


def _field_byte_width(field) -> int:
    """
    Compute the packed byte width for one PCD field.

    Args:
        field: PointField-like object.

    Returns:
        int: Total field width in bytes.

    Raises:
        ValueError: If the PointField datatype is unsupported.
    """
    info = _POINT_FIELD_DTYPES.get(field.datatype)
    if info is None:
        raise ValueError(f"Unsupported PointField datatype: {field.datatype}")
    return info[1] * _field_count(field)


def _packed_field_layout(msg: PointCloud2) -> tuple[list[tuple[PointField, int, int]], int]:
    """
    Build the packed byte layout for one PCD point.

    Args:
        msg: PointCloud2 message instance.

    Returns:
        tuple[list[tuple[PointField, int, int]], int]:
            ``(layout, packed_point_step)`` where each layout entry is
            ``(field, packed_offset, field_width)``.
    """
    layout = []
    packed_offset = 0
    for field in msg.fields:
        field_width = _field_byte_width(field)
        layout.append((field, packed_offset, field_width))
        packed_offset += field_width
    return layout, packed_offset


def _point_byte_view(msg: PointCloud2) -> np.ndarray:
    """
    Create a strided byte-level view over the PointCloud2 point storage.

    Args:
        msg: PointCloud2 message instance.

    Returns:
        numpy.ndarray: ``(height, width, point_step)`` uint8 view.
    """
    point_count = msg.width * msg.height
    if point_count == 0:
        return np.empty((0, 0, msg.point_step), dtype=np.uint8)

    return np.ndarray(
        shape=(msg.height, msg.width, msg.point_step),
        dtype=np.uint8,
        buffer=memoryview(msg.data),
        strides=(msg.row_step, msg.point_step, 1),
    )


def _is_packed_little_endian_layout(msg: PointCloud2, packed_step: int) -> bool:
    """
    Return whether the PointCloud2 byte layout already matches packed PCD bytes.

    Args:
        msg: PointCloud2 message instance.
        packed_step: Expected packed point size.

    Returns:
        bool: ``True`` when no repacking or byte swapping is required.
    """
    if msg.is_bigendian or msg.point_step != packed_step:
        return False

    expected_offset = 0
    for field in msg.fields:
        if field.offset != expected_offset:
            return False
        expected_offset += _field_byte_width(field)
    return True


def _packed_point_byte_matrix(msg: PointCloud2) -> np.ndarray:
    """
    Convert PointCloud2 storage into a contiguous packed little-endian byte matrix.

    Args:
        msg: PointCloud2 message instance.

    Returns:
        numpy.ndarray: ``(point_count, packed_point_step)`` uint8 matrix.
    """
    layout, packed_step = _packed_field_layout(msg)
    point_count = msg.width * msg.height
    if point_count == 0:
        return np.empty((0, packed_step), dtype=np.uint8)

    source = _point_byte_view(msg)
    if _is_packed_little_endian_layout(msg, packed_step):
        return np.ascontiguousarray(source).reshape(point_count, packed_step)

    packed = np.empty((msg.height, msg.width, packed_step), dtype=np.uint8)
    for field, packed_offset, field_width in layout:
        field_bytes = source[..., field.offset:field.offset + field_width]
        item_size = _POINT_FIELD_DTYPES[field.datatype][1]
        if msg.is_bigendian and item_size > 1:
            reshaped = field_bytes.reshape(msg.height, msg.width, _field_count(field), item_size)
            field_bytes = reshaped[..., ::-1].reshape(msg.height, msg.width, field_width)
        packed[..., packed_offset:packed_offset + field_width] = field_bytes
    return packed.reshape(point_count, packed_step)


def _build_structured_dtype(msg: PointCloud2, *, endian: str, packed: bool) -> np.dtype:
    """
    Build a structured numpy dtype for the PointCloud2 layout.

    Args:
        msg: PointCloud2 message instance.
        endian: Byte order prefix, ``"<"`` or ``">"``.
        packed: If ``True``, build a tightly packed dtype for PCD output.

    Returns:
        numpy.dtype: Structured dtype representing one point.
    """
    names = []
    formats = []
    offsets = []
    packed_offset = 0

    for field in msg.fields:
        count = _field_count(field)
        base_dtype = np.dtype(_field_dtype(field, endian=endian))
        field_format = (base_dtype, (count,)) if count > 1 else base_dtype
        names.append(field.name)
        formats.append(field_format)
        offsets.append(packed_offset if packed else field.offset)
        packed_offset += base_dtype.itemsize * count

    return np.dtype(
        {
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": packed_offset if packed else msg.point_step,
        }
    )


def pointcloud2_to_structured_array(msg: PointCloud2) -> np.ndarray:
    """
    Convert a PointCloud2 message into a structured numpy array of points.

    Args:
        msg: PointCloud2 message instance.

    Returns:
        numpy.ndarray: Structured array with one element per point.
    """
    dtype = _build_structured_dtype(msg, endian=">" if msg.is_bigendian else "<", packed=False)
    point_count = msg.width * msg.height
    if point_count == 0:
        return np.empty((0,), dtype=dtype)

    rows = []
    buffer = memoryview(msg.data)
    for row in range(msg.height):
        row_offset = row * msg.row_step
        rows.append(
            np.ndarray(
                shape=(msg.width,),
                dtype=dtype,
                buffer=buffer,
                offset=row_offset,
                strides=(msg.point_step,),
            )
        )
    return rows[0].copy() if len(rows) == 1 else np.concatenate(rows).copy()


def pointcloud2_to_pcd_array(msg: PointCloud2) -> np.ndarray:
    """
    Convert a PointCloud2 message into a tightly packed little-endian PCD array.

    Args:
        msg: PointCloud2 message instance.

    Returns:
        numpy.ndarray: Structured array in PCD-compatible field layout.
    """
    packed_dtype = _build_structured_dtype(msg, endian="<", packed=True)
    point_count = msg.width * msg.height
    if point_count == 0:
        return np.empty((0,), dtype=packed_dtype)

    packed_bytes = _packed_point_byte_matrix(msg)
    return np.frombuffer(packed_bytes, dtype=packed_dtype, count=point_count)


def build_pcd_header(msg: PointCloud2, data_mode: str) -> bytes:
    """
    Build a PCD v0.7 header for a PointCloud2 message.

    Args:
        msg: PointCloud2 message instance.
        data_mode: PCD data mode: ``ascii``, ``binary``, or ``binary_compressed``.

    Returns:
        bytes: UTF-8 encoded PCD header.
    """
    fields = []
    sizes = []
    types = []
    counts = []
    for field in msg.fields:
        _, size, pcd_type, _ = _POINT_FIELD_DTYPES[field.datatype]
        fields.append(field.name)
        sizes.append(str(size))
        types.append(pcd_type)
        counts.append(str(_field_count(field)))

    point_count = msg.width * msg.height
    header = [
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        f"FIELDS {' '.join(fields)}",
        f"SIZE {' '.join(sizes)}",
        f"TYPE {' '.join(types)}",
        f"COUNT {' '.join(counts)}",
        f"WIDTH {msg.width}",
        f"HEIGHT {msg.height}",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {point_count}",
        f"DATA {data_mode}",
    ]
    return ("\n".join(header) + "\n").encode("utf-8")


def format_pcd_ascii_data(msg: PointCloud2) -> bytes:
    """
    Serialize a PointCloud2 message into ASCII PCD point data.

    Args:
        msg: PointCloud2 message instance.

    Returns:
        bytes: UTF-8 encoded ASCII point rows.
    """
    packed = pointcloud2_to_pcd_array(msg)
    lines = []
    for point in packed:
        values = []
        for field in msg.fields:
            value = point[field.name]
            if np.isscalar(value):
                values.append(str(value.item()))
            else:
                values.extend(str(component.item()) for component in np.asarray(value).reshape(-1))
        lines.append(" ".join(values))
    return ("\n".join(lines) + ("\n" if lines else "")).encode("utf-8")


def format_pcd_binary_data(msg: PointCloud2) -> bytes:
    """
    Serialize a PointCloud2 message into packed binary PCD point data.

    Args:
        msg: PointCloud2 message instance.

    Returns:
        bytes: Packed binary point payload.
    """
    return _packed_point_byte_matrix(msg).tobytes()


def _pcd_struct_of_arrays_bytes(msg: PointCloud2, packed: np.ndarray) -> bytes:
    """
    Reorder packed point data into the field-major layout used by PCD compression.

    Args:
        msg: PointCloud2 message instance.
        packed: Packed PCD structured array.

    Returns:
        bytes: Field-major byte stream.
    """
    chunks = []
    layout, _ = _packed_field_layout(msg)
    for _field, packed_offset, field_width in layout:
        chunks.append(np.ascontiguousarray(packed[:, packed_offset:packed_offset + field_width]).tobytes())
    return b"".join(chunks)


def _lzf_compress(data: bytes) -> bytes:
    """
    Compress data using the LZF format expected by binary-compressed PCD files.

    Args:
        data: Uncompressed byte payload.

    Returns:
        bytes: LZF-compressed payload.
    """
    if _native_lzf_compress is not None:
        return _native_lzf_compress(data)

    if not data:
        return b""

    hlog = 14
    hsize = 1 << hlog
    max_lit = 1 << 5
    max_off = 1 << 13
    max_ref = (1 << 8) + (1 << 3)
    table = [-1] * hsize
    output = bytearray()
    literals = bytearray()
    i = 0
    data_len = len(data)

    def flush_literals() -> None:
        while literals:
            chunk = literals[:max_lit]
            del literals[:max_lit]
            output.append(len(chunk) - 1)
            output.extend(chunk)

    while i < data_len:
        if i < data_len - 2:
            hval = (data[i] << 16) | (data[i + 1] << 8) | data[i + 2]
            slot = ((hval * 57321) >> 9) & (hsize - 1)
            ref = table[slot]
            table[slot] = i
            off = i - ref - 1

            if (
                ref >= 0
                and off < max_off
                and data[ref:ref + 3] == data[i:i + 3]
            ):
                flush_literals()
                match_len = 3
                max_match = min(data_len - i, max_ref)
                while match_len < max_match and data[ref + match_len] == data[i + match_len]:
                    match_len += 1

                length_code = match_len - 2
                if length_code < 7:
                    output.append((length_code << 5) | (off >> 8))
                else:
                    output.append((7 << 5) | (off >> 8))
                    output.append(length_code - 7)
                output.append(off & 0xFF)

                i += match_len
                continue

        literals.append(data[i])
        if len(literals) == max_lit:
            flush_literals()
        i += 1

    flush_literals()
    return bytes(output)


def format_pcd_binary_compressed_data(msg: PointCloud2) -> bytes:
    """
    Serialize a PointCloud2 message into PCD binary-compressed payload bytes.

    Args:
        msg: PointCloud2 message instance.

    Returns:
        bytes: Binary-compressed payload including size prefix words.
    """
    packed = _packed_point_byte_matrix(msg)
    uncompressed = _pcd_struct_of_arrays_bytes(msg, packed)
    compressed = _lzf_compress(uncompressed)
    return struct.pack("<II", len(compressed), len(uncompressed)) + compressed


def write_pointcloud_pcd(msg: PointCloud2, path, data_mode: str) -> None:
    """
    Write a PointCloud2 message to a PCD v0.7 file.

    Args:
        msg: PointCloud2 message instance.
        path: Output path, including the ``.pcd`` suffix.
        data_mode: PCD data mode: ``ascii``, ``binary``, or ``binary_compressed``.

    Returns:
        None
    """
    if data_mode == "ascii":
        payload = format_pcd_ascii_data(msg)
    elif data_mode == "binary":
        payload = format_pcd_binary_data(msg)
    elif data_mode == "binary_compressed":
        payload = format_pcd_binary_compressed_data(msg)
    else:
        raise ValueError(f"Unsupported PCD data mode: {data_mode}")

    with open(path, "wb") as stream:
        stream.write(build_pcd_header(msg, data_mode))
        stream.write(payload)
