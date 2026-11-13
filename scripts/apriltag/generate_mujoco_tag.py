#!/usr/bin/env python3
"""Generate a tag36h11 ID 5 MJCF from the installed AprilTag codebook.

Each cell is a flat geom, so the viewer needs no texture files or UV mapping.
Black border width is 0.24 m; white quiet zone is included separately.
"""
import ctypes
import ctypes.util
from pathlib import Path


class Image(ctypes.Structure):
    _fields_ = [("width", ctypes.c_int), ("height", ctypes.c_int),
                ("stride", ctypes.c_int), ("buf", ctypes.POINTER(ctypes.c_uint8))]


def main():
    library = ctypes.util.find_library("apriltag")
    if not library:
        library = str(Path.home() / ".local/lib/libapriltag.so")
    lib = ctypes.CDLL(library)
    lib.tag36h11_create.restype = ctypes.c_void_p
    lib.apriltag_to_image.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.apriltag_to_image.restype = ctypes.POINTER(Image)
    utils_path = Path(library).with_name("libapriltag-utils.so")
    utils = ctypes.CDLL(str(utils_path)) if utils_path.exists() else lib
    utils.image_u8_destroy.argtypes = [ctypes.POINTER(Image)]
    lib.tag36h11_destroy.argtypes = [ctypes.c_void_p]
    family = lib.tag36h11_create()
    image = lib.apriltag_to_image(family, 5)
    pattern = image.contents
    pitch = 0.24 / 8
    lines = ['<mujoco>', '  <worldbody>',
             '    <!-- +x right, +y up, +z toward the robot; black border = 0.24 m. -->',
             '    <body name="apriltag_5" mocap="true" pos="2 0 0.5" quat="0.5 0.5 -0.5 -0.5">',
             '      <geom name="tag5_board" type="box" size="0.15 0.15 0.002"',
             '            rgba="1 1 1 1" contype="0" conaffinity="0" group="1"/>']
    for row in range(pattern.height):
        for col in range(pattern.width):
            if pattern.buf[row * pattern.stride + col] == 0:
                x = (col - (pattern.width - 1) / 2) * pitch
                y = ((pattern.height - 1) / 2 - row) * pitch
                lines.append(
                    f'      <geom type="box" pos="{x:.6f} {y:.6f} 0.0021" '
                    f'size="{pitch/2:.6f} {pitch/2:.6f} 0.0001" '
                    'rgba="0 0 0 1" contype="0" conaffinity="0" group="1"/>')
    lines += ['    </body>', '  </worldbody>', '</mujoco>', '']
    destination = Path(__file__).resolve().parents[2] / 'src/rl_sar_zoo/common/mjcf/apriltag_5.xml'
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text('\n'.join(lines))
    utils.image_u8_destroy(image)
    lib.tag36h11_destroy(family)
    print(destination)


if __name__ == '__main__':
    main()
