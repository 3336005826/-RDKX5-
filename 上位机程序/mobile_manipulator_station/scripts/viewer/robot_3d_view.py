#!/usr/bin/env python3

import math
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from PyQt5 import QtCore, QtGui, QtWidgets

try:
    from OpenGL.GL import (
        GL_AMBIENT,
        GL_AMBIENT_AND_DIFFUSE,
        GL_BLEND,
        GL_COLOR_BUFFER_BIT,
        GL_COLOR_MATERIAL,
        GL_CULL_FACE,
        GL_DEPTH_BUFFER_BIT,
        GL_DEPTH_TEST,
        GL_DIFFUSE,
        GL_FRONT_AND_BACK,
        GL_LEQUAL,
        GL_LIGHT0,
        GL_LIGHTING,
        GL_LINES,
        GL_MODELVIEW,
        GL_MULTISAMPLE,
        GL_NORMALIZE,
        GL_ONE_MINUS_SRC_ALPHA,
        GL_POSITION,
        GL_PROJECTION,
        GL_SMOOTH,
        GL_SPECULAR,
        GL_SRC_ALPHA,
        GL_TRIANGLES,
        glBegin,
        glBlendFunc,
        glCallList,
        glClear,
        glClearColor,
        glColor4f,
        glColorMaterial,
        glDeleteLists,
        glDepthFunc,
        glDisable,
        glEnable,
        glEnd,
        glEndList,
        glGenLists,
        glLightfv,
        glLineWidth,
        glLoadIdentity,
        glMaterialfv,
        glMatrixMode,
        glNewList,
        glNormal3f,
        glPopMatrix,
        glPushMatrix,
        glRotatef,
        glScalef,
        glShadeModel,
        glTranslatef,
        glVertex3f,
        glViewport,
    )
    from OpenGL.GLU import gluLookAt, gluPerspective

    OPENGL_AVAILABLE = True
    OPENGL_ERROR = ""
except Exception as exc:  # pragma: no cover - environment dependent
    OPENGL_AVAILABLE = False
    OPENGL_ERROR = str(exc)


Vec3 = Tuple[float, float, float]
Mat4 = List[List[float]]


@dataclass
class MeshTriangle:
    normal: Vec3
    vertices: Tuple[Vec3, Vec3, Vec3]


@dataclass
class SceneMesh:
    color: Tuple[float, float, float, float]
    triangles: List[MeshTriangle]


@dataclass
class SceneBounds:
    center: Vec3
    radius: float


def _identity_matrix() -> Mat4:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _matrix_multiply(a: Mat4, b: Mat4) -> Mat4:
    result = [[0.0] * 4 for _ in range(4)]
    for row in range(4):
        for col in range(4):
            result[row][col] = sum(a[row][idx] * b[idx][col] for idx in range(4))
    return result


def _translation_matrix(x: float, y: float, z: float) -> Mat4:
    matrix = _identity_matrix()
    matrix[0][3] = x
    matrix[1][3] = y
    matrix[2][3] = z
    return matrix


def _rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> Mat4:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    rot_x = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, cr, -sr, 0.0],
        [0.0, sr, cr, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    rot_y = [
        [cp, 0.0, sp, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-sp, 0.0, cp, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    rot_z = [
        [cy, -sy, 0.0, 0.0],
        [sy, cy, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    return _matrix_multiply(rot_z, _matrix_multiply(rot_y, rot_x))


def _scale_matrix(x: float, y: float, z: float) -> Mat4:
    matrix = _identity_matrix()
    matrix[0][0] = x
    matrix[1][1] = y
    matrix[2][2] = z
    return matrix


def _transform_matrix(xyz: Vec3, rpy: Vec3, scale: Optional[Vec3] = None) -> Mat4:
    matrix = _matrix_multiply(_translation_matrix(*xyz), _rotation_matrix_from_rpy(*rpy))
    if scale is not None:
        matrix = _matrix_multiply(matrix, _scale_matrix(*scale))
    return matrix


def _transform_point(matrix: Mat4, point: Vec3) -> Vec3:
    x, y, z = point
    return (
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z + matrix[0][3],
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z + matrix[1][3],
        matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z + matrix[2][3],
    )


def _transform_direction(matrix: Mat4, vector: Vec3) -> Vec3:
    x, y, z = vector
    return (
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z,
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z,
        matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z,
    )


def _normalize(vector: Vec3) -> Vec3:
    length = math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)
    if length <= 1e-9:
        return (0.0, 0.0, 1.0)
    return (vector[0] / length, vector[1] / length, vector[2] / length)


def _parse_vec3(text: Optional[str], default: Vec3 = (0.0, 0.0, 0.0)) -> Vec3:
    if not text:
        return default
    parts = text.strip().split()
    if len(parts) != 3:
        return default
    try:
        return (float(parts[0]), float(parts[1]), float(parts[2]))
    except ValueError:
        return default


def _parse_rgba(text: Optional[str]) -> Tuple[float, float, float, float]:
    rgba = _parse_vec4(text)
    return rgba if rgba is not None else (0.82, 0.84, 0.88, 1.0)


def _parse_vec4(text: Optional[str]) -> Optional[Tuple[float, float, float, float]]:
    if not text:
        return None
    parts = text.strip().split()
    if len(parts) != 4:
        return None
    try:
        return (float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]))
    except ValueError:
        return None


def _compute_triangle_normal(v0: Vec3, v1: Vec3, v2: Vec3) -> Vec3:
    ax, ay, az = v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]
    bx, by, bz = v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]
    normal = (
        ay * bz - az * by,
        az * bx - ax * bz,
        ax * by - ay * bx,
    )
    return _normalize(normal)


def _load_binary_stl(path: Path, face_count: int) -> List[MeshTriangle]:
    triangles: List[MeshTriangle] = []
    with path.open("rb") as handle:
        handle.seek(84)
        for _ in range(face_count):
            chunk = handle.read(50)
            if len(chunk) < 50:
                break
            unpacked = struct.unpack("<12fH", chunk)
            normal = _normalize((unpacked[0], unpacked[1], unpacked[2]))
            v0 = (unpacked[3], unpacked[4], unpacked[5])
            v1 = (unpacked[6], unpacked[7], unpacked[8])
            v2 = (unpacked[9], unpacked[10], unpacked[11])
            if normal == (0.0, 0.0, 1.0):
                normal = _compute_triangle_normal(v0, v1, v2)
            triangles.append(MeshTriangle(normal=normal, vertices=(v0, v1, v2)))
    return triangles


def _load_ascii_stl(path: Path) -> List[MeshTriangle]:
    triangles: List[MeshTriangle] = []
    current_normal: Vec3 = (0.0, 0.0, 1.0)
    current_vertices: List[Vec3] = []
    for raw_line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw_line.strip()
        if line.startswith("facet normal"):
            parts = line.split()
            current_normal = _normalize((float(parts[2]), float(parts[3]), float(parts[4])))
        elif line.startswith("vertex"):
            parts = line.split()
            current_vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
        elif line.startswith("endfacet"):
            if len(current_vertices) == 3:
                normal = current_normal
                if normal == (0.0, 0.0, 1.0):
                    normal = _compute_triangle_normal(*current_vertices)
                triangles.append(
                    MeshTriangle(
                        normal=normal,
                        vertices=(current_vertices[0], current_vertices[1], current_vertices[2]),
                    )
                )
            current_vertices = []
    return triangles


def load_stl_mesh(path: Path) -> List[MeshTriangle]:
    with path.open("rb") as handle:
        header = handle.read(84)
        if len(header) < 84:
            return []
        face_count = struct.unpack("<I", header[80:84])[0]
        expected_size = 84 + face_count * 50
        if expected_size == path.stat().st_size:
            return _load_binary_stl(path, face_count)
    return _load_ascii_stl(path)


def _find_package_root(urdf_path: Path, package_name: str) -> Optional[Path]:
    direct_root = urdf_path.parent.parent
    if direct_root.name == package_name and (direct_root / "package.xml").exists():
        return direct_root

    for parent in urdf_path.parents:
        candidate = parent / package_name
        if (candidate / "package.xml").exists():
            return candidate
        workspace_candidate = parent / "src" / package_name / package_name
        if (workspace_candidate / "package.xml").exists():
            return workspace_candidate
    return None


def resolve_mesh_path(urdf_path: Path, mesh_uri: str) -> Path:
    if mesh_uri.startswith("package://"):
        remainder = mesh_uri[len("package://") :]
        package_name, relative_path = remainder.split("/", 1)
        package_root = _find_package_root(urdf_path, package_name)
        if package_root is None:
            raise FileNotFoundError(f"无法定位 ROS 包: {package_name}")
        return (package_root / relative_path).resolve()
    return (urdf_path.parent / mesh_uri).resolve()


def compute_scene_bounds(meshes: List[SceneMesh]) -> SceneBounds:
    xs: List[float] = []
    ys: List[float] = []
    zs: List[float] = []
    for mesh in meshes:
        for triangle in mesh.triangles:
            for vertex in triangle.vertices:
                xs.append(vertex[0])
                ys.append(vertex[1])
                zs.append(vertex[2])
    if not xs:
        return SceneBounds(center=(0.0, 0.0, 0.0), radius=1.0)
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    min_z, max_z = min(zs), max(zs)
    center = ((min_x + max_x) * 0.5, (min_y + max_y) * 0.5, (min_z + max_z) * 0.5)
    radius = max(
        0.2,
        math.sqrt(
            max(
                (x - center[0]) ** 2 + (y - center[1]) ** 2 + (z - center[2]) ** 2
                for x, y, z in zip(xs, ys, zs)
            )
        ),
    )
    return SceneBounds(center=center, radius=radius)


def load_urdf_scene(urdf_path: Path) -> Tuple[List[SceneMesh], SceneBounds]:
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    links = {link.attrib["name"]: link for link in root.findall("link")}
    parent_to_children: Dict[str, List[ET.Element]] = {}
    child_links = set()
    for joint in root.findall("joint"):
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        if parent_node is None or child_node is None:
            continue
        parent_name = parent_node.attrib.get("link", "")
        child_name = child_node.attrib.get("link", "")
        if not parent_name or not child_name:
            continue
        parent_to_children.setdefault(parent_name, []).append(joint)
        child_links.add(child_name)

    root_links = [name for name in links if name not in child_links]
    if not root_links:
        raise ValueError("URDF 中未找到根 link")

    link_world: Dict[str, Mat4] = {root_links[0]: _identity_matrix()}
    queue = [root_links[0]]
    while queue:
        link_name = queue.pop(0)
        base_transform = link_world[link_name]
        for joint in parent_to_children.get(link_name, []):
            child_name = joint.find("child").attrib["link"]
            origin = joint.find("origin")
            xyz = _parse_vec3(origin.attrib.get("xyz") if origin is not None else None)
            rpy = _parse_vec3(origin.attrib.get("rpy") if origin is not None else None)
            joint_transform = _transform_matrix(xyz, rpy)
            link_world[child_name] = _matrix_multiply(base_transform, joint_transform)
            queue.append(child_name)

    mesh_cache: Dict[Path, List[MeshTriangle]] = {}
    scene_meshes: List[SceneMesh] = []
    for link_name, link in links.items():
        link_transform = link_world.get(link_name, _identity_matrix())
        for visual in link.findall("visual"):
            geometry = visual.find("geometry")
            if geometry is None:
                continue
            mesh_node = geometry.find("mesh")
            if mesh_node is None:
                continue

            mesh_filename = mesh_node.attrib.get("filename", "")
            if not mesh_filename:
                continue
            mesh_path = resolve_mesh_path(urdf_path, mesh_filename)
            if mesh_path not in mesh_cache:
                mesh_cache[mesh_path] = load_stl_mesh(mesh_path)

            origin = visual.find("origin")
            xyz = _parse_vec3(origin.attrib.get("xyz") if origin is not None else None)
            rpy = _parse_vec3(origin.attrib.get("rpy") if origin is not None else None)
            scale = _parse_vec3(mesh_node.attrib.get("scale"), (1.0, 1.0, 1.0))
            visual_transform = _matrix_multiply(link_transform, _transform_matrix(xyz, rpy, scale))

            color = (0.82, 0.84, 0.88, 1.0)
            material = visual.find("material")
            if material is not None:
                color_node = material.find("color")
                if color_node is not None:
                    color = _parse_rgba(color_node.attrib.get("rgba"))

            transformed_triangles: List[MeshTriangle] = []
            for triangle in mesh_cache[mesh_path]:
                vertices = tuple(_transform_point(visual_transform, vertex) for vertex in triangle.vertices)
                normal = _normalize(_transform_direction(visual_transform, triangle.normal))
                transformed_triangles.append(MeshTriangle(normal=normal, vertices=vertices))
            if transformed_triangles:
                scene_meshes.append(SceneMesh(color=color, triangles=transformed_triangles))

    return scene_meshes, compute_scene_bounds(scene_meshes)


class FallbackRobot3DWidget(QtWidgets.QFrame):
    def __init__(self, message: str = "", parent=None) -> None:
        super().__init__(parent)
        self.message = message or "当前环境无法加载 3D 机器人视窗。"
        self.setFrameShape(QtWidgets.QFrame.NoFrame)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        title = QtWidgets.QLabel("3D 机器人视窗不可用")
        title.setStyleSheet("font-size: 16px; font-weight: 700; color: #f8fafc;")
        body = QtWidgets.QLabel(self.message)
        body.setWordWrap(True)
        body.setStyleSheet("font-size: 12px; color: #cbd5e1;")
        body.setAlignment(QtCore.Qt.AlignCenter)

        spacer = QtWidgets.QWidget()
        spacer.setMinimumHeight(220)
        spacer.setStyleSheet(
            "background: #020617; border: 1px solid #1e293b; border-radius: 8px;"
        )

        inner = QtWidgets.QVBoxLayout(spacer)
        inner.setContentsMargins(18, 18, 18, 18)
        inner.addStretch(1)
        inner.addWidget(title, alignment=QtCore.Qt.AlignCenter)
        inner.addWidget(body, alignment=QtCore.Qt.AlignCenter)
        inner.addStretch(1)

        layout.addWidget(spacer)

    def set_urdf_path(self, _path: str) -> None:
        return

    def reset_view(self) -> None:
        return


if OPENGL_AVAILABLE:

    class Robot3DWidget(QtWidgets.QOpenGLWidget):
        def __init__(self, urdf_path: str = "", parent=None) -> None:
            super().__init__(parent)
            self.setMinimumHeight(260)
            self.scene_meshes: List[SceneMesh] = []
            self.scene_bounds = SceneBounds(center=(0.0, 0.0, 0.0), radius=1.0)
            self.display_lists: List[int] = []
            self.status_message = "等待加载 URDF 模型。"
            self.urdf_path = ""
            self.camera_yaw = 48.0
            self.camera_pitch = 28.0
            self.camera_distance = 2.6
            self.camera_pan_x = 0.0
            self.camera_pan_y = 0.0
            self.drag_mode: Optional[str] = None
            self.last_mouse_pos = QtCore.QPoint()
            self.gl_ready = False
            if urdf_path:
                self.set_urdf_path(urdf_path)

        def set_urdf_path(self, urdf_path: str) -> None:
            path_text = urdf_path.strip()
            self.urdf_path = path_text
            if not path_text:
                self.scene_meshes = []
                self.status_message = "未配置 3D 机器人 URDF 路径。"
                self._rebuild_display_lists()
                self.update()
                return

            path = Path(path_text)
            if not path.exists():
                self.scene_meshes = []
                self.status_message = f"URDF 不存在: {path}"
                self._rebuild_display_lists()
                self.update()
                return

            try:
                self.scene_meshes, self.scene_bounds = load_urdf_scene(path)
                self.status_message = f"已加载 3D 模型: {path.name}"
                self.reset_view()
            except Exception as exc:
                self.scene_meshes = []
                self.status_message = f"3D 模型加载失败: {exc}"
            self._rebuild_display_lists()
            self.update()

        def reset_view(self) -> None:
            radius = max(0.35, self.scene_bounds.radius)
            self.camera_yaw = 48.0
            self.camera_pitch = 28.0
            self.camera_distance = radius * 3.0
            self.camera_pan_x = 0.0
            self.camera_pan_y = 0.0
            self.update()

        def initializeGL(self) -> None:  # pragma: no cover - OpenGL runtime
            self.gl_ready = True
            glClearColor(0.04, 0.08, 0.13, 1.0)
            glEnable(GL_DEPTH_TEST)
            glDepthFunc(GL_LEQUAL)
            glEnable(GL_CULL_FACE)
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            glEnable(GL_MULTISAMPLE)
            glEnable(GL_LIGHTING)
            glEnable(GL_LIGHT0)
            glEnable(GL_COLOR_MATERIAL)
            glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
            glEnable(GL_NORMALIZE)
            glShadeModel(GL_SMOOTH)
            glLightfv(GL_LIGHT0, GL_POSITION, (4.0, 5.0, 8.0, 1.0))
            glLightfv(GL_LIGHT0, GL_AMBIENT, (0.25, 0.25, 0.25, 1.0))
            glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.9, 0.9, 0.9, 1.0))
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, (0.18, 0.18, 0.18, 1.0))
            self._rebuild_display_lists()

        def resizeGL(self, width: int, height: int) -> None:  # pragma: no cover - OpenGL runtime
            height = max(1, height)
            glViewport(0, 0, width, height)
            glMatrixMode(GL_PROJECTION)
            glLoadIdentity()
            gluPerspective(40.0, width / float(height), 0.01, 100.0)
            glMatrixMode(GL_MODELVIEW)

        def paintGL(self) -> None:  # pragma: no cover - OpenGL runtime
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glMatrixMode(GL_MODELVIEW)
            glLoadIdentity()

            center = self.scene_bounds.center
            radius = max(0.35, self.scene_bounds.radius)
            yaw_rad = math.radians(self.camera_yaw)
            pitch_rad = math.radians(self.camera_pitch)
            eye_x = center[0] + self.camera_pan_x + self.camera_distance * math.cos(pitch_rad) * math.cos(yaw_rad)
            eye_y = center[1] + self.camera_pan_y + self.camera_distance * math.cos(pitch_rad) * math.sin(yaw_rad)
            eye_z = center[2] + self.camera_distance * math.sin(pitch_rad)
            gluLookAt(
                eye_x,
                eye_y,
                eye_z,
                center[0] + self.camera_pan_x,
                center[1] + self.camera_pan_y,
                center[2],
                0.0,
                0.0,
                1.0,
            )

            self._draw_floor_grid(radius * 2.8)
            self._draw_axes(radius * 0.9)
            for display_list in self.display_lists:
                glCallList(display_list)

            self._draw_overlay()

        def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
            if event.button() == QtCore.Qt.LeftButton:
                self.drag_mode = "orbit"
            elif event.button() == QtCore.Qt.RightButton:
                self.drag_mode = "pan"
            else:
                self.drag_mode = None
            self.last_mouse_pos = event.pos()
            event.accept()

        def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:
            if self.drag_mode is None:
                return
            delta = event.pos() - self.last_mouse_pos
            self.last_mouse_pos = event.pos()
            if self.drag_mode == "orbit":
                self.camera_yaw += delta.x() * 0.55
                self.camera_pitch = max(-80.0, min(80.0, self.camera_pitch - delta.y() * 0.45))
            elif self.drag_mode == "pan":
                pan_scale = max(0.002, self.scene_bounds.radius * 0.0026)
                self.camera_pan_x -= delta.x() * pan_scale
                self.camera_pan_y += delta.y() * pan_scale
            self.update()

        def mouseReleaseEvent(self, event: QtGui.QMouseEvent) -> None:
            self.drag_mode = None
            event.accept()

        def wheelEvent(self, event: QtGui.QWheelEvent) -> None:
            delta = event.angleDelta().y()
            if delta == 0:
                return
            factor = 0.88 if delta > 0 else 1.14
            min_distance = max(0.15, self.scene_bounds.radius * 0.45)
            max_distance = max(1.0, self.scene_bounds.radius * 10.0)
            self.camera_distance = max(min_distance, min(max_distance, self.camera_distance * factor))
            self.update()
            event.accept()

        def _rebuild_display_lists(self) -> None:
            if not self.gl_ready:
                return
            for display_list in self.display_lists:
                glDeleteLists(display_list, 1)
            self.display_lists = []
            for mesh in self.scene_meshes:
                display_list = glGenLists(1)
                glNewList(display_list, 0x1300)  # GL_COMPILE
                glColor4f(*mesh.color)
                glBegin(GL_TRIANGLES)
                for triangle in mesh.triangles:
                    glNormal3f(*triangle.normal)
                    for vertex in triangle.vertices:
                        glVertex3f(*vertex)
                glEnd()
                glEndList()
                self.display_lists.append(display_list)

        def _draw_floor_grid(self, extent: float) -> None:
            glDisable(GL_LIGHTING)
            glLineWidth(1.0)
            glBegin(GL_LINES)
            glColor4f(0.20, 0.28, 0.38, 0.55)
            step = max(0.1, extent / 12.0)
            count = int(extent / step)
            for idx in range(-count, count + 1):
                value = idx * step
                glVertex3f(-extent, value, 0.0)
                glVertex3f(extent, value, 0.0)
                glVertex3f(value, -extent, 0.0)
                glVertex3f(value, extent, 0.0)
            glEnd()
            glEnable(GL_LIGHTING)

        def _draw_axes(self, size: float) -> None:
            glDisable(GL_LIGHTING)
            glLineWidth(2.0)
            glBegin(GL_LINES)
            glColor4f(0.96, 0.27, 0.37, 1.0)
            glVertex3f(0.0, 0.0, 0.0)
            glVertex3f(size, 0.0, 0.0)
            glColor4f(0.12, 0.84, 0.56, 1.0)
            glVertex3f(0.0, 0.0, 0.0)
            glVertex3f(0.0, size, 0.0)
            glColor4f(0.36, 0.60, 0.98, 1.0)
            glVertex3f(0.0, 0.0, 0.0)
            glVertex3f(0.0, 0.0, size)
            glEnd()
            glEnable(GL_LIGHTING)

        def _draw_overlay(self) -> None:
            painter = QtGui.QPainter(self)
            painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
            info_rect = QtCore.QRectF(14, 14, min(420, self.width() - 28), 54)
            painter.setPen(QtCore.Qt.NoPen)
            painter.setBrush(QtGui.QColor(15, 23, 42, 212))
            painter.drawRoundedRect(info_rect, 8, 8)
            painter.setPen(QtGui.QColor("#f8fafc"))
            painter.setFont(QtGui.QFont("Microsoft YaHei", 9, QtGui.QFont.Bold))
            painter.drawText(
                info_rect.adjusted(12, 8, -12, -24),
                QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
                self.status_message,
            )
            painter.setPen(QtGui.QColor("#cbd5e1"))
            painter.setFont(QtGui.QFont("Microsoft YaHei", 8))
            painter.drawText(
                info_rect.adjusted(12, 28, -12, -6),
                QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
                "左键旋转 | 右键平移 | 滚轮缩放",
            )
            painter.end()


else:

    class Robot3DWidget(FallbackRobot3DWidget):
        def __init__(self, urdf_path: str = "", parent=None) -> None:
            message = (
                "缺少 OpenGL 运行依赖，无法显示 3D 模型。\n"
                f"导入错误: {OPENGL_ERROR}\n"
                "请安装 windows_requirements.txt 中新增的 PyOpenGL 依赖。"
            )
            super().__init__(message=message, parent=parent)

