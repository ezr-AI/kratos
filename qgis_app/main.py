import os
import sys
import subprocess
from pathlib import Path

from PyQt5.QtCore import Qt, QSize, QProcess, QProcessEnvironment, QVariant
from PyQt5.QtGui import QColor, QIcon
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (
    QApplication,
    QFileDialog,
    QLabel,
    QMainWindow,
    QMenuBar,
    QMenu,
    QSplitter,
    QVBoxLayout,
    QWidget,
    QToolBar,
    QAction,
    QSizePolicy,
    QDialog,
    QFormLayout,
    QDialogButtonBox,
    QLineEdit,
    QSpinBox,
    QDoubleSpinBox,
    QMessageBox,
    QPushButton,
    QHBoxLayout,
    QColorDialog,
    QSlider,
    QDockWidget,
    QTreeView,
    QTreeWidget,
    QTreeWidgetItem,
    QFileSystemModel,
    QGroupBox,
    QProgressBar,
    QComboBox,
    QShortcut,
    QRadioButton,
    QToolButton,
    QInputDialog,
)
from qgis.core import (
    QgsApplication,
    QgsLayerTreeModel,
    QgsLayerTreeLayer,
    QgsLayerTreeGroup,
    QgsProject,
    QgsRasterLayer,
    QgsVectorLayer,
    QgsWkbTypes,
    QgsCoordinateReferenceSystem,
    QgsCoordinateTransform,
    QgsRectangle,
    QgsMarkerSymbol,
    QgsSingleSymbolRenderer,
    QgsUnitTypes,
    QgsDistanceArea,
    QgsFeatureRequest,
    QgsGeometry,
    QgsFields,
    QgsField,
    QgsVectorFileWriter,
    QgsFeature,
    QgsPointXY,
    QgsSpatialIndex,
)
from qgis.gui import QgsLayerTreeView, QgsMapCanvas
from qgis.gui import (
    QgsMapToolPan,
    QgsMapToolZoom,
    QgsMapToolEmitPoint,
    QgsRubberBand,
    QgsVertexMarker,
)
from pathlib import Path


class DistanceMeasureTool(QgsMapToolEmitPoint):
    def __init__(self, canvas, project, on_result, on_value=None):
        super().__init__(canvas)
        self.canvas = canvas
        self.project = project
        self.on_result = on_result
        self.on_value = on_value
        self.rubber = QgsRubberBand(canvas, QgsWkbTypes.LineGeometry)
        self.rubber.setColor(QColor(0, 180, 255, 200))
        self.rubber.setWidth(2)
        self.points = []
        self.markers = []
        self.drag_index = -1
        self.pick_px = 14

    def _add_marker(self, point):
        marker = QgsVertexMarker(self.canvas)
        marker.setCenter(point)
        marker.setColor(QColor(0, 180, 255))
        marker.setFillColor(QColor(255, 255, 255))
        marker.setIconType(QgsVertexMarker.ICON_CIRCLE)
        marker.setIconSize(12)
        marker.setPenWidth(2)
        self.markers.append(marker)

    def _clear_markers(self):
        for marker in self.markers:
            self.canvas.scene().removeItem(marker)
        self.markers = []

    def _redraw(self):
        self.rubber.reset(QgsWkbTypes.LineGeometry)
        self._clear_markers()
        for p in self.points:
            self._add_marker(p)
            self.rubber.addPoint(p, True)
        if len(self.points) == 2:
            da = QgsDistanceArea()
            da.setSourceCrs(self.project.crs(), self.project.transformContext())
            da.setEllipsoid(self.project.ellipsoid())
            dist = da.measureLine(self.points[0], self.points[1])
            self.on_result(f"Jarak: {dist:.2f} m")
            if self.on_value is not None:
                self.on_value(dist)

    def _pick_vertex_index(self, map_point):
        cx = self.toCanvasCoordinates(map_point)
        best = -1
        best_d2 = self.pick_px * self.pick_px
        for i, p in enumerate(self.points):
            px = self.toCanvasCoordinates(p)
            dx = px.x() - cx.x()
            dy = px.y() - cx.y()
            d2 = dx * dx + dy * dy
            if d2 <= best_d2:
                best = i
                best_d2 = d2
        return best

    def canvasPressEvent(self, event):
        if event.button() != Qt.LeftButton:
            return
        point = self.toMapCoordinates(event.pos())
        idx = self._pick_vertex_index(point)
        if idx >= 0:
            self.drag_index = idx
            return
        if len(self.points) < 2:
            self.points.append(point)
            self._redraw()
            if len(self.points) == 1:
                self.on_result("Klik titik kedua atau drag titik untuk edit")
        else:
            # Replace the nearest endpoint if user clicks elsewhere after complete.
            self.points[1] = point
            self._redraw()

    def canvasMoveEvent(self, event):
        if self.drag_index < 0:
            return
        point = self.toMapCoordinates(event.pos())
        self.points[self.drag_index] = point
        self._redraw()

    def canvasReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.drag_index = -1

    def clear(self):
        self.points = []
        self.drag_index = -1
        self.rubber.reset(QgsWkbTypes.LineGeometry)
        self._clear_markers()
        if self.on_value is not None:
            self.on_value(None)


class AreaMeasureTool(QgsMapToolEmitPoint):
    def __init__(self, canvas, project, on_result, on_value=None):
        super().__init__(canvas)
        self.canvas = canvas
        self.project = project
        self.on_result = on_result
        self.on_value = on_value
        self.points = []
        self.rubber = QgsRubberBand(canvas, QgsWkbTypes.PolygonGeometry)
        self.rubber.setColor(QColor(255, 0, 0, 180))
        self.rubber.setWidth(2)
        self.markers = []
        self.closed = False
        self.drag_index = -1
        self.pick_px = 14

    def _add_marker(self, point):
        marker = QgsVertexMarker(self.canvas)
        marker.setCenter(point)
        marker.setColor(QColor(255, 0, 0))
        marker.setFillColor(QColor(255, 255, 255))
        marker.setIconType(QgsVertexMarker.ICON_CIRCLE)
        marker.setIconSize(10)
        marker.setPenWidth(2)
        self.markers.append(marker)

    def _clear_markers(self):
        for marker in self.markers:
            self.canvas.scene().removeItem(marker)
        self.markers = []

    def _redraw(self):
        self.rubber.reset(QgsWkbTypes.PolygonGeometry)
        self._clear_markers()
        for p in self.points:
            self._add_marker(p)
            self.rubber.addPoint(p, True)
        if self.closed and len(self.points) >= 3:
            self.rubber.closePoints()
            da = QgsDistanceArea()
            da.setSourceCrs(self.project.crs(), self.project.transformContext())
            da.setEllipsoid(self.project.ellipsoid())
            area = da.measurePolygon(self.points)
            self.on_result(f"Luas: {area:.2f} m2")
            if self.on_value is not None:
                self.on_value(area)

    def _pick_vertex_index(self, map_point):
        cx = self.toCanvasCoordinates(map_point)
        best = -1
        best_d2 = self.pick_px * self.pick_px
        for i, p in enumerate(self.points):
            px = self.toCanvasCoordinates(p)
            dx = px.x() - cx.x()
            dy = px.y() - cx.y()
            d2 = dx * dx + dy * dy
            if d2 <= best_d2:
                best = i
                best_d2 = d2
        return best

    def canvasPressEvent(self, event):
        if event.button() == Qt.RightButton:
            self._finish()
            return
        if event.button() != Qt.LeftButton:
            return
        point = self.toMapCoordinates(event.pos())
        idx = self._pick_vertex_index(point)
        if idx >= 0 and self.closed:
            self.drag_index = idx
            return
        if self.closed:
            # Start a new polygon if user clicks after finishing.
            self._reset()
        self.points.append(point)
        self._redraw()

    def _finish(self):
        if len(self.points) < 3:
            self.on_result("Butuh >= 3 titik untuk luas")
            self._reset()
            return
        self.closed = True
        self._redraw()

    def canvasMoveEvent(self, event):
        if self.drag_index < 0:
            return
        point = self.toMapCoordinates(event.pos())
        self.points[self.drag_index] = point
        self._redraw()

    def canvasReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.drag_index = -1

    def _reset(self):
        self.points = []
        self.closed = False
        self.drag_index = -1
        self.rubber.reset(QgsWkbTypes.PolygonGeometry)
        self._clear_markers()
        if self.on_value is not None:
            self.on_value(None)

    def clear(self):
        self._reset()


class AOICountTool(QgsMapToolEmitPoint):
    def __init__(self, canvas, project, on_result, on_finished):
        super().__init__(canvas)
        self.canvas = canvas
        self.project = project
        self.on_result = on_result
        self.on_finished = on_finished
        self.points = []
        self.rubber = QgsRubberBand(canvas, QgsWkbTypes.PolygonGeometry)
        self.rubber.setColor(QColor(255, 180, 0, 180))
        self.rubber.setFillColor(QColor(255, 180, 0, 60))
        self.rubber.setWidth(2)
        self.markers = []
        self.closed = False

    def _add_marker(self, point):
        marker = QgsVertexMarker(self.canvas)
        marker.setCenter(point)
        marker.setColor(QColor(255, 180, 0))
        marker.setFillColor(QColor(255, 255, 255))
        marker.setIconType(QgsVertexMarker.ICON_CIRCLE)
        marker.setIconSize(10)
        marker.setPenWidth(2)
        self.markers.append(marker)

    def _clear_markers(self):
        for marker in self.markers:
            self.canvas.scene().removeItem(marker)
        self.markers = []

    def _redraw(self):
        self.rubber.reset(QgsWkbTypes.PolygonGeometry)
        self._clear_markers()
        for p in self.points:
            self._add_marker(p)
            self.rubber.addPoint(p, True)
        if self.closed and len(self.points) >= 3:
            self.rubber.closePoints()

    def canvasPressEvent(self, event):
        if event.button() == Qt.RightButton:
            self._finish()
            return
        if event.button() != Qt.LeftButton:
            return
        if self.closed:
            self._reset()
        point = self.toMapCoordinates(event.pos())
        self.points.append(point)
        self._redraw()
        self.on_result(f"AOI titik: {len(self.points)} (klik kanan untuk hitung)")

    def _finish(self):
        if len(self.points) < 3:
            self.on_result("AOI butuh >= 3 titik")
            return
        self.closed = True
        self._redraw()
        self.on_finished(list(self.points))

    def _reset(self):
        self.points = []
        self.closed = False
        self.rubber.reset(QgsWkbTypes.PolygonGeometry)
        self._clear_markers()

    def clear(self):
        self._reset()


class SelectFeatureTool(QgsMapToolEmitPoint):
    def __init__(self, canvas, project, get_layer, on_result):
        super().__init__(canvas)
        self.canvas = canvas
        self.project = project
        self.get_layer = get_layer
        self.on_result = on_result

    def canvasPressEvent(self, event):
        if event.button() != Qt.LeftButton:
            return
        layer = self.get_layer()
        if not isinstance(layer, QgsVectorLayer):
            self.on_result("Pilih layer vector untuk select")
            return
        if not layer.isEditable():
            self.on_result("Aktifkan edit mode dulu")
            return

        pt = self.toMapCoordinates(event.pos())
        click_geom = QgsGeometry.fromPointXY(QgsPointXY(pt))
        tol = max(1.0, self.canvas.mapUnitsPerPixel() * 12.0)
        rect = click_geom.boundingBox()
        rect.grow(tol)

        nearest_fid = None
        nearest_dist = None
        req = QgsFeatureRequest().setFilterRect(rect)
        for feat in layer.getFeatures(req):
            geom = feat.geometry()
            if not geom or geom.isEmpty():
                continue
            d = geom.distance(click_geom)
            if nearest_dist is None or d < nearest_dist:
                nearest_dist = d
                nearest_fid = feat.id()
        if nearest_fid is None:
            layer.removeSelection()
            self.on_result("Tidak ada feature dekat klik")
            return
        layer.selectByIds([nearest_fid])
        self.on_result(f"Selected feature id={nearest_fid}")


class AddPointFeatureTool(QgsMapToolEmitPoint):
    def __init__(self, canvas, get_layer, on_result):
        super().__init__(canvas)
        self.canvas = canvas
        self.get_layer = get_layer
        self.on_result = on_result

    def _map_to_layer_point(self, layer, map_point):
        src = self.canvas.mapSettings().destinationCrs()
        dst = layer.crs()
        if src.isValid() and dst.isValid() and src != dst:
            try:
                xform = QgsCoordinateTransform(src, dst, QgsProject.instance())
                return xform.transform(map_point)
            except Exception:
                return None
        return map_point

    def canvasPressEvent(self, event):
        if event.button() != Qt.LeftButton:
            return
        layer = self.get_layer()
        if not isinstance(layer, QgsVectorLayer):
            self.on_result("Pilih layer vector point")
            return
        if layer.geometryType() != QgsWkbTypes.PointGeometry:
            self.on_result("Layer aktif bukan point")
            return
        if not layer.isEditable():
            self.on_result("Aktifkan edit mode dulu")
            return

        pt = self.toMapCoordinates(event.pos())
        layer_pt = self._map_to_layer_point(layer, pt)
        if layer_pt is None:
            self.on_result("Gagal transform koordinat ke CRS layer")
            return
        feat = QgsFeature(layer.fields())
        feat.setGeometry(QgsGeometry.fromPointXY(QgsPointXY(layer_pt)))
        layer.beginEditCommand("Add point")
        if layer.addFeature(feat):
            layer.endEditCommand()
            layer.updateExtents()
            layer.triggerRepaint()
            extent = layer.extent()
            if not extent.isNull():
                self.canvas.setExtent(extent)
            self.canvas.refresh()
            self.on_result("Point ditambahkan")
        else:
            layer.destroyEditCommand()
            self.on_result("Gagal tambah point")


class AddLineFeatureTool(QgsMapToolEmitPoint):
    def __init__(self, canvas, get_layer, on_result):
        super().__init__(canvas)
        self.canvas = canvas
        self.get_layer = get_layer
        self.on_result = on_result
        self.points = []
        self.rubber = QgsRubberBand(canvas, QgsWkbTypes.LineGeometry)
        self.rubber.setColor(QColor(255, 160, 0, 220))
        self.rubber.setWidth(2)

    def _map_to_layer_point(self, layer, map_point):
        src = self.canvas.mapSettings().destinationCrs()
        dst = layer.crs()
        if src.isValid() and dst.isValid() and src != dst:
            try:
                xform = QgsCoordinateTransform(src, dst, QgsProject.instance())
                return xform.transform(map_point)
            except Exception:
                return None
        return map_point

    def _reset(self):
        self.points = []
        self.rubber.reset(QgsWkbTypes.LineGeometry)

    def clear(self):
        self._reset()

    def canvasPressEvent(self, event):
        layer = self.get_layer()
        if not isinstance(layer, QgsVectorLayer) or layer.geometryType() != QgsWkbTypes.LineGeometry:
            self.on_result("Pilih layer vector line")
            return
        if not layer.isEditable():
            if not layer.startEditing():
                self.on_result("Aktifkan edit mode dulu")
                return

        if event.button() == Qt.LeftButton:
            pt = self.toMapCoordinates(event.pos())
            layer_pt = self._map_to_layer_point(layer, pt)
            if layer_pt is None:
                self.on_result("Gagal transform koordinat ke CRS layer")
                return
            self.points.append(QgsPointXY(layer_pt))
            self.rubber.addPoint(pt, True)
            self.on_result(f"Line vertices: {len(self.points)}")
            return
        if event.button() == Qt.RightButton:
            if len(self.points) < 2:
                self.on_result("Line butuh >= 2 titik (lanjutkan klik titik)")
                return
            feat = QgsFeature(layer.fields())
            feat.setGeometry(QgsGeometry.fromPolylineXY(self.points))
            layer.beginEditCommand("Add line")
            if layer.addFeature(feat):
                layer.endEditCommand()
                layer.updateExtents()
                layer.triggerRepaint()
                extent = layer.extent()
                if not extent.isNull():
                    self.canvas.setExtent(extent)
                self.canvas.refresh()
                self.on_result("Line ditambahkan")
            else:
                layer.destroyEditCommand()
                self.on_result("Gagal tambah line")
            self._reset()


class AddPolygonFeatureTool(QgsMapToolEmitPoint):
    def __init__(self, canvas, get_layer, on_result):
        super().__init__(canvas)
        self.canvas = canvas
        self.get_layer = get_layer
        self.on_result = on_result
        self.points = []
        self.rubber = QgsRubberBand(canvas, QgsWkbTypes.PolygonGeometry)
        self.rubber.setColor(QColor(255, 120, 0, 220))
        self.rubber.setFillColor(QColor(255, 120, 0, 60))
        self.rubber.setWidth(2)

    def _map_to_layer_point(self, layer, map_point):
        src = self.canvas.mapSettings().destinationCrs()
        dst = layer.crs()
        if src.isValid() and dst.isValid() and src != dst:
            try:
                xform = QgsCoordinateTransform(src, dst, QgsProject.instance())
                return xform.transform(map_point)
            except Exception:
                return None
        return map_point

    def _reset(self):
        self.points = []
        self.rubber.reset(QgsWkbTypes.PolygonGeometry)

    def clear(self):
        self._reset()

    def canvasPressEvent(self, event):
        layer = self.get_layer()
        if not isinstance(layer, QgsVectorLayer) or layer.geometryType() != QgsWkbTypes.PolygonGeometry:
            self.on_result("Pilih layer vector polygon")
            return
        if not layer.isEditable():
            if not layer.startEditing():
                self.on_result("Aktifkan edit mode dulu")
                return

        if event.button() == Qt.LeftButton:
            pt = self.toMapCoordinates(event.pos())
            layer_pt = self._map_to_layer_point(layer, pt)
            if layer_pt is None:
                self.on_result("Gagal transform koordinat ke CRS layer")
                return
            self.points.append(QgsPointXY(layer_pt))
            self.rubber.addPoint(pt, True)
            self.on_result(f"Polygon vertices: {len(self.points)}")
            return
        if event.button() == Qt.RightButton:
            if len(self.points) < 3:
                self.on_result("Polygon butuh >= 3 titik (lanjutkan klik titik)")
                return
            ring = list(self.points)
            if ring[0] != ring[-1]:
                ring.append(QgsPointXY(ring[0]))
            feat = QgsFeature(layer.fields())
            feat.setGeometry(QgsGeometry.fromPolygonXY([ring]))
            layer.beginEditCommand("Add polygon")
            if layer.addFeature(feat):
                layer.endEditCommand()
                layer.updateExtents()
                layer.triggerRepaint()
                extent = layer.extent()
                if not extent.isNull():
                    self.canvas.setExtent(extent)
                self.canvas.refresh()
                self.on_result("Polygon ditambahkan")
            else:
                layer.destroyEditCommand()
                self.on_result("Gagal tambah polygon")
            self._reset()


class VertexEditTool(QgsMapToolEmitPoint):
    def __init__(self, canvas, get_layer, on_result):
        super().__init__(canvas)
        self.canvas = canvas
        self.get_layer = get_layer
        self.on_result = on_result
        self.drag_fid = None
        self.drag_vertex = -1
        self.edit_layer = None
        self.edit_cmd_open = False
        self.drag_origin_geom = None
        self.drag_preview_geom = None
        self.preview_band = None

    def _clear_preview(self):
        if self.preview_band is not None:
            self.canvas.scene().removeItem(self.preview_band)
            self.preview_band = None

    def clear(self):
        if self.edit_layer is not None and self.edit_cmd_open:
            self.edit_layer.destroyEditCommand()
        self.drag_fid = None
        self.drag_vertex = -1
        self.drag_origin_geom = None
        self.drag_preview_geom = None
        self.edit_layer = None
        self.edit_cmd_open = False
        self._clear_preview()

    def _pick_feature(self, layer, map_point):
        click_geom = QgsGeometry.fromPointXY(QgsPointXY(map_point))
        tol = max(1.0, self.canvas.mapUnitsPerPixel() * 14.0)
        rect = click_geom.boundingBox()
        rect.grow(tol)
        req = QgsFeatureRequest().setFilterRect(rect)
        nearest_fid = None
        nearest_dist = None
        for feat in layer.getFeatures(req):
            geom = feat.geometry()
            if not geom or geom.isEmpty():
                continue
            d = geom.distance(click_geom)
            if nearest_dist is None or d < nearest_dist:
                nearest_dist = d
                nearest_fid = feat.id()
        return nearest_fid

    def _map_to_layer_point(self, layer, map_point):
        src = self.canvas.mapSettings().destinationCrs()
        dst = layer.crs()
        if src.isValid() and dst.isValid() and src != dst:
            try:
                xform = QgsCoordinateTransform(src, dst, QgsProject.instance())
                return xform.transform(map_point)
            except Exception:
                return None
        return map_point

    def _pick_vertex_index(self, layer, fid, map_point):
        req = QgsFeatureRequest(fid)
        feat = next(layer.getFeatures(req), None)
        if feat is None:
            return -1
        geom = feat.geometry()
        if not geom or geom.isEmpty():
            return -1

        try:
            vtx, v_idx, _before, _after, sqr_dist = geom.closestVertex(QgsPointXY(map_point))
        except Exception:
            return -1

        tol = max(1.0, self.canvas.mapUnitsPerPixel() * 16.0)
        if sqr_dist > (tol * tol):
            return -1
        return v_idx

    def canvasPressEvent(self, event):
        if event.button() != Qt.LeftButton:
            return
        layer = self.get_layer()
        if not isinstance(layer, QgsVectorLayer):
            self.on_result("Pilih layer vector dulu")
            return
        if not layer.isEditable():
            self.on_result("Aktifkan edit mode dulu")
            return
        pt = self.toMapCoordinates(event.pos())
        layer_pt = self._map_to_layer_point(layer, pt)
        if layer_pt is None:
            self.on_result("Gagal transform koordinat ke CRS layer")
            return
        self.drag_fid = self._pick_feature(layer, layer_pt)
        if self.drag_fid is None:
            self.on_result("Tidak ada feature dekat klik")
            return
        self.drag_vertex = self._pick_vertex_index(layer, self.drag_fid, layer_pt)
        if self.drag_vertex < 0:
            self.drag_fid = None
            self.on_result("Klik lebih dekat ke vertex")
            return
        req = QgsFeatureRequest(self.drag_fid)
        feat = next(layer.getFeatures(req), None)
        if feat is None:
            self.drag_fid = None
            self.drag_vertex = -1
            self.on_result("Feature tidak ditemukan")
            return
        geom = feat.geometry()
        if not geom or geom.isEmpty():
            self.drag_fid = None
            self.drag_vertex = -1
            self.on_result("Geometry kosong")
            return
        self.drag_origin_geom = QgsGeometry(geom)
        self.drag_preview_geom = QgsGeometry(geom)
        self.edit_layer = layer
        self.edit_layer.beginEditCommand("Move vertex")
        self.edit_cmd_open = True
        self._clear_preview()
        if layer.geometryType() != QgsWkbTypes.PointGeometry:
            self.preview_band = QgsRubberBand(self.canvas, layer.geometryType())
            self.preview_band.setColor(QColor(255, 160, 0, 220))
            self.preview_band.setWidth(2)
            if layer.geometryType() == QgsWkbTypes.PolygonGeometry:
                self.preview_band.setFillColor(QColor(255, 160, 0, 40))
            self.preview_band.setToGeometry(self.drag_origin_geom, layer)
        self.on_result(f"Drag vertex: fid={self.drag_fid}, v={self.drag_vertex}")

    def canvasMoveEvent(self, event):
        if self.drag_fid is None:
            return
        layer = self.get_layer()
        if not isinstance(layer, QgsVectorLayer):
            return
        if self.drag_origin_geom is None:
            return
        pt = self.toMapCoordinates(event.pos())
        layer_pt = self._map_to_layer_point(layer, pt)
        if layer_pt is None:
            return
        geom = QgsGeometry(self.drag_origin_geom)
        if not geom.moveVertex(layer_pt.x(), layer_pt.y(), self.drag_vertex):
            return
        self.drag_preview_geom = geom
        if layer.geometryType() == QgsWkbTypes.PointGeometry:
            # Point edit is cheap; update directly for responsive drag.
            layer.changeGeometry(self.drag_fid, geom)
            layer.triggerRepaint()
            return
        if self.preview_band is not None:
            self.preview_band.setToGeometry(geom, layer)

    def canvasReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self.drag_fid is not None:
            ok = True
            if (
                self.edit_layer is not None
                and self.drag_preview_geom is not None
                and self.edit_layer.geometryType() != QgsWkbTypes.PointGeometry
            ):
                ok = self.edit_layer.changeGeometry(self.drag_fid, self.drag_preview_geom)
            if self.edit_layer is not None and self.edit_cmd_open:
                if ok:
                    self.edit_layer.endEditCommand()
                else:
                    self.edit_layer.destroyEditCommand()
            self.on_result(
                f"Vertex moved: fid={self.drag_fid}" if ok else "Gagal move vertex"
            )
            self.drag_fid = None
            self.drag_vertex = -1
            self.drag_origin_geom = None
            self.drag_preview_geom = None
            self.edit_layer = None
            self.edit_cmd_open = False
            self._clear_preview()
            self.canvas.refresh()


class MeasureDialog(QDialog):
    def __init__(self, parent=None, on_close=None):
        super().__init__(parent)
        self.setWindowTitle("Measure")
        self.resize(420, 180)
        self.mode = "distance"  # distance | area
        self.base_value = None
        self.on_close = on_close
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        form = QFormLayout()
        self.total_label = QLabel("Total")
        self.total_value = QLineEdit("")
        self.total_value.setReadOnly(True)
        self.unit_combo = QComboBox()
        self.unit_combo.currentIndexChanged.connect(self._update_display)
        row = QWidget()
        row_l = QHBoxLayout(row)
        row_l.setContentsMargins(0, 0, 0, 0)
        row_l.addWidget(self.total_value, 1)
        row_l.addWidget(self.unit_combo, 0)
        form.addRow(self.total_label, row)
        layout.addLayout(form)

        mode_row = QWidget()
        mode_l = QHBoxLayout(mode_row)
        mode_l.setContentsMargins(0, 0, 0, 0)
        self.radio_cart = QRadioButton("Cartesian")
        self.radio_ellip = QRadioButton("Ellipsoidal")
        self.radio_ellip.setChecked(True)
        mode_l.addWidget(self.radio_cart)
        mode_l.addWidget(self.radio_ellip)
        mode_l.addStretch(1)
        layout.addWidget(mode_row)

        btn_row = QWidget()
        btn_l = QHBoxLayout(btn_row)
        btn_l.setContentsMargins(0, 0, 0, 0)
        self.btn_new = QPushButton("New")
        self.btn_close = QPushButton("Close")
        self.btn_help = QPushButton("Help")
        btn_l.addStretch(1)
        btn_l.addWidget(self.btn_new)
        btn_l.addWidget(self.btn_close)
        btn_l.addWidget(self.btn_help)
        layout.addWidget(btn_row)

        self.btn_close.clicked.connect(self.close)
        self.btn_help.clicked.connect(self._show_help)
        self.btn_new.clicked.connect(self._on_new)

    def _show_help(self):
        QMessageBox.information(
            self,
            "Measure Help",
            "Left click to add points.\nRight click to finish area.\nPress Esc to cancel and return Pan.",
        )

    def _on_new(self):
        self.base_value = None
        self.total_value.setText("")

    def set_mode(self, mode: str):
        self.mode = mode
        self.unit_combo.blockSignals(True)
        self.unit_combo.clear()
        if mode == "distance":
            self.total_label.setText("Total")
            self.unit_combo.addItems(["meters", "kilometers"])
        else:
            self.total_label.setText("Total")
            self.unit_combo.addItems(["square meters", "hectares", "square kilometers"])
        self.unit_combo.blockSignals(False)
        self._update_display()

    def set_value(self, value):
        self.base_value = value
        self._update_display()

    def _update_display(self):
        if self.base_value is None:
            self.total_value.setText("")
            return
        unit = self.unit_combo.currentText()
        val = float(self.base_value)
        if self.mode == "distance":
            if unit == "kilometers":
                val /= 1000.0
                suffix = " km"
            else:
                suffix = " m"
        else:
            if unit == "hectares":
                val /= 10000.0
                suffix = " ha"
            elif unit == "square kilometers":
                val /= 1000000.0
                suffix = " km2"
            else:
                suffix = " m2"
        self.total_value.setText(f"{val:,.3f}{suffix}")

    def closeEvent(self, event):
        if callable(self.on_close):
            self.on_close()
        super().closeEvent(event)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Palm QGIS Lite")
        self.resize(1280, 800)

        self.project = QgsProject.instance()
        if not self.project.crs().isValid():
            self.project.setCrs(QgsCoordinateReferenceSystem("EPSG:4326"))
        root = self.project.layerTreeRoot()

        self.canvas = QgsMapCanvas()
        self.canvas.setCanvasColor(Qt.white)
        self.canvas.enableAntiAliasing(True)

        self.layer_tree = QgsLayerTreeView()
        self.layer_model = QgsLayerTreeModel(root)
        self.layer_model.setFlag(QgsLayerTreeModel.AllowNodeReorder)
        self.layer_model.setFlag(QgsLayerTreeModel.AllowNodeChangeVisibility)
        self.layer_model.setFlag(QgsLayerTreeModel.AllowNodeRename)
        self.layer_tree.setModel(self.layer_model)
        self.layer_tree.setDragDropMode(QgsLayerTreeView.InternalMove)
        self.layer_tree.setDefaultDropAction(Qt.MoveAction)
        self.layer_tree.setContextMenuPolicy(Qt.CustomContextMenu)
        self.layer_tree.customContextMenuRequested.connect(self._show_layer_context_menu)

        # Manual layer management is used in this app; avoid bridge auto-resets.
        self.bridge = None

        self.status_label = QLabel("Ready")
        self.coord_label = QLabel("Lat: -, Lon: -")
        self.scale_label = QLabel("Scale: -")
        self.crs_label = QLabel("CRS: -")
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setFixedWidth(220)
        self.progress_bar.setVisible(False)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(8, 8, 8, 8)
        left_split = QSplitter(Qt.Vertical)

        layers_box = QGroupBox("Layers")
        layers_layout = QVBoxLayout(layers_box)
        layers_layout.setContentsMargins(6, 6, 6, 6)
        layers_layout.addWidget(self.layer_tree)

        self.browser_view = QTreeView()
        self.browser_model = QFileSystemModel()
        self.browser_model.setRootPath(str(Path.cwd()))
        self.browser_view.setModel(self.browser_model)
        self.browser_view.setRootIndex(self.browser_model.index(str(Path.cwd())))
        for i in range(1, 4):
            self.browser_view.hideColumn(i)
        self.xyz_tree = QTreeWidget()
        self.xyz_tree.setHeaderHidden(True)
        self._populate_xyz_tiles()
        self.xyz_tree.itemDoubleClicked.connect(self._on_xyz_tile_double_clicked)

        browser_box = QGroupBox("Browser")
        browser_layout = QVBoxLayout(browser_box)
        browser_layout.setContentsMargins(6, 6, 6, 6)
        browser_layout.addWidget(self.xyz_tree)
        browser_layout.addWidget(self.browser_view)

        left_split.addWidget(layers_box)
        left_split.addWidget(browser_box)
        left_split.setSizes([400, 200])
        left_layout.addWidget(left_split)
        left_panel.setMinimumWidth(220)
        left_panel.setMaximumWidth(300)

        splitter = QSplitter()
        splitter.addWidget(left_panel)
        splitter.addWidget(self.canvas)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([240, 1000])

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(splitter)
        self.setCentralWidget(container)

        self._build_toolbar()
        self._build_menubar()
        self._bind_shortcuts()
        self.statusBar().addWidget(self.status_label, 1)
        self.statusBar().addWidget(self.progress_bar)
        self.statusBar().addWidget(self.coord_label)
        self.statusBar().addWidget(self.scale_label)
        self.statusBar().addWidget(self.crs_label)

        self._init_map_tools()
        self._bind_canvas_signals()
        self._bind_layer_tree_signals()
        self._build_properties_dock()
        self.detect_process = None
        self.detect_output_path = None
        self.detect_log_chunks = []
        self.detect_stdout_buffer = ""
        self.measure_dialog = MeasureDialog(self, on_close=self._on_measure_dialog_closed)
        self.measure_dialog.btn_new.clicked.connect(self._restart_measure_tool)

    def _bind_shortcuts(self) -> None:
        # Ensure ESC works even when focus is inside map canvas/child widgets.
        self.shortcut_cancel = QShortcut(QKeySequence(Qt.Key_Escape), self)
        self.shortcut_cancel.setContext(Qt.ApplicationShortcut)
        self.shortcut_cancel.activated.connect(self.cancel_current_operation)

    def _bind_layer_tree_signals(self) -> None:
        # Keep canvas render order synced with layer tree interactions.
        self.layer_model.rowsInserted.connect(self._on_layer_tree_changed)
        self.layer_model.rowsRemoved.connect(self._on_layer_tree_changed)
        self.layer_model.rowsMoved.connect(self._on_layer_tree_changed)
        self.layer_model.dataChanged.connect(self._on_layer_tree_changed)
        self._refresh_canvas_layers()

    def _on_layer_tree_changed(self, *args) -> None:
        self._refresh_canvas_layers()

    def _stop_detect_process(self) -> None:
        if self.detect_process is None:
            return
        pid = int(self.detect_process.processId())
        try:
            self.detect_process.terminate()
            if not self.detect_process.waitForFinished(2000):
                self.detect_process.kill()
                self.detect_process.waitForFinished(2000)
        except Exception:
            pass

        # On Windows, kill the whole process tree (cmd -> conda -> python).
        if os.name == "nt" and pid > 0:
            try:
                subprocess.run(
                    ["taskkill", "/PID", str(pid), "/T", "/F"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=False,
                )
            except Exception:
                pass

        self.detect_process = None
        self.act_detect.setEnabled(True)
        self.progress_bar.setVisible(False)
        self.progress_bar.setValue(0)

    def closeEvent(self, event) -> None:
        self._stop_detect_process()
        super().closeEvent(event)

    def keyPressEvent(self, event) -> None:
        if event.key() == Qt.Key_Escape:
            self.cancel_current_operation()
            event.accept()
            return
        super().keyPressEvent(event)

    def _init_map_tools(self) -> None:
        self.pan_tool = QgsMapToolPan(self.canvas)
        self.pan_tool.setAction(None)
        self.zoom_in_tool = QgsMapToolZoom(self.canvas, False)
        self.zoom_out_tool = QgsMapToolZoom(self.canvas, True)
        self.measure_dist_tool = DistanceMeasureTool(
            self.canvas, self.project, self.status_label.setText, self._on_distance_measure
        )
        self.measure_area_tool = AreaMeasureTool(
            self.canvas, self.project, self.status_label.setText, self._on_area_measure
        )
        self.aoi_count_tool = AOICountTool(
            self.canvas,
            self.project,
            self.status_label.setText,
            self._count_points_in_aoi,
        )
        self.select_tool = SelectFeatureTool(
            self.canvas,
            self.project,
            self._active_vector_layer,
            self.status_label.setText,
        )
        self.add_point_tool = AddPointFeatureTool(
            self.canvas,
            self._active_vector_layer,
            self.status_label.setText,
        )
        self.add_line_tool = AddLineFeatureTool(
            self.canvas,
            self._active_vector_layer,
            self.status_label.setText,
        )
        self.add_poly_tool = AddPolygonFeatureTool(
            self.canvas,
            self._active_vector_layer,
            self.status_label.setText,
        )
        self.vertex_tool = VertexEditTool(
            self.canvas,
            self._active_vector_layer,
            self.status_label.setText,
        )
        self.canvas.setMapTool(self.pan_tool)
        self.status_label.setText("Pan tool active")

    def _bind_canvas_signals(self) -> None:
        self.canvas.xyCoordinates.connect(self._update_coords)
        self.canvas.extentsChanged.connect(self._update_scale)
        self.project.crsChanged.connect(self._update_crs)
        self._update_scale()
        self._update_crs()

    def _update_coords(self, point) -> None:
        dest_crs = self.canvas.mapSettings().destinationCrs()
        wgs84 = QgsCoordinateReferenceSystem("EPSG:4326")
        if dest_crs.isValid() and dest_crs != wgs84:
            try:
                xform = QgsCoordinateTransform(dest_crs, wgs84, self.project)
                ll = xform.transform(point)
                self.coord_label.setText(f"Lat: {ll.y():.6f}, Lon: {ll.x():.6f}")
                return
            except Exception:
                pass
        # Already geographic or transform failed; fallback to current point.
        self.coord_label.setText(f"Lat: {point.y():.6f}, Lon: {point.x():.6f}")

    def _update_scale(self) -> None:
        self.scale_label.setText(f"Scale: 1:{int(self.canvas.scale()):,}")

    def _update_crs(self) -> None:
        crs = self.project.crs()
        if crs.isValid():
            self.crs_label.setText(f"CRS: {crs.authid()}")
        else:
            self.crs_label.setText("CRS: -")

    def _populate_xyz_tiles(self) -> None:
        self.xyz_tree.clear()
        root = QTreeWidgetItem(["XYZ Tiles"])
        root.setFlags(root.flags() & ~Qt.ItemIsSelectable)
        root.setExpanded(True)
        self.xyz_tree.addTopLevelItem(root)

        # Mapzen terrain tiles are no longer generally available without keys.
        # This entry keeps the familiar label while using a public terrain tile source.
        tiles = [
            ("Mapzen Global Terrain", "https://tile.opentopomap.org/{z}/{x}/{y}.png"),
            ("OpenStreetMap", "https://tile.openstreetmap.org/{z}/{x}/{y}.png"),
        ]
        for name, url in tiles:
            item = QTreeWidgetItem([name])
            item.setData(0, Qt.UserRole, url)
            root.addChild(item)

    def _on_xyz_tile_double_clicked(self, item, _column) -> None:
        url = item.data(0, Qt.UserRole)
        if not url:
            return
        self.add_xyz_tile_layer(item.text(0), url)

    def add_xyz_tile_layer(self, name: str, url: str) -> None:
        source = f"type=xyz&url={url}"
        for lyr in self.project.mapLayers().values():
            if isinstance(lyr, QgsRasterLayer) and lyr.source() == source:
                self.status_label.setText(f"Layer already added: {name}")
                return

        layer = QgsRasterLayer(source, name, "wms")
        if not layer.isValid():
            QMessageBox.critical(self, "Error", f"Gagal menambah XYZ tile: {name}")
            return

        self.project.addMapLayer(layer, False)
        # Keep basemap low in stack.
        self.project.layerTreeRoot().addLayer(layer)
        self._refresh_canvas_layers()
        if len(self.project.mapLayers()) == 1:
            self.canvas.setExtent(layer.extent())
        self.canvas.refresh()
        self.status_label.setText(f"Loaded XYZ tile: {name}")

    def _build_menubar(self) -> None:
        menubar = QMenuBar(self)
        file_menu = QMenu("File", self)
        view_menu = QMenu("View", self)
        layer_menu = QMenu("Layer", self)
        create_layer_menu = QMenu("Create Layer", self)
        edit_menu = QMenu("Edit", self)

        act_open_raster = QAction("Open GeoTIFF", self)
        act_open_vector = QAction("Open KML/GeoJSON", self)
        act_new_shapefile = QAction("New Shapefile Layer...", self)
        act_clear_layers = QAction("Clear All Layers", self)
        act_measure_line = QAction("Measure Line", self)
        act_measure_area = QAction("Measure Area", self)
        act_measure_bearing = QAction("Measure Bearing", self)
        act_measure_angle = QAction("Measure Angle", self)
        act_toggle_edit = QAction("Toggle Editing", self)
        act_select_feature = QAction("Select Feature", self)
        act_add_point = QAction("Add Point", self)
        act_add_line = QAction("Add Line", self)
        act_add_polygon = QAction("Add Polygon", self)
        act_vertex = QAction("Vertex Tool", self)
        act_delete_selected = QAction("Delete Selected", self)
        act_undo_edit = QAction("Undo Edit", self)
        act_redo_edit = QAction("Redo Edit", self)
        act_save_edits = QAction("Save Edits", self)
        act_rollback = QAction("Rollback Edits", self)
        act_exit = QAction("Exit", self)

        act_open_raster.triggered.connect(self.open_raster)
        act_open_vector.triggered.connect(self.open_vector)
        act_new_shapefile.triggered.connect(self.create_new_shapefile_layer)
        act_clear_layers.triggered.connect(self.clear_layers)
        act_measure_line.triggered.connect(self.set_measure_distance)
        act_measure_area.triggered.connect(self.set_measure_area)
        act_measure_line.setShortcut("Ctrl+Shift+M")
        act_measure_area.setShortcut("Ctrl+Shift+J")
        act_measure_bearing.triggered.connect(
            lambda: QMessageBox.information(self, "Measure", "Measure Bearing belum tersedia.")
        )
        act_measure_angle.triggered.connect(
            lambda: QMessageBox.information(self, "Measure", "Measure Angle belum tersedia.")
        )
        act_toggle_edit.triggered.connect(self.toggle_editing)
        act_select_feature.triggered.connect(self.set_select_tool)
        act_add_point.triggered.connect(self.set_add_point_tool)
        act_add_line.triggered.connect(self.set_add_line_tool)
        act_add_polygon.triggered.connect(self.set_add_polygon_tool)
        act_vertex.triggered.connect(self.set_vertex_tool)
        act_delete_selected.triggered.connect(self.delete_selected_features)
        act_undo_edit.triggered.connect(self.undo_edit)
        act_redo_edit.triggered.connect(self.redo_edit)
        act_save_edits.triggered.connect(self.save_edits)
        act_rollback.triggered.connect(self.rollback_edits)
        act_exit.triggered.connect(self.close)

        file_menu.addAction(act_open_raster)
        file_menu.addAction(act_open_vector)
        file_menu.addSeparator()
        file_menu.addAction(act_exit)

        # Placeholder view menu
        view_menu.addAction("Full Extent", self.zoom_full_extent)
        view_menu.addAction("Zoom to Layer", self.zoom_to_active_layer)
        create_layer_menu.addAction(act_new_shapefile)
        layer_menu.addMenu(create_layer_menu)
        edit_menu.addAction(act_toggle_edit)
        edit_menu.addAction(act_select_feature)
        edit_menu.addSeparator()
        edit_menu.addAction(act_add_point)
        edit_menu.addAction(act_add_line)
        edit_menu.addAction(act_add_polygon)
        edit_menu.addAction(act_vertex)
        edit_menu.addSeparator()
        edit_menu.addAction(act_delete_selected)
        edit_menu.addAction(act_undo_edit)
        edit_menu.addAction(act_redo_edit)
        edit_menu.addAction(act_save_edits)
        edit_menu.addAction(act_rollback)

        menubar.addMenu(file_menu)
        menubar.addMenu(view_menu)
        menubar.addMenu(layer_menu)
        self.setMenuBar(menubar)

    def _build_properties_dock(self) -> None:
        self.props_dock = QDockWidget("Properties", self)
        self.props_widget = QWidget()
        self.props_layout = QFormLayout(self.props_widget)
        self.props_name = QLabel("-")
        self.props_type = QLabel("-")
        self.props_crs = QLabel("-")
        self.props_count = QLabel("-")
        self.props_layout.addRow("Name", self.props_name)
        self.props_layout.addRow("Type", self.props_type)
        self.props_layout.addRow("CRS", self.props_crs)
        self.props_layout.addRow("Count", self.props_count)
        self.props_dock.setWidget(self.props_widget)
        self.addDockWidget(Qt.RightDockWidgetArea, self.props_dock)
        self.layer_tree.currentLayerChanged.connect(self._update_properties)
        self.layer_tree.currentLayerChanged.connect(self._bind_active_layer_signals)

    def _update_properties(self, layer) -> None:
        if layer is None:
            self.props_name.setText("-")
            self.props_type.setText("-")
            self.props_crs.setText("-")
            self.props_count.setText("-")
            return
        self.props_name.setText(layer.name())
        if isinstance(layer, QgsRasterLayer):
            self.props_type.setText("Raster")
            self.props_count.setText("-")
        elif isinstance(layer, QgsVectorLayer):
            self.props_type.setText("Vector")
            self.props_count.setText(str(layer.featureCount()))
        else:
            self.props_type.setText("Layer")
            self.props_count.setText("-")
        # Show effective display CRS (project CRS) to match canvas behavior.
        project_crs = self.project.crs()
        if project_crs.isValid():
            self.props_crs.setText(project_crs.authid())
        else:
            self.props_crs.setText("-")

    def _bind_active_layer_signals(self, layer) -> None:
        if layer is None or not isinstance(layer, QgsVectorLayer):
            return
        # Update properties panel when features change in edit mode.
        layer.featureAdded.connect(lambda _fid: self._update_properties(layer))
        layer.featuresDeleted.connect(lambda _fids: self._update_properties(layer))
        layer.editingStopped.connect(lambda: self._update_properties(layer))

    def _show_layer_context_menu(self, pos) -> None:
        idx = self.layer_tree.indexAt(pos)
        if idx.isValid():
            self.layer_tree.setCurrentIndex(idx)
        layer = self.layer_tree.currentLayer()
        if layer is None:
            return

        menu = QMenu(self)
        act_zoom = menu.addAction("Zoom to Layer")
        act_set_crs = menu.addAction("Set Layer CRS...")
        act_remove = menu.addAction("Remove Layer")
        if isinstance(layer, QgsVectorLayer):
            menu.addSeparator()
            act_toggle_edit = menu.addAction(
                "Stop Editing" if layer.isEditable() else "Toggle Editing"
            )
            act_delete_selected = menu.addAction("Delete Selected")
            act_undo_edit = menu.addAction("Undo Edit")
            act_redo_edit = menu.addAction("Redo Edit")
            act_save_edits = menu.addAction("Save Edits")
            act_rollback = menu.addAction("Rollback Edits")
            menu.addSeparator()
            act_save_as = menu.addAction("Save Layer As...")
        else:
            act_toggle_edit = None
            act_delete_selected = None
            act_undo_edit = None
            act_redo_edit = None
            act_save_edits = None
            act_rollback = None
            act_save_as = None

        chosen = menu.exec_(self.layer_tree.viewport().mapToGlobal(pos))
        if chosen == act_zoom:
            self.zoom_to_active_layer()
        elif chosen == act_set_crs:
            self.set_active_layer_crs()
        elif chosen == act_remove:
            self.remove_active_layer()
        elif act_toggle_edit is not None and chosen == act_toggle_edit:
            self.toggle_editing()
        elif act_delete_selected is not None and chosen == act_delete_selected:
            self.delete_selected_features()
        elif act_undo_edit is not None and chosen == act_undo_edit:
            self.undo_edit()
        elif act_redo_edit is not None and chosen == act_redo_edit:
            self.redo_edit()
        elif act_save_edits is not None and chosen == act_save_edits:
            self.save_edits()
        elif act_rollback is not None and chosen == act_rollback:
            self.rollback_edits()
        elif act_save_as is not None and chosen == act_save_as:
            self.save_active_layer_as()

    def set_active_layer_crs(self) -> None:
        layer = self.layer_tree.currentLayer()
        if layer is None:
            self.status_label.setText("No active layer")
            return
        current = layer.crs().authid() if layer.crs().isValid() else "EPSG:4326"
        text, ok = QInputDialog.getText(
            self,
            "Set Layer CRS",
            "Masukkan CRS (contoh: EPSG:32748):",
            text=current,
        )
        if not ok:
            return
        crs_text = text.strip()
        if not crs_text:
            self.status_label.setText("CRS dibatalkan")
            return
        crs = QgsCoordinateReferenceSystem(crs_text)
        if not crs.isValid():
            QMessageBox.critical(self, "Error", f"CRS tidak valid: {crs_text}")
            return

        # Assign layer CRS (definition only, no coordinate reprojection).
        if not hasattr(layer, "setCrs"):
            QMessageBox.critical(self, "Error", "Layer ini tidak mendukung set CRS.")
            return
        layer.setCrs(crs)
        self._update_properties(layer)
        self._refresh_canvas_layers()
        self.canvas.refresh()
        self.status_label.setText(f"{layer.name()} CRS -> {crs.authid()}")

    def set_pan_tool(self) -> None:
        self.canvas.setMapTool(self.pan_tool)
        self.status_label.setText("Pan tool active")

    def cancel_current_operation(self) -> None:
        self._clear_temp_map_tools()
        self.measure_dist_tool.clear()
        self.measure_area_tool.clear()
        self.canvas.setMapTool(self.pan_tool)
        self.canvas.refresh()
        self.status_label.setText("Operation canceled, Pan tool active")

    def set_zoom_in_tool(self) -> None:
        self.canvas.setMapTool(self.zoom_in_tool)
        self.status_label.setText("Zoom in tool active")

    def set_zoom_out_tool(self) -> None:
        self.canvas.setMapTool(self.zoom_out_tool)
        self.status_label.setText("Zoom out tool active")

    def set_measure_distance(self) -> None:
        self.measure_dialog.set_mode("distance")
        self.measure_dialog.show()
        self.measure_dialog.raise_()
        self.canvas.setMapTool(self.measure_dist_tool)
        self.status_label.setText("Klik 2 titik untuk jarak")

    def set_measure_area(self) -> None:
        self.measure_dialog.set_mode("area")
        self.measure_dialog.show()
        self.measure_dialog.raise_()
        self.canvas.setMapTool(self.measure_area_tool)
        self.status_label.setText("Klik titik poligon, klik kanan untuk selesai")

    def clear_measurements(self) -> None:
        self.measure_dist_tool.clear()
        self.measure_area_tool.clear()
        self.measure_dialog.set_value(None)
        self.canvas.refresh()
        self.status_label.setText("Measurement cleared")

    def _on_measure_dialog_closed(self):
        self.measure_dist_tool.clear()
        self.measure_area_tool.clear()
        self.canvas.refresh()
        self.set_pan_tool()

    def _on_distance_measure(self, meters):
        if self.measure_dialog.mode != "distance":
            self.measure_dialog.set_mode("distance")
        self.measure_dialog.set_value(meters)

    def _on_area_measure(self, sq_m):
        if self.measure_dialog.mode != "area":
            self.measure_dialog.set_mode("area")
        self.measure_dialog.set_value(sq_m)

    def _restart_measure_tool(self):
        if self.measure_dialog.mode == "distance":
            self.measure_dist_tool.clear()
            self.canvas.setMapTool(self.measure_dist_tool)
            self.status_label.setText("Measure line baru: klik 2 titik")
        else:
            self.measure_area_tool.clear()
            self.canvas.setMapTool(self.measure_area_tool)
            self.status_label.setText("Measure area baru: klik titik poligon")

    def set_count_aoi_tool(self) -> None:
        self._clear_temp_map_tools()
        self.canvas.setMapTool(self.aoi_count_tool)
        self.status_label.setText("Klik titik AOI, klik kanan untuk hitung pohon")

    def clear_aoi_count(self) -> None:
        self.aoi_count_tool.clear()
        self.canvas.refresh()
        self.status_label.setText("AOI cleared")

    def set_select_tool(self) -> None:
        self._clear_temp_map_tools()
        self.canvas.setMapTool(self.select_tool)
        self.status_label.setText("Select feature: klik pada feature")

    def set_add_point_tool(self) -> None:
        self._clear_temp_map_tools()
        self.canvas.setMapTool(self.add_point_tool)
        self.status_label.setText("Add point: klik pada peta")

    def set_add_line_tool(self) -> None:
        self._clear_temp_map_tools()
        self.canvas.setMapTool(self.add_line_tool)
        self.status_label.setText("Add line: klik kiri tambah titik, klik kanan selesai")

    def set_add_polygon_tool(self) -> None:
        self._clear_temp_map_tools()
        self.canvas.setMapTool(self.add_poly_tool)
        self.status_label.setText("Add polygon: klik kiri tambah titik, klik kanan selesai")

    def set_add_feature_tool(self) -> None:
        layer = self._active_vector_layer()
        if layer is None:
            self.status_label.setText("Pilih layer vector dulu")
            return
        g = layer.geometryType()
        if g == QgsWkbTypes.PointGeometry:
            self.set_add_point_tool()
            return
        if g == QgsWkbTypes.LineGeometry:
            self.set_add_line_tool()
            return
        if g == QgsWkbTypes.PolygonGeometry:
            self.set_add_polygon_tool()
            return
        self.status_label.setText("Geometry layer tidak didukung untuk Add Feature")

    def set_vertex_tool(self) -> None:
        self._clear_temp_map_tools()
        self.canvas.setMapTool(self.vertex_tool)
        self.status_label.setText("Vertex tool: klik dekat vertex lalu drag untuk ubah bentuk")

    def _clear_temp_map_tools(self) -> None:
        self.add_line_tool.clear()
        self.add_poly_tool.clear()
        self.aoi_count_tool.clear()
        self.vertex_tool.clear()

    def _build_toolbar(self) -> None:
        tb = QToolBar("Tools")
        tb.setMovable(False)
        tb.setIconSize(QSize(16, 16))

        icon_dir = Path(__file__).with_name("icons")
        def icon(name: str) -> QIcon:
            path = icon_dir / f"{name}.svg"
            return QIcon(str(path)) if path.exists() else QIcon()

        act_open_raster = QAction(icon("open_raster"), "Open GeoTIFF", self)
        act_open_vector = QAction(icon("open_vector"), "Open KML/GeoJSON", self)
        act_new_vector = QAction(icon("new_shapefile"), "New Shapefile Layer", self)
        act_pan = QAction(icon("pan"), "Pan", self)
        act_zoom_in = QAction(icon("zoom_in"), "Zoom In", self)
        act_zoom_out = QAction(icon("zoom_out"), "Zoom Out", self)
        act_full_extent = QAction(icon("full_extent"), "Full Extent", self)
        act_zoom_layer = QAction(icon("zoom_layer"), "Zoom to Layer", self)
        self.act_detect = QAction(icon("detect"), "Detect Palms (YOLO)", self)
        act_measure_dist = QAction(icon("measure_dist"), "Measure Distance", self)
        act_measure_area = QAction(icon("measure_area"), "Measure Area", self)
        act_measure_clear = QAction(icon("measure_clear"), "Clear Measure", self)
        act_measure_bearing = QAction("Measure Bearing", self)
        act_measure_angle = QAction("Measure Angle", self)
        act_toggle_edit = QAction(icon("toggle_editing"), "Toggle Editing", self)
        act_add_feature = QAction(icon("add_feature"), "Add Feature", self)
        act_select_feature = QAction(icon("select"), "Select Feature", self)
        act_add_point = QAction(icon("add_point"), "Add Point", self)
        act_add_line = QAction(icon("add_line"), "Add Line", self)
        act_add_polygon = QAction(icon("add_polygon"), "Add Polygon", self)
        act_vertex = QAction(icon("vertex"), "Vertex Tool", self)
        act_delete_selected = QAction(icon("delete_feature"), "Delete Selected", self)
        act_undo_edit = QAction(icon("undo"), "Undo Edit", self)
        act_redo_edit = QAction(icon("redo"), "Redo Edit", self)
        act_save_edits = QAction(icon("save"), "Save Edits", self)
        act_rollback = QAction(icon("rollback"), "Rollback", self)
        act_attr_table = QAction(icon("table"), "Attribute Table", self)
        act_symbology = QAction(icon("symbology"), "Symbology", self)

        act_open_raster.triggered.connect(self.open_raster)
        act_open_vector.triggered.connect(self.open_vector)
        act_new_vector.triggered.connect(self.create_new_shapefile_layer)
        act_pan.triggered.connect(self.set_pan_tool)
        act_zoom_in.triggered.connect(self.set_zoom_in_tool)
        act_zoom_out.triggered.connect(self.set_zoom_out_tool)
        act_full_extent.triggered.connect(self.zoom_full_extent)
        act_zoom_layer.triggered.connect(self.zoom_to_active_layer)
        self.act_detect.triggered.connect(self.detect_palm_trees)
        act_measure_dist.triggered.connect(self.set_measure_distance)
        act_measure_area.triggered.connect(self.set_measure_area)
        act_measure_clear.triggered.connect(self.clear_measurements)
        act_measure_bearing.triggered.connect(
            lambda: QMessageBox.information(self, "Measure", "Measure Bearing belum tersedia.")
        )
        act_measure_angle.triggered.connect(
            lambda: QMessageBox.information(self, "Measure", "Measure Angle belum tersedia.")
        )
        act_toggle_edit.triggered.connect(self.toggle_editing)
        act_add_feature.triggered.connect(self.set_add_feature_tool)
        act_select_feature.triggered.connect(self.set_select_tool)
        act_add_point.triggered.connect(self.set_add_point_tool)
        act_add_line.triggered.connect(self.set_add_line_tool)
        act_add_polygon.triggered.connect(self.set_add_polygon_tool)
        act_vertex.triggered.connect(self.set_vertex_tool)
        act_delete_selected.triggered.connect(self.delete_selected_features)
        act_undo_edit.triggered.connect(self.undo_edit)
        act_redo_edit.triggered.connect(self.redo_edit)
        act_save_edits.triggered.connect(self.save_edits)
        act_rollback.triggered.connect(self.rollback_edits)
        act_attr_table.triggered.connect(self.open_attribute_table)
        act_symbology.triggered.connect(self.open_symbology)

        tb.addAction(act_open_raster)
        tb.addAction(act_open_vector)
        tb.addAction(act_new_vector)
        tb.addSeparator()
        tb.addAction(act_pan)
        tb.addAction(act_zoom_in)
        tb.addAction(act_zoom_out)
        tb.addSeparator()
        tb.addAction(act_full_extent)
        tb.addAction(act_zoom_layer)
        tb.addSeparator()
        tb.addAction(self.act_detect)
        tb.addSeparator()
        tb.addAction(act_toggle_edit)
        tb.addAction(act_add_feature)
        tb.addSeparator()
        measure_menu = QMenu(self)
        measure_menu.addAction(act_measure_dist)
        measure_menu.addAction(act_measure_area)
        measure_menu.addAction(act_measure_bearing)
        measure_menu.addAction(act_measure_angle)
        measure_menu.addSeparator()
        measure_menu.addAction(act_measure_clear)

        measure_btn = QToolButton(tb)
        measure_btn.setPopupMode(QToolButton.MenuButtonPopup)
        measure_btn.setText("Measure")
        measure_btn.setIcon(icon("measure_dist"))
        measure_btn.setMenu(measure_menu)
        measure_btn.clicked.connect(self.set_measure_distance)
        tb.addWidget(measure_btn)
        tb.addSeparator()
        tb.addAction(act_attr_table)
        tb.addAction(act_symbology)

        self.addToolBar(tb)

    def open_raster(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Open GeoTIFF", "", "GeoTIFF (*.tif *.tiff)"
        )
        if not path:
            return
        layer = QgsRasterLayer(path, Path(path).name)
        if not layer.isValid():
            self.status_label.setText("Failed to load GeoTIFF")
            return
        # QGIS-like behavior: keep project CRS stable; render layer with OTF reprojection.
        self.project.addMapLayer(layer, False)
        self.project.layerTreeRoot().addLayer(layer)
        self.canvas.setDestinationCrs(self.project.crs())
        self._refresh_canvas_layers()
        # Focus to newly loaded raster, transformed into project CRS if needed.
        extent = layer.extent()
        dest_crs = self.project.crs()
        if (
            layer.crs().isValid()
            and dest_crs.isValid()
            and layer.crs() != dest_crs
        ):
            try:
                xform = QgsCoordinateTransform(layer.crs(), dest_crs, self.project)
                extent = xform.transformBoundingBox(extent)
            except Exception:
                pass
        self.canvas.setExtent(extent)
        self.canvas.refresh()
        self.status_label.setText(f"Loaded {Path(path).name}")

    def create_new_shapefile_layer(self) -> None:
        dialog = QDialog(self)
        dialog.setWindowTitle("New Shapefile Layer")
        layout = QFormLayout(dialog)

        name_input = QLineEdit("new_layer")
        geom_input = QComboBox()
        geom_input.addItems(["Point", "LineString", "Polygon"])
        crs_default = self.project.crs().authid() if self.project.crs().isValid() else "EPSG:4326"
        crs_input = QLineEdit(crs_default)

        layout.addRow("Layer name", name_input)
        layout.addRow("Geometry", geom_input)
        layout.addRow("CRS", crs_input)
        layout.addRow("Storage", QLabel("Temporary (save later via Save Layer As...)"))

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addRow(buttons)

        if dialog.exec_() != QDialog.Accepted:
            return

        name = name_input.text().strip() or "new_layer"
        geom = geom_input.currentText()
        crs = QgsCoordinateReferenceSystem(crs_input.text().strip())
        if not crs.isValid():
            QMessageBox.critical(self, "Error", "CRS tidak valid. Contoh: EPSG:32748")
            return

        uri = f"{geom}?crs={crs.authid()}&field=id:integer"
        mem_layer = QgsVectorLayer(uri, name, "memory")
        if not mem_layer.isValid():
            QMessageBox.critical(self, "Error", "Gagal membuat layer memory.")
            return

        # Ensure minimal schema exists and editable behavior matches QGIS-like empty layer.
        pr = mem_layer.dataProvider()
        if pr.fields().indexFromName("id") < 0:
            fields = QgsFields()
            fields.append(QgsField("id", QVariant.Int))
            pr.addAttributes(fields)
            mem_layer.updateFields()
        layer = mem_layer

        self.project.addMapLayer(layer, False)
        self.project.layerTreeRoot().insertLayer(0, layer)
        self.canvas.setDestinationCrs(self.project.crs())
        self._refresh_canvas_layers()
        self.canvas.refresh()
        self.layer_tree.setCurrentLayer(layer)
        self.status_label.setText(
            f"Created temporary layer {layer.name()} (use Save Layer As to export)"
        )

    # Backward-compatible alias (existing call sites/custom scripts)
    def create_new_vector_layer(self) -> None:
        self.create_new_shapefile_layer()

    def save_active_layer_as(self) -> None:
        layer = self.layer_tree.currentLayer()
        if layer is None or not isinstance(layer, QgsVectorLayer):
            QMessageBox.information(self, "Info", "Pilih layer vector terlebih dahulu.")
            return

        default_name = f"{layer.name()}.geojson"
        out_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Layer As",
            default_name,
            "GeoJSON (*.geojson);;Shapefile (*.shp);;GeoPackage (*.gpkg)",
        )
        if not out_path:
            return

        ext = Path(out_path).suffix.lower()
        options = QgsVectorFileWriter.SaveVectorOptions()
        if ext == ".shp":
            options.driverName = "ESRI Shapefile"
        elif ext == ".gpkg":
            options.driverName = "GPKG"
            options.layerName = layer.name()
        else:
            if ext != ".geojson":
                out_path = f"{out_path}.geojson"
            options.driverName = "GeoJSON"

        result = QgsVectorFileWriter.writeAsVectorFormatV3(
            layer,
            out_path,
            self.project.transformContext(),
            options,
        )
        err_code = result[0] if isinstance(result, tuple) else result
        if err_code != QgsVectorFileWriter.NoError:
            QMessageBox.critical(self, "Error", "Gagal menyimpan layer.")
            return
        self.status_label.setText(f"Layer saved: {Path(out_path).name}")

    def open_vector(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Open Vector",
            "",
            "Vector (*.kml *.geojson *.json *.gpkg *.shp)",
        )
        if not path:
            return
        layer = QgsVectorLayer(path, Path(path).name, "ogr")
        if not layer.isValid():
            self.status_label.setText("Failed to load vector")
            return
        # Detection outputs are written in raster/project CRS coordinates.
        if path.lower().endswith(".detections.geojson") and self.project.crs().isValid():
            layer.setCrs(self.project.crs())
        # If CRS is missing, assume project CRS.
        if not layer.crs().isValid():
            if self.project.crs().isValid():
                layer.setCrs(self.project.crs())
            else:
                layer.setCrs(QgsCoordinateReferenceSystem("EPSG:32748"))
        # Style point layers for visibility.
        if layer.geometryType() == QgsWkbTypes.PointGeometry:
            if path.lower().endswith(".detections.geojson"):
                self._apply_detection_style(layer)
            else:
                symbol = QgsMarkerSymbol.createSimple(
                    {
                        "name": "circle",
                        "color": "255,0,0",
                        "size": "8",
                        "outline_color": "255,255,255",
                        "outline_width": "1",
                    }
                )
                symbol.setSizeUnit(QgsUnitTypes.RenderPixels)
                layer.setRenderer(QgsSingleSymbolRenderer(symbol))
        self.project.addMapLayer(layer, False)
        self.project.layerTreeRoot().insertLayer(0, layer)
        self.canvas.setDestinationCrs(self.project.crs())
        self._refresh_canvas_layers()
        self._zoom_to_all_layers()
        self.canvas.refresh()
        self.status_label.setText(f"Loaded {Path(path).name}")

    def clear_layers(self) -> None:
        self.project.removeAllMapLayers()
        self._refresh_canvas_layers()
        self.canvas.refresh()
        self.status_label.setText("Cleared")

    def _active_vector_layer(self):
        layer = self.layer_tree.currentLayer()
        if isinstance(layer, QgsVectorLayer):
            return layer
        return None

    def toggle_editing(self) -> None:
        layer = self._active_vector_layer()
        if layer is None:
            self.status_label.setText("Pilih layer vector dulu")
            return
        if layer.isEditable():
            choice = QMessageBox.question(
                self,
                "Stop Editing",
                f"Simpan perubahan layer '{layer.name()}'?",
                QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel,
                QMessageBox.Save,
            )
            if choice == QMessageBox.Cancel:
                self.status_label.setText("Stop editing dibatalkan")
                return
            if choice == QMessageBox.Save:
                if layer.commitChanges():
                    self.status_label.setText(f"Edits saved: {layer.name()}")
                else:
                    self.status_label.setText(f"Gagal save edits: {layer.name()}")
                    return
            else:
                layer.rollBack()
                self.status_label.setText(f"Edits discarded: {layer.name()}")
        else:
            if layer.startEditing():
                self.status_label.setText(f"Editing ON: {layer.name()}")
            else:
                self.status_label.setText(f"Gagal start editing: {layer.name()}")
        layer.triggerRepaint()
        self.canvas.refresh()

    def save_edits(self) -> None:
        layer = self._active_vector_layer()
        if layer is None:
            self.status_label.setText("Pilih layer vector dulu")
            return
        if not layer.isEditable():
            self.status_label.setText("Layer tidak dalam mode edit")
            return
        if layer.commitChanges():
            self.status_label.setText(f"Edits saved: {layer.name()}")
        else:
            self.status_label.setText(f"Gagal save edits: {layer.name()}")
        layer.triggerRepaint()
        self.canvas.refresh()

    def rollback_edits(self) -> None:
        layer = self._active_vector_layer()
        if layer is None:
            self.status_label.setText("Pilih layer vector dulu")
            return
        if not layer.isEditable():
            self.status_label.setText("Layer tidak dalam mode edit")
            return
        layer.rollBack()
        layer.triggerRepaint()
        self.canvas.refresh()
        self.status_label.setText(f"Rollback edits: {layer.name()}")

    def delete_selected_features(self) -> None:
        layer = self._active_vector_layer()
        if layer is None:
            self.status_label.setText("Pilih layer vector dulu")
            return
        if not layer.isEditable():
            self.status_label.setText("Aktifkan edit mode dulu")
            return
        fids = layer.selectedFeatureIds()
        if not fids:
            self.status_label.setText("Tidak ada feature terpilih")
            return
        layer.beginEditCommand("Delete selected features")
        ok = layer.deleteFeatures(fids)
        if ok:
            layer.endEditCommand()
        else:
            layer.destroyEditCommand()
        layer.removeSelection()
        layer.triggerRepaint()
        self.canvas.refresh()
        if ok:
            self.status_label.setText(f"Deleted {len(fids)} feature(s)")
        else:
            self.status_label.setText("Gagal delete selected feature(s)")

    def undo_edit(self) -> None:
        layer = self._active_vector_layer()
        if layer is None:
            self.status_label.setText("Pilih layer vector dulu")
            return
        if not layer.isEditable():
            self.status_label.setText("Aktifkan edit mode dulu")
            return
        stack = layer.undoStack()
        if stack is None or not stack.canUndo():
            self.status_label.setText("Tidak ada perubahan untuk undo")
            return
        stack.undo()
        layer.triggerRepaint()
        self.canvas.refresh()
        self.status_label.setText("Undo edit")

    def redo_edit(self) -> None:
        layer = self._active_vector_layer()
        if layer is None:
            self.status_label.setText("Pilih layer vector dulu")
            return
        if not layer.isEditable():
            self.status_label.setText("Aktifkan edit mode dulu")
            return
        stack = layer.undoStack()
        if stack is None or not stack.canRedo():
            self.status_label.setText("Tidak ada perubahan untuk redo")
            return
        stack.redo()
        layer.triggerRepaint()
        self.canvas.refresh()
        self.status_label.setText("Redo edit")

    def remove_active_layer(self) -> None:
        layer = self.layer_tree.currentLayer()
        if layer is None:
            self.status_label.setText("No active layer")
            return
        layer_name = layer.name()
        self.project.removeMapLayer(layer.id())
        self._refresh_canvas_layers()
        self.canvas.refresh()
        self.status_label.setText(f"Removed {layer_name}")

    def _zoom_to_all_layers(self) -> None:
        layers = list(self.project.mapLayers().values())
        if not layers:
            return
        combined = None
        dest_crs = self.project.crs()
        for layer in layers:
            extent = layer.extent()
            if extent.isNull():
                continue
            if layer.crs().isValid() and dest_crs.isValid() and layer.crs() != dest_crs:
                try:
                    xform = QgsCoordinateTransform(layer.crs(), dest_crs, self.project)
                    extent = xform.transformBoundingBox(extent)
                except Exception:
                    pass
            if combined is None:
                combined = QgsRectangle(extent)
            else:
                combined.combineExtentWith(extent)
        if combined:
            self.canvas.setExtent(combined)
            self.canvas.refresh()

    def _refresh_canvas_layers(self) -> None:
        root = self.project.layerTreeRoot()
        layers = self._collect_visible_layers(root, parent_visible=True)
        if not layers:
            self.canvas.setLayers([])
            return
        # Keep render order aligned with layer tree (top item in panel stays on top).
        self.canvas.setLayers(layers)

    def _collect_visible_layers(self, node, parent_visible: bool):
        collected = []
        if isinstance(node, QgsLayerTreeLayer):
            if parent_visible and node.itemVisibilityChecked():
                lyr = node.layer()
                if lyr is not None:
                    collected.append(lyr)
            return collected
        if isinstance(node, QgsLayerTreeGroup):
            visible = parent_visible and node.itemVisibilityChecked()
            for child in node.children():
                collected.extend(self._collect_visible_layers(child, visible))
            return collected
        for child in node.children():
            collected.extend(self._collect_visible_layers(child, parent_visible))
        return collected

    def _is_layer_visible(self, layer) -> bool:
        if layer is None:
            return False
        node = self.project.layerTreeRoot().findLayer(layer.id())
        if node is None:
            return False
        return bool(node.itemVisibilityChecked())

    def _candidate_point_layers(self):
        layer = self.layer_tree.currentLayer()
        if (
            isinstance(layer, QgsVectorLayer)
            and layer.geometryType() == QgsWkbTypes.PointGeometry
            and self._is_layer_visible(layer)
        ):
            return [layer]

        candidates = []
        for lyr in self.project.mapLayers().values():
            if not isinstance(lyr, QgsVectorLayer):
                continue
            if lyr.geometryType() != QgsWkbTypes.PointGeometry:
                continue
            if not self._is_layer_visible(lyr):
                continue
            name = lyr.name().lower()
            if "detect" in name or name.endswith(".detections.geojson"):
                candidates.append(lyr)

        if candidates:
            return candidates

        # Fallback: all visible point layers.
        return [
            lyr
            for lyr in self.project.mapLayers().values()
            if isinstance(lyr, QgsVectorLayer)
            and lyr.geometryType() == QgsWkbTypes.PointGeometry
            and self._is_layer_visible(lyr)
        ]

    def _count_points_in_aoi(self, points) -> None:
        if len(points) < 3:
            self.status_label.setText("AOI tidak valid")
            return

        point_layers = self._candidate_point_layers()
        if not point_layers:
            QMessageBox.information(
                self,
                "AOI Count",
                "Tidak ada layer titik aktif/terlihat untuk dihitung.",
            )
            return

        poly_map = QgsGeometry.fromPolygonXY([points])
        if poly_map is None or poly_map.isEmpty():
            self.status_label.setText("AOI tidak valid")
            return

        da = QgsDistanceArea()
        da.setSourceCrs(self.project.crs(), self.project.transformContext())
        da.setEllipsoid(self.project.ellipsoid())
        area_m2 = da.measurePolygon(points)
        area_ha = area_m2 / 10000.0 if area_m2 > 0 else 0.0

        total = 0
        lines = []
        for layer in point_layers:
            poly_layer = QgsGeometry(poly_map)
            if (
                layer.crs().isValid()
                and self.project.crs().isValid()
                and layer.crs() != self.project.crs()
            ):
                try:
                    xform = QgsCoordinateTransform(
                        self.project.crs(),
                        layer.crs(),
                        self.project,
                    )
                    poly_layer.transform(xform)
                except Exception:
                    continue

            count = 0
            req = QgsFeatureRequest().setFilterRect(poly_layer.boundingBox())
            for feat in layer.getFeatures(req):
                geom = feat.geometry()
                if not geom or geom.isEmpty():
                    continue
                if poly_layer.contains(geom):
                    count += 1

            total += count
            density = (count / area_ha) if area_ha > 0 else 0.0
            lines.append(f"{layer.name()}: {count} titik ({density:.2f} titik/ha)")

        if not lines:
            self.status_label.setText("AOI count: 0")
            QMessageBox.information(self, "AOI Count", "Tidak ada titik di AOI.")
            return

        summary = (
            f"AOI area: {area_ha:.3f} ha\n"
            f"Total titik: {total}\n\n"
            + "\n".join(lines)
        )
        self.status_label.setText(f"AOI count total: {total} titik")
        QMessageBox.information(self, "AOI Count", summary)

    def zoom_full_extent(self) -> None:
        self._zoom_to_all_layers()
        self.status_label.setText("Zoomed to full extent")

    def zoom_to_active_layer(self) -> None:
        layer = self.layer_tree.currentLayer()
        if layer is None:
            self.status_label.setText("No active layer")
            return
        extent = layer.extent()
        if extent.isNull():
            self.status_label.setText("Layer has no extent")
            return
        dest_crs = self.project.crs()
        if layer.crs().isValid() and dest_crs.isValid() and layer.crs() != dest_crs:
            try:
                xform = QgsCoordinateTransform(layer.crs(), dest_crs, self.project)
                extent = xform.transformBoundingBox(extent)
            except Exception:
                pass
        self.canvas.setExtent(extent)
        self.canvas.refresh()
        self.status_label.setText(f"Zoomed to {layer.name()}")

    def open_attribute_table(self) -> None:
        layer = self.layer_tree.currentLayer()
        if layer is None or not isinstance(layer, QgsVectorLayer):
            QMessageBox.information(self, "Info", "Pilih layer vector terlebih dahulu.")
            return
        try:
            from qgis.gui import QgsAttributeTableDialog
            dlg = QgsAttributeTableDialog(layer, self)
            dlg.show()
        except Exception:
            QMessageBox.information(
                self,
                "Info",
                "Attribute table tidak tersedia di QGIS LTR ini.",
            )

    def open_symbology(self) -> None:
        layer = self.layer_tree.currentLayer()
        if layer is None:
            QMessageBox.information(self, "Info", "Pilih layer terlebih dahulu.")
            return
        dialog = QDialog(self)
        dialog.setWindowTitle("Symbology")
        layout = QFormLayout(dialog)

        if isinstance(layer, QgsRasterLayer):
            opacity = QSlider(Qt.Horizontal)
            opacity.setRange(0, 100)
            opacity.setValue(int(layer.opacity() * 100))
            layout.addRow("Opacity", opacity)

            def apply_raster():
                layer.setOpacity(opacity.value() / 100.0)
                self.canvas.refresh()

            buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
            buttons.accepted.connect(apply_raster)
            buttons.accepted.connect(dialog.accept)
            buttons.rejected.connect(dialog.reject)
            layout.addRow(buttons)
            dialog.exec_()
            return

        if isinstance(layer, QgsVectorLayer) and layer.geometryType() == QgsWkbTypes.PointGeometry:
            size = QSpinBox()
            size.setRange(1, 30)
            size.setValue(8)
            color_btn = QPushButton("Pick Color")
            color = {"value": QColor(255, 0, 0)}

            def pick_color():
                chosen = QColorDialog.getColor(color["value"], dialog, "Pick Color")
                if chosen.isValid():
                    color["value"] = chosen

            color_btn.clicked.connect(pick_color)
            layout.addRow("Size (px)", size)
            layout.addRow("Color", color_btn)

            def apply_vector():
                symbol = QgsMarkerSymbol.createSimple(
                    {
                        "name": "cross",
                        "color": f"{color['value'].red()},{color['value'].green()},{color['value'].blue()}",
                        "size": str(size.value()),
                        "outline_color": "255,255,255",
                        "outline_width": "1",
                    }
                )
                symbol.setSizeUnit(QgsUnitTypes.RenderPixels)
                layer.setRenderer(QgsSingleSymbolRenderer(symbol))
                layer.triggerRepaint()
                self.canvas.refresh()

            buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
            buttons.accepted.connect(apply_vector)
            buttons.accepted.connect(dialog.accept)
            buttons.rejected.connect(dialog.reject)
            layout.addRow(buttons)
            dialog.exec_()
            return

        QMessageBox.information(self, "Info", "Symbology hanya untuk raster dan point.")

    def _detect_dialog(self) -> dict | None:
        dialog = QDialog(self)
        dialog.setWindowTitle("Deteksi Pohon (YOLOv8)")
        layout = QFormLayout(dialog)

        default_model = Path("best.pt")
        if default_model.exists():
            model_input = QLineEdit(str(default_model.resolve()))
        else:
            fallback = Path("yolov9_trees.onnx")
            if fallback.exists():
                model_input = QLineEdit(str(fallback.resolve()))
            else:
                model_input = QLineEdit(str(Path("yolov8n.pt").resolve()))
        image_input = QLineEdit(self._get_default_raster_path())
        env_input = QLineEdit("mmyolo")

        tile_size = QSpinBox()
        tile_size.setRange(256, 4096)
        tile_size.setValue(1024)

        overlap_pct = QDoubleSpinBox()
        overlap_pct.setRange(0.0, 50.0)
        overlap_pct.setSingleStep(5.0)
        overlap_pct.setValue(12.5)

        conf = QDoubleSpinBox()
        conf.setRange(0.0, 1.0)
        conf.setSingleStep(0.05)
        conf.setValue(0.25)

        iou = QDoubleSpinBox()
        iou.setRange(0.0, 1.0)
        iou.setSingleStep(0.05)
        iou.setValue(0.45)

        def pick_model():
            path, _ = QFileDialog.getOpenFileName(
                self, "Pilih Model YOLO", "", "Model (*.pt *.onnx)"
            )
            if path:
                model_input.setText(path)

        def pick_image():
            path, _ = QFileDialog.getOpenFileName(
                self, "Pilih GeoTIFF", "", "GeoTIFF (*.tif *.tiff)"
            )
            if path:
                image_input.setText(path)

        model_btn = QAction("Browse", dialog)
        image_btn = QAction("Browse", dialog)

        model_btn_widget = QPushButton("Browse")
        model_btn_widget.clicked.connect(pick_model)
        image_btn_widget = QPushButton("Browse")
        image_btn_widget.clicked.connect(pick_image)

        model_row = QWidget()
        model_row_layout = QHBoxLayout(model_row)
        model_row_layout.setContentsMargins(0, 0, 0, 0)
        model_row_layout.addWidget(model_input)
        model_row_layout.addWidget(model_btn_widget)

        image_row = QWidget()
        image_row_layout = QHBoxLayout(image_row)
        image_row_layout.setContentsMargins(0, 0, 0, 0)
        image_row_layout.addWidget(image_input)
        image_row_layout.addWidget(image_btn_widget)

        layout.addRow("Model (.pt)", model_row)
        layout.addRow("GeoTIFF", image_row)
        layout.addRow("Env Conda", env_input)
        layout.addRow("Tile Size", tile_size)
        layout.addRow("Overlap (%)", overlap_pct)
        layout.addRow("Conf", conf)
        layout.addRow("IoU", iou)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addRow(buttons)

        if dialog.exec_() != QDialog.Accepted:
            return None

        overlap_px = int(tile_size.value() * (overlap_pct.value() / 100.0))
        return {
            "model": model_input.text().strip(),
            "image": image_input.text().strip(),
            "env": env_input.text().strip(),
            "tile": tile_size.value(),
            "overlap": overlap_px,
            "conf": conf.value(),
            "iou": iou.value(),
        }

    def _normalize_raster_source(self, source: str) -> str:
        if not source:
            return ""
        # Some providers append options after '|', keep filesystem path only.
        return source.split("|", 1)[0]

    def _get_default_raster_path(self) -> str:
        layer = self.layer_tree.currentLayer()
        if isinstance(layer, QgsRasterLayer):
            return self._normalize_raster_source(layer.source())
        for lyr in self.project.mapLayers().values():
            if isinstance(lyr, QgsRasterLayer):
                return self._normalize_raster_source(lyr.source())
        return ""

    def detect_palm_trees(self) -> None:
        if self.detect_process is not None:
            QMessageBox.information(self, "Info", "Deteksi masih berjalan.")
            return
        params = self._detect_dialog()
        if not params:
            return
        if not params["image"]:
            params["image"] = self._get_default_raster_path()
        if not params["model"] or not params["image"]:
            QMessageBox.information(self, "Info", "Model dan GeoTIFF wajib diisi.")
            return

        out_geojson = Path(params["image"]).with_suffix(".detections.geojson")
        script_path = Path(__file__).with_name("yolo_detect.py")
        if not script_path.exists():
            QMessageBox.critical(self, "Error", "yolo_detect.py tidak ditemukan.")
            return
        program, arguments = self._build_detector_command(params, script_path, out_geojson)
        self.status_label.setText("Running YOLO detection in background...")
        self.detect_output_path = out_geojson
        self.detect_log_chunks = []
        self.detect_stdout_buffer = ""
        self.progress_bar.setVisible(True)
        self.progress_bar.setRange(0, 0)
        self.progress_bar.setValue(0)
        self.act_detect.setEnabled(False)

        proc = QProcess(self)
        proc.setProgram(program)
        proc.setArguments(arguments)
        proc_env = QProcessEnvironment.systemEnvironment()
        proc_env.remove("PYTHONHOME")
        proc_env.remove("PYTHONPATH")
        proc.setProcessEnvironment(proc_env)
        proc.readyReadStandardOutput.connect(self._on_detect_stdout)
        proc.readyReadStandardError.connect(self._on_detect_stderr)
        proc.finished.connect(self._on_detect_finished)
        proc.start()
        self.detect_process = proc
        if not proc.waitForStarted(5000):
            self.act_detect.setEnabled(True)
            self.detect_process = None
            self.progress_bar.setVisible(False)
            self.progress_bar.setValue(0)
            QMessageBox.critical(self, "Error", "Gagal memulai proses deteksi.")
            return

    def _build_detector_command(self, params: dict, script_path: Path, out_geojson: Path):
        args = [
            "-u",
            str(script_path),
            "--image",
            params["image"],
            "--model",
            params["model"],
            "--tile",
            str(params["tile"]),
            "--overlap",
            str(params["overlap"]),
            "--conf",
            str(params["conf"]),
            "--iou",
            str(params["iou"]),
            "--output",
            str(out_geojson),
        ]

        conda_prefix_root = Path(
            os.environ.get("CONDA_PREFIX", r"C:\Users\ACER\miniconda3")
        )
        # Normalize when CONDA_PREFIX points to an env path (.../envs/<name>).
        if conda_prefix_root.name.lower() != "miniconda3":
            parts = [p.lower() for p in conda_prefix_root.parts]
            if "envs" in parts:
                envs_idx = parts.index("envs")
                if envs_idx > 0:
                    conda_prefix_root = Path(*conda_prefix_root.parts[:envs_idx])
        env_python = conda_prefix_root / "envs" / params["env"] / "python.exe"
        if env_python.exists():
            return str(env_python), args

        # Fallback to conda run only if env python path is unavailable.
        conda_exe = os.environ.get("CONDA_EXE")
        conda_bat = conda_prefix_root / "Scripts" / "conda.bat"
        if conda_exe is None:
            conda_exe = str(conda_bat) if conda_bat.exists() else "conda"
        fallback_args = [
            "/c",
            conda_exe,
            "run",
            "-n",
            params["env"],
            "python",
        ] + args
        return "cmd", fallback_args

    def _on_detect_stdout(self) -> None:
        if self.detect_process is None:
            return
        text = bytes(self.detect_process.readAllStandardOutput()).decode(
            "utf-8", errors="replace"
        )
        if text:
            self.detect_log_chunks.append(text)
            self.detect_stdout_buffer += text
            while "\n" in self.detect_stdout_buffer:
                line, self.detect_stdout_buffer = self.detect_stdout_buffer.split("\n", 1)
                self._handle_detect_output_line(line.strip())

    def _on_detect_stderr(self) -> None:
        if self.detect_process is None:
            return
        text = bytes(self.detect_process.readAllStandardError()).decode(
            "utf-8", errors="replace"
        )
        if text:
            self.detect_log_chunks.append(text)

    def _handle_detect_output_line(self, line: str) -> None:
        if line.startswith("STATUS "):
            self.status_label.setText(line[len("STATUS ") :].strip())
            return
        if not line.startswith("PROGRESS "):
            return
        payload = line[len("PROGRESS ") :]
        if "/" not in payload:
            return
        done_s, total_s = payload.split("/", 1)
        try:
            done = int(done_s)
            total = int(total_s)
        except ValueError:
            return
        if total <= 0:
            return
        if self.progress_bar.maximum() == 0:
            self.progress_bar.setRange(0, 100)
        pct = int((done / total) * 100)
        self.progress_bar.setValue(max(0, min(100, pct)))
        self.status_label.setText(f"Detecting palms... {done}/{total} tiles")

    def _on_detect_finished(self, exit_code: int, _exit_status) -> None:
        self.act_detect.setEnabled(True)
        self.progress_bar.setVisible(False)
        self.progress_bar.setValue(0)
        proc = self.detect_process
        self.detect_process = None
        log_text = "".join(self.detect_log_chunks).strip()
        self.detect_log_chunks = []
        if self.detect_stdout_buffer.strip():
            self._handle_detect_output_line(self.detect_stdout_buffer.strip())
        self.detect_stdout_buffer = ""
        out_geojson = self.detect_output_path
        self.detect_output_path = None

        if exit_code != 0:
            log_path = Path(__file__).with_name("yolo_detect.log")
            msg = log_text if log_text else f"Process exit code: {exit_code}"
            log_path.write_text(msg, encoding="utf-8")
            QMessageBox.critical(
                self,
                "Error",
                f"Deteksi gagal. Log disimpan di:\n{log_path}",
            )
            self.status_label.setText("Deteksi gagal")
            if proc is not None:
                proc.deleteLater()
            return

        if out_geojson is None:
            self.status_label.setText("Deteksi selesai, output tidak ditemukan")
            if proc is not None:
                proc.deleteLater()
            return

        # Remove existing detection layer with same source/name
        for lyr in list(self.project.mapLayers().values()):
            if lyr.name() == out_geojson.name or getattr(lyr, "source", lambda: "")() == str(out_geojson):
                self.project.removeMapLayer(lyr.id())

        layer = QgsVectorLayer(str(out_geojson), out_geojson.name, "ogr")
        if not layer.isValid():
            QMessageBox.critical(self, "Error", "Gagal memuat hasil deteksi.")
            return
        # Detection output coordinates are written in raster/project CRS.
        layer.setCrs(self.project.crs())
        if layer.geometryType() == QgsWkbTypes.PointGeometry:
            self._apply_detection_style(layer)
        self.project.addMapLayer(layer, False)
        self.project.layerTreeRoot().insertLayer(0, layer)
        self.canvas.setDestinationCrs(self.project.crs())
        self._refresh_canvas_layers()
        self._zoom_to_all_layers()
        self.canvas.refresh()
        count = layer.featureCount()
        self.status_label.setText(f"Deteksi selesai: {count} pohon")
        if proc is not None:
            proc.deleteLater()

    def _apply_detection_style(self, layer: QgsVectorLayer) -> None:
        symbol = QgsMarkerSymbol.createSimple(
            {
                "name": "circle",
                "color": "255,0,220,220",
                "size": "16",
                "outline_color": "0,0,0,220",
                "outline_width": "1.5",
            }
        )
        symbol.setSizeUnit(QgsUnitTypes.RenderPixels)
        layer.setRenderer(QgsSingleSymbolRenderer(symbol))
        layer.triggerRepaint()



def main() -> int:
    qgis_prefix = os.environ.get("QGIS_PREFIX_PATH")
    if not qgis_prefix:
        print("QGIS_PREFIX_PATH is not set.")
        print("Run via python-qgis-ltr.bat or set QGIS_PREFIX_PATH.")
        return 1

    QgsApplication.setPrefixPath(qgis_prefix, True)
    qgs = QgsApplication([], False)
    qgs.initQgis()

    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    code = app.exec_()

    qgs.exitQgis()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
