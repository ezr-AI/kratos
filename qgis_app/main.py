import os
import sys
from pathlib import Path

from PyQt5.QtCore import Qt, QSize
from PyQt5.QtGui import QColor, QIcon
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
    QFileSystemModel,
    QGroupBox,
)
from qgis.core import (
    QgsApplication,
    QgsLayerTreeModel,
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
)
from qgis.gui import QgsLayerTreeMapCanvasBridge, QgsLayerTreeView, QgsMapCanvas
from qgis.gui import (
    QgsMapToolPan,
    QgsMapToolZoom,
    QgsMapToolEmitPoint,
    QgsRubberBand,
)
import subprocess
from pathlib import Path


class DistanceMeasureTool(QgsMapToolEmitPoint):
    def __init__(self, canvas, project, on_result):
        super().__init__(canvas)
        self.canvas = canvas
        self.project = project
        self.on_result = on_result
        self._first = None
        self.rubber = QgsRubberBand(canvas, QgsWkbTypes.LineGeometry)
        self.rubber.setColor(QColor(0, 180, 255, 200))
        self.rubber.setWidth(2)

    def canvasReleaseEvent(self, event):
        if event.button() != Qt.LeftButton:
            return
        point = self.toMapCoordinates(event.pos())
        if self._first is None:
            self._first = point
            self.rubber.reset(QgsWkbTypes.LineGeometry)
            self.rubber.addPoint(point, True)
            self.on_result("Pilih titik kedua untuk jarak")
            return
        da = QgsDistanceArea()
        da.setSourceCrs(self.project.crs(), self.project.transformContext())
        da.setEllipsoid(self.project.ellipsoid())
        dist = da.measureLine(self._first, point)
        self.rubber.addPoint(point, True)
        self._first = None
        self.on_result(f"Jarak: {dist:.2f} m")

    def clear(self):
        self._first = None
        self.rubber.reset(QgsWkbTypes.LineGeometry)


class AreaMeasureTool(QgsMapToolEmitPoint):
    def __init__(self, canvas, project, on_result):
        super().__init__(canvas)
        self.canvas = canvas
        self.project = project
        self.on_result = on_result
        self.points = []
        self.rubber = QgsRubberBand(canvas, QgsWkbTypes.PolygonGeometry)
        self.rubber.setColor(QColor(255, 0, 0, 180))
        self.rubber.setWidth(2)

    def canvasReleaseEvent(self, event):
        if event.button() == Qt.RightButton:
            self._finish()
            return
        if event.button() != Qt.LeftButton:
            return
        point = self.toMapCoordinates(event.pos())
        self.points.append(point)
        self.rubber.addPoint(point, True)

    def _finish(self):
        if len(self.points) < 3:
            self.on_result("Butuh >= 3 titik untuk luas")
            self._reset()
            return
        da = QgsDistanceArea()
        da.setSourceCrs(self.project.crs(), self.project.transformContext())
        da.setEllipsoid(self.project.ellipsoid())
        area = da.measurePolygon(self.points)
        self.on_result(f"Luas: {area:.2f} m²")
        # Keep polygon visible for review

    def _reset(self):
        self.points = []
        self.rubber.reset(QgsWkbTypes.PolygonGeometry)

    def clear(self):
        self._reset()


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Palm QGIS Lite")
        self.resize(1280, 800)

        self.project = QgsProject.instance()
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

        self.bridge = QgsLayerTreeMapCanvasBridge(root, self.canvas)
        self.bridge.setAutoSetupOnFirstLayer(True)

        self.status_label = QLabel("Ready")
        self.coord_label = QLabel("X: -, Y: -")
        self.scale_label = QLabel("Scale: -")
        self.crs_label = QLabel("CRS: -")

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

        browser_box = QGroupBox("Browser")
        browser_layout = QVBoxLayout(browser_box)
        browser_layout.setContentsMargins(6, 6, 6, 6)
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
        self.statusBar().addWidget(self.status_label, 1)
        self.statusBar().addWidget(self.coord_label)
        self.statusBar().addWidget(self.scale_label)
        self.statusBar().addWidget(self.crs_label)

        self._init_map_tools()
        self._bind_canvas_signals()
        self._build_properties_dock()

    def _init_map_tools(self) -> None:
        self.pan_tool = QgsMapToolPan(self.canvas)
        self.pan_tool.setAction(None)
        self.zoom_in_tool = QgsMapToolZoom(self.canvas, False)
        self.zoom_out_tool = QgsMapToolZoom(self.canvas, True)
        self.measure_dist_tool = DistanceMeasureTool(
            self.canvas, self.project, self.status_label.setText
        )
        self.measure_area_tool = AreaMeasureTool(
            self.canvas, self.project, self.status_label.setText
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
        self.coord_label.setText(f"X: {point.x():.2f}, Y: {point.y():.2f}")

    def _update_scale(self) -> None:
        self.scale_label.setText(f"Scale: 1:{int(self.canvas.scale()):,}")

    def _update_crs(self) -> None:
        crs = self.project.crs()
        if crs.isValid():
            self.crs_label.setText(f"CRS: {crs.authid()}")
        else:
            self.crs_label.setText("CRS: -")

    def _build_menubar(self) -> None:
        menubar = QMenuBar(self)
        file_menu = QMenu("File", self)
        view_menu = QMenu("View", self)

        act_open_raster = QAction("Open GeoTIFF", self)
        act_open_vector = QAction("Open KML/GeoJSON", self)
        act_exit = QAction("Exit", self)

        act_open_raster.triggered.connect(self.open_raster)
        act_open_vector.triggered.connect(self.open_vector)
        act_exit.triggered.connect(self.close)

        file_menu.addAction(act_open_raster)
        file_menu.addAction(act_open_vector)
        file_menu.addSeparator()
        file_menu.addAction(act_exit)

        # Placeholder view menu
        view_menu.addAction("Full Extent", self.zoom_full_extent)
        view_menu.addAction("Zoom to Layer", self.zoom_to_active_layer)

        menubar.addMenu(file_menu)
        menubar.addMenu(view_menu)
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
        crs = layer.crs()
        self.props_crs.setText(crs.authid() if crs.isValid() else "-")

    def set_pan_tool(self) -> None:
        self.canvas.setMapTool(self.pan_tool)
        self.status_label.setText("Pan tool active")

    def set_zoom_in_tool(self) -> None:
        self.canvas.setMapTool(self.zoom_in_tool)
        self.status_label.setText("Zoom in tool active")

    def set_zoom_out_tool(self) -> None:
        self.canvas.setMapTool(self.zoom_out_tool)
        self.status_label.setText("Zoom out tool active")

    def set_measure_distance(self) -> None:
        self.canvas.setMapTool(self.measure_dist_tool)
        self.status_label.setText("Klik 2 titik untuk jarak")

    def set_measure_area(self) -> None:
        self.canvas.setMapTool(self.measure_area_tool)
        self.status_label.setText("Klik titik poligon, klik kanan untuk selesai")

    def clear_measurements(self) -> None:
        self.measure_dist_tool.clear()
        self.measure_area_tool.clear()
        self.canvas.refresh()
        self.status_label.setText("Measurement cleared")

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
        act_clear = QAction(icon("clear"), "Clear Layers", self)
        act_pan = QAction(icon("pan"), "Pan", self)
        act_zoom_in = QAction(icon("zoom_in"), "Zoom In", self)
        act_zoom_out = QAction(icon("zoom_out"), "Zoom Out", self)
        act_full_extent = QAction(icon("full_extent"), "Full Extent", self)
        act_zoom_layer = QAction(icon("zoom_layer"), "Zoom to Layer", self)
        act_detect = QAction(icon("detect"), "Detect Palms (YOLO)", self)
        act_measure_dist = QAction(icon("measure_dist"), "Measure Distance", self)
        act_measure_area = QAction(icon("measure_area"), "Measure Area", self)
        act_measure_clear = QAction(icon("measure_clear"), "Clear Measure", self)
        act_attr_table = QAction(icon("table"), "Attribute Table", self)
        act_symbology = QAction(icon("symbology"), "Symbology", self)

        act_open_raster.triggered.connect(self.open_raster)
        act_open_vector.triggered.connect(self.open_vector)
        act_clear.triggered.connect(self.clear_layers)
        act_pan.triggered.connect(self.set_pan_tool)
        act_zoom_in.triggered.connect(self.set_zoom_in_tool)
        act_zoom_out.triggered.connect(self.set_zoom_out_tool)
        act_full_extent.triggered.connect(self.zoom_full_extent)
        act_zoom_layer.triggered.connect(self.zoom_to_active_layer)
        act_detect.triggered.connect(self.detect_palm_trees)
        act_measure_dist.triggered.connect(self.set_measure_distance)
        act_measure_area.triggered.connect(self.set_measure_area)
        act_measure_clear.triggered.connect(self.clear_measurements)
        act_attr_table.triggered.connect(self.open_attribute_table)
        act_symbology.triggered.connect(self.open_symbology)

        tb.addAction(act_open_raster)
        tb.addAction(act_open_vector)
        tb.addAction(act_clear)
        tb.addSeparator()
        tb.addAction(act_pan)
        tb.addAction(act_zoom_in)
        tb.addAction(act_zoom_out)
        tb.addSeparator()
        tb.addAction(act_full_extent)
        tb.addAction(act_zoom_layer)
        tb.addSeparator()
        tb.addAction(act_detect)
        tb.addSeparator()
        tb.addAction(act_measure_dist)
        tb.addAction(act_measure_area)
        tb.addAction(act_measure_clear)
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
        # Set project CRS from raster for on-the-fly reprojection
        if layer.crs().isValid():
            self.project.setCrs(layer.crs())
        else:
            self.project.setCrs(QgsCoordinateReferenceSystem("EPSG:32748"))
        # Avoid duplicate nodes when re-adding same file
        # Add raster below vectors (so points stay visible)
        self.project.addMapLayer(layer, False)
        self.project.layerTreeRoot().addLayer(layer)
        self.canvas.setDestinationCrs(self.project.crs())
        self._refresh_canvas_layers()
        self._zoom_to_all_layers()
        self.canvas.refresh()
        self.status_label.setText(f"Loaded {Path(path).name}")

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
        # If CRS is missing, assume project CRS (detections are in raster CRS)
        if not layer.crs().isValid():
            if self.project.crs().isValid():
                layer.setCrs(self.project.crs())
            else:
                layer.setCrs(QgsCoordinateReferenceSystem("EPSG:32748"))
        # Style vector for visibility (red outline / points)
        if layer.geometryType() == QgsWkbTypes.PointGeometry:
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
        # Respect layer tree order; ensure vectors draw above rasters
        root = self.project.layerTreeRoot()
        layers = []
        for node in root.children():
            lyr = getattr(node, "layer", lambda: None)()
            if lyr:
                layers.append(lyr)
        if not layers:
            self.canvas.setLayers([])
            return
        rasters = [l for l in layers if isinstance(l, QgsRasterLayer)]
        vectors = [l for l in layers if isinstance(l, QgsVectorLayer)]
        ordered = rasters + vectors
        self.canvas.setLayers(ordered)

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
        image_input = QLineEdit("")
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

    def detect_palm_trees(self) -> None:
        params = self._detect_dialog()
        if not params:
            return
        if not params["model"] or not params["image"]:
            QMessageBox.information(self, "Info", "Model dan GeoTIFF wajib diisi.")
            return

        out_geojson = Path(params["image"]).with_suffix(".detections.geojson")
        conda_exe = os.environ.get("CONDA_EXE")
        conda_bat = os.path.join(
            os.environ.get("CONDA_PREFIX", r"C:\Users\ACER\miniconda3"),
            "Scripts",
            "conda.bat",
        )
        if conda_exe is None:
            conda_exe = conda_bat if os.path.exists(conda_bat) else "conda"
        script_path = Path(__file__).with_name("yolo_detect.py")
        if not script_path.exists():
            QMessageBox.critical(self, "Error", "yolo_detect.py tidak ditemukan.")
            return
        cmd = [
            "cmd",
            "/c",
            conda_exe,
            "run",
            "-n",
            params["env"],
            "python",
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
        try:
            self.status_label.setText("Running YOLOv8 detection...")
            env = os.environ.copy()
            env.pop("PYTHONHOME", None)
            env.pop("PYTHONPATH", None)
            result = subprocess.run(cmd, capture_output=True, text=True, env=env)
            if result.returncode != 0:
                msg = result.stderr or result.stdout or "Unknown error"
                log_path = Path(__file__).with_name("yolo_detect.log")
                log_path.write_text(msg, encoding="utf-8")
                QMessageBox.critical(
                    self,
                    "Error",
                    f"Deteksi gagal. Log disimpan di:\n{log_path}",
                )
                return
        except Exception as exc:
            QMessageBox.critical(self, "Error", f"Deteksi gagal: {exc}")
            return

        # Remove existing detection layer with same source/name
        for lyr in list(self.project.mapLayers().values()):
            if lyr.name() == out_geojson.name or getattr(lyr, "source", lambda: "")() == str(out_geojson):
                self.project.removeMapLayer(lyr.id())

        layer = QgsVectorLayer(str(out_geojson), out_geojson.name, "ogr")
        if not layer.isValid():
            QMessageBox.critical(self, "Error", "Gagal memuat hasil deteksi.")
            return
        # Detection output coordinates are in raster CRS; force project CRS.
        layer.setCrs(self.project.crs())
        if layer.geometryType() == QgsWkbTypes.PointGeometry:
            symbol = QgsMarkerSymbol.createSimple(
                {
                    "name": "cross",
                    "color": "255,0,0",
                    "size": "12",
                    "outline_color": "255,255,255",
                    "outline_width": "1.2",
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
        count = layer.featureCount()
        self.status_label.setText(f"Deteksi selesai: {count} pohon")



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
