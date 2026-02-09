import os
import sys
from pathlib import Path

import numpy as np
import rasterio
from rasterio.enums import Resampling
import rioxarray as rxr
from qtpy.QtWidgets import QApplication, QFileDialog
import napari


def pick_file() -> str | None:
    app = QApplication.instance() or QApplication(sys.argv)
    path, _ = QFileDialog.getOpenFileName(
        None, "Open GeoTIFF", str(Path.home()), "GeoTIFF (*.tif *.tiff)"
    )
    return path or None


def _dtype_limits(dtype) -> tuple[float, float] | None:
    if np.issubdtype(dtype, np.integer):
        info = np.iinfo(dtype)
        return float(info.min), float(info.max)
    if np.issubdtype(dtype, np.floating):
        return None
    return None


def open_raster(path: str, preview_scale: float):
    if preview_scale < 1.0:
        with rasterio.open(path) as src:
            out_height = max(1, int(src.height * preview_scale))
            out_width = max(1, int(src.width * preview_scale))
            data = src.read(
                out_shape=(src.count, out_height, out_width),
                resampling=Resampling.average,
            )
        if data.shape[0] in (3, 4):
            data = data.transpose(1, 2, 0)
            return data, {"rgb": True}
        return data, {"channel_axis": 0}

    # Full-res lazy loading with dask (tile-based, RAM-friendly)
    da = rxr.open_rasterio(
        path,
        chunks={"x": 1024, "y": 1024},
        masked=False,
        cache=False,
    )
    # Use .data to keep it lazy (dask array)
    data = da.data

    opts: dict = {}
    limits = _dtype_limits(da.dtype)
    if limits:
        opts["contrast_limits"] = limits

    if "band" in da.dims and da.rio.count in (3, 4):
        data = da.transpose("y", "x", "band").data
        opts["rgb"] = True
    else:
        opts["channel_axis"] = 0
    return data, opts


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else None
    if not path:
        path = pick_file()
    if not path:
        print("No file selected.")
        return 1

    preview_scale = float(os.environ.get("PREVIEW_SCALE", "1.0"))
    data, opts = open_raster(path, preview_scale)
    viewer = napari.Viewer(title=f"Palm GeoTIFF Viewer - {Path(path).name}")
    viewer.add_image(
        data,
        name=Path(path).name,
        contrast_limits=opts.get("contrast_limits"),
        rgb=opts.get("rgb", False),
        channel_axis=opts.get("channel_axis"),
    )
    napari.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
