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


def _quick_percentiles_from_overview(path: str) -> tuple[float, float] | None:
    with rasterio.open(path) as src:
        factors = src.overviews(1)
        if factors:
            level = max(factors)
            scale = 1 / level
            out_height = max(1, int(src.height * scale))
            out_width = max(1, int(src.width * scale))
            sample = src.read(
                1,
                out_shape=(out_height, out_width),
                resampling=Resampling.average,
            ).astype(np.float32)
        else:
            scale = 512 / max(src.width, src.height)
            out_height = max(1, int(src.height * scale))
            out_width = max(1, int(src.width * scale))
            sample = src.read(
                1,
                out_shape=(out_height, out_width),
                resampling=Resampling.average,
            ).astype(np.float32)
    sample = sample[np.isfinite(sample)]
    if sample.size == 0:
        return None
    p2 = float(np.percentile(sample, 2))
    p98 = float(np.percentile(sample, 98))
    return p2, p98


def _build_multiscale_from_overviews(path: str, chunks: dict):
    with rasterio.open(path) as src:
        factors = src.overviews(1)
    if not factors:
        return None
    levels = sorted(factors)
    pyramid = []

    # Full-res as dask (RAM friendly)
    try:
        full = rxr.open_rasterio(
            path,
            chunks=chunks,
            masked=False,
            cache=False,
        )
        pyramid.append(full)
    except Exception:
        pass

    # Overviews as in-memory arrays (small, stable)
    for level in levels:
        try:
            with rasterio.open(path, overview_level=level) as src:
                data = src.read()
            pyramid.append(data)
        except Exception:
            continue

    if not pyramid:
        return None
    return pyramid


def _build_multiscale_coarsen(da, factors):
    pyramid = [da]
    for f in factors:
        pyramid.append(da.coarsen(y=f, x=f, boundary="trim").mean())
    return pyramid


def open_raster(path: str, preview_scale: float, use_full_res: bool):
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
    chunks = {"x": 512, "y": 512}
    pyramid = _build_multiscale_from_overviews(path, chunks)
    if pyramid is None:
        da = rxr.open_rasterio(
            path,
            chunks=chunks,
            masked=False,
            cache=False,
        )
        pyramid = _build_multiscale_coarsen(da, [2, 4, 8, 16, 32, 64])
    opts: dict = {}
    # Optionally drop full-res level for speed (like QGIS uses overviews)
    if not use_full_res and len(pyramid) > 1:
        pyramid = pyramid[1:]

    first = pyramid[0]
    is_xarray = hasattr(first, "dims")
    if (is_xarray and "band" in first.dims and first.rio.count in (3, 4)) or (
        isinstance(first, np.ndarray) and first.ndim == 3 and first.shape[0] in (3, 4)
    ):
        converted = []
        # First level may be xarray (dask)
        if is_xarray:
            converted.append(first.transpose("y", "x", "band").data)
        else:
            converted.append(np.transpose(first, (1, 2, 0)))
        for p in pyramid[1:]:
            if isinstance(p, np.ndarray):
                # numpy array in (band, y, x)
                converted.append(np.transpose(p, (1, 2, 0)))
            else:
                converted.append(p.transpose("y", "x", "band").data)
        pyramid = converted
        opts["rgb"] = True
        opts["multiscale"] = True
        return pyramid, opts

    # Single band
    converted = []
    if is_xarray:
        converted.append(first.isel(band=0).data)
    elif isinstance(first, np.ndarray):
        converted.append(first[0])
    for p in pyramid[1:]:
        if isinstance(p, np.ndarray):
            converted.append(p[0])
        else:
            converted.append(p.isel(band=0).data)
    pyramid = converted
    opts["multiscale"] = True
    limits = _quick_percentiles_from_overview(path)
    if limits:
        opts["contrast_limits"] = limits
    return pyramid, opts


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else None
    if not path:
        path = pick_file()
    if not path:
        print("No file selected.")
        return 1

    preview_scale = float(os.environ.get("PREVIEW_SCALE", "1.0"))
    use_full_res = os.environ.get("USE_FULL_RES", "1") == "1"
    data, opts = open_raster(path, preview_scale, use_full_res)
    viewer = napari.Viewer(title=f"Palm GeoTIFF Viewer - {Path(path).name}")
    layer = viewer.add_image(
        data,
        name=Path(path).name,
        contrast_limits=opts.get("contrast_limits"),
        rgb=opts.get("rgb", False),
        multiscale=opts.get("multiscale", False),
    )
    try:
        layer.interpolation = "linear"
    except Exception:
        pass
    napari.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
