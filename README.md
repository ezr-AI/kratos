# Palm GeoTIFF Desktop Viewer

Desktop viewer sederhana untuk GeoTIFF besar dengan pan & zoom.
Menggunakan napari + rasterio (lazy loading via dask).

## Setup
```powershell
conda env create -f environment.yml
```

## Jalankan
```powershell
run.bat
```

Atau langsung:
```powershell
conda activate palm-viewer
python app\main.py "C:\Users\ACER\Documents\O7.tif"
```
