"""Nén giao diện web: web/* → data/www/*.gz (LittleFS).

Chạy tay:      python tools/gzip_www.py
Tự động:       platformio.ini có `extra_scripts = pre:tools/gzip_www.py`
               → chạy trước mỗi lần build / buildfs / uploadfs.

ESPAsyncWebServer tự phục vụ bản .gz khi trình duyệt xin file gốc.
"""
import gzip
import os

EXTS = (".html", ".css", ".js", ".svg", ".json", ".ico", ".webmanifest")


def project_dir():
    try:
        return env.subst("$PROJECT_DIR")  # noqa: F821 — biến SCons khi chạy qua PlatformIO
    except NameError:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def build(root):
    src = os.path.join(root, "web")
    dst = os.path.join(root, "data", "www")
    if not os.path.isdir(src):
        return
    os.makedirs(dst, exist_ok=True)
    total = 0
    for name in sorted(os.listdir(src)):
        if not name.endswith(EXTS):
            continue
        s = os.path.join(src, name)
        d = os.path.join(dst, name + ".gz")
        if not os.path.exists(d) or os.path.getmtime(d) < os.path.getmtime(s):
            with open(s, "rb") as f:
                data = f.read()
            with open(d, "wb") as f:
                # mtime=0 → file nén ổn định, không đổi nếu nội dung không đổi
                f.write(gzip.compress(data, compresslevel=9, mtime=0))
        total += os.path.getsize(d)
    # Xóa bản .gz mồ côi (file nguồn đã bị xóa)
    for name in os.listdir(dst):
        if name.endswith(".gz") and not os.path.exists(os.path.join(src, name[:-3])):
            os.remove(os.path.join(dst, name))
    print("gzip_www: data/www = %.1f KB (gzip)" % (total / 1024.0))


try:
    Import("env")  # noqa: F821
except NameError:
    pass
build(project_dir())
