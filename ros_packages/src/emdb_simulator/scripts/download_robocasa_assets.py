#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import shutil
import sys
import urllib.request
from pathlib import Path
from zipfile import BadZipFile, ZipFile


ASSET_LAYOUT = {
    "textures": "textures",
    "generative_textures": "generative_textures",
    "fixtures": "fixtures",
    "fixtures_lightwheel": "fixtures_lightwheel",
    "objaverse": "objects/objaverse",
    "aigen_objs": "objects/aigen_objs",
    "objects_lightwheel": "objects/lightwheel",
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Download RoboCasa assets into emdb_simulator/assets/robocasa"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "robocasa_box_links.json",
        help="Path to JSON file with Box shared links",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "assets" / "robocasa",
        help="Destination root folder",
    )
    parser.add_argument(
        "--types",
        nargs="+",
        default=["all"],
        help='Asset types to download, or "all"',
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing extracted folders",
    )
    parser.add_argument(
        "--keep-zips",
        action="store_true",
        help="Keep downloaded zip files in _downloads/",
    )
    return parser.parse_args()


def load_links(config_path: Path) -> dict[str, str]:
    with config_path.open("r", encoding="utf-8") as f:
        return json.load(f)


def to_direct_box_zip(shared_url: str) -> str:
    shared_id = shared_url.rstrip("/").split("/")[-1]
    base = shared_url.split("/s/")[0]
    return f"{base}/shared/static/{shared_id}.zip"


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def remove_dir(path: Path):
    if path.exists():
        shutil.rmtree(path)


def print_progress(filename: str):
    def _hook(blocks: int, block_size: int, total_size: int):
        if total_size > 0:
            downloaded = blocks * block_size
            pct = min(100.0, downloaded * 100.0 / total_size)
            sys.stdout.write(
                f"\rDownloading {filename}: {pct:5.1f}% "
                f"({downloaded / 1e6:.1f} MB / {total_size / 1e6:.1f} MB)"
            )
        else:
            sys.stdout.write(f"\rDownloading {filename} ...")
        sys.stdout.flush()

    return _hook


def download_file(url: str, dst: Path):
    ensure_dir(dst.parent)
    urllib.request.urlretrieve(url, filename=str(dst), reporthook=print_progress(dst.name))
    sys.stdout.write("\n")


def extract_zip(zip_path: Path, out_dir: Path):
    ensure_dir(out_dir)
    with ZipFile(zip_path, "r") as zf:
        bad_file = zf.testzip()
        if bad_file is not None:
            raise BadZipFile(f"Corrupted member inside zip: {bad_file}")
        zf.extractall(out_dir)


def download_one(asset_name: str, shared_url: str, output_root: Path, overwrite: bool, keep_zips: bool):
    if asset_name not in ASSET_LAYOUT:
        print(f"[skip] Unknown asset type in config: {asset_name}")
        return

    target_dir = output_root / ASSET_LAYOUT[asset_name]
    download_dir = output_root / "_downloads"
    zip_path = download_dir / f"{asset_name}.zip"
    direct_url = to_direct_box_zip(shared_url)

    if target_dir.exists() and any(target_dir.iterdir()) and not overwrite:
        print(f"[skip] {asset_name}: already exists at {target_dir}")
        return

    if overwrite and target_dir.exists():
        print(f"[info] removing existing folder: {target_dir}")
        remove_dir(target_dir)

    print(f"[info] asset:   {asset_name}")
    print(f"[info] source:  {shared_url}")
    print(f"[info] target:  {target_dir}")

    download_file(direct_url, zip_path)

    print(f"[info] extracting {zip_path.name} -> {target_dir}")
    extract_zip(zip_path, target_dir)

    if not keep_zips and zip_path.exists():
        zip_path.unlink()
        print(f"[info] deleted zip: {zip_path}")

    print(f"[done] {asset_name}\n")


def main():
    args = parse_args()

    links = load_links(args.config)
    output_root = args.output.resolve()
    ensure_dir(output_root)

    selected = list(links.keys()) if "all" in args.types else args.types

    for asset_name in selected:
        if asset_name not in links:
            print(f"[warn] {asset_name} not found in {args.config}")
            continue

        download_one(
            asset_name=asset_name,
            shared_url=links[asset_name],
            output_root=output_root,
            overwrite=args.overwrite,
            keep_zips=args.keep_zips,
        )


if __name__ == "__main__":
    main()