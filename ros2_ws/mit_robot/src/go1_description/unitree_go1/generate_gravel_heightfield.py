#!/usr/bin/env python3
"""Regenerate the deterministic flat and sloped gravel heightfields."""

from pathlib import Path
import math
import random

from PIL import Image


WIDTH = 129
HEIGHT = 81
# 固定随机种子，确保重新生成的地形与仓库中的资产可复现。
SEED = 20260813
ASSET_DIRECTORY = Path(__file__).with_name("assets")
FLAT_OUTPUT = ASSET_DIRECTORY / "gravel_heightfield.png"
SLOPED_OUTPUT = ASSET_DIRECTORY / "gravel_slope_heightfield.png"


def smoothstep(value: float) -> float:
    """将输入限制到 [0, 1]，并返回平滑的三次插值权重."""
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def normalize(samples: list[float]) -> Image.Image:
    """Convert nonnegative height samples to a normalized grayscale image."""
    peak = max(samples)
    pixels = [round(255.0 * max(0.0, value) / peak) for value in samples]
    image = Image.new("L", (WIDTH, HEIGHT))
    image.putdata(pixels)
    return image


def generate_heightfield(sloped: bool) -> Image.Image:
    """生成平地或带长坡的确定性碎石高度场."""
    rng = random.Random(SEED)
    # Randomized phases keep the surface irregular while the seed makes the
    # checked-in terrain exactly reproducible.
    phases = [rng.uniform(0.0, 2.0 * math.pi) for _ in range(8)]
    samples: list[float] = []

    for row in range(HEIGHT):
        y = 2.0 * row / (HEIGHT - 1) - 1.0
        for column in range(WIDTH):
            x = 2.0 * column / (WIDTH - 1) - 1.0

            if sloped:
                # A long smooth ridge makes +X travel climb for the first half
                # and descend for the second half. Gravel remains superimposed.
                progress = 0.5 * (x + 1.0)
                hill = 0.72 * math.sin(math.pi * progress) ** 1.25
                broad = (
                    0.07 * math.sin(2.1 * math.pi * x + phases[0])
                    + 0.05 * math.sin(1.8 * math.pi * y + phases[1])
                    + 0.04 * math.sin(
                        2.6 * math.pi * (x + 0.45 * y) + phases[2]
                    )
                )
                gravel = (
                    0.045 * math.sin(7.0 * math.pi * x + 3.2 * y + phases[3])
                    + 0.040 * math.sin(8.5 * math.pi * y - 2.1 * x + phases[4])
                    + 0.035 * math.sin(10.0 * math.pi * (x + y) + phases[5])
                    + 0.030 * math.sin(
                        13.0 * math.pi * (x - 0.6 * y) + phases[6]
                    )
                    + 0.025 * math.sin(
                        17.0 * math.pi * (x + 0.2 * y) + phases[7]
                    )
                )
                surface = 0.15 + hill + broad + gravel
            else:
                # This patch has no macroscopic grade: only the original broad
                # compacted-gravel undulation and short crossing wavelengths.
                broad = (
                    0.26 * math.sin(2.1 * math.pi * x + phases[0])
                    + 0.20 * math.sin(1.8 * math.pi * y + phases[1])
                    + 0.16 * math.sin(
                        2.6 * math.pi * (x + 0.45 * y) + phases[2]
                    )
                )
                gravel = (
                    0.12 * math.sin(7.0 * math.pi * x + 3.2 * y + phases[3])
                    + 0.10 * math.sin(8.5 * math.pi * y - 2.1 * x + phases[4])
                    + 0.08 * math.sin(10.0 * math.pi * (x + y) + phases[5])
                    + 0.06 * math.sin(
                        13.0 * math.pi * (x - 0.6 * y) + phases[6]
                    )
                    + 0.04 * math.sin(
                        17.0 * math.pi * (x + 0.2 * y) + phases[7]
                    )
                )
                surface = 0.58 + broad + gravel

            # Separate longitudinal and lateral ramps make all four borders
            # exactly flat. The wider lateral fade avoids sharp side walls on
            # the newly narrowed strip.
            x_envelope = smoothstep((1.0 - abs(x)) / 0.15)
            y_envelope = smoothstep((1.0 - abs(y)) / 0.25)
            envelope = x_envelope * y_envelope
            samples.append(envelope * surface)

    return normalize(samples)


def main() -> None:
    """生成并保存 MuJoCo 使用的两张灰度高度场图片."""
    ASSET_DIRECTORY.mkdir(parents=True, exist_ok=True)
    generate_heightfield(sloped=False).save(FLAT_OUTPUT, optimize=True)
    generate_heightfield(sloped=True).save(SLOPED_OUTPUT, optimize=True)
    print(f"wrote {FLAT_OUTPUT} ({WIDTH}x{HEIGHT}, seed={SEED})")
    print(f"wrote {SLOPED_OUTPUT} ({WIDTH}x{HEIGHT}, seed={SEED})")


if __name__ == "__main__":
    main()
