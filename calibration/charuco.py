"""ChArUco board definition and detection (OpenCV 4.5.4 legacy aruco API, as on the Jetson)."""
import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

BOARD_JSON = Path(__file__).parent / "board" / "board.json"


@dataclass
class BoardSpec:
    squares_x: int
    squares_y: int
    square_mm: float
    marker_mm: float
    dictionary: str

    @classmethod
    def load(cls, path=BOARD_JSON):
        d = json.loads(Path(path).read_text())
        return cls(d["squares_x"], d["squares_y"], d["square_mm"], d["marker_mm"], d["dictionary"])


class Charuco:
    def __init__(self, spec: BoardSpec):
        self.spec = spec
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec.dictionary))
        self.board = cv2.aruco.CharucoBoard_create(
            spec.squares_x, spec.squares_y, spec.square_mm, spec.marker_mm, self.dictionary)
        self.params = cv2.aruco.DetectorParameters_create()
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        # Board-frame (mm) coordinates of every inner chessboard corner, indexed by ChArUco id.
        self.object_points = np.asarray(self.board.chessboardCorners, dtype=np.float64).reshape(-1, 3)
        self.n_corners = len(self.object_points)

    def detect(self, gray):
        """Return (corners Nx1x2 float32, ids Nx1 int32) or (None, None) if nothing usable."""
        marker_corners, marker_ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        if marker_ids is None or len(marker_ids) == 0:
            return None, None
        marker_corners, marker_ids, _, _ = cv2.aruco.refineDetectedMarkers(
            gray, self.board, marker_corners, marker_ids, rejected)
        n, corners, ids = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, gray, self.board)
        if not n or ids is None:
            return None, None
        return corners, ids
