import os
import sys
import math
import datetime

import cv2
import dlib
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt


# -------------------------------------------------------------------
# Paths
# -------------------------------------------------------------------
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)

MODEL_DIR = os.path.join(ROOT_DIR, "models")
IMAGE_DIR = os.path.join(ROOT_DIR, "images")
RESULTS_DIR = os.path.join(ROOT_DIR, "results")

PREDICTOR_PATH = os.path.join(MODEL_DIR, "shape_predictor_68_face_landmarks.dat")
IMAGE_PATH = os.path.join(IMAGE_DIR, "sample_face.png")  # change if needed


# -------------------------------------------------------------------
# Safety checks
# -------------------------------------------------------------------
if not os.path.exists(PREDICTOR_PATH):
    print(
        f"[ERROR] Facial landmark model not found.\n"
        f"Expected:\n  {PREDICTOR_PATH}\n\n"
        f"Download from:\n"
        f"  https://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2\n"
        f"Extract and place the .dat file into the 'models' folder."
    )
    sys.exit(1)

if not os.path.exists(IMAGE_PATH):
    print(
        f"[ERROR] Input image not found.\n"
        f"Expected:\n  {IMAGE_PATH}\n\n"
        f"Put a face image in the 'images' folder and update IMAGE_PATH if needed."
    )
    sys.exit(1)

# Ensure results folder exists
os.makedirs(RESULTS_DIR, exist_ok=True)


# -------------------------------------------------------------------
# Helpers
# -------------------------------------------------------------------
def calc_distance(x1, y1, x2, y2) -> float:
    """Euclidean distance between two points."""
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx * dx + dy * dy)


def build_landmark_graph(landmarks, distance_threshold: float = 51.0) -> tuple[nx.Graph, dict]:
    """
    Build a graph from 68 facial landmarks.

    nodes: 0..67
    edges: between nodes whose distance is below distance_threshold
    pos:   dict[node] = (x, -y) for plotting
    """
    xs = [landmarks.part(i).x for i in range(68)]
    ys = [landmarks.part(i).y for i in range(68)]

    pos = {i: (xs[i], -ys[i]) for i in range(68)}

    G = nx.Graph()
    G.add_nodes_from(range(68))

    for i in range(68):
        for j in range(i + 1, 68):
            if calc_distance(xs[i], ys[i], xs[j], ys[j]) < distance_threshold:
                G.add_edge(i, j)

    return G, pos


# -------------------------------------------------------------------
# Main pipeline
# -------------------------------------------------------------------
def main():
    # 1. Load models
    detector = dlib.get_frontal_face_detector()
    predictor = dlib.shape_predictor(PREDICTOR_PATH)

    # 2. Read image
    img = cv2.imread(IMAGE_PATH)
    if img is None:
        print(f"[ERROR] Failed to read image from {IMAGE_PATH}")
        sys.exit(1)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 3. Detect faces
    faces = detector(gray)
    if len(faces) == 0:
        print("[ERROR] No faces detected in the image.")
        sys.exit(1)

    face = faces[0]
    landmarks = predictor(gray, face)

    # 4. Draw facial keypoints
    for i in range(68):
        x = landmarks.part(i).x
        y = landmarks.part(i).y
        cv2.circle(img, (x, y), radius=2, color=(0, 255, 255), thickness=-1)

    # 5. Save results
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    image_output_path = os.path.join(RESULTS_DIR, f"landmarks_{timestamp}.png")

    cv2.imwrite(image_output_path, img)
    print(f"[INFO] Saved landmark image to: {image_output_path}")

    # show window
    cv2.imshow("Facial Landmarks", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # 6. Build and save graph
    G, pos = build_landmark_graph(landmarks, distance_threshold=51.0)
    plt.figure(figsize=(6, 6))
    nx.draw(
        G,
        pos,
        node_size=10,
        node_color="#e64105",
        edge_color="#aaaaaa",
        with_labels=False,
    )
    plt.title("Facial Landmark Graph")
    plt.axis("equal")

    graph_output_path = os.path.join(RESULTS_DIR, f"graph_{timestamp}.png")
    plt.savefig(graph_output_path, bbox_inches="tight", dpi=300)
    print(f"[INFO] Saved graph to: {graph_output_path}")
    plt.show()


# -------------------------------------------------------------------
# Entry point
# -------------------------------------------------------------------
if __name__ == "__main__":
    main()
