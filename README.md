# Face Key Points Detector

This project detects **68 facial landmarks** using `dlib` and visualizes them with **OpenCV** and **NetworkX**.  
It builds a **graph** where each node is a facial keypoint and edges represent spatial connections based on distance.

---

## Features

- Detects 68 facial landmarks using the pretrained dlib model  
- Visualizes landmarks directly on the image using OpenCV  
- Builds a NetworkX graph of landmarks and their spatial relations  
- Automatically saves both visualizations in the `results/` folder  
- Includes clear error messages and modular structure (`main()`)

---

## 📂 Project Structure

```text
Face-Key-Points-Detector/
│
├─ src/
│   └─ detect_keypoints.py        # main script
│
├─ models/
│   ├─ haarcascade_frontalface_alt2.xml
│   └─ shape_predictor_68_face_landmarks.dat
│
├─ images/
│   └─ sample_face.png            # example input
│
├─ results/                       # automatically created on run
│   ├─ landmarks_YYYYMMDD_HHMMSS.png
│   └─ graph_YYYYMMDD_HHMMSS.png
│
├─ requirements.txt
└─ .gitignore


