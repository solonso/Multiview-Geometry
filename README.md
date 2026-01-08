# Multiview Geometry Laboratory Projects

This repository contains comprehensive laboratory work on multiview geometry and computer vision, covering fundamental topics from camera calibration to stereo visual odometry. The labs are implemented in MATLAB and explore various geometric transformations and estimation techniques used in computer vision applications.

## Overview

This project consists of four laboratory exercises that progressively build understanding of multiview geometry:

1. **Lab 1**: Camera Calibration and Projection Matrix Estimation
2. **Lab 2**: Feature Extraction and Image Registration
3. **Lab 3**: Epipolar Geometry and Stereo Vision
4. **Lab 4**: Stereo Visual Odometry

---

## Lab 1: Camera Calibration and Projection Matrix Estimation

**Location**: `lab1/`

### Objectives
- Understand camera intrinsic and extrinsic parameters
- Compute projection matrices from camera parameters
- Estimate projection matrices using the Hall method
- Analyze the effects of noise and sample size on estimation accuracy

### Key Topics Covered
- **Camera Parameters**: Intrinsic matrix (focal length, principal point) and extrinsic parameters (rotation, translation)
- **Projection Matrix**: 3×4 matrix mapping 3D world points to 2D image coordinates
- **Hall Method**: Linear method for estimating projection matrices from correspondences
- **RQ Decomposition**: Technique to extract intrinsic parameters and rotation from projection matrix

### Main Functions
- `points_3d_gen()`: Generates random 3D points in world coordinates
- `points_2d_gen()`: Projects 3D points onto 2D image plane
- `estimate_params()`: Implements Hall method for projection matrix estimation
- `get_intrinsics_from_proj_matrix()`: Extracts intrinsic matrix and rotation via RQ decomposition

### Experiments
- Projection matrix estimation with 6, 10, and 50 points
- Noise analysis: comparing noise-free vs. noisy 2D projections
- Error metrics: projection error, intrinsic parameter error, rotation error

---

## Lab 2: Feature Extraction and Image Registration

**Location**: `lab2/`

### Objectives
- Extract and match SIFT features between images
- Estimate different types of homographies
- Perform image registration and warping
- Evaluate matching quality under varying conditions

### Key Topics Covered
- **SIFT Features**: Scale-Invariant Feature Transform for robust feature detection
- **Feature Matching**: Matching descriptors using distance ratios
- **Homography Estimation**: Planar transformations including:
  - Translation
  - Similarity (rotation + scale + translation)
  - Affine
  - Projective (full homography)
- **Image Registration**: Aligning and warping images using estimated homographies

### Main Functions
- `siftprecomputed()`: Extracts SIFT features from images
- `matchsiftmodif()`: Matches features between two images with configurable distance ratio
- `computeHomographyEmpty()`: Estimates homography matrices for different transformation models
- `showwarpedimages()`: Visualizes image registration results
- `projectionerrorvec()`: Computes reprojection errors

### Experiments
- Feature extraction and visualization with different image pairs
- Matching analysis with varying distance ratios (0.4 to 0.9)
- Homography estimation and comparison across transformation models
- Evaluation using synthetic and real underwater image datasets

### Datasets
- **DataSet01**: Synthetic images with ground truth correspondences
- **MiamiSet00**: Underwater images for real-world testing

---

## Lab 3: Epipolar Geometry and Stereo Vision

**Location**: `lab3/`

### Objectives
- Understand the fundamental matrix and epipolar geometry
- Implement the 8-point algorithm for fundamental matrix estimation
- Enforce rank-2 constraint on fundamental matrix
- Analyze epipolar lines, epipoles, and their geometric properties

### Key Topics Covered
- **Fundamental Matrix**: 3×3 matrix encoding epipolar geometry between two views
- **8-Point Algorithm**: Linear method for estimating fundamental matrix from correspondences
- **Rank-2 Constraint**: Fundamental matrix must have rank 2 (enforced via SVD)
- **Normalization**: Improving 8-point algorithm robustness through coordinate normalization
- **Epipolar Geometry**: Epipolar lines, epipoles, and their relationships

### Main Functions
- `mvg_projectPointToImagePlane()`: Projects 3D points onto image planes
- `mvg_compute_epipolar_geom_modif()`: Computes epipolar line coefficients
- `mvg_show_epipolar_lines()`: Visualizes epipolar lines
- `mvg_show_epipoles()`: Displays epipoles in both images
- `mvg_compute_distances_to_epip_lines()`: Computes distances from points to epipolar lines

### Experiments
- Analytical fundamental matrix computation from camera parameters
- 8-point algorithm implementation with and without rank-2 enforcement
- Comparison of normalized vs. unnormalized 8-point algorithm
- Noise analysis: effects of Gaussian noise on fundamental matrix estimation
- Visualization of epipolar geometry with different fundamental matrices

### Key Insights
- Rank-2 enforcement significantly improves fundamental matrix accuracy
- Normalization reduces sensitivity to coordinate scaling
- Noise robustness improves with proper normalization and rank enforcement

---

## Lab 4: Stereo Visual Odometry

**Location**: `lab4/`

### Objectives
- Implement stereo visual odometry for camera motion estimation
- Track camera pose over image sequences
- Estimate 6-DOF camera motion from stereo pairs

### Key Topics Covered
- **Stereo Vision**: Using two cameras for depth estimation
- **Visual Odometry**: Estimating camera motion from image sequences
- **Pose Estimation**: Recovering 6-DOF camera transformations
- **Trajectory Estimation**: Tracking camera path over time

### Files
- `u1999088_u1999124_stereo_vo_lab_session_2024.mlx`: Live script containing the stereo visual odometry implementation
- Report documentation in PDF format

---

## Requirements

- **MATLAB** (R2018b or later recommended)
- **Image Processing Toolbox**
- **Computer Vision Toolbox** (for some visualization functions)
- **Statistics and Machine Learning Toolbox** (for some utility functions)

## Repository Structure

```
Multiview-Geometry/
├── lab1/                          # Camera calibration
│   ├── main.m                     # Main script for Lab 1
│   └── Multiview_Geometry_Lab1.pdf
│
├── lab2/                          # Feature extraction and registration
│   ├── lab2_section2_example.m    # SIFT feature extraction example
│   ├── lab2_section3_empty.m      # Matching analysis script
│   ├── lab2_section4_empty.m      # Homography estimation script
│   ├── computeHomographyEmpty.m   # Homography computation function
│   ├── match.m                    # Feature matching function
│   ├── matchsiftmodif.m           # Modified SIFT matching
│   ├── siftprecomputed.m          # SIFT feature extraction
│   ├── DataSet01/                 # Synthetic test dataset
│   ├── MiamiSet00/                # Underwater image dataset
│   └── MVG_Lab2_feature_extraction_image_registration_underwater_2024_v2.pdf
│
├── lab3/                          # Epipolar geometry
│   ├── lab3_empty.m               # Main script for Lab 3
│   ├── mvg_compute_epipolar_geom_modif.m
│   ├── mvg_show_epipolar_lines.m
│   ├── mvg_show_epipoles.m
│   ├── mvg_projectPointToImagePlane.m
│   ├── mvg_compute_distances_to_epip_lines.m
│   └── MVG_Lab3_epipolar_geometry_stereo_2024_v2.pdf
│
├── lab4/                          # Stereo visual odometry
│   ├── u1999088_u1999124_stereo_vo_lab_session_2024.mlx
│   └── u1999088_u1999124_Stereo Visual Odometer MVG Lab4_report.docx.pdf
│
└── README.md                      # This file
```

## Getting Started

1. **Clone the repository**:
   ```bash
   git clone https://github.com/solonso/Multiview-Geometry.git
   cd Multiview-Geometry
   ```

2. **Open MATLAB** and navigate to the repository directory

3. **Run individual labs**:
   - Lab 1: Execute `lab1/main.m`
   - Lab 2: Start with `lab2/lab2_section2_example.m`
   - Lab 3: Run `lab3/lab3_empty.m`
   - Lab 4: Open and run `lab4/u1999088_u1999124_stereo_vo_lab_session_2024.mlx`

## Key Concepts

### Camera Models
- **Pinhole Camera Model**: Basic camera model with intrinsic and extrinsic parameters
- **Projection Matrix**: Maps 3D world coordinates to 2D image coordinates

### Geometric Transformations
- **Homography**: Planar projective transformation preserving straight lines
- **Fundamental Matrix**: Describes geometric relationship between two uncalibrated views

### Feature-Based Methods
- **SIFT**: Robust local feature descriptor invariant to scale, rotation, and illumination
- **Feature Matching**: Establishing correspondences between images

### Estimation Techniques
- **Linear Estimation**: Direct methods (Hall method, 8-point algorithm)
- **Constraint Enforcement**: Ensuring geometric properties (rank-2 for fundamental matrix)
- **Normalization**: Improving numerical stability and accuracy

## Experimental Results

Each lab includes experiments demonstrating:
- **Accuracy Analysis**: Comparison of estimated vs. ground truth parameters
- **Noise Robustness**: Performance under varying noise levels
- **Sample Size Effects**: How estimation quality improves with more data points
- **Visualization**: Graphical representation of geometric relationships

## Authors

- Solomon Chibuzo Nwafor (@solonso)
- Muhammad Faran Akram (@mfa006)

---

