import markdown
from weasyprint import HTML, CSS

md_content = """
<div class="header-container">
    <div class="logo-box">
        <img src="https://upload.wikimedia.org/wikipedia/en/thumb/1/1c/IIT_Kharagpur_Logo.svg/1200px-IIT_Kharagpur_Logo.svg.png" width="90">
    </div>
    <div class="header-text">
        <h2>Indian Institute of Technology Kharagpur</h2>
        <p>Department of Industrial and Systems Engineering<br>
        BTP-I Mid-Semester Project Summary<br>
        Autumn Semester 2026–27</p>
    </div>
</div>

<hr style="border-top: 1px solid black; margin: 5px 0;">

<table style="width: 100%; font-size: 11pt; border: none; margin-bottom: 20px;">
    <tr>
        <td><b>Name:</b> Shaurya Baranwal</td>
        <td><b>Roll No.:</b> 23IM10052</td>
        <td style="text-align: right;"><b>Supervisor:</b> Prof. Subhajit Sidhanta</td>
    </tr>
</table>

<h2 style="text-align: center; margin-bottom: 20px;">3D Environment Reconstruction using UAV LiDAR Data</h2>

### Background and Motivation
Unmanned Aerial Vehicles (UAVs) equipped with Light Detection and Ranging (LiDAR) sensors provide an unparalleled ability to rapidly capture highly accurate, dense 3D structural data of large-scale environments. Such 3D reconstructed maps are essential for autonomous navigation, infrastructure inspection, and detailed topographical surveying. However, processing raw LiDAR point clouds in real-time involves significant computational challenges, particularly in handling the immense volume of unorganized data and accurately estimating the trajectory of the UAV under complex flight dynamics.

Recent advancements in LiDAR Inertial Odometry (LIO), such as the FAST-LIO2 algorithm, have demonstrated that incorporating incremental KD-tree structures allows for efficient state estimation and point cloud registration. This project investigates the application of such state-of-the-art LIO frameworks on public UAV LiDAR datasets (e.g., MARS-LVIG) to reconstruct 3D surroundings. The study aims to eventually extend these pipelines by incorporating adaptive voxel-based resolution scaling to dynamically manage map density, improving both storage efficiency and real-time computational performance without losing critical environmental details.

### Aim and Objectives
The aim of this project is to develop and evaluate a robust pipeline for 3D surrounding reconstruction using UAV LiDAR data, bridging the gap between raw sensor data acquisition and usable 3D occupancy maps. Crucially, this pipeline is being designed for eventual integration with a physical custom-built UAV platform equipped with a LiDAR sensor, which is currently being developed by the project team. 
The main objectives are:
* Establish a complete data pipeline capable of converting and streaming UAV LiDAR datasets into modern ROS2 architectures.
* Integrate and configure the FAST-LIO2 odometry framework to process raw point clouds and generate registered, dense 3D maps.
* Develop custom visualization and validation tools to analyze the reconstructed environments in real-time.
* Design an adaptive voxel-based resolution scaling algorithm to optimize map density based on LiDAR distance and structural complexity.
* Prepare the software architecture for seamless integration and real-time deployment on the team's custom-built LiDAR-equipped UAV.

### Proposed System Architecture
The proposed pipeline separates data acquisition, state estimation, mapping, and map optimization. 
The system initiates by ingesting raw dataset logs (MARS-LVIG) containing custom-serialized LiDAR and IMU packets. A data conversion layer translates this into standardized ROS2 topics (`livox_ros_driver2/msg/CustomMsg`). These streams are fed into the FAST-LIO2 node, which performs tight coupling of LiDAR and IMU measurements via an Iterated Kalman Filter (IKF). 
The output registered point clouds are continuously accumulated into a global map (PCD format). In future phases, this map will be processed through an adaptive voxel-grid optimizer to dynamically adjust resolution before being converted into a volumetric 3D occupancy grid (OctoMap) for path-planning applications.

### Current Progress
Significant progress has been made in developing the foundational data pipeline and validating the core state estimation algorithm. The primary dataset being used for initial validation is the publicly available MARS-LVIG dataset, which contains high-fidelity UAV LiDAR and IMU recordings.
The current achievements include:
* **ROS2 Pipeline Migration:** A custom data translation layer was developed to successfully convert legacy ROS1 Livox hardware serializations into modern ROS2 Jazzy `CustomMsg` standards, handling complex binary serialization anomalies.
* **Odometry Integration:** The FAST-LIO2 framework was compiled, integrated, and deployed within a user-local unprivileged ROS2 environment.
* **System Debugging and QoS Tuning:** Addressed critical Quality of Service (QoS) mismatch issues between the ROS2 message publishing layer and FAST-LIO2 subscribers, ensuring reliable and lossless data ingestion.
* **Baseline Map Generation:** The pipeline was successfully executed on the `HKairport01` dataset sequence. The state estimation accurately initialized the incremental KD-tree and consecutively processed incoming point clouds, extracting a dense global map comprising over 20,000 spatial map points.
* **Visualization Tools:** A custom Python-based publisher utilizing Open3D was developed to project the generated point cloud map (`scans.pcd`) back into RViz for real-time remote visualization and structural inspection.

### Planned Experimental Study and Further Development
The next phase will focus on post-processing the registered point clouds and introducing the planned algorithmic contributions. 
First, the generated PCD maps will be converted into 3D occupancy grids (OctoMap) to enable probabilistic representations of free and occupied space, which is critical for downstream autonomous path-planning tasks.

Subsequently, the core algorithmic research contribution will be implemented: an adaptive voxel-grid scaling algorithm. Unlike standard uniform voxel downsampling, this approach will dynamically adjust the voxel resolution based on the distance from the sensor and the local feature density. 
Finally, the complete 19GB MARS-LVIG dataset will be processed through this optimized pipeline to evaluate the computational overhead, mapping accuracy, and memory efficiency of the adaptive resolution approach against standard fixed-resolution pipelines.

### Expected Outcome
The expected outcome is an experimentally validated, ROS2-compatible framework for UAV LiDAR reconstruction that can efficiently process large-scale datasets. The project will provide a methodology for adaptive map resolution, demonstrating improvements in computational efficiency and storage utilization while preserving crucial structural details for autonomous navigation.

<br><br><br>

<table style="width: 100%; font-size: 11pt; border: none; margin-top: 50px;">
    <tr>
        <td style="width: 50%;"><b>Submitted by</b><br><br><br>Shaurya Baranwal<br>Roll No.: 23IM10052<br>Department of Industrial and Systems Engineering<br>IIT Kharagpur</td>
        <td style="width: 50%; text-align: right; vertical-align: top;"><b>Supervisor's Signature</b><br><br><br>____________________________________<br>Prof. Subhajit Sidhanta<br>Department of Industrial and Systems Engineering<br>IIT Kharagpur</td>
    </tr>
</table>
"""

html_body = markdown.markdown(md_content)

full_html = f"""
<!DOCTYPE html>
<html>
<head>
<style>
    @page {{
        size: A4;
        margin: 20mm 20mm 20mm 20mm;
        @bottom-center {{
            content: counter(page);
        }}
    }}
    body {{
        font-family: 'Times New Roman', Times, serif;
        font-size: 11pt;
        line-height: 1.3;
        text-align: justify;
    }}
    h1, h2, h3, h4 {{
        font-family: 'Times New Roman', Times, serif;
        margin-top: 15px;
        margin-bottom: 5px;
    }}
    h3 {{
        font-size: 12pt;
    }}
    p {{
        margin-top: 5px;
        margin-bottom: 10px;
    }}
    ul {{
        margin-top: 0px;
        margin-bottom: 10px;
        padding-left: 20px;
    }}
    li {{
        margin-bottom: 3px;
    }}
    .header-container {{
        width: 100%;
        text-align: center;
        margin-bottom: 10px;
    }}
    .logo-box {{
        display: inline-block;
        vertical-align: middle;
        margin-right: 15px;
    }}
    .header-text {{
        display: inline-block;
        vertical-align: middle;
        text-align: center;
    }}
    .header-text h2 {{
        margin: 0;
        font-size: 16pt;
    }}
    .header-text p {{
        margin: 5px 0 0 0;
        font-size: 11pt;
    }}
</style>
</head>
<body>
{html_body}
</body>
</html>
"""

pdf_file = "/home/shaurya/BTP_UAV_LiDAR_Reconstruction/Reports/Mid_term_report_23IM10052.pdf"
HTML(string=full_html).write_pdf(pdf_file)
print(f"Generated {pdf_file}")
