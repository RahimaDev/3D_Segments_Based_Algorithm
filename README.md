# 3D segments based registration framework by RAhima Djahel (ENPC), Pascal Monasse (ENPC) and Bruno Vallet (IGN).

# Required dependencies: PCL, Eigen, OpenCV.

3Dseg_Registration: an efficient algorithm for Image/LiDAR data registration.

# Test:
./3Ddeg_Registration ../data/img_lines.txt ../data/lidar_lines.txt 0.54 4


Where:


img_lines.txt: txt file contains informations about 3D segments reconstructed from an image sequence.

lidar_lines: txt fole contains informations about 3D segments extracted from LiDAR scan

0.54: is a distance threshold above which a line is just
considered an outlier (can be adapted by the users).

4: is a distance threshold to avoid selecting coplanar segment pairs (can be adapted by the users).

# To visualize the result:

cloudcompare.CloudCompare TR_lines.obj LIDAR_LINES.obj


# If you use our algorithm in any of your publications or projects, please cite our paper:

DJAHEL, Rahima, MONASSE, Pascal, et VALLET, Bruno. A 3D SEGMENTS BASED ALGORITHM FOR HETEROGENEOUS DATA REGISTRATION. The International Archives of Photogrammetry, Remote Sensing and Spatial Information Sciences, 2022, vol. 43, p. 129-136.


# If you have any questions, you can send an email to :

rahima.djahel@enpc.fr

rdjahel@gmail.com
