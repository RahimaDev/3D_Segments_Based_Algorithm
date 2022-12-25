# 3D segments based registration framework by Rahima Djahel (ENPC), Pascal Monasse (ENPC) and Bruno Vallet (IGN).

# Required dependencies: PCL, Eigen, OpenCV.

3Dseg_Registration: an efficient algorithm for Image/LiDAR data registration.
# Test on synthetic data

./Test_3DS  1.2 2


1.2: is a distance threshold above which a line is just considered an outlier (can be adapted by the users).

2: is a distance threshold to avoid selecting coplanar segment pairs (can be adapted by the users).



# Test on real data:
./3Ddeg_Registration ../data/img_lines.txt ../data/lidar_lines.txt 1.2     2  0


Where:


img_lines.txt: txt file contains informations about 3D segments reconstructed from an image sequence.

lidar_lines: txt file contains informations about 3D segments extracted from LiDAR scan

1.2: is a distance threshold above which a line is just
considered an outlier (can be adapted by the users).

2: is a distance threshold to avoid selecting coplanar segment pairs (can be adapted by the users).

0: is the verticality 

If the image and/or LiDAR can be vertically oriented (V=1), we obviously associate the vertical cluster of the image data to the vertical cluster of the LiDAR data, and then associate
any non vertical cluster of the image data with any non vertical cluster of the LiDAR data. In the other case (at least
one data-set cannot be vertically oriented) (V=0), we associate any pair of clusters of the image data to any pair of clusters of LiDAR data if they have a compatible angle.

# To visualize the result:

cloudcompare.CloudCompare TR_lines.obj LIDAR_LINES.obj


# If you use our algorithm in any of your publications or projects, please cite our paper:

DJAHEL, Rahima, MONASSE, Pascal, et VALLET, Bruno. A 3D SEGMENTS BASED ALGORITHM FOR HETEROGENEOUS DATA REGISTRATION. The International Archives of Photogrammetry, Remote Sensing and Spatial Information Sciences, 2022, vol. 43, p. 129-136.


# If you have any questions, you can send an email to :

rahima.djahel@enpc.fr

rdjahel@gmail.com
