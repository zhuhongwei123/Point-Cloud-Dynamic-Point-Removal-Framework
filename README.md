# Point-Cloud-Dynamic-Point-Removal-Framework

This code corresponds to our paper, which presents a framework for dynamic point removal. Our approach is based on Removert, but we have added several new functions to the code. As our paper is still under review, we cannot display all the code here, and some of our innovations have not been shown. However, the code can be compiled and run, including our ground segmentation and batch processing code.

The files RANSAC_Seg.h and Lidar2BEV.h in the Include directory contain the method for ground point segmentation, which is used for the comparative experiment in our paper. The clusting.h file contains a clustering algorithm, which is a module needed for the innovation in our thesis.

In particular, we have reserved a part of the code that demonstrates our most innovative 3D bounding box in the removert_main.cpp file.
