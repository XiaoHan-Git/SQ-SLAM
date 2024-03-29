# SQ-SLAM
SQ-SLAM: Monocular Semantic SLAM Based on Superquadric Object Representation

Paper: [[arXiv:2209.10817](https://arxiv.org/pdf/2209.10817)]

Video: [YouTube](https://www.youtube.com/watch?v=PTB3og5J6QQ)

## Abstract

Object SLAM uses additional semantic information to detect and map objects in the scene, in order to improve the system's perception and map representation capabilities. Quadrics and cubes are often used to represent objects, but their single shape limits the accuracy of object map and thus affects the application of downstream tasks. In this paper, we introduce superquadrics (SQ) with shape parameters into SLAM for representing objects, and propose a separate parameter estimation method that can accurately estimate object pose and adapt to different shapes. Furthermore, we present a lightweight data association strategy for correctly associating semantic observations in multiple views with object landmarks. We implement a monocular semantic SLAM system with real-time performance and conduct comprehensive experiments on public datasets. The results show that our method is able to build accurate object map and has advantages in object representation. 

## Code

We have uploaded files related to SuperQuadrics. For other object-level SLAM content, please refer to [RO-MAP](https://github.com/XiaoHan-Git/RO-MAP). The only  difference, after excluding the NeRF part, is that RO-MAP uses bounding  boxes.
