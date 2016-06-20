
#ifndef _PRIMITIVE_H_
#define _PRIMITIVE_H_

#include <iostream>
#include <vector>
#include "rayCast.h"
#include "material.h"
#include "bitmap_image.hpp"

inline void insertTriangle(World &world, std::vector<Eigen::Vector3f>, Eigen::Vector3f, Material *mat);

inline void insertTriangle(World &world, std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>, Eigen::Vector3f, Material *mat);

inline void insertTriangle(World &world, std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector2f>, Eigen::Vector3f, Material *mat);

void insertQuad(World &world, Eigen::Vector3f v0, Eigen::Vector3f v1, Eigen::Vector3f v2, Eigen::Vector3f v3, Eigen::Vector3f normal, Material *mat);

void insertCube(World &world, float x1, float x2, float y1, float y2, float z1, float z2, Material *mat);

#endif

