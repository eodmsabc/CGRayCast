
#ifndef _PRIMITIVE_H_
#define _PRIMITIVE_H_

#include <iostream>
#include <vector>
#include "rayCast.h"
#include "material.h"
#include "bitmap_image.hpp"

void insertTriangle(World &world, Eigen::Vector3f v0, Eigen::Vector3f v1, Eigen::Vector3f v2, Eigen::Vector3f normal, Material *mat);

void insertTriangle(World &world, Eigen::Vector3f v0, Eigen::Vector3f v1, Eigen::Vector3f v2, Eigen::Vector3f normal, Material *mat, bitmap_image *t);

void insertQuad(World &world, Eigen::Vector3f v0, Eigen::Vector3f v1, Eigen::Vector3f v2, Eigen::Vector3f v3, Eigen::Vector3f normal, Material *mat);

void insertQuad(World &world, Eigen::Vector3f v0, Eigen::Vector3f v1, Eigen::Vector3f v2, Eigen::Vector3f v3, Eigen::Vector3f normal, Material *mat, bitmap_image *t);

void insertCube(World &world, float x1, float x2, float y1, float y2, float z1, float z2, Material *mat);

#endif

