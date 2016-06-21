
#ifndef _MATERIAL_H_
#define _MATERIAL_H_

#include <Eigen/Dense>
#include "bitmap_image.hpp"

class Material {
public:
    Eigen::Vector3f ambient;
    Eigen::Vector3f diffuse;
    Eigen::Vector3f specular;
    float shininess;
    float reflectivity;
    float alpha;
    float refidx_p;
    float refidx_n;

    float texture_a;
    float texture_d;
    float texture_s;
    bool bumpmapped;
    bitmap_image *texture;

    Material(){}
    Material(Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f, float, float, float, float, float);
    Material(float, float, float, float, float, float, float, float, float, float, float, float, float, float);
    Material(float, float, float, float, float, float, float, float, bitmap_image*);
    Material(float, float, float, float, float, float, float, float, float, float, float, bitmap_image*, bool);
};

#endif

