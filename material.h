
#ifndef _MATERIAL_H_
#define _MATERIAL_H_

#include <eigen3/Eigen/Dense>

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
    Material(Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f, float, float, float, float, float);
    Material(float, float, float, float, float, float, float, float, float, float, float, float, float, float);
};

#endif

