
#include "material.h"

Material::Material(Eigen::Vector3f amb, Eigen::Vector3f dif, Eigen::Vector3f spe, float sh, float refl, float a, float rp, float rn) {
    ambient = amb;
    diffuse = dif;
    specular = spe;
    shininess = sh;
    reflectivity = (refl > 1)? 1: refl;
    alpha = (a > 1)? 1: a;
    refidx_p = rp;
    refidx_n = rn;
}

