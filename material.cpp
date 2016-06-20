
#include "material.h"

Material::Material(Eigen::Vector3f amb, Eigen::Vector3f dif, Eigen::Vector3f spe, float sh, float refl, float a, float rp, float rn) {
    ambient = amb;
    diffuse = dif;
    specular = spe;
    shininess = sh * 128;
    reflectivity = (refl > 1)? 1 : refl;
    alpha = (a > 1)? 1 : a;
    refidx_p = rp;
    refidx_n = rn;
    texture = NULL;
}

Material::Material(float ar, float ag, float ab, float dr, float dg, float db, float sr, float sg, float sb, float sh, float refl, float a, float rp, float rn) {
    ambient = Eigen::Vector3f(ar, ag, ab);
    diffuse = Eigen::Vector3f(dr, dg, db);
    specular = Eigen::Vector3f(sr, sg, sb);
    shininess = sh * 128;
    reflectivity = (refl > 1)? 1 : refl;
    alpha = (a > 1)? 1 : a;
    refidx_p = rp;
    refidx_n = rn;
    texture = NULL;
}

Material::Material(float ta, float td, float ts, float sh, float refl, float a, float rp, float rn, bitmap_image *image) {
    texture_a = ta;
    texture_d = td;
    texture_s = ts;
    shininess = sh * 128;
    reflectivity = (refl > 1)? 1 : refl;
    alpha = (a > 1)? 1 : a;
    refidx_p = rp;
    refidx_n = rn;
    texture = image;
}
