
#ifndef _RAY_CAST_H_
#define _RAY_CAST_H_

#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "material.h"
#include "bitmap_image.hpp"

#define INF 100000

class Ray{
public:
    Eigen::Vector3f start;
    Eigen::Vector3f direction;
    float refidx;
    Ray(Eigen::Vector3f, Eigen::Vector3f);
    Ray(Eigen::Vector3f, Eigen::Vector3f, float);
};

struct Target {
    Eigen::Vector3f point;
    Eigen::Vector3f normal;
    Material *material;
};

class Triangle {
public:
    std::vector<Eigen::Vector3f> vertices;
    std::vector<Eigen::Vector3f> vn;
    std::vector<Eigen::Vector2f> vt;
    Eigen::Vector3f normal;
    Material *material;

    Triangle(){}
    Triangle(std::vector<Eigen::Vector3f>, Eigen::Vector3f, Material*);
    Triangle(std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>, Eigen::Vector3f, Material*);
    Triangle(std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector2f>, Eigen::Vector3f, Material*);
    
    Eigen::Vector3f barycentric(Eigen::Vector3f);
};

class Sphere {
public:
    Eigen::Vector3f center;
    float radius;
    Material *material;
    Sphere(Eigen::Vector3f, float, Material*);

    Eigen::Vector2f theta_phi_conversion(Eigen::Vector3f);
};

class World {
public:
    std::vector<Triangle> planeList;
    std::vector<Sphere> sphereList;
    std::vector<Eigen::Vector3f> pointLights;

    World();
    void insertLight(float, float, float);
    void insertLight(Eigen::Vector3f);
    void insert(Triangle);
    void insert(Sphere);
    
    // Ray Cast
    bool rayCast(Ray, float, Target &);
    bool rayCast(Ray, float);
};

bool sameSideWithCenter(Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f);
bool isIntersect(Triangle, Ray, float &);
bool isIntersect(Sphere, Ray, float &);

#endif

