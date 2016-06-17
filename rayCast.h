
#ifndef _RAY_CAST_H_
#define _RAY_CAST_H_

#include <iostream>
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "material.h"

#define INF 100000

class Ray{
public:
    Eigen::Vector3f start;
    Eigen::Vector3f direction;
    float refidx;
    Ray(Eigen::Vector3f, Eigen::Vector3f, float);
};

struct Target {
    Eigen::Vector3f point;
    Eigen::Vector3f normal;
    Material *material;
};

class Plane {
public:
    std::vector<Eigen::Vector3f> vertices;
    Eigen::Vector3f normal;
    Material *material;
    Plane();
    Plane(std::vector<Eigen::Vector3f>, Eigen::Vector3f, Material*);
};

class PNode {
public:
    Plane plane;
    PNode *left;
    PNode *right;

    PNode(Plane);
    PNode(Plane, PNode*, PNode*);
};

class Sphere {
public:
    Eigen::Vector3f center;
    float radius;
    Material *material;
    Sphere(Eigen::Vector3f, float, Material*);
};

class World {
public:
    PNode *bsproot;
    std::vector<Plane> planeList;
    std::vector<Sphere> sphereList;
    bool bspTrigger;
    std::vector<Eigen::Vector3f> pointLights;

    World(bool);
    void insertLight(Eigen::Vector3f);
    void insert(Plane);
    void insert(Sphere);
    
    // Ray Cast
    bool rayCast(Ray, float, Target &);
    bool rayCast(Ray, float);

private:
    // Binary Space Partitioning
    void insertBSP(Plane, PNode*);
    void split(Eigen::Vector3f, float, std::vector<Eigen::Vector3f>&, std::vector<Eigen::Vector3f>&);
};

bool isIntersect(Plane, Ray, float &);
bool isIntersect(Sphere, Ray, float &);

#endif

