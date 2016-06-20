
#include "rayCast.h"

Ray::Ray(Eigen::Vector3f s, Eigen::Vector3f d) {
    start = s;
    direction = d.normalized();
    refidx = 1.0;
}

Ray::Ray(Eigen::Vector3f s, Eigen::Vector3f d, float ri) {
    start = s;
    direction = d.normalized();
    refidx = ri;
}

Triangle::Triangle() {}

Triangle::Triangle(std::vector<Eigen::Vector3f> v, Eigen::Vector3f n, Material *mat) {
    vertices = v;
    normal = n.normalized();
    material = mat;
}

Triangle::Triangle(std::vector<Eigen::Vector3f> v, std::vector<Eigen::Vector2f> t, Eigen::Vector3f n, Material *mat) {
    vertices = v;
    texture = t;
    normal = n.normalized();
    material = mat;
}

Eigen::Vector3f Triangle::barycentric(Eigen::Vector3f p) {
    return Eigen::Vector3f(0, 0, 0);
}

Sphere::Sphere(Eigen::Vector3f c, float r, Material *mat) {
    center = c;
    radius = r;
    material = mat;
}
/*
Light::Light(Eigen::Vector3f p, Eigen::Vector3f c) {
    position = p;
    color = c;
}
*/
World::World() {}

void World::insertLight(float x, float y, float z) {
    pointLights.push_back(Eigen::Vector3f(x, y, z));
}

void World::insertLight(Eigen::Vector3f l) {
    pointLights.push_back(l);
}

void World::insert(Triangle p) {
        planeList.push_back(p);
}

void World::insert(Sphere s) {
    sphereList.push_back(s);
}

// Ray Cast
bool World::rayCast(Ray r, float threshold, Target &target) {
    float minDist = INF;

    int planeNum = planeList.size();
    for (int i = 0; i < planeNum; i++) {
        float dist;
        if (isIntersect(planeList[i], r, dist)) {
            if (dist < minDist && dist < threshold) {
                target.point = r.start + r.direction * dist;
                target.normal = planeList[i].normal;
                target.material = planeList[i].material;
                minDist = dist;
            }
        }
    }
    
    int sphereNum = sphereList.size();
    for (int i = 0; i < sphereNum; i++) {
        float dist;
        if (isIntersect(sphereList[i], r, dist)) {
            if (dist < minDist && dist < threshold) {
                target.point = r.start + r.direction * dist;
                target.normal = (target.point - sphereList[i].center).normalized();
                target.material = sphereList[i].material;
                minDist = dist;
            }
        }
    }

    if (minDist > INF - 10)
        return false;

    return true;
}

bool World::rayCast(Ray r, float threshold) {
    Target dummy;
    return rayCast(r, threshold, dummy);
}

bool sameSideWithCenter(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c, Eigen::Vector3f target) {
    Eigen::Vector3f line = a - b;
    Eigen::Vector3f cc = c - b;
    float t = line.dot(cc) / line.dot(line);
    Eigen::Vector3f h = cc - line * t;
    return h.dot(target - b) > 0;
}

// Intersection
bool isIntersect(Triangle p, Ray r, float &dist) {
    dist = p.normal.dot(p.vertices[0] - r.start) / p.normal.dot(r.direction);
    if (dist < 0.0001) return false;

    Eigen::Vector3f center(0, 0, 0);
    int size = p.vertices.size();
    for (int i = 0; i < size; i++) {
        center += p.vertices[i];
    }
    center /= size;
    for (int i = 0; i < size; i++) {
        if (!sameSideWithCenter(p.vertices[i], p.vertices[(i+1)%size], center, r.start + r.direction * dist)) return false;
    }
    return true;
}

bool isIntersect(Sphere s, Ray r, float &dist) {
    Eigen::Vector3f spc = s.center - r.start;

    float raydist = r.direction.dot(spc);
    Eigen::Vector3f vr = r.direction * raydist;
    float sub = (vr - spc).norm();
    float distdiffsq = s.radius * s.radius - sub * sub;
    if (distdiffsq < 0) distdiffsq = 0;
    float distdiff = sqrt(distdiffsq);

    if (raydist <= 0) {
        // far surface
        if (spc.norm() >= s.radius - 0.0001) return false;
        dist = raydist + distdiff;
        return true;
    }

    // near surface
    if (spc.norm() < s.radius + 0.0001) {
        dist = raydist + distdiff;
        return true;
    }

    if (sub >= s.radius) {
        return false;
    }

    dist = raydist - distdiff;
    return true;
}


