
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

Plane::Plane() {}

Plane::Plane(std::vector<Eigen::Vector3f> v, Eigen::Vector3f n, Material *mat) {
    vertices = v;
    normal = n.normalized();
    material = mat;
}

PNode::PNode(Plane p, PNode *l, PNode *r) {
    plane = p;
    left = l;
    right = r;
}

PNode::PNode(Plane p) {
    PNode(p, NULL, NULL);
}

Sphere::Sphere(Eigen::Vector3f c, float r, Material *mat) {
    center = c;
    radius = r;
    material = mat;
}

Light::Light(Eigen::Vector3f p, Eigen::Vector3f c) {
    position = p;
    color = c;
}

World::World(bool bt) {
    bspTrigger = bt;
}

void World::insertLight(Light l) {
    pointLights.push_back(l);
}

void World::insertLight(Eigen::Vector3f p, Eigen::Vector3f c) {
    pointLights.push_back(Light(p, c));
}

void World::insert(Plane p) {
    if (bspTrigger) {
        insertBSP(p, bsproot);
    }
    else {
        planeList.push_back(p);
    }
}

void World::insert(Sphere s) {
    sphereList.push_back(s);
}

// Binary Space Partitioning
void World::insertBSP(Plane p, PNode *node) {
    // TODO
}

void World::split(Eigen::Vector3f norm, float d, std::vector<Eigen::Vector3f> &left, std::vector<Eigen::Vector3f> &right) {
    // TODO
}

// Ray Cast
bool World::rayCast(Ray r, float threshold, Target &target) {
    float minDist = INF;

    if (bspTrigger) {
        //
    }
    else {
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

// Intersection
bool isIntersect(Plane p, Ray r, float &dist) {
    // Plane collide
    float d = p.normal.dot(p.vertices[0]);
    bool samedir = r.direction.dot(p.normal) > 0;
    bool small = r.start.dot(p.normal) < d;
    if (samedir ^ small) return false;
    // Inside polygin
    return false;
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


