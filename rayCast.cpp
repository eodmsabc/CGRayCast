
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

Triangle::Triangle(std::vector<Eigen::Vector3f> v, Eigen::Vector3f n, Material *mat) {
    vertices = v;
    for (int i = 0; i < 3; i++) vn.push_back(n);
    normal = n;
    material = mat;
}

Triangle::Triangle(std::vector<Eigen::Vector3f> v, std::vector<Eigen::Vector3f> vvn, Eigen::Vector3f n, Material *mat) {
    vertices = v;
    vn = vvn;
    normal = n;
    material = mat;
}

Triangle::Triangle(std::vector<Eigen::Vector3f> v, std::vector<Eigen::Vector3f> vvn, std::vector<Eigen::Vector2f> tx, Eigen::Vector3f n, Material *mat) {
    vertices = v;
    vn = vvn;
    vt = tx;
    normal = n;
    material = mat;
}

Eigen::Vector3f Triangle::barycentric(Eigen::Vector3f p) {
    Eigen::Vector3f u = vertices[1] - vertices[0];
    Eigen::Vector3f v = vertices[2] - vertices[0];
    Eigen::Vector3f w = p - vertices[0];
    float temp = u.dot(v);
    float uu = u.dot(u);
    float vv = v.dot(v);
    float s = (temp * w.dot(v) - vv * w.dot(u)) / (temp * temp - uu * vv);
    float t = (temp * w.dot(u) - uu * w.dot(v)) / (temp * temp - uu * vv);
    return Eigen::Vector3f(1 - s - t, s, t);
}

Sphere::Sphere(Eigen::Vector3f c, float r, Material *mat) {
    center = c;
    radius = r;
    material = mat;
    texture = NULL;
}

Sphere::Sphere(Eigen::Vector3f c, float r, Material *mat, bitmap_image *texture_image) {
    center = c;
    radius = r;
    material = mat;
    texture = texture_image;
}

Eigen::Vector2f Sphere::theta_phi_conversion(Eigen::Vector3f p) {
    Eigen::Vector3f v = p - center;
    float phi = acos(v(1) / radius);
    v(1) = 0;
    float theta = 0;
    return Eigen::Vector2f(theta, phi);
}

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
                Eigen::Vector3f b = planeList[i].barycentric(r.start + r.direction * dist);
                Eigen::Vector3f normal_base(0, 0, 0);
                for(int j = 0; j < 3; j++) {
                    normal_base += planeList[i].vn[j] * b(j);
                }
                target.normal = normal_base.normalized();
                target.material = planeList[i].material;
                if (planeList[i].material -> texture != NULL) {
                    Material *m = planeList[i].material;
                    Eigen::Vector2f tcoord(0, 0);
                    for (int j = 0; j < 3; j++) {
                        tcoord += planeList[i].vt[j] * b(j);
                    }
                    int w = tcoord(0), h = tcoord(1);
                    unsigned char red, green, blue;
                    (*(m -> texture)).get_pixel(w, h, red, green, blue);
                    m -> ambient = Eigen::Vector3f(red, green, blue) * 0.1 / 255;
                    m -> diffuse = Eigen::Vector3f(red, green, blue) * 0.6 / 255;
                    m -> specular = Eigen::Vector3f(0.3, 0.3, 0.3);
                    m -> shininess = 40;
                    m -> reflectivity = 0.3;
                    m -> alpha = 1;
                    target.material = m;
                }
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
                if (sphereList[i].material -> texture != NULL) {
                    Material *m = sphereList[i].material;
                    int w = m -> texture -> width();
                    int h = m -> texture -> height();
                }
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

    /*
    Eigen::Vector3f center(0, 0, 0);
    int size = p.vertices.size();
    for (int i = 0; i < size; i++) {
        center += p.vertices[i];
    }
    center /= size;
    for (int i = 0; i < size; i++) {
        if (!sameSideWithCenter(p.vertices[i], p.vertices[(i+1)%size], center, r.start + r.direction * dist)) return false;
    }
    */
    Eigen::Vector3f b = p.barycentric(r.start + r.direction * dist);
    for (int i = 0; i < 3; i++) {
        if (b(i) < 0) return false;
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


