#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "material.h"
#include "rayCast.h"
#include "bitmap_image.hpp"

using namespace std;
using namespace Eigen;

bool debug = false;

// PIXEL
const unsigned int WIDTH = 640;
const float ASPECT = 9.0 / 16.0;
const unsigned int HEIGHT = WIDTH * ASPECT;
const unsigned int PIXEL_COUNT = WIDTH * HEIGHT;
const float CAM_HEIGHT = 2.0f;
const float VPWIDTH = 16.0;
const float VPHEIGHT = VPWIDTH * ASPECT;
const float VPMINX = -VPWIDTH / 2.0;
const float VPMINY = CAM_HEIGHT - VPHEIGHT / 2.0;
const float VPDX = VPWIDTH / (WIDTH - 1);
const float VPDY = VPHEIGHT / (HEIGHT - 1);

Vector3f camloc (0, CAM_HEIGHT, 10);

// DATA
float data_r[PIXEL_COUNT];
float data_g[PIXEL_COUNT];
float data_b[PIXEL_COUNT];

const int RAY_TRACE_DEPTH = 6;
const float SHADOW_FACTOR = 0.3;
bool supersampling = false;

inline Vector3f pixelRayDirection(float i, float j) {
    return Vector3f(VPMINX + VPWIDTH / (WIDTH - 1) * i, VPMINY + VPHEIGHT / (HEIGHT - 1) * j, 0) - camloc;
}

Vector3f rayTracer(World &, Ray, int);

// COLORS
Vector3f BLACK (0.0, 0.0, 0.0);
Vector3f DARK_GRAY (0.7, 0.7, 0.7);
Vector3f LIGHT_GRAY (0.2, 0.2, 0.2);
Vector3f WHITE (1.0, 1.0, 1.0);
Vector3f RED (1.0, 0.0, 0.0);
Vector3f GREEN (0.0, 1.0, 0.0);
Vector3f BLUE (0.0, 0.0, 1.0);
Vector3f CYAN (0.0, 1.0, 1.0);
Vector3f MAGENTA (1.0, 0.0, 1.0);
Vector3f YELLOW (1.0, 1.0, 0.0);

Vector3f default_color = LIGHT_GRAY;

// REFRACTION INDICES
float RI_AIR = 1.0;
float RI_WATER = 1.33;
float RI_GLASS = 1.5;
float RI_EMERALD = 1.58;
float RI_RUBY = 1.76;

// MATERIALS
Material *ROOM = new Material(0.1, 0.1, 0.1, 0.6, 0.6, 0.6, 0.3, 0.3, 0.3, 0.5, 0.7, 1.0, RI_AIR, RI_AIR);
Material *EMERALD = new Material(0.0215, 0.1745, 0.0215, 0.07568, 0.61424, 0.07568, 0.633, 0.727811, 0.633, 0.6, 0.3, 0.2, RI_AIR, RI_GLASS);
Material *RUBY = new Material(0.1745, 0.01175, 0.01175, 0.6142, 0.0414, 0.0414, 0.7278, 0.627, 0.627, 0.6, 0.2, 0.5, RI_AIR, RI_RUBY);
Material *GOLD = new Material(0.24725, 0.1995, 0.0745, 0.75164, 0.60648, 0.22648, 0.62828, 0.5558, 0.366065, 0.4, 0.0, 1.0, RI_AIR, RI_AIR);
Material *GOLD_IN_EMERALD = new Material(0.24725, 0.2, 0.0745, 0.75, 0.6, 0.22648, 0.62828, 0.556, 0.366, 0.4, 0.0, 1.0, RI_EMERALD, RI_AIR);
Material *GLASS = new Material(0, 0, 0, 0, 0, 0, 0.1, 0.1, 0.1, 0.4, 0.0, 0.0, RI_AIR, 0.9);
Material *CHESSBOARD = new Material(0.1, 0.6, 0.3, 0.6, 0.2, 1.0, 1.0, 1.0, new bitmap_image("textures/chessboard.bmp"));
Material *WORLDMAP = new Material(0.2, 0.7, 0.1, 0.6, 0.1, 1.0, 1.0, 1.0, new bitmap_image("textures/worldmap.bmp"));
Material *TEXT = new Material(0.1, 0.6, 0.3, 0.6, 0.2, 1.0, 1.0, 1.0, new bitmap_image("textures/text.bmp"));

// TEXTURE IMAGES
//bitmap_image *chessboard = new bitmap_image("textures/chessboard.bmp");

inline void insertTriangle(World &world, vector<Vector3f>, Vector3f, Material *mat);
inline void insertTriangle(World &world, vector<Vector3f>, vector<Vector3f>, Vector3f, Material *mat);
inline void insertTriangle(World &world, vector<Vector3f>, vector<Vector3f>, vector<Vector2f>, Vector3f, Material *mat);
inline void insertQuad(World &world, Vector3f v0, Vector3f v1, Vector3f v2, Vector3f v3, Vector3f normal, Material *mat);
inline void insertCube(World &world, float x1, float x2, float y1, float y2, float z1, float z2, Material *mat);

void insertItems(World &world) {
    // ROOM
    insertCube(world, 25, -25, 25, -25, 10, -40, ROOM);

    // OBJECTS
//    world.insert(Sphere(Vector3f(1, 6, -7), 3, EMERALD));
//    world.insert(Sphere(Vector3f(-5, 5, -29), 7, GOLD));
    world.insert(Sphere(Vector3f(0, 2, -15), 6, WORLDMAP));
//    world.insert(Sphere(Vector3f(2, 2, 0), 4, GLASS));

    
    insertCube(world, 2, 8, -3, 3, -8, -5, EMERALD);
    //insertQuad(world, Vector3f(0, -3, -10), Vector3f(0, 0, -10), Vector3f(3, 0, -10), Vector3f(3, -3, -10), Vector3f(0, 0, 1), EMERALD);
    //insertCube(world, 4, 14, -5, 5, -19, -9, EMERALD);
    //world.insert(Sphere(Vector3f(9, 0, -14), 4, GOLD_IN_EMERALD));
    insertQuad(world, Vector3f(-25, -6, -40), Vector3f(25, -6, -40), Vector3f(25, -6, 10), Vector3f(-25, -6, 10), Vector3f(0, 1, 0), CHESSBOARD);
}

int main(int argc, char* argv[]) {
    World world;
    world.insertLight(Vector3f(-4, 15, 3));
    world.insertLight(Vector3f(8, 0, 5));

    insertItems(world);

    // OUTPUT
    bitmap_image image(WIDTH, HEIGHT);
    for (int j = 0; j < HEIGHT; j++) {
        for (int i = 0; i < WIDTH; i++) {
            Vector3f color(0, 0, 0);
            if (!supersampling) {
                color = rayTracer(world, Ray(camloc, pixelRayDirection(i, HEIGHT - j + 1), RI_AIR), RAY_TRACE_DEPTH);
            }
            else {
                color += rayTracer(world, Ray(camloc, pixelRayDirection(i, HEIGHT - j + 1), RI_AIR), RAY_TRACE_DEPTH) * 0.25;
                color += rayTracer(world, Ray(camloc, pixelRayDirection(i - 0.33, HEIGHT - j + 1), RI_AIR), RAY_TRACE_DEPTH) * 0.125;
                color += rayTracer(world, Ray(camloc, pixelRayDirection(i + 0.33, HEIGHT - j + 1), RI_AIR), RAY_TRACE_DEPTH) * 0.125;
                color += rayTracer(world, Ray(camloc, pixelRayDirection(i, HEIGHT - j + 1.33), RI_AIR), RAY_TRACE_DEPTH) * 0.125;
                color += rayTracer(world, Ray(camloc, pixelRayDirection(i, HEIGHT - j + 0.67), RI_AIR), RAY_TRACE_DEPTH) * 0.125;
                color += rayTracer(world, Ray(camloc, pixelRayDirection(i - 0.33, HEIGHT - j + 0.67), RI_AIR), RAY_TRACE_DEPTH) * 0.0625;
                color += rayTracer(world, Ray(camloc, pixelRayDirection(i - 0.33, HEIGHT - j + 1.33), RI_AIR), RAY_TRACE_DEPTH) * 0.0625;
                color += rayTracer(world, Ray(camloc, pixelRayDirection(i + 0.33, HEIGHT - j + 0.67), RI_AIR), RAY_TRACE_DEPTH) * 0.0625;
                color += rayTracer(world, Ray(camloc, pixelRayDirection(i + 0.33, HEIGHT - j + 1.33), RI_AIR), RAY_TRACE_DEPTH) * 0.0625;
            }
            data_r[j * WIDTH + i] = color(0) > 0.9999? 0.99999 : color(0);
            data_g[j * WIDTH + i] = color(1) > 0.9999? 0.99999 : color(1);
            data_b[j * WIDTH + i] = color(2) > 0.9999? 0.99999 : color(2);
        }
        image.import_rgb(data_r, data_g, data_b);
        image.save_image("myimage.bmp");
    }

    return 0;
}

inline Vector3f reflectDirection(Vector3f in, Vector3f normal) {
    Vector3f n = normal * (normal.dot(-in));
    return n * 2 + in;
}

inline Vector3f localIllumination(Vector3f point, Vector3f normal, Vector3f raydir, Material *mat, World world) {
    int size = world.pointLights.size();
    Vector3f color = Vector3f(0, 0, 0);
    for (int i = 0; i < size; i++) {
        Vector3f lightpos = world.pointLights[i];

        // ambient
        color += mat -> ambient;
        Vector3f lightvec = lightpos - point;

        // diffuse
        float difFactor = lightvec.normalized().dot(normal);
        if (difFactor < 0) difFactor *= mat -> alpha - 1;
        float shadow = 1;
        if (world.rayCast(Ray(point, lightvec), lightvec.norm())) shadow = SHADOW_FACTOR;
        color += (mat -> diffuse) * difFactor * shadow;

        // Specular
        Vector3f lightout = reflectDirection(-lightvec.normalized(), normal);
        float speFactor_sub = lightout.dot(-raydir);
        float speFactor = (speFactor_sub < 0)? 0 : pow(speFactor_sub, mat -> shininess);
        color += (mat -> specular) * speFactor * shadow;
    }
    return color;
}

Vector3f rayTracer(World &world, Ray ray, int depth) {
    if (depth == 0) return default_color;

    Target target;
    if (world.rayCast(ray, INF, target)) {
        Material *mat = target.material;

        float alpha = mat -> alpha;
        float reflectivity = mat -> reflectivity;

        // COLORS
        Vector3f object_color = Vector3f(0, 0, 0);
        Vector3f reflection = Vector3f(0, 0, 0);
        Vector3f refraction = Vector3f(0, 0, 0);
        

        // Object Color
        if (reflectivity < 1 && alpha > 0) {
            object_color = localIllumination(target.point, target.normal, ray.direction, mat, world) * (1 - reflectivity) * alpha;
        }

        // Reflection
        Vector3f reflDir = reflectDirection(ray.direction, target.normal);
        reflection = rayTracer(world, Ray(target.point, reflDir, ray.refidx), depth - 1);
        
        // Refraction
        if (alpha < 1) {
            float nn = target.normal.dot(ray.direction);
            float ridx = (nn > 0)? mat -> refidx_p : mat -> refidx_n;
            float inc_angle = acos(abs(nn));

            // Total Reflection Check
            float sin_ref_ang = sin(inc_angle) * ray.refidx / ridx;
            if (sin_ref_ang >= 1) {
                return object_color + reflection * (1 - alpha * (1 - reflectivity));
            }
            reflection *= reflectivity * alpha;

            float rev = (nn > 0)? -1.0f : 1.0f;
            float angle = asin(sin_ref_ang) - inc_angle;
            Vector3f refrDir = AngleAxisf(rev * angle, ray.direction.cross(target.normal).normalized()) * ray.direction;

            if (depth >= 1 && debug) {
                cout << "refidx : " << ridx << endl;
                cout << "nowdir : " << ray.direction.transpose() << endl;
                cout << "target : " << target.point.transpose() << endl;
                cout << "normal : " << target.normal.transpose() << endl;
                cout << "refdir : " << refrDir.normalized().transpose() << endl;
            }
            
            if (debug) cout << "depth : " << depth << ", refract to : " << refrDir.normalized().transpose() << endl;

            refraction = rayTracer(world, Ray(target.point, refrDir, ridx), depth - 1) * (1 - alpha);
        }
        return object_color + reflection * reflectivity * alpha + refraction;
    }
    else {
        return default_color;
    }
}

inline void insertTriangle(World &world, vector<Vector3f> v, Vector3f normal, Material *mat) {
    world.insert(Triangle(v, normal, mat));
}

inline void insertTriangle(World &world, vector<Vector3f> v, vector<Vector3f> vn, Vector3f normal, Material *mat) {
    world.insert(Triangle(v, vn, normal, mat));
}

inline void insertTriangle(World &world, vector<Vector3f> v, vector<Vector3f> vn, vector<Vector2f> vt, Vector3f normal, Material *mat) {
    world.insert(Triangle(v, vn, vt, normal, mat));
}

inline void insertQuad(World &world, Vector3f v0, Vector3f v1, Vector3f v2, Vector3f v3, Vector3f normal, Material *mat) {
    vector<Vector3f> vertices1;
    vertices1.push_back(v0);
    vertices1.push_back(v1);
    vertices1.push_back(v2);
    vector<Vector3f> vertices2;
    vertices2.push_back(v0);
    vertices2.push_back(v2);
    vertices2.push_back(v3);
    if (mat -> texture == NULL) {
        insertTriangle(world, vertices1, normal, mat);
        insertTriangle(world, vertices2, normal, mat);
    }
    else {
        float w = mat -> texture -> width();
        float h = mat -> texture -> height();
        Vector2f t0(0, h - 1);
        Vector2f t1(w - 1, h - 1);
        Vector2f t2(w - 1, 0);
        Vector2f t3(0, 0);
        vector<Eigen::Vector3f> vn;
        for (int i = 0; i < 3; i ++) vn.push_back(normal);
        vector<Eigen::Vector2f> vt1;
        vt1.push_back(t0);
        vt1.push_back(t1);
        vt1.push_back(t2);
        std::vector<Eigen::Vector2f> vt2;
        vt2.push_back(t0);
        vt2.push_back(t2);
        vt2.push_back(t3);
        insertTriangle(world, vertices1, vn, vt1, normal, mat);
        insertTriangle(world, vertices2, vn, vt2, normal, mat);
    }
}

inline void insertCube(World &world, float x1, float x2, float y1, float y2, float z1, float z2, Material *mat) {
    Vector3f v0(x1, y1, z1);
    Vector3f v1(x1, y1, z2);
    Vector3f v2(x1, y2, z1);
    Vector3f v3(x1, y2, z2);
    Vector3f v4(x2, y1, z1);
    Vector3f v5(x2, y1, z2);
    Vector3f v6(x2, y2, z1);
    Vector3f v7(x2, y2, z2);
    insertQuad(world, v0, v1, v3, v2, Vector3f(-1,  0,  0), mat);
    insertQuad(world, v4, v5, v7, v6, Vector3f( 1,  0,  0), mat);
    insertQuad(world, v0, v1, v5, v4, Vector3f( 0, -1,  0), mat);
    insertQuad(world, v2, v3, v7, v6, Vector3f( 0,  1,  0), mat);
    insertQuad(world, v0, v2, v6, v4, Vector3f( 0,  0, -1), mat);
    insertQuad(world, v1, v3, v7, v5, Vector3f( 0,  0,  1), mat);

}

