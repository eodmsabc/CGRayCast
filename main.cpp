#include <iostream>
#include <fstream>
#include <cmath>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "material.h"
#include "rayCast.h"
#include "primitive.h"
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

Vector3f pixelRayDirection(int i, int j) {
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
Material *mat1 = new Material(MAGENTA * 0.1, MAGENTA * 0.5, MAGENTA * 0.4,  2, 0.0,  0.2, RI_AIR, RI_GLASS);
Material *mat2 = new Material(CYAN * 0.1, CYAN * 0.5, CYAN * 0.4,  2, 0.2,  1.0, RI_AIR, RI_GLASS);
Material *mat3 = new Material(YELLOW * 0.1, YELLOW * 0.5, YELLOW * 0.4,  2, 0.0,  1, RI_AIR, RI_GLASS);
Material *mat4 = new Material(BLUE * 0.1, BLUE* 0.5, BLUE * 0.4,  2, 0.1,  1.0, RI_AIR, RI_GLASS);
Material *ROOM = new Material(0.1, 0.1, 0.1, 0.6, 0.6, 0.6, 0.3, 0.3, 0.3, 0.5, 0.0, 1.0, RI_AIR, RI_AIR);
Material *EMERALD = new Material(0.0215, 0.1745, 0.0215, 0.07568, 0.61424, 0.07568, 0.633, 0.727811, 0.633, 0.6, 0.3, 0.2, RI_AIR, RI_GLASS);
Material *RUBY = new Material(0.1745, 0.01175, 0.01175, 0.61424, 0.04136, 0.04136, 0.727811, 0.626959, 0.626959, 0.6, 0.2, 0.5, RI_AIR, 
RI_RUBY);
Material *GOLD = new Material(0.24725, 0.1995, 0.0745, 0.75164, 0.60648, 0.22648, 0.628281, 0.555802, 0.366065, 0.4, 0.0, 1.0, RI_AIR, RI_AIR);
Material *GOLD_IN_EMERALD = new Material(0.24725, 0.1995, 0.0745, 0.75164, 0.60648, 0.22648, 0.628281, 0.555802, 0.366065, 0.4, 0.0, 1.0, RI_EMERALD, RI_AIR);

void insertItems(World &world) {
    // ROOM
    insertCube(world, 20, -20, 20, -20, 20, -40, ROOM);

    // OBJECTS
    //world.insert(Sphere(Vector3f(2, 5, -4), 3, mat1));
    world.insert(Sphere(Vector3f(-1, 6, -6), 3, EMERALD));
    world.insert(Sphere(Vector3f(-5, 5, -29), 8, GOLD));
    //world.insert(Sphere(Vector3f(-10, 4, -5), 4, mat4));
    
    insertCube(world, 1, 9, -4, 4, -24, -16, EMERALD);
    //insertQuad(world, Vector3f(0, -3, -10), Vector3f(0, 0, -10), Vector3f(3, 0, -10), Vector3f(3, -3, -10), Vector3f(0, 0, 1), EMERALD);
    //insertCube(world, 4, 14, -5, 5, -19, -9, EMERALD);
    //world.insert(Sphere(Vector3f(9, 0, -14), 4, GOLD_IN_EMERALD));
}

int main(int argc, char* argv[]) {
    World world;
    world.insertLight(Vector3f(-4, 11, 3));
    insertItems(world);
    
    bitmap_image image(WIDTH, HEIGHT);

    for (int j = 0; j < HEIGHT; j++) {
        for (int i = 0; i < WIDTH; i++) {
            Vector3f color = rayTracer(world, Ray(camloc, pixelRayDirection(i, HEIGHT - j + 1), RI_AIR), RAY_TRACE_DEPTH);
            data_r[j * WIDTH + i] = color(0);
            data_g[j * WIDTH + i] = color(1);
            data_b[j * WIDTH + i] = color(2);
        }
        image.import_rgb(data_r, data_g, data_b);
        image.save_image("myimage.bmp");
    }

    return 0;
}

Vector3f reflectDirection(Vector3f in, Vector3f normal) {
    Vector3f n = normal * (normal.dot(-in));
    return n * 2 + in;
}

Vector3f localIllumination(Vector3f point, Vector3f normal, Vector3f raydir, Material *mat, World world) {
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

