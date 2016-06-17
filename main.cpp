#include <iostream>
#include <fstream>
#include <cmath>
#include <string>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

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
const unsigned int SIZE = WIDTH * HEIGHT;
const float CAM_HEIGHT = 2.0f;
const float VPWIDTH = 16.0;
const float VPHEIGHT = VPWIDTH * ASPECT;
const float VPMINX = -VPWIDTH / 2.0;
const float VPMINY = CAM_HEIGHT - VPHEIGHT / 2.0;
const float VPDX = VPWIDTH / (WIDTH - 1);
const float VPDY = VPHEIGHT / (HEIGHT - 1);

Vector3f camloc (0, CAM_HEIGHT, 10);

Vector3f pixelRayDirection(int i, int j) {
    return Vector3f(VPMINX + VPWIDTH / (WIDTH - 1) * i, VPMINY + VPHEIGHT / (HEIGHT - 1) * j, 0) - camloc;
}

// RAY TRACE DEPTH
const int RAY_TRACE_DEPTH = 6;
const float SHADOW_FACTOR = 0.3;

// COLORS
Vector3f default_color (0.4, 0.4, 0.4);
Vector3f BLACK (0.0, 0.0, 0.0);
Vector3f DARK_GRAY (0.7, 0.7, 0.7);
Vector3f LIGHT_GRAY (0.3, 0.3, 0.3);
Vector3f WHITE (1.0, 1.0, 1.0);
Vector3f RED (1.0, 0.0, 0.0);
Vector3f GREEN (0.0, 1.0, 0.0);
Vector3f BLUE (0.0, 0.0, 1.0);
Vector3f CYAN (0.0, 1.0, 1.0);
Vector3f MAGENTA (1.0, 0.0, 1.0);
Vector3f YELLOW (1.0, 1.0, 0.0);

// REFRACTION INDICES
float RI_AIR = 1.0;
float RI_WATER = 1.33;
float RI_GLASS = 1.5;

// MATERIALS
Material *mat1 = new Material(MAGENTA, MAGENTA, MAGENTA,  2, 0.0,  0.2, RI_AIR, RI_GLASS);
Material *mat2 = new Material(CYAN, CYAN, CYAN,  2, 0.2,  1.0, RI_AIR, RI_GLASS);
Material *mat3 = new Material(YELLOW, YELLOW, YELLOW,  2, 0.0,  0.8, RI_AIR, RI_GLASS);
Material *mat4 = new Material(BLUE, BLUE, BLUE,  2, 0.1,  1.0, RI_AIR, RI_GLASS);

void insertItems(World &world) {
    world.insert(Sphere(Vector3f(0, 5, 0), 3, mat1));
    world.insert(Sphere(Vector3f(-4, 6, -10), 3, mat2));
    world.insert(Sphere(Vector3f(-7, 5, -40), 15, mat3));
    world.insert(Sphere(Vector3f(-10, 4, -5), 4, mat4));
}

vector< vector<Vector3f> > data;
float pixels[HEIGHT][WIDTH][3];

bool firstTime = true;
int height_count = 0;

///////////////////////////////////////////////
World wtest(false);

Vector3f rayTracer(World &, Ray, int);

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case 27:
            exit(0);
            break;
    }
}

void mouse(int button, int state, int x, int y) {
    switch (button) {
        case GLUT_LEFT_BUTTON:
            if (state == GLUT_DOWN) {
                y = HEIGHT - y;
                debug = true;
                Vector3f color = rayTracer(wtest, Ray(camloc, pixelRayDirection(x, y), RI_AIR), RAY_TRACE_DEPTH);
                for (int k = 0; k < 3; k++) {
                    pixels[y][x][k] = color(k);
                }
                debug = false;
                cout << "Position : " << x << ", " << y << " >>> Color : " << color.transpose() << endl;
            }
    }
    return;
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT);
    if (firstTime) {
        for (int i = 0; i < WIDTH; i++) {
            for (int j = 0; j < HEIGHT; j++) {
                for (int k = 0; k < 3; k++) {
                    pixels[j][i][k] = 0.0f;
                }
            }
        }
        firstTime = false;
    }

    if (height_count < HEIGHT) {
        for (int i = 0; i < WIDTH; i++) {
            Vector3f color = rayTracer(wtest, Ray(camloc, pixelRayDirection(i, height_count), RI_AIR), RAY_TRACE_DEPTH);
            for (int k = 0; k < 3; k++) {
                pixels[height_count][i][k] = color(k);
            }
        }
        height_count++;
    }
    
    glDrawPixels(WIDTH, HEIGHT, GL_RGB, GL_FLOAT, pixels);
    glutSwapBuffers();
}

unsigned timeStep = 20;
void Timer(int unused) {
    glutPostRedisplay();
    glutTimerFunc(timeStep, Timer, 0);
}

int main(int argc, char* argv[]) {
    World world (false);
    world.setLight(Vector3f(-4, 11, 3));
    
    insertItems(world);
    data.clear();

    //////////////////////////// for debug
    wtest = world;
    ///////////////////////////

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

    glutInitWindowSize(WIDTH, HEIGHT);
    glutInitWindowPosition(50, 50);
    glutCreateWindow("Ray Tracing");

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutTimerFunc(timeStep, Timer, 0);
    glutKeyboardFunc(keyboard);

    glutMainLoop();
    return 0;
}

Vector3f reflectDirection(Vector3f in, Vector3f normal) {
    Vector3f n = normal * (normal.dot(-in));
    return n * 2 + in;
}

Vector3f rayTracer(World &world, Ray ray, int depth) {
    if (depth == 0) return default_color;

    Target target;
    if (world.rayCast(ray, INF, target)) {
        Material *mat = target.material;


        Vector3f matcolor = mat -> color;
        float alpha = mat -> alpha;
        float reflectivity = mat -> reflectivity;

        // COLORS
        Vector3f object_color = Vector3f(0, 0, 0);
        Vector3f reflection = Vector3f(0, 0, 0);
        Vector3f refraction = Vector3f(0, 0, 0);
        

        // Object Color
        if (reflectivity < 1 && alpha > 0) {
            // Ambient
            object_color = matcolor * (mat -> famb);

            // Diffuse
            Vector3f lightvec = world.light - target.point;
            float difFactor = lightvec.normalized().dot(target.normal);
            if (difFactor < 0) difFactor *= alpha - 1;
            float shadow = 1;
            if (world.rayCast(Ray(target.point, world.light - target.point, ray.refidx), lightvec.norm())) shadow = SHADOW_FACTOR;
            object_color += matcolor * (mat -> fdif) * difFactor * shadow;

            // Specular
            Vector3f lightout = reflectDirection(-lightvec.normalized(), target.normal);
            float speFactor_sub = lightout.dot(-ray.direction);
            float speFactor = (speFactor_sub < 0)? 0 : pow(speFactor_sub, mat -> shininess);
            object_color += matcolor * (mat -> fspe) * speFactor * shadow;

            object_color *= (1 - reflectivity) * alpha;
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
        if (depth == RAY_TRACE_DEPTH) return Vector3f(0.1, 0.1, 0.1);
        return default_color;
    }
}

