
#include "primitive.h"

inline void insertTriangle(World &world, std::vector<Eigen::Vector3f> v, Eigen::Vector3f normal, Material *mat) {
    world.insert(Triangle(v, normal, mat));
}

inline void insertTriangle(World &world, std::vector<Eigen::Vector3f> v, std::vector<Eigen::Vector3f> vn, Eigen::Vector3f normal, Material *mat) {
    world.insert(Triangle(v, vn, normal, mat));
}

inline void insertTriangle(World &world, std::vector<Eigen::Vector3f> v, std::vector<Eigen::Vector3f> vn, std::vector<Eigen::Vector2f> vt, Eigen::Vector3f normal, Material *mat) {
    world.insert(Triangle(v, vn, vt, normal, mat));
}

void insertQuad(World &world, Eigen::Vector3f v0, Eigen::Vector3f v1, Eigen::Vector3f v2, Eigen::Vector3f v3, Eigen::Vector3f normal, Material *mat) {
    std::vector<Eigen::Vector3f> vertices1;
    vertices1.push_back(v0);
    vertices1.push_back(v1);
    vertices1.push_back(v2);
    std::vector<Eigen::Vector3f> vertices2;
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
        Eigen::Vector2f t0(0, h - 1);
        Eigen::Vector2f t1(w - 1, h - 1);
        Eigen::Vector2f t2(w - 1, 0);
        Eigen::Vector2f t3(0, 0);
        std::vector<Eigen::Vector3f> vn;
        for (int i = 0; i < 3; i ++) vn.push_back(normal);
        std::vector<Eigen::Vector2f> vt1;
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

void insertCube(World &world, float x1, float x2, float y1, float y2, float z1, float z2, Material *mat) {
    Eigen::Vector3f v0(x1, y1, z1);
    Eigen::Vector3f v1(x1, y1, z2);
    Eigen::Vector3f v2(x1, y2, z1);
    Eigen::Vector3f v3(x1, y2, z2);
    Eigen::Vector3f v4(x2, y1, z1);
    Eigen::Vector3f v5(x2, y1, z2);
    Eigen::Vector3f v6(x2, y2, z1);
    Eigen::Vector3f v7(x2, y2, z2);
    insertQuad(world, v0, v1, v3, v2, Eigen::Vector3f(-1,  0,  0), mat);
    insertQuad(world, v4, v5, v7, v6, Eigen::Vector3f( 1,  0,  0), mat);
    insertQuad(world, v0, v1, v5, v4, Eigen::Vector3f( 0, -1,  0), mat);
    insertQuad(world, v2, v3, v7, v6, Eigen::Vector3f( 0,  1,  0), mat);
    insertQuad(world, v0, v2, v6, v4, Eigen::Vector3f( 0,  0, -1), mat);
    insertQuad(world, v1, v3, v7, v5, Eigen::Vector3f( 0,  0,  1), mat);

}

