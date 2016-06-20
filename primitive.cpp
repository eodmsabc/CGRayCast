
#include "primitive.h"

void insertTriangle(World &world, Eigen::Vector3f v0, Eigen::Vector3f v1, Eigen::Vector3f v2, Eigen::Vector3f normal, Material *mat) {
    std::vector<Eigen::Vector3f> vertices;
    vertices.push_back(v0);
    vertices.push_back(v1);
    vertices.push_back(v2);
    world.insert(Triangle(vertices, normal, mat));
}

void insertTriangle(World &world, Eigen::Vector3f v0, Eigen::Vector3f v1, Eigen::Vector3f v2, Eigen::Vector3f normal, Material *mat, bitmap_image *t, Eigen::Vector2f t0, Eigen::Vector2f t1, Eigen::Vector2f t2) {
    std::vector<Eigen::Vector3f> vertices;
    std::vector<Eigen::Vector2f> texture;
    vertices.push_back(v0);
    vertices.push_back(v1);
    vertices.push_back(v2);
    texture.push_back(t0);
    texture.push_back(t1);
    texture.push_back(t2);
    world.insert(Triangle(vertices, texture, normal, mat, t));
}

void insertQuad(World &world, Eigen::Vector3f v0, Eigen::Vector3f v1, Eigen::Vector3f v2, Eigen::Vector3f v3, Eigen::Vector3f normal, Material *mat) {
    insertTriangle(world, v0, v1, v2, normal, mat);
    insertTriangle(world, v0, v2, v3, normal, mat);
}

void insertQuad(World &world, Eigen::Vector3f v0, Eigen::Vector3f v1, Eigen::Vector3f v2, Eigen::Vector3f v3, Eigen::Vector3f normal, Material *mat, bitmap_image *t) {
    float w = t -> width();
    float h = t -> height();
    Eigen::Vector2f t0(0, 0);
    Eigen::Vector2f t1(w - 1, 0);
    Eigen::Vector2f t2(w - 1, h - 1);
    Eigen::Vector2f t3(0, h - 1);
    insertTriangle(world, v0, v1, v2, normal, mat, t, t0, t1, t2);
    insertTriangle(world, v0, v2, v3, normal, mat, t, t0, t2, t3);
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

