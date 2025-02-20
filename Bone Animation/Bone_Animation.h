#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class Bone_Animation
{
public:
    Bone_Animation();
    ~Bone_Animation();

    void init();
    void update(float delta_time);
    void reset();

    bool isMoving;
    int tree_depth;

    // Here the head of each vector is the root bone
    std::vector<glm::vec3> scale_vector;
    std::vector<glm::vec3> rotation_degree_vector;
    std::vector<glm::vec4> colors;

    glm::vec3 root_position;
    std::vector<glm::mat4> bone_mat = std::vector<glm::mat4>(4, glm::mat4(1.0f));
    glm::vec3 target;
    glm::vec3 endEffector;
    glm::vec4 target_colors;

private:
    std::vector<glm::vec3> pivot;
    std::vector<glm::vec3> X_axis;
    std::vector<glm::vec3> Y_axis;
    std::vector<glm::vec3> Z_axis;

    void Rotation();
    void JacobianTranspose();

    std::vector<glm::mat4> rotateX;
    std::vector<glm::mat4> rotateY;
    std::vector<glm::mat4> rotateZ;
    std::vector<glm::mat4> rotate_mat = std::vector<glm::mat4>(4, glm::mat4(1.0f));
    std::vector<glm::mat4> translate_mat_origin = std::vector<glm::mat4>(4, glm::mat4(1.0f));
    std::vector<glm::mat4> translate_mat_link = std::vector<glm::mat4>(4, glm::mat4(1.0f));
    std::vector<glm::mat4> translate_mat_end = std::vector<glm::mat4>(4, glm::mat4(1.0f));
    std::vector<glm::mat4> world_mat = std::vector<glm::mat4>(4, glm::mat4(1.0f));

    int joint_num;
    const float threshold = 1e-6;
    float distance;
    float alpha;
};
