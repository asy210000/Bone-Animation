#include "Bone_Animation.h"
#include <glm/gtx/euler_angles.hpp>


Bone_Animation::Bone_Animation()
{
}


Bone_Animation::~Bone_Animation()
{
}

void Bone_Animation::init()
{
    root_position = { 2.0f,0.5f,2.0f };

    target = { 3.0f,8.0f,3.0f };

    target_colors = { 0.0f, 1.0f, 0.0f, 1.0f };

    scale_vector =
    {
        {1.0f,1.0f,1.0f},
        {0.5f,4.0f,0.5f},
        {0.5f,3.0f,0.5f},
        {0.5f,2.0f,0.5f}
    };

    rotation_degree_vector =
    {
        {0.0f,0.0f,0.0f},
        {0.0f,30.0f,0.0f},
        {0.0f,30.0f,0.0f},
        {0.0f,30.0f,0.0f}
    };

    colors =
    {
        {0.7f,0.0f,0.0f,1.0f},
        {0.7f,0.7f,0.0f,1.0f},
        {0.7f,0.0f,0.7f,1.0f},
        {0.0f,0.7f,0.7f,1.0f}
    };

    bone_mat = { glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f) };

    isMoving = false;
    tree_depth = (int)scale_vector.size();
    joint_num = tree_depth - 1;

    Rotation();
}

void Bone_Animation::update(float delta_time)
{

    pivot.clear();
    X_axis.clear();
    Y_axis.clear();
    Z_axis.clear();

    Rotation();

    if (isMoving)
    {
        JacobianTranspose();
    }
}


void Bone_Animation::Rotation()
{
    // Pre-calculate radians and euler angles
    std::vector<glm::mat4> rotateX(tree_depth), rotateY(tree_depth), rotateZ(tree_depth);
    for (int i = 0; i < tree_depth; ++i) {
        float X_angle = glm::radians(rotation_degree_vector[i][2]);
        float Y_angle = glm::radians(rotation_degree_vector[i][0]);
        float Z_angle = glm::radians(rotation_degree_vector[i][1]);

        rotateX[i] = glm::eulerAngleX(X_angle);
        rotateY[i] = glm::eulerAngleY(Y_angle);
        rotateZ[i] = glm::eulerAngleZ(Z_angle);
    }

    // Compute transformations
    glm::mat4 identity_matrix = glm::mat4(1.0f);
    for (int i = 0; i < tree_depth; ++i) {
        rotate_mat[i] = rotateX[i] * rotateZ[i] * rotateY[i];

        glm::vec3 translation_offset(0.0f, scale_vector[i][1] / 2.0f, 0.0f);
        translate_mat_origin[i] = glm::translate(identity_matrix, translation_offset);
        translate_mat_end[i] = glm::translate(identity_matrix, glm::vec3(0.0f, scale_vector[i][1], 0.0f));

        if (i == 0) {
            glm::vec3 root_offset(root_position[0], root_position[1] - translation_offset.y, root_position[2]);
            translate_mat_link[i] = glm::translate(identity_matrix, root_offset);
            world_mat[i] = translate_mat_link[i] * rotate_mat[i];
            bone_mat[i] = glm::translate(identity_matrix, root_position);
        }
        else {
            translate_mat_link[i] = glm::translate(identity_matrix, glm::vec3(0.0f, scale_vector[i - 1][1], 0.0f));
            world_mat[i] = world_mat[i - 1] * translate_mat_link[i] * rotate_mat[i];
            bone_mat[i] = world_mat[i] * translate_mat_origin[i];

            glm::mat4 prev_world_transform = world_mat[i - 1];
            pivot.push_back(glm::vec3(prev_world_transform * translate_mat_end[i - 1] * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)));
            X_axis.push_back(glm::normalize(glm::vec3(prev_world_transform * glm::vec4(1.0f, 0.0f, 0.0f, 0.0f))));
            Y_axis.push_back(glm::normalize(glm::vec3(prev_world_transform * rotateX[i] * rotateZ[i] * glm::vec4(0.0f, 1.0f, 0.0f, 0.0f))));
            Z_axis.push_back(glm::normalize(glm::vec3(prev_world_transform * rotateX[i] * glm::vec4(0.0f, 0.0f, 1.0f, 0.0f))));
        }
    }

    endEffector = world_mat[tree_depth - 1] * translate_mat_end[tree_depth - 1] * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    distance = glm::distance(target, endEffector);
}



void Bone_Animation::reset()
{
    isMoving = false;

    const glm::vec3 default_rotation(0.0f, 30.0f, 0.0f);

    rotation_degree_vector =
    {
        glm::vec3(0.0f, 0.0f, 0.0f), 
        default_rotation,
        default_rotation,
        default_rotation
    };

    bone_mat = std::vector<glm::mat4>(4, glm::mat4(1.0f));
}


void Bone_Animation::JacobianTranspose() {
    if (distance < threshold) {
        return;
    }

    // Reserve space to avoid reallocations
    std::vector<glm::mat3> Jacobian_trans(joint_num);
    std::vector<glm::vec3> alpha_numerator(joint_num);

    // Vector to accumulate the weighted Jacobian matrices for the denominator of alpha
    glm::vec3 alpha_denominator(0.0f);
    // Variable to accumulate the squared norms for the numerator of alpha
    float alpha_numerator_sum_squares = 0.0f;

    glm::vec3 target_minus_endEffector = target - endEffector;

    for (int i = 0; i < joint_num; i++) {
        glm::vec3 r = endEffector - pivot[i];
        Jacobian_trans[i] = glm::mat3(
            glm::cross(X_axis[i], r),
            glm::cross(Y_axis[i], r),
            glm::cross(Z_axis[i], r)
        );

        // Directly use the transpose of the Jacobian to compute the numerator
        glm::vec3 current_numerator = glm::transpose(Jacobian_trans[i]) * target_minus_endEffector;
        alpha_numerator[i] = current_numerator;
        alpha_numerator_sum_squares += glm::dot(current_numerator, current_numerator);

        // Accumulate the denominator
        alpha_denominator += Jacobian_trans[i] * current_numerator;
    }

    float alpha = alpha_numerator_sum_squares / glm::dot(alpha_denominator, alpha_denominator);

    for (int i = 0; i < joint_num; i++) {
        glm::vec3 deltaTheta = alpha * alpha_numerator[i];
        rotation_degree_vector[i + 1] += glm::vec3(deltaTheta.y, deltaTheta.z, deltaTheta.x);
    }
}
