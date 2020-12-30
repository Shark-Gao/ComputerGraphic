#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    float radian = MY_PI / 180 * rotation_angle;
    Eigen::Matrix4f translate;
    translate << cos(radian), -sin(radian), 0, 0, 
                sin(radian), cos(radian), 0, 0, 
                0, 0, 1, 0, 
                0, 0, 0, 1;

    model = translate * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    // float halfFovRadian = MY_PI / 180 * eye_fov / 2;

    // Vector4f p1;
    // p1 << 0, tan(halfFovRadian) * zFar, -zFar, 1;

    // Eigen::Matrix4f Mp2o;
    // Mp2o << zNear, 0, 0, 0, 
    //             0, zNear, 0, 0, 
    //             0, 0, zNear + zFar, -zNear * zFar, 
    //             0, 0, 1, 0;
    // projection = Mp2o * projection;

    // p1 = Mp2o * p1;

    // float t = p1[1];
    // float b = -p1[1];
    // float l = -t * aspect_ratio;
    // float r = t * aspect_ratio;

    // // Eigen::Matrix4f MorthoTranslate;
    // // MorthoTranslate << 1, 0, 0, 0,
    // // 0, 1, 0, 0,
    // // 0, 0, 1, -(zNear + zFar) / 2,
    // // 0, 0, 0, 1;

    // Eigen::Matrix4f MorthoScale;
    // MorthoScale << 1 / (t * aspect_ratio), 0, 0, 0,
    // 0, 1/(t), 0, 0,
    // 0, 0, 2/ (zNear - zFar), 0,
    // 0, 0, 0, 1;

    // projection = MorthoScale * projection;

    float n = -zNear;
    float f = -zFar;
    float t = std::abs(n) * std::tan(.5f * MY_PI / 180 * eye_fov);
    float b = -t;
    float r = aspect_ratio * t;
    float l = -r;
    

    Eigen::Matrix4f perspective_to_orthogonal = Eigen::Matrix4f::Identity();
    perspective_to_orthogonal(0,0) = n;
    perspective_to_orthogonal(1,1) = n;
    perspective_to_orthogonal(2,2) = n + f;
    perspective_to_orthogonal(2,3) = -f * n;
    perspective_to_orthogonal(3,2) = 1;
    perspective_to_orthogonal(3,3) = 0;

    Eigen::Matrix4f scale_to_canonical = Eigen::Matrix4f::Identity();
    scale_to_canonical(0,0) = 2.f/(r-l);
    scale_to_canonical(1,1) = 2.f/(t-b);
    scale_to_canonical(2,2) = 2.f/(n-f);

    Eigen::Matrix4f orthogonal_projection = scale_to_canonical;

    projection = orthogonal_projection * perspective_to_orthogonal;

    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    angle = MY_PI / 180 * angle;
    Eigen::Matrix3f anlyRotateMatrix;
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();

    Eigen::Matrix3f N;
    N <<    0, -axis[2], axis[1],
            axis[2], 0, -axis[0],
            -axis[1], axis[0], 0;
    anlyRotateMatrix = cos(angle) * I + (1 - cos(angle)) * axis * axis.transpose() + sin(angle) * N;

    Eigen::Matrix4f anlyRotateMatrix2 = Eigen::Matrix4f::Identity();
    anlyRotateMatrix2 << anlyRotateMatrix.row(0), 0,
    anlyRotateMatrix.row(1), 0,
    anlyRotateMatrix.row(2), 0,
    0,0,0,1;
    return anlyRotateMatrix2;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

         //r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(Vector3f(0,1,0), angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(Vector3f(0,1, 0), angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
