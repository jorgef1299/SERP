/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
#define __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__

#include <vector>
#include <functional>
#include <set>
#include "opencv2/opencv.hpp"


namespace AStar
{
    struct Vec2i
    {
        int x, y;

        bool operator == (const Vec2i& coordinates_);
    };

    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;
    using CoordinateList = std::vector<Vec2i>;

    struct Node
    {
        uint G, H;
        Vec2i coordinates;
        Node *parent;

        Node(Vec2i coord_, Node *parent_ = nullptr);
        uint getScore();
    };

    using NodeSet = std::vector<Node*>;

    class Generator
    {
        bool detectCollision(Vec2i coordinates_);
        Node* findNodeOnList(NodeSet& nodes_, Vec2i coordinates_);
        void releaseNodes(NodeSet& nodes_);

    public:
        Generator();
        void setWorldSize(Vec2i worldSize_);
        void setDiagonalMovement(bool enable_);
        void setHeuristic(HeuristicFunction heuristic_);
        CoordinateList findPath(Vec2i source_, Vec2i target_);
        void addCollision(Vec2i coordinates_);
        void removeCollision(Vec2i coordinates_);
        void clearCollisions();

    private:
        HeuristicFunction heuristic;
        CoordinateList direction, walls;
        Vec2i worldSize;
        uint directions;
    };

    class Heuristic
    {
        static Vec2i getDelta(Vec2i source_, Vec2i target_);

    public:
        static uint manhattan(Vec2i source_, Vec2i target_);
        static uint euclidean(Vec2i source_, Vec2i target_);
        static uint octagonal(Vec2i source_, Vec2i target_);
    };
}



#define N_TILES_X 80
#define N_TILES_Y 50
#define THRESHOLD_WHITE_PIXELS 0.01
#define POINTS_PER_LINE 10
#define KERNEL_SIZE 9

struct line_connection {
    float cost;
    AStar::CoordinateList line_points;
    AStar::Vec2i initial_point;
    AStar::Vec2i final_point;
};


bool check_line_in_tile(cv::Mat& tile);

float calculate_slope(const AStar::Vec2i& pt_i, const AStar::Vec2i& pt_f);

float calculate_orientation_error(float& angle_1, float& angle_2);

AStar::CoordinateList extract_line_points(AStar::CoordinateList& line_points);

float calculate_line_cost(AStar::CoordinateList extracted_points);

cv::Mat create_occupancy_grid_map(cv::Mat& cropped_image, AStar::Generator& generator, uint8_t n_tiles_x, uint8_t n_tiles_y);

bool compare_costs(const line_connection& a, const line_connection& b);

int convert_coordinates_to_occupancy_map(const AStar::CoordinateList& original_input_pos, const AStar::CoordinateList& original_output_pos, AStar::CoordinateList& occupancy_initial_points, AStar::CoordinateList& occupancy_final_points, cv::Mat& cropped_image, cv::Mat& occupancy_map);

std::vector<line_connection> find_line_connections(AStar::Generator& generator, cv::Mat& occupancy_grid_map, std::vector<AStar::Vec2i>& initial_points, std::vector<AStar::Vec2i>& final_points);
#endif // __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
