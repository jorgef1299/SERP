#include "AStar.hpp"
#include <algorithm>
#include <math.h>

using namespace std::placeholders;

bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

AStar::Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

AStar::Node::Node(Vec2i coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

AStar::uint AStar::Node::getScore()
{
    return G + H;
}

AStar::Generator::Generator()
{
    setDiagonalMovement(false);
    setHeuristic(&Heuristic::manhattan);
    direction = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
}

void AStar::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
}

void AStar::Generator::setDiagonalMovement(bool enable_)
{
    directions = (enable_ ? 8 : 4);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

void AStar::Generator::addCollision(Vec2i coordinates_)
{
    walls.push_back(coordinates_);
}

void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void AStar::Generator::clearCollisions()
{
    walls.clear();
}

AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_)
{
    Node *current = nullptr;
    NodeSet openSet, closedSet;
    openSet.reserve(100);
    closedSet.reserve(100);
    openSet.push_back(new Node(source_));

    while (!openSet.empty()) {
        auto current_it = openSet.begin();
        current = *current_it;

        for (auto it = openSet.begin(); it != openSet.end(); it++) {
            auto node = *it;
            if (node->getScore() <= current->getScore()) {
                current = node;
                current_it = it;
            }
        }

        if (current->coordinates == target_) {
            break;
        }

        closedSet.push_back(current);
        openSet.erase(current_it);

        for (uint i = 0; i < directions; ++i) {
            Vec2i newCoordinates(current->coordinates + direction[i]);
            if (detectCollision(newCoordinates) ||
                findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }

            uint totalCost = current->G + ((i < 4) ? 10 : 14);

            Node *successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) {
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                openSet.push_back(successor);
            }
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    CoordinateList path;
    while (current != nullptr) {
        path.push_back(current->coordinates);
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

    return path;
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(Vec2i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }
    return false;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}


bool check_line_in_tile(cv::Mat& tile)
{
    uint16_t total_pixels = tile.rows * tile.cols;
    uint16_t sum = 0;
    for(int i=0; i < tile.rows; i++) {
        const uint8_t* row_i = tile.ptr<uint8_t>(i);
        for(int j=0; j < tile.cols; j++) {
            if(row_i[j] == 255) {
                sum++;
                if((float)sum/total_pixels > THRESHOLD_WHITE_PIXELS) {
                    return true;
                }
            }
        }
    }
    return false;
}

float calculate_slope(const AStar::Vec2i& pt_i, const AStar::Vec2i& pt_f)
{
    float angle = atan2(pt_f.y-pt_i.y, pt_f.x-pt_i.x);
    return angle;
}

float calculate_orientation_error(float& angle_1, float& angle_2)
{
    return std::abs(angle_2 - angle_1);
}

AStar::CoordinateList extract_line_points(AStar::CoordinateList& line_points) {
    unsigned int num_line_points = line_points.size();
    AStar::CoordinateList extracted_points;
    extracted_points.push_back(line_points[num_line_points-1]);
    unsigned int ref = num_line_points - (num_line_points/POINTS_PER_LINE);
    unsigned int ref_dec = num_line_points/POINTS_PER_LINE;
    for(uint8_t i=1; i < POINTS_PER_LINE-1; i++) {
        extracted_points.push_back(line_points[ref]);
        ref -= ref_dec;
    }
    extracted_points.push_back(line_points[0]);
    return extracted_points;
}

float calculate_line_cost(AStar::CoordinateList extracted_points) {
    // Calculate total consecutive orientation errors
    float angle_1, angle_2;
    float cost=0;

    for(int i=0; i < extracted_points.size()-4; i=i+1) {
        angle_1 = calculate_slope(extracted_points[i], extracted_points[i+2]);
        angle_2 = calculate_slope(extracted_points[i+2], extracted_points[i+4]);
        cost += calculate_orientation_error(angle_1, angle_2);
    }
        angle_1 = calculate_slope(extracted_points[0], extracted_points[3]);
        angle_2 = calculate_slope(extracted_points[extracted_points.size()-4], extracted_points[extracted_points.size()-1]);
        cost += 0.8*calculate_orientation_error(angle_1, angle_2);
    return cost;
}

cv::Mat create_occupancy_grid_map(cv::Mat& cropped_image, AStar::Generator& generator, uint8_t n_tiles_x, uint8_t n_tiles_y) {
    cv::Mat occupancy_grid_map;
    uint16_t occupancy_map_row=0, occupancy_map_col=0;
    uint16_t n_pixels_x, n_pixels_y;
    bool exists_line;

    // Initialize occupancy grid map
    occupancy_grid_map = cv::Mat::zeros(N_TILES_Y, N_TILES_X, CV_8UC1);
    generator.clearCollisions();

    // Fill map
    for(int i=0; i < cropped_image.rows;) {
        uint8_t* occupancy_map_row_i = occupancy_grid_map.ptr<uint8_t>(occupancy_map_row);
        occupancy_map_col = 0;
        for(int j=0; j < cropped_image.cols;) {
            n_pixels_x = (uint16_t)(((float)cropped_image.cols / N_TILES_X)*(occupancy_map_col+1)) - j;
            n_pixels_y = (uint16_t)(((float)cropped_image.rows / N_TILES_Y)*(occupancy_map_row+1)) - i;
            if(occupancy_map_col == N_TILES_X-1) n_pixels_x = cropped_image.cols-j;
            if(occupancy_map_row == N_TILES_Y-1) n_pixels_y = cropped_image.rows-i;
            cv::Mat tile = cropped_image(cv::Rect(j,i, n_pixels_x, n_pixels_y));
            exists_line = check_line_in_tile(tile);
            if(!exists_line) {
                generator.addCollision({occupancy_map_col, occupancy_map_row});
                occupancy_map_row_i[occupancy_map_col] = 0;
            }
            else {
                occupancy_map_row_i[occupancy_map_col] = 255;
            }
            occupancy_map_col++;
            j=j+n_pixels_x;
        }
        occupancy_map_row++;
        i=i+n_pixels_y;
    }
    return occupancy_grid_map;
}

bool compare_costs(const Line_connection& a, const Line_connection& b)
{
    return a.cost < b.cost;
}

int convert_coordinates_to_occupancy_map(const std::vector<Point>& original_input_pos, const std::vector<Point>& original_output_pos, const cv::Mat& cropped_image, const cv::Mat& occupancy_map, std::vector<Point>& occupancy_initial_points, std::vector<Point>& occupancy_final_points)
{
    float pixels_per_tile_x = (float)cropped_image.cols / std::min(N_TILES_X, cropped_image.cols);
    float pixels_per_tile_y = (float)cropped_image.rows / std::min(N_TILES_Y, cropped_image.rows);

    AStar::Vec2i estimated_pos;
    bool stop;
    Point point;

    // Input points
    for(uint8_t i=0; i < original_input_pos.size(); i++) {
        estimated_pos.x = original_input_pos[i].point_coordinates.x / pixels_per_tile_x;
        estimated_pos.y = original_input_pos[i].point_coordinates.y / pixels_per_tile_y;

        stop = false;
        uint16_t c_initial = std::max(estimated_pos.x, 0);
        if(c_initial + KERNEL_SIZE >= occupancy_map.cols) {
            c_initial = occupancy_map.cols-KERNEL_SIZE-1;
        }
        uint16_t r_initial = std::max(estimated_pos.y, 0);
        if(r_initial + KERNEL_SIZE >= occupancy_map.rows) {
            r_initial = occupancy_map.rows-KERNEL_SIZE-1;
        }
        for(uint16_t c=c_initial; c < c_initial + KERNEL_SIZE; c++) {
            if(stop) break;
            for(uint16_t r=r_initial; r < r_initial + KERNEL_SIZE; r++) {
                if(occupancy_map.at<uint8_t>(r, c) == 255) {
                    stop = true;
                    point = original_input_pos[i];
                    point.point_coordinates = {c,r};
                    occupancy_initial_points.push_back(point);
                    break;
                }
            }
        }
        if(!stop) {
            return -1;
        }
    }
    // Output points
    for(uint8_t i=0; i < original_output_pos.size(); i++) {
        estimated_pos.x = original_output_pos[i].point_coordinates.x / pixels_per_tile_x;
        estimated_pos.y = original_output_pos[i].point_coordinates.y / pixels_per_tile_y;
        stop = false;
        uint16_t c_initial = std::max(estimated_pos.x, 0);
        if(c_initial + KERNEL_SIZE >= occupancy_map.cols) {
            c_initial = occupancy_map.cols-KERNEL_SIZE-1;
        }
        uint16_t r_initial = std::max(estimated_pos.y, 0);
        if(r_initial + KERNEL_SIZE >= occupancy_map.rows) {
            r_initial = occupancy_map.rows-KERNEL_SIZE-1;
        }
        for(uint16_t c=c_initial; c > c_initial - KERNEL_SIZE; c--) {
            if(stop) break;
            for(uint16_t r=r_initial; r < r_initial + KERNEL_SIZE; r++) {
                if(occupancy_map.at<uint8_t>(r, c) == 255) {
                    stop = true;
                    point = original_output_pos[i];
                    point.point_coordinates = {c,r};
                    occupancy_final_points.push_back(point);
                    break;
                }
            }
        }
        if(!stop) {
            return -1;
        }
    }
}

std::vector<Line> find_line_connections(AStar::Generator& generator, cv::Mat& occupancy_grid_map, std::vector<Point>& initial_points, std::vector<Point>& final_points)
{
    AStar::CoordinateList path;
    std::vector<Line> detected_connections;
    std::vector<std::vector<Line_connection>> all_lines_costs;
    float cost;

    // Calculate the cost for each possible connection
    for(uint8_t i=0; i < initial_points.size(); i++) {
        std::vector<Line_connection> vec_line_costs;
        for(uint8_t j=0; j < final_points.size(); j++) {
            // Find optimal path between each initial and final point
            path = generator.findPath(initial_points[i].point_coordinates, final_points[j].point_coordinates);
            if(path.size() <= 1) { // No Path found
                std::vector<Line> v;
                return v;
            }
            // Get some points (equidistants) from the line
            AStar::CoordinateList extracted_points = extract_line_points(path);
            // Create structure to save line info
            Line_connection struct_line;
            struct_line.cost =  calculate_line_cost(extracted_points);
            struct_line.initial_point = initial_points[i].point_coordinates;
            struct_line.final_point = final_points[j].point_coordinates;
            //struct_line.line_points = path;
            // Add results to the vector
            vec_line_costs.push_back(struct_line);
        }
        all_lines_costs.push_back(vec_line_costs);
    }

    // Find best connections
    uint8_t better_connection = 0;
    AStar::Vec2i final_point;
    float better_cost;

    float min;
    int better_match;
    bool is_line_selected[final_points.size()] = {false};
    for(uint8_t i=0; i < final_points.size(); i++) {
        min = all_lines_costs[0][i].cost;
        better_match = -1;
        for(uint8_t j=0; j < all_lines_costs.size(); j++) {
            if(all_lines_costs[j][i].cost < min && all_lines_costs[j][i].cost == std::min_element(all_lines_costs[j].begin(), all_lines_costs[j].end(), compare_costs)->cost) {
                min = all_lines_costs[j][i].cost;
                better_match = j;
            }
        }
        if(better_match > -1) {
            Line line;
            line.input = initial_points[better_match];
            line.input.point_coordinates = all_lines_costs[better_match][i].initial_point;
            line.output = final_points[i];
            line.output.point_coordinates = all_lines_costs[better_match][i].final_point;
            detected_connections.push_back(line);
            all_lines_costs.erase(all_lines_costs.begin()+better_match);
            is_line_selected[i] = true;
        }
    }

    for(uint8_t i=0; i < final_points.size(); i++) {
        if(is_line_selected[i] == false) {
            min = all_lines_costs[0][i].cost / std::min_element(all_lines_costs[0].begin(), all_lines_costs[0].end(), compare_costs)->cost;
            better_match = 0;
            for(uint8_t j=1; j < all_lines_costs.size(); j++) {
                if(all_lines_costs[j][i].cost / std::min_element(all_lines_costs[j].begin(), all_lines_costs[j].end(), compare_costs)->cost < min) {
                    min = all_lines_costs[j][i].cost / std::min_element(all_lines_costs[j].begin(), all_lines_costs[j].end(), compare_costs)->cost;
                    better_match = j;
                }
            }
            Line line;
            line.input = initial_points[better_match];
            line.input.point_coordinates = all_lines_costs[better_match][i].initial_point;
            line.output = final_points[i];
            line.output.point_coordinates = all_lines_costs[better_match][i].final_point;
            detected_connections.push_back(line);
            all_lines_costs.erase(all_lines_costs.begin()+better_match);
        }
    }
    return detected_connections;
}
