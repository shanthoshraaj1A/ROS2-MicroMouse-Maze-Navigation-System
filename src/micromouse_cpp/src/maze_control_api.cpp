/**
 * @file maze_control_api.cpp
 * @brief Implementation of MMS communication API
 */

#include "micromouse_cpp/maze_control_api.hpp"

#include <cstdlib>
#include <iostream>

namespace micromouse {

// =============================================================================
// Cell Implementation
// =============================================================================

bool Cell::operator==(const Cell& other) const noexcept {
    return x == other.x && y == other.y;
}

bool Cell::operator<(const Cell& other) const noexcept {
    return (y < other.y) || (y == other.y && x < other.x);
}

// =============================================================================
// Direction Helper Functions
// =============================================================================

int dx(Dir d) {
    return d == Dir::East ? 1 : d == Dir::West ? -1 : 0;
}

int dy(Dir d) {
    return d == Dir::North ? 1 : d == Dir::South ? -1 : 0;
}

char dir_char(Dir d) {
    switch (d) {
        case Dir::North: return 'n';
        case Dir::East:  return 'e';
        case Dir::South: return 's';
        default:         return 'w';
    }
}

Dir opposite(Dir d) {
    switch (d) {
        case Dir::North: return Dir::South;
        case Dir::East:  return Dir::West;
        case Dir::South: return Dir::North;
        default:         return Dir::East;
    }
}

std::string dir_to_string(Dir d) {
    switch (d) {
        case Dir::North: return "NORTH";
        case Dir::East:  return "EAST";
        case Dir::South: return "SOUTH";
        default:         return "WEST";
    }
}

// =============================================================================
// MazeControlAPI Implementation
// =============================================================================

int MazeControlAPI::get_maze_width() {
    std::cout << "mazeWidth" << std::endl;
    std::string response;
    std::cin >> response;
    return atoi(response.c_str());
}

int MazeControlAPI::get_maze_height() {
    std::cout << "mazeHeight" << std::endl;
    std::string response;
    std::cin >> response;
    return atoi(response.c_str());
}

bool MazeControlAPI::has_wall_front() {
    std::cout << "wallFront" << std::endl;
    std::string response;
    std::cin >> response;
    return response == "true";
}

bool MazeControlAPI::has_wall_left() {
    std::cout << "wallLeft" << std::endl;
    std::string response;
    std::cin >> response;
    return response == "true";
}

bool MazeControlAPI::has_wall_right() {
    std::cout << "wallRight" << std::endl;
    std::string response;
    std::cin >> response;
    return response == "true";
}

void MazeControlAPI::move_forward() {
    std::cout << "moveForward" << std::endl;
    std::string response;
    std::cin >> response;
    if (response != "ack") {
        std::cerr << "moveForward error: " << response << std::endl;
    }
}

void MazeControlAPI::turn_left() {
    std::cout << "turnLeft" << std::endl;
    std::string ack;
    std::cin >> ack;
}

void MazeControlAPI::turn_right() {
    std::cout << "turnRight" << std::endl;
    std::string ack;
    std::cin >> ack;
}

void MazeControlAPI::set_wall(int x, int y, char direction) {
    std::cout << "setWall " << x << " " << y << " " << direction << std::endl;
}

void MazeControlAPI::set_color(int x, int y, char color) {
    std::cout << "setColor " << x << " " << y << " " << color << std::endl;
}

void MazeControlAPI::clear_color(int x, int y) {
    std::cout << "clearColor " << x << " " << y << std::endl;
}

void MazeControlAPI::clear_all_color() {
    std::cout << "clearAllColor" << std::endl;
}

void MazeControlAPI::set_text(int x, int y, const std::string& text) {
    std::cout << "setText " << x << " " << y << " " << text << std::endl;
}

void MazeControlAPI::clear_text(int x, int y) {
    std::cout << "clearText " << x << " " << y << std::endl;
}

void MazeControlAPI::clear_all_text() {
    std::cout << "clearAllText" << std::endl;
}

}  // namespace micromouse