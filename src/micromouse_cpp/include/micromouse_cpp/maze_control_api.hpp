#pragma once
/**
 * @file maze_control_api.hpp
 * @brief MMS (Micro Mouse Simulator) communication API
 * 
 * Provides interface for communicating with the MMS simulator
 * via stdin/stdout protocol.
 */

#include <string>

namespace micromouse {

// =============================================================================
// Basic Types
// =============================================================================

/**
 * @brief Grid cell position in the maze
 */
struct Cell {
    int x{0};
    int y{0};
    
    bool operator==(const Cell& other) const noexcept;
    bool operator<(const Cell& other) const noexcept;
};

/**
 * @brief Cardinal directions
 */
enum class Dir { North = 0, East = 1, South = 2, West = 3 };

/**
 * @brief Get x offset for direction
 */
int dx(Dir d);

/**
 * @brief Get y offset for direction
 */
int dy(Dir d);

/**
 * @brief Convert direction to MMS character ('n', 'e', 's', 'w')
 */
char dir_char(Dir d);

/**
 * @brief Get opposite direction
 */
Dir opposite(Dir d);

/**
 * @brief Convert direction to string
 */
std::string dir_to_string(Dir d);

// =============================================================================
// MazeControlAPI
// =============================================================================

/**
 * @brief API for communicating with MMS simulator
 * 
 * All methods are static. Communication uses stdout for commands
 * and stdin for responses. Movement commands wait for "ack" response.
 */
class MazeControlAPI {
public:
    // -------------------------------------------------------------------------
    // Maze Information
    // -------------------------------------------------------------------------
    
    /**
     * @brief Get maze width from simulator
     */
    static int get_maze_width();
    
    /**
     * @brief Get maze height from simulator
     */
    static int get_maze_height();
    
    // -------------------------------------------------------------------------
    // Wall Sensing
    // -------------------------------------------------------------------------
    
    /**
     * @brief Check if wall exists in front of robot
     */
    static bool has_wall_front();
    
    /**
     * @brief Check if wall exists to left of robot
     */
    static bool has_wall_left();
    
    /**
     * @brief Check if wall exists to right of robot
     */
    static bool has_wall_right();
    
    // -------------------------------------------------------------------------
    // Movement (all wait for ack)
    // -------------------------------------------------------------------------
    
    /**
     * @brief Move robot forward one cell
     */
    static void move_forward();
    
    /**
     * @brief Turn robot left 90 degrees
     */
    static void turn_left();
    
    /**
     * @brief Turn robot right 90 degrees
     */
    static void turn_right();
    
    // -------------------------------------------------------------------------
    // Visualization
    // -------------------------------------------------------------------------
    
    /**
     * @brief Display a wall in the simulator
     * @param x Cell x coordinate
     * @param y Cell y coordinate
     * @param direction Wall direction ('n', 'e', 's', 'w')
     */
    static void set_wall(int x, int y, char direction);
    
    /**
     * @brief Set cell background color
     * @param x Cell x coordinate
     * @param y Cell y coordinate
     * @param color Color character (e.g., 'r', 'g', 'b', 'c', 'y')
     */
    static void set_color(int x, int y, char color);
    
    /**
     * @brief Clear cell background color
     */
    static void clear_color(int x, int y);
    
    /**
     * @brief Clear all cell colors
     */
    static void clear_all_color();
    
    /**
     * @brief Set text in a cell
     */
    static void set_text(int x, int y, const std::string& text);
    
    /**
     * @brief Clear text in a cell
     */
    static void clear_text(int x, int y);
    
    /**
     * @brief Clear all cell text
     */
    static void clear_all_text();
};

}  // namespace micromouse