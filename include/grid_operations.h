#pragma once

#include <SFML/Graphics.hpp>
#include <math.h>


struct GridParameters{
    size_t grid_height;
    size_t grid_width;
    size_t step_size;
    std::pair<size_t,size_t> origin;
    sf::Color grid_color;

};



class Grid{

    public :

        Grid(GridParameters &grid_parameters);
        std::vector<sf::Vertex>* init_grid();
        sf::Text* init_texts();
        sf::RectangleShape* draw_cell(size_t x, size_t y,sf::Color color );
        sf::Vertex* draw_line(size_t x1 , size_t y1, size_t x2 , size_t y2);
        sf::Vertex* draw_line_without_grid(size_t x1 , size_t y1, size_t x2 , size_t y2);


    private:
        GridParameters _grid_parameters;
        std::vector<sf::Vertex> _vertice_array;
        sf::Text _origin_text;
        sf::Font _font;
        sf::Vector2f _cell_vector;
        sf::Vector2f _cell_position;
        sf::Vertex _line_points[2];
        
};

