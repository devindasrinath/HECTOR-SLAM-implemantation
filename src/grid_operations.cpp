#include "grid_operations.h"

Grid::Grid(GridParameters &grid_parameters) :
_grid_parameters(grid_parameters) {
}


std::vector<sf::Vertex>* Grid::init_grid()
{
    /* vertical grid lines*/
    for(size_t i = 0; i<_grid_parameters.grid_width; i+=_grid_parameters.step_size){
        if(i == _grid_parameters.origin.second ){
            _vertice_array.push_back(sf::Vertex(sf::Vector2f(i, 0), sf::Color::Red));
            _vertice_array.push_back(sf::Vertex(sf::Vector2f(i, _grid_parameters.grid_height), sf::Color::Red));
        }
        else{
            _vertice_array.push_back(sf::Vertex(sf::Vector2f(i, 0), _grid_parameters.grid_color));
            _vertice_array.push_back(sf::Vertex(sf::Vector2f(i, _grid_parameters.grid_height), _grid_parameters.grid_color));
        }
    }

    /* horizontal grid lines*/
    for(size_t i = 0; i<_grid_parameters.grid_height; i+=_grid_parameters.step_size){
        if(i == _grid_parameters.origin.first ){
            _vertice_array.push_back(sf::Vertex(sf::Vector2f(0, i), sf::Color::Red));
            _vertice_array.push_back(sf::Vertex(sf::Vector2f(_grid_parameters.grid_width, i), sf::Color::Red));
        }
        else{
            _vertice_array.push_back(sf::Vertex(sf::Vector2f(0, i), _grid_parameters.grid_color));
            _vertice_array.push_back(sf::Vertex(sf::Vector2f(_grid_parameters.grid_width, i), _grid_parameters.grid_color));
        }
    }

    return &_vertice_array;

}

sf::Text* Grid::init_texts(){
    
    _font.loadFromFile("./fonts/open-sans/OpenSans-Bold.ttf");

    _origin_text.setString(std::string("0,0"));
    _origin_text.setPosition(_grid_parameters.origin.first,_grid_parameters.grid_height-_grid_parameters.origin.second );
    _origin_text.setOrigin(0,0);
    _origin_text.setFont(_font);
    _origin_text.setColor(sf::Color::White);
    _origin_text.setCharacterSize(12);

    return &_origin_text;
}

sf::RectangleShape* Grid::draw_cell(size_t x, size_t y,sf::Color color = sf::Color::Yellow){
    auto p_rectangle_shape = new sf::RectangleShape; 
    _cell_vector.x = (float) _grid_parameters.step_size;
    _cell_vector.y = (float) _grid_parameters.step_size;
    _cell_position.x =  ((x-0.5)*_grid_parameters.step_size) + _grid_parameters.origin.first;
    _cell_position.y = _grid_parameters.grid_height - ((y+0.5)*_grid_parameters.step_size) - _grid_parameters.origin.second;
    p_rectangle_shape->setFillColor(color);
    p_rectangle_shape->setSize(_cell_vector);
    p_rectangle_shape->setPosition(_cell_position);

    return p_rectangle_shape;

}


sf::Vertex* Grid::draw_line(size_t x1 , size_t y1, size_t x2 , size_t y2)
{

    _line_points[0] = sf::Vertex(sf::Vector2f((x1*_grid_parameters.step_size) + _grid_parameters.origin.first
        , _grid_parameters.grid_height - (y1*_grid_parameters.step_size) - _grid_parameters.origin.second));
    _line_points[0].color = sf::Color::Blue;

    _line_points[1] = sf::Vertex(sf::Vector2f((x2*_grid_parameters.step_size) + _grid_parameters.origin.first
        , _grid_parameters.grid_height - (y2*_grid_parameters.step_size) - _grid_parameters.origin.second));
    _line_points[1].color = sf::Color::Blue;

    return _line_points;
}

sf::Vertex* Grid::draw_point(double x1 , double y1)
{
    auto _point = new sf::Vertex();

    _point->position = (sf::Vector2f((x1*_grid_parameters.step_size) + _grid_parameters.origin.first
        , _grid_parameters.grid_height - (y1*_grid_parameters.step_size) - _grid_parameters.origin.second));

    _point->color = sf::Color::Blue;

    return _point;

}

sf::Vertex* Grid::draw_line_without_grid(size_t x1 , size_t y1, size_t x2 , size_t y2)
{

    _line_points[0] = sf::Vertex(sf::Vector2f(x1 + _grid_parameters.origin.first
        , _grid_parameters.grid_height - y1 - _grid_parameters.origin.second));
    _line_points[0].color = sf::Color::Blue;

    _line_points[1] = sf::Vertex(sf::Vector2f(x2 + _grid_parameters.origin.first
        , _grid_parameters.grid_height - y2 - _grid_parameters.origin.second));
    _line_points[1].color = sf::Color::Blue;

    return _line_points;
}



sf::CircleShape* Grid::draw_circle(double x1 , double y1, size_t r)
{
    auto _p_circle = new sf::CircleShape();
    _p_circle->setRadius(r);
    _p_circle->setPosition(sf::Vector2f(x1 *_grid_parameters.step_size+ _grid_parameters.origin.first
        , _grid_parameters.grid_height - y1*_grid_parameters.step_size - _grid_parameters.origin.second));
    _p_circle->setOrigin(r, r) ;   
    _p_circle->setFillColor(sf::Color::Red);
    _p_circle->setOutlineColor(sf::Color::Yellow);
    _p_circle->setOutlineThickness(2);


    return _p_circle;
}