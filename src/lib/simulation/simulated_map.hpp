// Copyright © 2021 Gianmarco Rampulla. All Rights Reserved.

/**
 * @file simulated_map.hpp
 * @brief Implementation of the `simulated_map` component handling collision and collision avoidance.
 */

#ifndef FCPP_SIMULATED_MAP_H_
#define FCPP_SIMULATED_MAP_H_

#include <cstring>
#include <stack>
#include <queue>
#include <set>

#include "lib/common/traits.hpp"
#include "lib/component/base.hpp"
#include "lib/data/color.hpp"
#include "lib/data/vec.hpp"

#include "external/stb_image/stb_image.h"
#include "stb_image_write.h"




/**
 * @brief Namespace containing all the objects in the FCPP library.
 */
namespace fcpp {

//! @brief Namespace for all FCPP components.
namespace component {

//! @brief Namespace of tags to be used for initialising components.
namespace tags {
    //! @brief Declaration tag associating to the dimensionality of the space.
    template <intmax_t n>
    struct dimension;

    //! @brief Declaration tag associating to the bounding coordinates of the grid area.
    template <intmax_t xmin, intmax_t ymin, intmax_t xmax, intmax_t ymax, intmax_t den>
    struct area;

    //! @brief Net initialisation tag associating to the minimum coordinates of the grid area.
    struct area_min;

    //! @brief Net initialisation tag associating to the maximum coordinates of the grid area.
    struct area_max;

    //! @brief Net initialisation tag associating to the path of the image representing obstacles.
    struct obstacles {};

    //! @brief Net initialisation tag associating to the color of the obstacles.
    struct obstacles_color {};

    //! @brief Net initialisation tag associating to the threshold in which consider the specified obstacles_color.
    struct obstacles_color_threshold {};

}


//! @cond INTERNAL
namespace details {
    //! @brief Converts a number sequence to a vec (general form).
    template <typename T>
    struct numseq_to_vec_map;
    //! @brief Converts a number sequence to a vec (empty form).
    template <>
    struct numseq_to_vec_map<common::number_sequence<>> {
        constexpr static vec<0> min{};
        constexpr static vec<0> max{};
    };
    //! @brief Converts a number sequence to a vec (active form).
    template <intmax_t xmin, intmax_t ymin, intmax_t xmax, intmax_t ymax, intmax_t den>
    struct numseq_to_vec_map<common::number_sequence<xmin,ymin,xmax,ymax,den>> {
        constexpr static vec<2> min{xmin*1.0/den,ymin*1.0/den};
        constexpr static vec<2> max{xmax*1.0/den,ymax*1.0/den};
    };

}
//! @endcond


/**
 * @brief Component handling node collision and collision avoidance.
 *
 * <b>Declaration tags:</b>
 * - \ref tags::dimension defines the dimensionality of the space (defaults to 2).
 * - \ref tags::area defines the area in which collision is considered.
 *
 * <b>Net initialisation tags:</b>
 * - \ref tags::area_min associates to a vector representing minimum area coordinate.
 * - \ref tags::area_max associates o a vector representing maximum area coordinate.
 * - \ref tags::obstacles associates to a path of the image representing obstacles.
 * - \ref tags::obstacles_colors associates to a color used to identify which pixel on the bitmaps are obstacles.
 * - \ref tags::obstacles_color_threshold associates to a real number used to have a margin error for colors in different image format.
 *
 */
template <class... Ts>
struct simulated_map {
    //! @brief The dimensionality of the space.
    constexpr static intmax_t dimension = common::option_num<tags::dimension, 2, Ts...>;

    //! @brief Bounding coordinates of the grid area.
    using area = common::option_nums<tags::area, Ts...>;

    static_assert(area::size == 5 or area::size == 0, "the bounding coordinates must be 4 integers");

    /**
     * @brief The actual component.
     *
     * Component functionalities are added to those of the parent by inheritance at multiple levels: the whole component class inherits tag for static checks of correct composition, while `node` and `net` sub-classes inherit actual behaviour.
     * Further parametrisation with F enables <a href="https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern">CRTP</a> for static emulation of virtual calls.
     *
     * @param F The final composition of all components.
     * @param P The parent component to inherit from.
     */
    template <typename F, typename P>
    struct component : public P {
        //! @cond INTERNAL
        DECLARE_COMPONENT(simulated_map);
        //! @endcond

        //! @brief The local part of the component.
        using node = typename P::node;

        //! @brief The global part of the component.
        class net : public P::net {

          public: // visible by node objects and the main program
            //! @brief Type for representing a position.
            using position_type = vec<dimension>;

            //! @brief Constructor from a tagged tuple.
            template <typename S, typename T>
            net(common::tagged_tuple <S, T> const& t) : P::net(t) {

                static_assert((S::template intersect<tags::obstacles>::size == 1) <= (area::size == 5 or S::template intersect<tags::area_min, tags::area_max>::size == 2), "no option area defined and no area_min and area_max defined either");

                if (S::template intersect<tags::obstacles>::size == 0) {
                    m_viewport_min = to_pos_type(make_vec(0, 0));
                    m_viewport_max = to_pos_type(make_vec(1, 1));
                }
                else {
                    //variables to avoid linking issues
                    constexpr auto max = details::numseq_to_vec_map<area>::max;
                    constexpr auto min = details::numseq_to_vec_map<area>::min;
                    m_viewport_min = to_pos_type(common::get_or<tags::area_min>(t, min));
                    m_viewport_max = to_pos_type(common::get_or<tags::area_max>(t, max));
                }

                position_type viewport_size = m_viewport_max - m_viewport_min;
                load_bitmap(common::get_or<tags::obstacles>(t, ""),
                                 common::get_or<tags::obstacles_color>(t, color(BLACK)),
                                 common::get_or<tags::obstacles_color_threshold>(t, 0.5));
                m_index_scales = {m_bitmap[0].size() / viewport_size[0], m_bitmap.size() / viewport_size[1]};
                m_index_factors = {viewport_size[0] / m_bitmap[0].size(), viewport_size[1] / m_bitmap.size()};

                fill_closest();

                mark_potential_points(common::get_or<tags::obstacles>(t, ""));
            }

            //! @brief Returns the position of the closest empty space starting from node_position
            position_type closest_space(position_type position) {
                if (!is_obstacle(position)) return position;
                index_type index = position_to_index(position);
                return index_to_position(m_closest[index[1]][index[0]], position);
            }

            //! @brief Returns the position of the closest obstacle starting from node_position
            position_type closest_obstacle(position_type position) {
                if (is_obstacle(position)) return position;
                if (!is_in_area(position)) return closest_obstacle(get_nearest_edge_position(position));
                index_type index = position_to_index(position);
                return index_to_position(m_closest[index[1]][index[0]], position);
            }

            //! @brief Returns true if a specific position is a obstacle otherwise false
            bool is_obstacle(position_type position) {
                if (!is_in_area(position)) return false;
                index_type index = position_to_index(position);
                return m_bitmap[index[1]][index[0]];
            }

          private: // implementation details

            //! @brief Type for representing a bitmap index.
            using index_type = std::array<size_t, 2>;

            //! @brief Type for representing a bfs queue pair of point and its source point from which was generated.
            using matrix_pair_type = std::pair<index_type,index_type>;

            //! @brief Converts a node position to an equivalent bitmap index
            inline index_type position_to_index(position_type const& position) {
                index_type index_to_return;
                //linear scaling
                for (int i = 0; i < 2; i++)
                    index_to_return[i] = static_cast<size_t>(std::min(std::max(std::floor(m_index_scales[i] * (position[i] - m_viewport_min[i])), real_t(0)),m_viewport_max[i]-1));
                return index_to_return;
            }

            //! @brief Converts a bitmap index to an equivalent node position
            position_type index_to_position(index_type const& index, position_type position) {
                //linear scaling inverse formula
                for (int i = 0; i < 2; i++) {
                    real_t x = index[i] * m_index_factors[i] + m_viewport_min[i];
                    position[i] = std::min(std::max(position[i], x), x + m_index_factors[i]);
                }
                return position;
            }

            //! @brief Checks if a position is contained in the predefined area
            bool is_in_area(position_type position) {
                for (int i = 0; i < 2; i++)
                    if (position[i] < m_viewport_min[i] || position[i] > m_viewport_max[i])
                        return false;
                return true;
            }

            //! @brief Calculates the nearest in area position (edge position) starting from a generic node position
            position_type get_nearest_edge_position(position_type position) {
                for (int i = 0; i < 2; i++)
                    position[i] = std::min(std::max(position[i], m_viewport_min[i]), m_viewport_max[i]);
                return position;
            }

            //! @brief Fills m_bitmap by reading and parsing a stored bitmap given the bitmap file path
            void load_bitmap(std::string const& path, color const& color, real_t threshold) {
                if (path.empty()) {
                    m_bitmap = {{false}};
                    return;
                }
                int bitmap_width, bitmap_height, channels_per_pixel, row_index, col_index = 0;
                std::string real_path;
                threshold *= 255;
#if _WIN32
                real_path = std::string(".\\textures\\").append(path);
#else
                //real_path = std::string("./textures/").append(path);
                real_path = path;
#endif
                unsigned char *bitmap_data = stbi_load(real_path.c_str(), &bitmap_width, &bitmap_height, &channels_per_pixel, 0);
                if (bitmap_data == nullptr) throw std::runtime_error("Error in image loading");

                row_index = bitmap_height - 1;
                m_bitmap = std::vector<std::vector<bool>>(bitmap_height,std::vector<bool>(bitmap_width,true));

                while (row_index >= 0) {
                    unsigned char* pixel_ptr = (bitmap_data + (channels_per_pixel * ((bitmap_width - row_index) * bitmap_width + col_index)));
                    for (int j = 0; j < channels_per_pixel && m_bitmap[row_index][col_index]; j++)
                        m_bitmap[row_index][col_index] = m_bitmap[row_index][col_index] && std::abs(color.rgba[j] * 255 - pixel_ptr[j]) < threshold;
                    col_index++;
                    if (col_index == bitmap_width) {
                        row_index--;
                        col_index = 0;
                    }
                }

                stbi_image_free(bitmap_data);
            }

            void mark_potential_points(std::string const& path) {

                constexpr static std::array<std::array<int, 2>, 8> deltas = {{{-1, 0}, {1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}}};
                int bitmap_width, bitmap_height, channels_per_pixel, row_index, col_index;
                std::string real_path;
#if _WIN32
                real_path = std::string(".\\textures\\").append(path);
#else
                //real_path = std::string("./textures/").append(path);
                real_path = path;
#endif
                unsigned char *bitmap_data = stbi_load(real_path.c_str(), &bitmap_width, &bitmap_height, &channels_per_pixel, 3);
                if (bitmap_data == nullptr) throw std::runtime_error("Error in image loading");

                std::vector<std::vector<bool>> visited(m_bitmap.size(), std::vector<bool>(m_bitmap[0].size()));
                std::vector<std::vector<std::pair<index_type,color>>> draw_buffers;
                std::vector<std::pair<index_type,int>> point_values;

                //prepare 2 buffers
                draw_buffers.emplace_back();
                draw_buffers.emplace_back();

                row_index = bitmap_height - 1;
                col_index = 0;

                while (row_index >= 0) {

                    if(!m_bitmap[row_index][col_index]) {

                        if (col_index >= 0 && col_index < m_bitmap[0].size() && row_index >= 0 && row_index < m_bitmap.size() && !m_bitmap[row_index][col_index]) {

                            int distance = get_distance(m_closest[row_index][col_index], col_index, row_index);
                            bool max = true;
                            for (std::array<int, 2> const &d: deltas) {
                                size_t n_x = col_index + d[0];
                                size_t n_y = row_index + d[1];
                                if (n_x >= 0 && n_x < m_bitmap[0].size() && n_y >= 0 && n_y < m_bitmap.size() && !m_bitmap[n_y][n_x]) {
                                    int n_distance = get_distance(m_closest[n_y][n_x], n_x, n_y);
                                    max &= distance >= n_distance;
                                }
                            }

                            if (max) {
                                index_type t;
                                t[0] = col_index;
                                t[1] = row_index;
                                draw_buffers[0].emplace_back(t, color(RED));

                                if (!visited[row_index][col_index]) {
                                    std::array<int,2> b = start_it_dfs(visited, row_index, col_index);
                                    if(b[0] > 0 && b[1] > 1) {
                                        //draw_buffers[1].emplace_back(b, color(BLUE));
                                        m_barycenters.emplace_back(b, color(BLUE));
                                    }

                                }
                            }
                        }
                    }
                    col_index++;
                    if (col_index == bitmap_width) {
                        row_index--;
                        col_index = 0;
                    }
                }

                create_rooms();

                static std::array<color,13> colors = {color(RED), color(BLUE), color(BROWN), color(LAVENDER), color(CYAN), color(GOLD), color(GREEN), color(FUCHSIA), color(IVORY), color(TOMATO), color(LEMON_CHIFFON), color(SALMON)};

                for(int r = 0; r < m_bitmap.size(); r++) {
                    for(int c = 0; c < m_bitmap[0].size(); c++) {
                        if(m_map_rooms[r][c] != -1)
                            write_point_on_image(bitmap_data, visited, r, c, bitmap_width, channels_per_pixel, colors[m_map_rooms[r][c]],false);
                    }
                }

                /*
                for (std::vector<std::pair<index_type,color>> const& p_room : m_rooms) {
                    for (std::pair<index_type, color> t: p_room) {
                        write_point_on_image(bitmap_data, visited, t.first[1], t.first[0], bitmap_width, channels_per_pixel, t.second,false);
                    }
                }
                 */

                for (std::vector<std::pair<index_type,color>> const& p_buffer : draw_buffers) {
                    for (std::pair<index_type, color> t: p_buffer) {
                        write_point_on_image(bitmap_data, visited, t.first[1], t.first[0], bitmap_width, channels_per_pixel, t.second);
                    }
                }

                write_point_on_image(bitmap_data, visited, m_barycenters[2].first[1], m_barycenters[2].first[0], bitmap_width, channels_per_pixel, color(GREEN),false);


                stbi_write_jpg("debugPoints.jpg", bitmap_width, bitmap_height, 3, bitmap_data, 100);

                stbi_image_free(bitmap_data);

            }

            void write_point_on_image(unsigned char* pointer, std::vector<std::vector<bool>>& visited, int row, int col, int bitmap_width, int channels, color clr, bool visit = true) {

                if(col >= 0 && col < m_bitmap[0].size() && row >= 0 && row < m_bitmap.size()) {
                    for (int j = 0; j < channels; j++) {
                        (pointer + (channels * ((bitmap_width - row) * bitmap_width + col)))[j] = clr.rgba[j] * 255;
                    }
                    if(visit)
                    visited[row][col] = true;
                }
            }

            int get_distance(index_type const& closest, int x, int y) {

                int deltaX = std::abs((int)closest[0] - x);
                int deltaY = std::abs((int)closest[1] - y);

                return std::min(deltaX, deltaY) + (2 * std::max(deltaX, deltaY));

            }


            //! @brief create rooms for navigations
            void create_rooms() {

                constexpr static std::array<std::array<int, 3>, 8> deltas = {{{-1, 0, 2}, {1, 0, 2}, {0, 1, 2}, {0, -1, 2}, {1, 1, 3}, {-1, 1, 3}, {1, -1, 3}, {-1, -1, 3}}};
                int rooms_count = -1;
                int rooms_color = 0;
                //dynamic queues vector with source queue
                m_map_rooms = std::vector<std::vector<int>>(m_bitmap.size(), std::vector<int>(m_bitmap[0].size(),-1));
                std::vector<std::vector<int>> distances(m_bitmap.size(), std::vector<int>(m_bitmap[0].size(),-1));
                std::vector<std::vector<std::pair<std::array<int,2>,int>>> queues(1);

                //start bfs
                //for(std::pair<index_type,color> barycentre : m_barycenters) {

                    std::pair<std::array<int,2>,color> barycentre = m_barycenters[2];

                    queues[0].emplace_back(barycentre.first,0);
                    m_rooms.emplace_back();
                    rooms_count++;
                    if(rooms_count > 0 && !m_rooms[rooms_count-1].empty()) rooms_color++;

                    std::vector<std::vector<bool>> visited(m_bitmap.size(), std::vector<bool>(m_bitmap[0].size()));

                    //start bfs
                    for (size_t i = 0; i < queues.size(); ++i) {
                        for (std::pair<std::array<int,2>,int> const& elem : queues[i]) {

                            std::array<int,2> const& point = elem.first;
                            int distance = elem.second;

                            if (!visited[point[1]][point[0]]) {
                                distances[point[1]][point[0]] = distance;
                                m_map_rooms[point[1]][point[0]] = rooms_count;
                                visited[point[1]][point[0]] = true;

                                for (std::array<int,3> const& d : deltas) {
                                    //add queues to add nodes at distance i+2 and i+3
                                    while (i+d[2] >= queues.size()) queues.emplace_back();
                                    int n_x = point[0] + d[0];
                                    int n_y = point[1] + d[1];
                                    if (n_x >= 0 && n_x < m_bitmap[0].size() && n_y >= 0 && n_y < m_bitmap.size()) {

                                        if (m_bitmap[n_y][n_x]) {

                                            std::pair<int ,int> OB = {  barycentre.first[0] - n_x ,barycentre.first[1] - n_y};
                                            for(int r = 0; r < m_bitmap.size(); r++) {
                                                for(int c = 0; c < m_bitmap[0].size(); c++) {
                                                    std::pair<int, int> OP = {c - n_x,r - n_y};
                                                    if (OP.first * OB.first + OP.second * OB.second <= 80) {
                                                        visited[r][c] = true;
                                                        m_map_rooms[r][c] = -1;
                                                    }
                                                }
                                            }
                                        }
                                        else {
                                            std::array<int,2> new_p{};
                                            new_p[0] = n_x;
                                            new_p[1] = n_y;
                                            queues[i+d[2]].push_back({new_p, distance + d[2]});
                                        }
                                    }
                                }
                            }
                        }
                        queues[i].clear();
                    }
                }


            //! @brief Fills m_closest by parsing the bitmap with two sequential bfs
            void fill_closest() {
                constexpr static std::array<std::array<int, 3>, 8> deltas = {{{-1, 0, 2}, {1, 0, 2}, {0, 1, 2}, {0, -1, 2}, {1, 1, 3}, {-1, 1, 3}, {1, -1, 3}, {-1, -1, 3}}};
                m_closest = std::vector<std::vector<index_type>>(m_bitmap.size(),std::vector<index_type>(m_bitmap[0].size()));

                for (int obstacle = 1; obstacle >= 0; obstacle--) {
                    //dynamic queues vector with source queue
                    std::vector<std::vector<matrix_pair_type>> queues(1);
                    //new visited matrix
                    std::vector<std::vector<bool>> visited(m_bitmap.size(), std::vector<bool>(m_bitmap[0].size()));
                    //load source points
                    for (size_t r = 0; r < m_bitmap.size(); r++) {
                        for (size_t c = 0; c < m_bitmap[0].size(); c++) {
                            if (m_bitmap[r][c] == obstacle)
                                queues[0].emplace_back(std::pair<index_type, index_type>({c, r}, {c, r}));
                        }
                    }
                    //start bfs
                    for (size_t i = 0; i < queues.size(); ++i) {
                        for (matrix_pair_type const& elem : queues[i]) {
                            index_type const& point = elem.first;
                            if (!visited[point[1]][point[0]]) {
                                if (i > 0) m_closest[point[1]][point[0]] = elem.second;
                                visited[point[1]][point[0]] = true;
                                for (std::array<int,3> const& d : deltas) {
                                    //add queues to add nodes at distance i+2 and i+3
                                    while (i+d[2] >= queues.size()) queues.emplace_back();
                                    size_t n_x = point[0] + d[0];
                                    size_t n_y = point[1] + d[1];
                                    if (n_x >= 0 && n_x < m_bitmap[0].size() && n_y >= 0 && n_y < m_bitmap.size())
                                        queues[i+d[2]].push_back({{n_x, n_y}, elem.second});
                                }
                            }
                        }
                        queues[i].clear();
                    }
                    //end bfs
                }
            }

            std::array<int,2> start_it_dfs(std::vector<std::vector<bool>>& visited, int row_index, int column_index) {

                constexpr static std::array<std::array<int, 2>, 8> deltas = {{{-1, 0}, {1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}}};

                std::stack<std::pair<std::pair<size_t,size_t>,int>> stack;
                int tollerance = 5;
                int x_sum = 0,y_sum = 0,count = 0;

                stack.push({{column_index, row_index},tollerance});

                while(!stack.empty()) {

                    std::pair<std::pair<size_t,size_t>,int> p = stack.top();
                    stack.pop();

                    visited[p.first.second][p.first.first] = true;

                    if (p.second == 0) continue;

                    if(p.second == tollerance) {
                        count++;
                        x_sum += p.first.first;
                        y_sum += p.first.second;
                    }

                    for (std::array<int, 2> const &d: deltas) {

                        size_t n_x = p.first.first + d[0];
                        size_t n_y = p.first.second + d[1];

                        if (n_x >= 0 && n_x < m_bitmap[0].size() && n_y >= 0 && n_y < m_bitmap.size() && !m_bitmap[n_y][n_x] && !visited[n_y][n_x]) {

                            int distance_n = get_distance(m_closest[n_y][n_x], n_x, n_y);
                            bool max = true;
                            for (std::array<int, 2> const &d: deltas) {
                                size_t n_x2 = n_x + d[0];
                                size_t n_y2 = n_y + d[1];
                                if (n_x2 >= 0 && n_x2 < m_bitmap[0].size() && n_y2 >= 0 && n_y2 < m_bitmap.size() &&
                                    !m_bitmap[n_y2][n_x2]) {
                                    int n_distance = get_distance(m_closest[n_y2][n_x2], n_x2, n_y2);
                                    max &= distance_n >= n_distance;
                                }
                            }

                            if (max) {
                                stack.push({{n_x, n_y},tollerance});
                            } else if (p.second > 0) {
                                stack.push({{n_x, n_y},p.second-1});
                            }
                        }
                    }
                }
                std::array<int,2> barycentre{};
                barycentre[0] = x_sum / count;
                barycentre[1] = y_sum / count;
                return barycentre;
            }

            //! @brief Convert a generic vector to a compatible position on the map
            template<size_t n>
            position_type to_pos_type(vec<n> const& vec) {
                position_type t;
                for (size_t i = 0; i < vec.dimension; ++i)
                    t[i] = vec[i];
                return t;
            }

            /**
            * @brief Bitmap representation
            *
            * a true value means there is an obstacle otherwise false
            */
            std::vector<std::vector<bool>> m_bitmap;

            /**
            * @brief Matrix containing data to implements closest_space() and closest_obstacle()
            *
            * if a indexed position is empty it contains the nearest obstacle otherwise the nearest empty space position
            */
            std::vector<std::vector<index_type>> m_closest;

            //! @brief Vector of maximum coordinate of the grid area.
            position_type m_viewport_max;

            //! @brief Vector of minimum coordinate of the grid area.
            position_type m_viewport_min;

            //! @brief Array containing cached values of m_index_size / m_viewport_size
            std::array<real_t,2> m_index_scales;

            //! @brief Array containing cached values of m_index_size * m_viewport_size
            std::array<real_t,2> m_index_factors;

            //! @brief Matrix containing each point of a specific room
            std::vector<std::vector<std::pair<index_type,color>>> m_rooms;

            //! @brief Array containing calculated barycenters
            std::vector<std::pair<std::array<int,2>,color>> m_barycenters;

            //! @brief Array containing calculated barycenters
            std::vector<std::vector<int>> m_map_rooms;


        };
    };
};
}
}
#endif // FCPP_SIMULATED_MAP_H_
