// Copyright © 2021 Gianmarco Rampulla. All Rights Reserved.

/**
 * @file simulated_map.hpp
 * @brief Implementation of the `simulated_map` component handling collision and collision avoidance.
 */

#ifndef FCPP_SIMULATED_MAP_H_
#define FCPP_SIMULATED_MAP_H_

#include "lib/component/base.hpp"
#include "lib/data/vec.hpp"
#include "lib/data/color.hpp"
#include "lib/common/traits.hpp"

#include <cstring>
#include <queue>
#include "./external/stb_image/stb_image.h"

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

                static_assert(area::size == 5 or S::template intersect<tags::area_min, tags::area_max>::size == 2,
                              "no option area defined and no area_min and area_max defined either");

                //variables to avoid linking issues
                constexpr auto max = details::numseq_to_vec_map<area>::max;
                constexpr auto min = details::numseq_to_vec_map<area>::min;
                m_viewport_min = common::get_or<tags::area_min>(t, min);
                m_viewport_max = common::get_or<tags::area_max>(t, max);
                load_bitmap(common::get_or<tags::obstacles>(t, ""),
                                 common::get_or<tags::obstacles_color>(t, color(BLACK)),
                                 common::get_or<tags::obstacles_color_threshold>(t, 0.5));
                fill_closest();
                position_type viewport_size = m_viewport_max - m_viewport_min;

                m_index_scales = {m_bitmap[0].size() / viewport_size[0], m_bitmap.size() / viewport_size[1]};
                m_index_factors = {viewport_size[0] / m_bitmap[0].size(), viewport_size[1] / m_bitmap.size()};
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
                    index_to_return[i] = static_cast<size_t>(std::round(m_index_scales[i] * (position[i] - m_viewport_min[i])));
                return index_to_return;
            }

            //! @brief Converts a bitmap index to an equivalent node position
            position_type index_to_position(index_type const& index, position_type position) {
                //linear scaling inverse formula
                for (int i = 0; i < 2; i++)
                    position[i] = std::round(index[i] * m_index_factors[i] + m_viewport_min[i]);
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
            void load_bitmap(std::string const& path, color const& color, real_t const& threshold) {
                int bitmap_width, bitmap_height, channels_per_pixel, line_index = 0;
                std::string real_path;

                if (path.empty()) {
                    m_bitmap = {{false}};
                    return;
                }

                #if _WIN32
                   real_path = std::string(".\\textures\\").append(path);
                #else
                   real_path = std::string("./textures/").append(path);
                #endif

                unsigned char *bitmap_data = stbi_load(real_path.c_str(), &bitmap_width, &bitmap_height, &channels_per_pixel, 0);
                if (bitmap_data != nullptr) {
                    m_bitmap.resize(bitmap_height);
                    m_bitmap[0].resize(bitmap_width, true);

                    for (int i = 0; i < bitmap_height * bitmap_width; i++) {
                        unsigned char *pixelOffset = bitmap_data + i * channels_per_pixel;
                        if (i != 0 && i % bitmap_width == 0) {
                            line_index++;
                            m_bitmap[line_index].resize(bitmap_width, true);
                        }
                        for (int j = 0; j < channels_per_pixel; j++)
                            m_bitmap[line_index][i%bitmap_width] = m_bitmap[line_index][i%bitmap_width] && std::abs(color.rgba[j] * 255 - pixelOffset[j]) < threshold;
                    }
                    stbi_image_free(bitmap_data);
                }
                else throw std::runtime_error("Error in image loading");

            }

            //! @brief Fills m_closest by parsing the bitmap with two sequential bfs
            void fill_closest() {
                constexpr static std::array<std::array<int, 3>, 8> deltas = {{{-1, 0, 2}, {1, 0, 2}, {0, 1, 2}, {0, -1, 2}, {1, 1, 3}, {-1, 1, 3}, {1, -1, 3}, {-1, -1, 3}}};

                m_closest = std::vector<std::vector<index_type>>(m_bitmap.size(),std::vector<index_type>(m_bitmap[0].size()));
                //triple queues are generated compared to normal width in order to express i+2 and i+3 distances without indexing error
                std::vector<std::queue<matrix_pair_type>> queues(m_bitmap.size()*3);
                for (int obstacle = 1; obstacle >= 0; obstacle--) {
                    //new visited matrix
                    std::vector<std::vector<bool>> visited(m_bitmap.size(), std::vector<bool>(m_bitmap[0].size()));
                    //load source points
                    for (int r = 0; r < m_bitmap.size(); r++)
                        for (int c = 0; c < m_bitmap[0].size(); c++)
                            if (m_bitmap[r][c] == obstacle) queues[0].push({{static_cast<size_t>(c), static_cast<size_t>(r)},{static_cast<size_t>(c), static_cast<size_t>(r)}});
                    //start bfs
                    for (int i = 0; i < queues.size(); i++) {
                        std::queue<matrix_pair_type>& current_queue = queues[i];

                        while (!current_queue.empty()) {
                            matrix_pair_type elem = current_queue.front();
                            index_type point = elem.first;
                            if (!visited[point[1]][point[0]]) {

                                if ((obstacle == 0 && m_bitmap[point[1]][point[0]]) || obstacle == 1)
                                    m_closest[point[1]][point[0]] = elem.second;
                                visited[point[1]][point[0]] = true;

                                for (int j = 0; j < 4; j++) {
                                    if (is_in_bound(point[0] + deltas[j][0], point[1] + deltas[j][1]))
                                        queues[i+2].push({{point[0] + deltas[j][0], point[1] + deltas[j][1]}, elem.second});
                                    if (is_in_bound(point[0] + deltas[j+4][0], point[1] + deltas[j+4][1]))
                                        queues[i+3].push({{point[0] + deltas[j+4][0], point[1] + deltas[j+4][1]}, elem.second});
                                }
                            }
                            current_queue.pop();
                        }
                        //clear queue
                        current_queue = std::queue<matrix_pair_type>();
                    }
                }
            }

            //! @brief Checks if an index is in the bitmap size limits
            inline bool is_in_bound(int x, int y) {
                return x >= 0 && x < m_bitmap[0].size() && y >= 0 && y < m_bitmap.size();
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

        };
    };
};
}
}
#endif // FCPP_SIMULATED_MAP_H_
