#ifndef __MAP_H
#define __MAP_H

#include <vector>
#include <stdexcept>

namespace s8 {
    namespace map {
        struct MapCoordinate {
            int i; //Row index.
            int j; //Col index.

            MapCoordinate() : i(0), j(0) {}
            MapCoordinate(int i, int j) : i(i), j(j) {}
        };

        std::string to_string(MapCoordinate coordinate) {
            return "(" + std::to_string(coordinate.i) + ", " + std::to_string(coordinate.j) + ")";
        }

        class Map {
            class Row {
                const size_t cols;
                std::vector<int> row;

            public:
                Row(size_t cols, size_t default_value) : cols(cols) {
                    row = std::vector<int>(cols, default_value);
                }

                size_t num_cols() const {
                    return cols;
                }

                int & operator[] (size_t index) throw(std::out_of_range) {
                    return row.at(index);
                }
            };

        size_t rows;
        size_t cols;

        size_t origo_row;
        size_t origo_col;

        std::vector<Row> matrix;

        public:
            Map() : rows(0), cols(0), origo_row(0), origo_col(0) {}

            Map(size_t rows, size_t cols, int default_value = 0) : rows(rows), cols(cols) {
                matrix = std::vector<Row>(rows, Row(cols, default_value));
                origo_row = rows / 2;
                origo_col = cols / 2;
                ROS_INFO("%ldx%ld with origo (%ld,%ld)", rows, cols, origo_row, origo_col);
            }

            Map & operator= (Map && map) {
                rows = map.rows;
                cols = map.cols;
                origo_row = map.origo_row;
                origo_col = map.origo_col;
                matrix = std::move(map.matrix);
                return *this;
            }

            size_t num_rows() const {
                return rows;
            }

            size_t num_cols() const {
                return cols;
            }

            size_t num_cells() const {
                return rows * cols;
            }

            size_t get_origo_row() const {
                return origo_row;
            }
            
            size_t get_origo_col() const {
                return origo_col;
            }

            MapCoordinate get_origo() const {
                return MapCoordinate(origo_row, origo_col);
            }

            size_t row_relative_origo(size_t index) const {
                return origo_row + index;
            }

            size_t col_relative_origo(size_t index) const {
                return origo_col + index;
            }

            MapCoordinate coordinate_relative_origo(size_t i, size_t j) const {
                return MapCoordinate(row_relative_origo(i), col_relative_origo(j));
            }

            MapCoordinate coordinate_relative_origo(MapCoordinate coordinate) const {
                return coordinate_relative_origo(coordinate.i, coordinate.j);
            }

            Row & operator[] (size_t index) throw (std::out_of_range) {
                return matrix.at(index);
            }

            int & operator[] (MapCoordinate coordinate) throw (std::out_of_range) {
                return matrix.at(coordinate.i)[coordinate.i];
            }
        };
    }
}

#endif
