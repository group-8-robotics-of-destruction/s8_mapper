#ifndef __MAP_H
#define __MAP_H

#include <vector>
#include <stdexcept>

namespace s8 {
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

    std::vector<Row> matrix;

    public:
        Map() : rows(0), cols(0) {}

        Map(size_t rows, size_t cols, int default_value = 0) : rows(rows), cols(cols) {
            matrix = std::vector<Row>(rows, Row(cols, default_value));
            ROS_INFO("%ldx%ld", rows, cols);
        }

        Map & operator= (Map && map) {
            rows = map.rows;
            cols = map.cols;
            matrix = std::move(map.matrix);
            return *this;
        }

        size_t num_rows() const {
            return rows;
        }

        size_t num_cols() const {
            return cols;
        }

        Row & operator[] (size_t index) throw (std::out_of_range) {
            return matrix.at(index);
        }
    };
}

#endif
