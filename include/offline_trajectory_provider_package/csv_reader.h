/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace utilities {

/**
 * @brief The CsvReader class.
 * 
 * This class provides a CSV file reader.
 * 
 * The class provides functions to open a file, count the number of lines in the
 * file, read a specific line from the file, and close the file.
 */
class CsvReader {
 public:
  // Constructor
  CsvReader();

  // Destructor
  ~CsvReader();

  void openFile(const std::string& file_name);
  void countFileLines();
  void storeFileToStdVector();
  // std::string readLine(const int line_idx);
  Eigen::VectorXd readLineAsVector(const int line_idx);
  void closeFile();

  int getCountFileLines() const { return file_line_count_; }

 private:
  std::string file_name_;
  std::ifstream file_stream_;

  // Pointer to the data
  std::shared_ptr<std::vector<std::vector<std::string>>> p_data_;

  // Variable to store the number of lines in the file
  int file_line_count_;

  // Variable to store the lines of the file as a vector of strings
  std::vector<std::string> file_lines_vector_;
};

}  // namespace utilities