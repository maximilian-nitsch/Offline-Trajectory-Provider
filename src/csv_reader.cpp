/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#include "csv_reader.h"

#include <iostream>
#include <stdexcept>

namespace utilities {

/**
 * @brief Constructor of the CsvReader class.
 * 
 * The constructor initializes the file name and the file line count with empty values.
 * The pointer to the data is initialized with an empty vector of vectors of strings.
 * 
 * @param[in] None
 * 
 * @return None
*/
CsvReader::CsvReader()
    : file_name_(""), file_line_count_(0), file_lines_vector_() {

  // Create a shared pointer to a vector of vectors of strings
  p_data_ = std::make_shared<std::vector<std::vector<std::string>>>();
}

/**
 * @brief Destructor of the CsvReader class.
 * 
 * The destructor closes the file stream if it is open.
 * 
 * @param[in] None
 * 
 * @return None
*/
CsvReader::~CsvReader() {
  if (file_stream_.is_open()) {
    file_stream_.close();
  }
}

/**
 * @brief Count the number of lines in a file.
 * 
 * This function counts the number of lines in a file.
 * 
 * @param[in] None
 * 
 * @return None
*/
void CsvReader::countFileLines() {
  // Check if file is open
  if (!file_stream_.is_open()) {
    throw std::runtime_error("File is not open: " + file_name_);
  }

  // Count lines
  int line_count = 0;
  std::string line;

  while (std::getline(file_stream_, line)) {
    line_count++;
  }

  // Reset file stream
  file_stream_.clear();
  file_stream_.seekg(0, std::ios::beg);

  file_line_count_ = line_count;
}

/**
 * @brief Open a file.
 * 
 * This function opens and stores a file stream. The name of the file to open is passed as an argument.
 * 
 * @param file_name The name of the file to open.
 * 
 * @return None
*/
void CsvReader::openFile(const std::string& file_name) {
  // Set file name
  file_name_ = file_name;

  // Open file
  file_stream_.open(file_name_);

  // Check if file is open and return file stream
  if (!file_stream_.is_open()) {
    throw std::runtime_error("Could not open file: " + file_name_);
  }

  // Count lines in file
  countFileLines();

  // Store file lines in vector
  storeFileToStdVector();
}

// /**
//  * @brief Read a line from the file.
//  *
//  * This function reads a line from the file. The index of the line to read is passed as an argument.
//  *
//  * @param line_idx The index of the line to read.
//  *
//  * @return The line as a string.
// */
// std::string CsvReader::readLine(const int line_idx) {
//   // Check if file is open
//   if (!file_stream_.is_open()) {
//     throw std::runtime_error("File is not open: " + file_name_);
//   }

//   std::string line;
//   // Seek to the beginning of the file
//   file_stream_.seekg(0, std::ios::beg);

//   // Read lines until the desired index
//   for (int i = 0; i < line_idx; ++i) {
//     if (!std::getline(file_stream_, line)) {
//       // Throw exception if the index is out of range
//       throw std::runtime_error("Could not read line " +
//                                std::to_string(line_idx) +
//                                " from file: " + file_name_);
//     }
//   }

//   // Read the line at the desired index
//   if (!std::getline(file_stream_, line)) {
//     // Throw expcetion if the index is out of range
//     throw std::runtime_error("Could not read line " + std::to_string(line_idx) +
//                              " from file: " + file_name_);
//   }

//   return line;
// }

/**
 * @brief Read a line from the file and store the data in an Eigen vector.
 * 
 * This function reads a line from the file and stores the data in an Eigen vector.
 * The index of the line to read is passed as an argument.
 * 
 * @param line_idx The index of the line to read.
 * 
 * @return The data from the line as an Eigen vector.
*/
Eigen::VectorXd CsvReader::readLineAsVector(const int line_idx) {
  // Read line
  // std::string line = readLine(line_idx);
  // Read line from vector
  std::string line = file_lines_vector_.at(line_idx);

  // Split line
  std::vector<std::string> line_split;
  std::string token;
  std::istringstream token_stream(line);

  // Split line by comma and store in vector
  while (std::getline(token_stream, token, ',')) {
    line_split.push_back(token);
  }

  // Define vector
  Eigen::VectorXd line_vector(line_split.size());

  // Convert string to doubles and store in Eigen vector
  for (size_t i = 0; i < line_split.size(); i++) {
    line_vector(i) = std::stod(line_split[i]);
  }

  return line_vector;
}

/**
 * @brief Store all lines of a CSV file in a vector.
 * 
 * This function stores all the CSVlines of a CSV file in a vector.
 * The lines are stored as strings.
 * 
 * @param[in] None
 * 
 * @return None
*/
void CsvReader::storeFileToStdVector() {
  // Check if file is open
  if (!file_stream_.is_open()) {
    throw std::runtime_error("File is not open: " + file_name_);
  }

  // Temporary string to store lines
  std::string line;

  // Store lines of file in vector
  while (std::getline(file_stream_, line)) {
    file_lines_vector_.push_back(line);
  }
}

/**
 * @brief Close the file stream.
 * 
 * This function closes the file stream. It also resets the file line count.
 * 
 * @param[in] None
 * 
 * @return None
*/
void CsvReader::closeFile() {
  // Close file
  if (file_stream_.is_open()) {
    file_stream_.close();
  }

  // Reset file line count
  file_line_count_ = 0;

  // Reset file lines
  file_lines_vector_.clear();
}

}  // namespace utilities