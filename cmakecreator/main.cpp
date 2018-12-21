#include <iostream>
#include <fstream>
#include <unistd.h>
#include <boost/process.hpp>

std::string cmake_version();

int main(int argc, char **argv)
{
  int ch;
  std::string project_name = "project_name";
  std::string program_name = "${PROJECT_NAME}";
  while ((ch = getopt(argc, argv, "hp:e:")) != -1)
  {
    switch (ch)
    {
    case 'h':
      std::cout << "-h help" << std::endl;
      std::cout << "-p project" << "    set project name" << std::endl;
      std::cout << "-e name" << "    set program name" << std::endl;
      return 0;
    case 'p':
      project_name = optarg;
      break;
    case 'e':
      program_name = optarg;
      break;
    default:
      break;
    }
  }

  std::ofstream ofs("CMakeLists.txt", std::ofstream::out);

  ofs << cmake_version() << std::endl << std::endl;

  ofs << "project(" << project_name << ")" << std::endl << std::endl;

  ofs << "set (CMAKE_CXX_FLAGS \"${CMAKE_CXX_FLAGS} -std=c++11\")" << std::endl << std::endl;

  ofs << "#include_directories(/usr/include)" << std::endl;
  ofs << "#link_directories(/usr/lib)" << std::endl << std::endl;

  ofs << "add_executable(" + program_name + " main.cpp)" << std::endl;
  ofs << "#target_sources(" + program_name + " PRIVATE test.cpp)" << std::endl;
  ofs << "#target_link_libraries(" + program_name + " rt)" << std::endl;
  ofs << "#target_link_libraries(" + program_name + " pthread)" << std::endl;

  ofs.close();

  return 0;
}

std::string cmake_version()
{
  boost::process::ipstream pipe_stream;
  boost::process::child c("cmake --version", boost::process::std_out > pipe_stream);
  std::string line;
  while (pipe_stream && std::getline(pipe_stream, line) && !line.empty())
    break;

  const std::string sub = "version";
  std::size_t found = line.find(sub);
  if (found != std::string::npos)
  {
    line = line.substr(found + sub.length() + 1);
  }
  if (line[0] == '2')
    line = line.substr(0, 3);
  else
    line = "3.0";

  return std::string("cmake_minimum_required(VERSION ") + line + ")";
}
