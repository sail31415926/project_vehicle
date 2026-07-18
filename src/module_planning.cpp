// ==== module_planning.cpp ====
#include "module_planning.h"
#include <fstream>
#include <sstream>
#include <iostream>

std::vector<ReferencePoint> module_planning(const std::string& csv_file) {
    std::vector<ReferencePoint> ref_path;
    std::ifstream file(csv_file);
    if (!file.is_open()) {
        std::cerr << "致命错误: 无法打开参考轨迹文件 " << csv_file << std::endl;
        return ref_path;
    }
    std::string line, word;
    std::getline(file, line); // 跳过表头
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::vector<double> row;
        while (std::getline(ss, word, ',')) {
            row.push_back(std::stod(word));
        }
        if (row.size() >= 5) {
            ref_path.emplace_back(row[0], row[1], row[2], row[3], row[4]);
        }
    }
    file.close();
    std::cout << "成功读取贝塞尔规划轨迹，共 " << ref_path.size() << " 个点。" << std::endl;
    return ref_path;
}