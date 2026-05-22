#include "depth_cone_map/DataAssociator.hpp"
#include <vector>
#include "depth_cone_map/Cone.hpp"

DataAssociator::DataAssociator(){
   faiss_indexes.emplace(ConeColor::BLUE, IndexWithId());
   faiss_indexes.emplace(ConeColor::YELLOW, IndexWithId());
   faiss_indexes.emplace(ConeColor::ORANGE, IndexWithId());
   faiss_indexes.emplace(ConeColor::LARGE_ORANGE, IndexWithId());
}


void DataAssociator::buildIndex(const std::vector<Cone>& cones){
    std::vector<cv::Point3f> faiss_blue_cone;
    std::vector<cv::Point3f> faiss_yellow_cone;
    std::vector<cv::Point3f> faiss_orange_cone;
    std::vector<cv::Point3f> faiss_large_orange_cone;
    std::vector<faiss::idx_t> blue_idx;
    std::vector<faiss::idx_t> yellow_idx;
    std::vector<faiss::idx_t> orange_idx;
    std::vector<faiss::idx_t> large_orange_idx;
    for (const auto& cone: cones){
        switch(cone.color){
            case ConeColor::BLUE:
            faiss_blue_cone.push_back(cone.position_world_frame);
            blue_idx.push_back(cone.id);
            break;
            case ConeColor::UNKNOWN:
            break;
            case ConeColor::YELLOW:
            faiss_yellow_cone.push_back(cone.position_world_frame);
            yellow_idx.push_back(cone.id);
            break;
            case ConeColor::ORANGE:
            faiss_orange_cone.push_back(cone.position_world_frame);
            orange_idx.push_back(cone.id);
            break;
            case ConeColor::LARGE_ORANGE:
            faiss_large_orange_cone.push_back(cone.position_world_frame);
            large_orange_idx.push_back(cone.id);
            break;
        }
    }
    for(auto& index_struct : faiss_indexes){
        index_struct.second.index->reset();
        index_struct.second.id.reset();
    }

    faiss_indexes.at(ConeColor::BLUE).id.add_with_ids(faiss_blue_cone.size(), reinterpret_cast<float*>(faiss_blue_cone.data()), blue_idx.data());
    faiss_indexes.at(ConeColor::YELLOW).id.add_with_ids(faiss_yellow_cone.size(), reinterpret_cast<float*>(faiss_yellow_cone.data()), yellow_idx.data());
    faiss_indexes.at(ConeColor::ORANGE).id.add_with_ids(faiss_orange_cone.size(), reinterpret_cast<float*>(faiss_orange_cone.data()), orange_idx.data());
    faiss_indexes.at(ConeColor::LARGE_ORANGE).id.add_with_ids(faiss_large_orange_cone.size(), reinterpret_cast<float*>(faiss_large_orange_cone.data()), large_orange_idx.data());
}


faiss::idx_t DataAssociator::searchNearestCone(const Cone& cone){
    faiss::idx_t I; // Indice
    float D; // distanza
    float query_pt[3] = {cone.position_world_frame.x, cone.position_world_frame.y, cone.position_world_frame.z};
    faiss_indexes.at(cone.color).id.search(1, query_pt, 1, &D, &I);
    return I;

}
