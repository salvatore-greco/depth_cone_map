#include "depth_cone_map/DataAssociator.hpp"
#include <vector>
#include "depth_cone_map/Cone.hpp"

DataAssociator::DataAssociator(): cones_vector_size(0){
   faiss_indexes.emplace(ConeColor::BLUE, IndexWithId());
   faiss_indexes.emplace(ConeColor::YELLOW, IndexWithId());
   faiss_indexes.emplace(ConeColor::ORANGE, IndexWithId());
   faiss_indexes.emplace(ConeColor::LARGE_ORANGE, IndexWithId());
}


void DataAssociator::updateIndex(const std::vector<Cone>& cones){
    std::vector<cv::Point3f> faiss_blue_cone;
    std::vector<cv::Point3f> faiss_yellow_cone;
    std::vector<cv::Point3f> faiss_orange_cone;
    std::vector<cv::Point3f> faiss_large_orange_cone;
    std::vector<faiss::idx_t> blue_idx;
    std::vector<faiss::idx_t> yellow_idx;
    std::vector<faiss::idx_t> orange_idx;
    std::vector<faiss::idx_t> large_orange_idx;
    for(size_t i = this->cones_vector_size; i<cones.size(); i++){
        switch(cones[i].color){
            case ConeColor::BLUE:
            faiss_blue_cone.push_back(cones[i].position_world_frame);
            blue_idx.push_back(cones[i].id);
            break;
            case ConeColor::UNKNOWN:
            break;
            case ConeColor::YELLOW:
            faiss_yellow_cone.push_back(cones[i].position_world_frame);
            yellow_idx.push_back(cones[i].id);
            break;
            case ConeColor::ORANGE:
            faiss_orange_cone.push_back(cones[i].position_world_frame);
            orange_idx.push_back(cones[i].id);
            break;
            case ConeColor::LARGE_ORANGE:
            faiss_large_orange_cone.push_back(cones[i].position_world_frame);
            large_orange_idx.push_back(cones[i].id);
            break;
        }
    }

    if (!faiss_blue_cone.empty()) faiss_indexes.at(ConeColor::BLUE).id.add_with_ids(faiss_blue_cone.size(), reinterpret_cast<float*>(faiss_blue_cone.data()), blue_idx.data());
    if(!faiss_yellow_cone.empty()) faiss_indexes.at(ConeColor::YELLOW).id.add_with_ids(faiss_yellow_cone.size(), reinterpret_cast<float*>(faiss_yellow_cone.data()), yellow_idx.data());
    if(!faiss_orange_cone.empty()) faiss_indexes.at(ConeColor::ORANGE).id.add_with_ids(faiss_orange_cone.size(), reinterpret_cast<float*>(faiss_orange_cone.data()), orange_idx.data());
    if(!faiss_large_orange_cone.empty()) faiss_indexes.at(ConeColor::LARGE_ORANGE).id.add_with_ids(faiss_large_orange_cone.size(), reinterpret_cast<float*>(faiss_large_orange_cone.data()), large_orange_idx.data());
    this->cones_vector_size = cones.size();
}


faiss::idx_t DataAssociator::searchNearestCone(const Cone& cone){
    faiss::idx_t I; // Indice
    float D; // distanza
    float query_pt[3] = {cone.position_world_frame.x, cone.position_world_frame.y, cone.position_world_frame.z};
    faiss_indexes.at(cone.color).id.search(1, query_pt, 1, &D, &I);
    return I;

}
