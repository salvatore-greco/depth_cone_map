#ifndef DATAASSOCIATOR_HPP
#define DATAASSOCIATOR_HPP

#include <faiss/IndexFlat.h>
#include <faiss/IndexIDMap.h>
#include <faiss/MetricType.h>
#include "Cone.hpp"
#include <memory>
#include <unordered_map>
#include <vector>

struct IndexWithId{
    IndexWithId(): index(std::make_unique<faiss::IndexFlatL2>(3)), id(index.get()){};

    IndexWithId(const IndexWithId&) = delete;
    IndexWithId& operator=(const IndexWithId&) = delete;

    IndexWithId(IndexWithId&&) noexcept = default;
    IndexWithId& operator=(IndexWithId&&) noexcept = default;

    std::unique_ptr<faiss::IndexFlatL2> index;
    faiss::IndexIDMap id;
};

class DataAssociator {

    public:
        DataAssociator();
        void buildIndex(const std::vector<Cone>& cones);
        faiss::idx_t searchNearestCone(const Cone& cone);
    private:
        std::unordered_map<ConeColor, IndexWithId> faiss_indexes;
};


#endif
