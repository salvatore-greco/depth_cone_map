#ifndef CONEADAPTOR_HPP
#define CONEADAPTOR_HPP

#include "Cone.hpp"
#include <map>
#include <vector>
// nanoflann kdtree è solo una mappa di indici. I coni vivono in un altra struttura dati.
// source: https://jlblancoc.github.io/nanoflann/classnanoflann_1_1KDTreeSingleIndexDynamicAdaptor__.html
class ConeAdaptor{
   public:

    ConeAdaptor(std::vector<Cone>& cones): cones(cones){};
    // Must return the number of data points
    inline size_t kdtree_get_point_count() const {
        return cones.size();
    }

    // Must return the dim'th component of the idx'th point in the class:
    // ritorna x,y,z del cono (a seconda del valore di dim) dell'idx cono nella struttura dati
    // sono id progressivi, i coni non verranno mai rimossi
    inline float kdtree_get_pt(const size_t idx, int dim) const {
        if(dim == 0) return cones[idx].position_world_frame.x;
        if (dim == 1) return cones[idx].position_world_frame.y;
        return cones[idx].position_world_frame.z;

    }

    // unused
    template <class BBOX>
    bool kdtree_get_bbox(BBOX &bb) const {
            return false;
    }

    private:
        std::vector<Cone>& cones;
};

#endif
