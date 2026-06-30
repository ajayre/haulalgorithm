#include "haul_kdtree.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <nanoflann.hpp>

struct HaulKdTreeImpl
{
    std::vector<double> OwnedCoords;
    const double* Coords;
    std::size_t Count;

    struct PointCloud
    {
        const double* Coords;
        std::size_t Count;

        inline std::size_t kdtree_get_point_count() const
        {
            return Count;
        }

        inline double kdtree_get_pt(const std::size_t idx, const std::size_t dim) const
        {
            return Coords[(idx * 2) + dim];
        }

        template <class Bbox>
        bool kdtree_get_bbox(Bbox& /*bbox*/) const
        {
            return false;
        }
    };

    PointCloud Cloud;
    using KdTree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, PointCloud>,
        PointCloud,
        2>;
    std::unique_ptr<KdTree> Tree;
};

static int QueryKnn
    (
    HaulKdTreeImpl* tree,
    double x,
    double y,
    int k,
    int* out_indices,
    double* out_dist_sq
    )
{
    if ((tree == nullptr) || (k <= 0) || (out_indices == nullptr) || (out_dist_sq == nullptr))
    {
        return 0;
    }

    k = std::min(k, static_cast<int>(tree->Count));
    std::vector<std::size_t> Indices(static_cast<std::size_t>(k));
    std::vector<double> Dists(static_cast<std::size_t>(k));
    double Query[2] = { x, y };
    nanoflann::KNNResultSet<double> Result(static_cast<std::size_t>(k));
    Result.init(Indices.data(), Dists.data());
    tree->Tree->findNeighbors(Result, Query, nanoflann::SearchParameters());
    int Found = static_cast<int>(Result.size());
    for (int Iter = 0; Iter < Found; Iter += 1)
    {
        out_indices[Iter] = static_cast<int>(Indices[static_cast<std::size_t>(Iter)]);
        out_dist_sq[Iter] = Dists[static_cast<std::size_t>(Iter)];
    }

    return Found;
}

extern "C"
{

HaulKdTree* haul_kdtree_build
    (
    int n_points,
    const double* coords,
    int copy_coords
    )
{
    if ((n_points <= 0) || (coords == nullptr))
    {
        return nullptr;
    }

    HaulKdTreeImpl* Handle = new HaulKdTreeImpl();
    Handle->Count = static_cast<std::size_t>(n_points);
    if (copy_coords != 0)
    {
        Handle->OwnedCoords.assign(coords, coords + (static_cast<std::size_t>(n_points) * 2));
        Handle->Coords = Handle->OwnedCoords.data();
    }
    else
    {
        Handle->Coords = coords;
    }

    Handle->Cloud.Coords = Handle->Coords;
    Handle->Cloud.Count = Handle->Count;
    Handle->Tree = std::make_unique<HaulKdTreeImpl::KdTree>(2, Handle->Cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    Handle->Tree->buildIndex();
    return reinterpret_cast<HaulKdTree*>(Handle);
}

void haul_kdtree_free
    (
    HaulKdTree* tree
    )
{
    delete reinterpret_cast<HaulKdTreeImpl*>(tree);
}

int haul_kdtree_query_knn
    (
    HaulKdTree* tree,
    double x,
    double y,
    int k,
    int* out_indices,
    double* out_dist_sq
    )
{
    return QueryKnn(reinterpret_cast<HaulKdTreeImpl*>(tree), x, y, k, out_indices, out_dist_sq);
}

void haul_kdtree_query_batch_k1
    (
    HaulKdTree* tree,
    int n_query,
    const double* query_coords,
    int* out_indices,
    double* out_dist_sq
    )
{
    if ((tree == nullptr) || (n_query <= 0) || (query_coords == nullptr) || (out_indices == nullptr) || (out_dist_sq == nullptr))
    {
        return;
    }

    HaulKdTreeImpl* Handle = reinterpret_cast<HaulKdTreeImpl*>(tree);
    for (int Query = 0; Query < n_query; Query += 1)
    {
        double X = query_coords[Query * 2];
        double Y = query_coords[(Query * 2) + 1];
        QueryKnn(Handle, X, Y, 1, &out_indices[Query], &out_dist_sq[Query]);
    }
}

}
