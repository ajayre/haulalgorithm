#ifndef HAUL_KDTREE_H
#define HAUL_KDTREE_H

#ifdef _WIN32
#if defined(HAUL_KDTREE_BUILD)
#define HAUL_KDTREE_API __declspec(dllexport)
#else
#define HAUL_KDTREE_API __declspec(dllimport)
#endif
#else
#define HAUL_KDTREE_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct HaulKdTree HaulKdTree;

/** Build a 2D KD-tree over interleaved coords [x0,y0,x1,y1,...]. Copies coords if copy_coords != 0. */
HAUL_KDTREE_API HaulKdTree* haul_kdtree_build(int n_points, const double* coords, int copy_coords);

HAUL_KDTREE_API void haul_kdtree_free(HaulKdTree* tree);

/** k-nearest neighbors from (x,y). Returns count written (<= k). Distances are squared Euclidean. */
HAUL_KDTREE_API int haul_kdtree_query_knn(
    HaulKdTree* tree,
    double x,
    double y,
    int k,
    int* out_indices,
    double* out_dist_sq);

/** Batch k=1 queries. query_coords length 2*n_query. */
HAUL_KDTREE_API void haul_kdtree_query_batch_k1(
    HaulKdTree* tree,
    int n_query,
    const double* query_coords,
    int* out_indices,
    double* out_dist_sq);

#ifdef __cplusplus
}
#endif

#endif
