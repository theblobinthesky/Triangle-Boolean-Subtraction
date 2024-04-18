#include "occl_cull.h"
#include <glm/vec2.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/norm.hpp>
#include <cmath>
#include <string.h>
#include <queue>

#define DEBUG false

#if DEBUG
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

inline bool bbox_intersect(const BBox& a, const BBox& b) {
    return !(b.tl.x > a.br.x || b.br.x < a.tl.x
                || b.tl.y > a.br.y || b.br.y < a.tl.y);
}

std::string to_string(const glm::vec2& v) {
    char str[256];
    // See https://stackoverflow.com/a/21162120.
    sprintf(str, "{%.9g, %.9g}", v.x, v.y);

    return str;
}

std::string to_string(const triangle& tri) {
    return "{" + to_string(tri.pts[0]) + ", " + to_string(tri.pts[1]) + ", " + to_string(tri.pts[2]) + "}";
}

void print(const std::vector<triangle>& tris) {
    printf("{");

    for (size_t i = 0; i < tris.size(); i++) {
        const triangle& t = tris[i];
        printf("{%s}", to_string(t).c_str());
        
        if (i != tris.size() - 1) printf(", ");
    }

    printf("}\n");
}

void print(const std::vector<glm::vec2>& pts) {
    printf("{");

    for (size_t i = 0; i < pts.size(); i++) {
        const glm::vec2& p = pts[i];
        printf("%s", to_string(p).c_str());
        
        if (i != pts.size() - 1) printf(", ");
    }

    printf("}\n");
}

struct line {
    glm::vec2 pts[2];
};

std::string to_string(const line& line) {
    return glm::to_string(line.pts[0]) + ", " + glm::to_string(line.pts[1]);    
}

glm::vec2 orth(const glm::vec2& v) {
    return glm::vec2(v.y, -v.x);
}

int minuend_side(int i) {
    return i / 3;
} 

int subtr_side(int i) {
    return i % 3;
}

int tri_side(int t, int i) {
    return (t == 0) ? minuend_side(i) : subtr_side(i);
}

f32 signed_tri_height(const glm::vec2& side, const glm::vec2& ground) {
    f32 dot = glm::dot(side, orth(ground));
    f32 ground_len = glm::length(ground);

    if (f32_eq(ground_len, 0.0f)) {
        return 0.0f;
    } else {
        return dot / ground_len;
    }
}

f32 tri_area(const triangle& tri) {
    // See https://de.wikipedia.org/wiki/Dreiecksfl%C3%A4che#Mit_Koordinaten_in_der_Ebene
    glm::mat2 m;
    m[0] = tri.pts[1] - tri.pts[0];
    m[1] = tri.pts[2] - tri.pts[0];
    return 0.5f * std::abs(glm::determinant(m));
}

f32 tri_min_height_to_ground_ratio(const triangle& tri) {
    f32 min_ratio = 999999.0f;

    for (int i = 0; i < 3; i++) {
        glm::vec2 ground = tri.pts[(i + 1) % 3] - tri.pts[i];
        f32 ground_len = glm::length(ground);
        f32 height = std::abs(signed_tri_height(tri.pts[(i + 2) % 3] - tri.pts[i], ground));
        min_ratio = std::min(min_ratio, height / ground_len);        
    }

    return min_ratio;
}

// TODO: Move up.
void max_with_idx(f32 contender, int contender_idx, f32& current, int &current_idx) {
    if (contender > current) {
        current = contender;
        current_idx = contender_idx;
    }
}

f32 tri_proximity_to(const triangle& tri, const glm::vec2& pt) {
    // bool inside = heights[0] <= 0 && heights[1] <= 0 && heights[2] <= 0;
    // In case (inside == true), we would want the minimum height to get the closest point-
    // In case (inside == false), we would like the maximum height to get the furthest away point.
    // So both cases can be handled using the same loop.

    f32 max = -99999.0f;

    for (int i = 0; i < 3; i++) {
        f32 height = signed_tri_height(pt - tri.pts[i], tri.pts[(i + 1) % 3] - tri.pts[i]);
        max = std::max(height, max);
    }

    return max;
}

inline bool tri_inside(f32 proximity, f32 e = 1e-4) {
    return proximity < e;
}

triangle triangle_align_corners(const triangle& a, const triangle& b) {
    f32 min_value = 9999999.0f;
    int off;

    for (int i = 0; i < 3; i++) {
        f32 dist2 = glm::distance2(a.pts[0], b.pts[i]);
        
        if (dist2 < min_value) {
            min_value = dist2;
            off = i;
        }
    }

    return {b.pts[off], b.pts[(off + 1) % 3], b.pts[(off + 2) % 3]};
}

void tris_from_cc_quadrilateral(std::vector<triangle>& tris, const quadrilateral&& q) {
    if (tri_inside(tri_proximity_to({q.pts[0], q.pts[1], q.pts[3]}, q.pts[2]))) {
        tris.push_back({{q.pts[0], q.pts[1], q.pts[2]}});
        tris.push_back({{q.pts[0], q.pts[2], q.pts[3]}});
    } else {
        tris.push_back({{q.pts[0], q.pts[1], q.pts[3]}});
        tris.push_back({{q.pts[1], q.pts[2], q.pts[3]}});
    }
}

bool tri_is_winding_cc(const triangle& tri) {
    f32 z = glm::cross(glm::vec3(tri.pts[1] - tri.pts[0], 0), glm::vec3(tri.pts[2] - tri.pts[0], 0)).z;
    return z >= 0;
}

template <size_t N>
void tris_first_to_cc_winding_others_simult(triangle& tri, line* lines[N]) {
    if (!tri_is_winding_cc(tri)) {
        tri = { tri.pts[0], tri.pts[2], tri.pts[1] };

        for (size_t i = 0; i < N; i++) {
            line *l = lines[i];
            *l = { l->pts[1], l->pts[0] };
        }
    }
}

inline glm::vec2 tri_get_inters_point(const glm::vec2 pts[], f32 fac_arr[], int fac_idx) {
    int side = minuend_side(fac_idx);
    f32 fac = fac_arr[fac_idx];
    return pts[side] + fac * (pts[(side + 1) % 3] - pts[side]);
}

inline line tri_next_two_inters_points(const glm::vec2 pts[], f32 fac_arr[], int indices[2]) {
    return {
        tri_get_inters_point(pts, fac_arr, indices[0]),
        tri_get_inters_point(pts, fac_arr, indices[1])
    };
}

int tri_get_rem_index(int indices[2]) {
    return 3 - indices[0] - indices[1];
}

int tri_get_common_point_of_sides(int side0, int side1) {
    constexpr int common_point_table[3][3] = {
        {0, 1, 0},
        {1, 1, 2},
        {0, 2, 2}
    };

    return common_point_table[side0][side1];
}

triangle tri_align_as_common_side0_side1(const triangle& tri, int side0, int side1) {
    int common = tri_get_common_point_of_sides(side0, side1);
    int other0 = (side0 == common) ? (side0 + 1) % 3 : side0;
    int other1 = (side1 == common) ? (side1 + 1) % 3 : side1;

    return {tri.pts[common], tri.pts[other0], tri.pts[other1]};
}

triangle tri_align_as_other_fac0_fac1(const triangle& tri, int side, f32 fac0, f32 fac1) {
    int indices[2];
    if (fac0 <= fac1) {
        indices[0] = side;
        indices[1] = (side + 1) % 3;
    } else {
        indices[0] = (side + 1) % 3;
        indices[1] = side;
    }

    int other = tri_get_rem_index(indices);
    return { tri.pts[other], tri.pts[indices[0]], tri.pts[indices[1]] };
}

void get_near_and_far_minuend_4inters(f32 fac_arr[], int inters_indices[], int indices[]) {
    int side0 = minuend_side(inters_indices[0]);
    int common_point = tri_get_common_point_of_sides(side0, minuend_side(inters_indices[2]));

    // Swap near and far indices if necessary.
    int off = 0;
    if ((side0 != common_point) != (fac_arr[inters_indices[0]] > fac_arr[inters_indices[1]])) {
        off = 2;
    }

    indices[(0 + off) % 4] = inters_indices[0];
    indices[(1 + off) % 4] = inters_indices[2];
    indices[(2 + off) % 4] = inters_indices[1];
    indices[(3 + off) % 4] = inters_indices[3];
}

constexpr uint fac_arr_size = 9;

void normalize_wrt_norm_line(const glm::vec2& p0, const glm::vec2& p1, 
    const glm::vec2& norm_line_p, const glm::vec2& norm_line_dir, f32 normalized[2]) {
    f32 len2_dir = glm::length2(norm_line_dir);
    normalized[0] = glm::dot(p0 - norm_line_p, norm_line_dir) / len2_dir;
    normalized[1] = glm::dot(p1 - norm_line_p, norm_line_dir) / len2_dir;
}

void line_get_inters_factor(const glm::vec2& p1, const glm::vec2& d1, const glm::vec2& p2, const glm::vec2& d2, f32 inters_facs[2]) {
    glm::mat2 m;
    m[0] = d1;
    m[1] = -d2;

    f32 len_mul = glm::length(d1) * glm::length(d2); // Very large and very small d1, d2 vectors must be accounted for.
    if (f32_eq(glm::determinant(m), 0.0f, len_mul * 1e-3)) {
        inters_facs[0] = inters_facs[1] = F32_NAN;
    } else {
        glm::vec2 fac = glm::inverse(m) * (p2 - p1);
        
        f32 e = 1e-4;
        if (fac.x < -e || fac.x > 1 + e || fac.y < -e || fac.y > 1 + e) {
            inters_facs[0] = inters_facs[1] = F32_NAN;
        } else {
            inters_facs[0] = fac.x;
            inters_facs[1] = fac.y;
        }
    }
}

void tris_get_inters(const triangle& minuend, const triangle& subtr, f32 fac_arr[], int inters_indices[], int& inters_count) {
    glm::vec2 minuend_sides[3];
    glm::vec2 subtr_sides[3];
    
    for (int i = 0; i < 3; i++) {
        minuend_sides[i] = minuend.pts[(i + 1) % 3] - minuend.pts[i];
        subtr_sides[i] = subtr.pts[(i + 1) % 3] - subtr.pts[i];
    }

    f32 full_fac_arr[fac_arr_size][2];
    for (int i = 0; i < 9; i++) {
        int m = minuend_side(i);;
        int s = subtr_side(i);

        line_get_inters_factor(minuend.pts[m], minuend_sides[m], subtr.pts[s], subtr_sides[s], full_fac_arr[i]);
    }

    // TODO: This could be one large loop!
    // Remove double intersections on the subtrahend. Double inters. on the minuend cannot be removed!
    for (int i = 0; i < (int)fac_arr_size; i++) {
        int n = 3 * minuend_side(i) + ((subtr_side(i) + 1) % 3);

        // It's important to ONLY delete 0.0 if its next to 1.0.
        if (f32_eq(full_fac_arr[i][1], 1.0f) && f32_eq(full_fac_arr[n][1], 0.0f)) {
            full_fac_arr[n][0] = F32_NAN;
        }
    }

    // If the minuend has no double intersection where there should be one, add one.
    for (int i = 0; i < (int)fac_arr_size; i++) {
        int other_m;
        f32 fac;

        if (f32_eq(full_fac_arr[i][0], 0)) {
            other_m = (minuend_side(i) + 2) % 3;
            fac = 1;
        } else if (f32_eq(full_fac_arr[i][0], 1)) {
            other_m = (minuend_side(i) + 1) % 3;
            fac = 0;
        } else {
            continue;
        }

        // If the intersection already exists, don't add it twice!
        bool test = true;
        for (int s = 0; s < 3; s++) {
            int other = 3 * other_m + s;
            if (f32_eq(full_fac_arr[other][0], fac)) {
                test = false;
                break;
            }
        }

        if (test) {
            int other = 3 * other_m + subtr_side(i);
            full_fac_arr[other][0] = fac;
            full_fac_arr[other][1] = full_fac_arr[i][1];
        }
    }

    // Copy over. TODO: Speed
    for (size_t i = 0; i < fac_arr_size; i++) {
        fac_arr[i] = full_fac_arr[i][0];
    }

    DEBUG_PRINT("fac_arr: ");
    for (int i = 0; i < 9; i++) {
        DEBUG_PRINT("[%d] %f, ", i, fac_arr[i]);
    }
    DEBUG_PRINT("\n");

    for (size_t i = 0; i < fac_arr_size; i++) {
        f32 fac = fac_arr[i];

        if (!std::isnan(fac)) {
            inters_indices[inters_count++] = i;
        }
    }
}

template<int(*FUNC)(int)>
void get_side_to_inters(int side_inters[3][2], int side_icount[3], int inters_indices[], int inters_count) {
    for (int i = 0; i < inters_count; i++) {
        int t = FUNC(inters_indices[i]);
        side_inters[t][side_icount[t]++] = inters_indices[i];
    }
}

#define PROX_INSIDE 0
#define PROX_OUTSIDE 1

// Returns whether the start point was chosen confidently.
bool tri_choose_start_point(const triangle& tri, const triangle& ref, int &start_pt, int &start_flag) {
    // Calculate the distance if the tri point is inside or outside the ref triangle.
    // Then choose the larger one.

    f32 inner_prox = -999999.0f, outer_prox = -999999.0f;
    int inner_pt = -1, outer_pt = -1;
    
    for (int i = 0; i < 3; i++) {
        f32 prox = tri_proximity_to(ref, tri.pts[i]);

        if (prox <= 0) {
            max_with_idx(-prox, i, inner_prox, inner_pt);
        }

        if (prox >= 0) {
            max_with_idx(prox, i, outer_prox, outer_pt);
        }
    }

    if (inner_prox < outer_prox) {
        start_pt = outer_pt;
        start_flag = PROX_OUTSIDE;
    } else {
        start_pt = inner_pt;
        start_flag = PROX_INSIDE;
    }

    f32 e = 1e-6;
    return std::max(inner_prox, outer_prox) >= e;
}

template<bool FORCE>
bool sort_with_max_2_elements(int i, f32 fac_arr[], int side_inters[3][2], int side_icount[3]) {
    assert(side_icount[i] <= 2);

    if (side_icount[i] == 2) {
        int *inters = side_inters[i];
        if (FORCE || fac_arr[inters[0]] > fac_arr[inters[1]]) {
            int tmp = inters[0];
            inters[0] = inters[1];
            inters[1] = tmp;
            
            return true;
        }
    }

    return false;
}

void get_mll_inters_and_walk_minuend(const triangle& minuend, const triangle& subtr, f32 fac_arr[fac_arr_size], int inters_indices[], int& inters_count, 
    int mll_inters_indices[], int& mll_inters_count, int minuend_outside_indices[3], int& minuend_outside_count, int side_inters[3][2], int side_icount[3]) {
    get_side_to_inters<minuend_side>(side_inters, side_icount, inters_indices, inters_count);

    int init_pt, init_flag;
    if (!tri_choose_start_point(minuend, subtr, init_pt, init_flag)) {
        // Assume all minuend points are inside if there is certain start point.
        return;
    }

    // Compute the most likely inters for each side based on the intersections and sampled points between intersections.
    int pt_prox_flag = init_flag;
    int mll_side_inters[3][2] = {};
    int mll_side_icount[3] = {};

    for (int i = 0; i < 3; i++) {
        int pt = (init_pt + i) % 3;

        DEBUG_PRINT("side_inters: ");
        for (int i = 0; i < side_icount[pt]; i++) {
            DEBUG_PRINT("%s, ", to_string(tri_get_inters_point(minuend.pts, fac_arr, side_inters[pt][i])).c_str()); 
        }
        DEBUG_PRINT("\n");

        // Sort the intersections on the minuend sides.
        bool was_sorted = sort_with_max_2_elements<false>(pt, fac_arr, side_inters, side_icount);

        int icount = side_icount[pt];
        constexpr int MAX_INTERS_PER_SIDE = 2;
        f32 prox_samples[MAX_INTERS_PER_SIDE];

        if (icount > 0) {
            glm::vec2 inters_pts[3];
            for (int i = 0; i < icount; i++) {
                int inters_idx = side_inters[pt][i];
                inters_pts[i] = tri_get_inters_point(minuend.pts, fac_arr, inters_idx);
            }

            // Sample points exactly in the middle between the intersection points 
            // and figure out, what their proximities are wrt. the subtrahend.
            DEBUG_PRINT("sample_pt: ");

            for (int i = 0; i < icount - 1; i++) {
                glm::vec2 sample_pt = 0.5f * (inters_pts[i] + inters_pts[i + 1]);
                prox_samples[i] = tri_proximity_to(subtr, sample_pt);    
                DEBUG_PRINT("%s, ", to_string(sample_pt).c_str());
            }

            // The last sample is between the last intersection point and the line end.
            glm::vec2 sample_pt = 0.5f * (inters_pts[icount - 1] + minuend.pts[(pt + 1) % 3]);
            prox_samples[icount - 1] = tri_proximity_to(subtr, sample_pt);
            DEBUG_PRINT("%s\n", to_string(sample_pt).c_str());
        }

        // Now calculate the most likely proximities based on the samples' proximities.
        int prox_prev_known = pt_prox_flag;
        for (int i = 0; i < icount; i++) {
            f32 prox_sample = prox_samples[i];
            int prox_after_inters;

            f32 e = 1e-7;
            if (prox_prev_known == PROX_INSIDE) {
                prox_after_inters = (prox_sample >= e) ? PROX_OUTSIDE : PROX_INSIDE;
            } else {
                prox_after_inters = (prox_sample <= -e) ? PROX_INSIDE : PROX_OUTSIDE;
            }
            
            if (prox_prev_known != prox_after_inters) {
                // The proximities changed. Therefore the intersection causes the minuend to enter/exit the subtrahend.
                // We must keep it!
                DEBUG_PRINT("proximity changed. prox_prev_known == %d, prox_after_inters == %d, prox_sample == %.9g\n", 
                    prox_prev_known, prox_after_inters, prox_sample);

                int mll_icount = mll_side_icount[pt]++;
                mll_side_inters[pt][mll_icount] = side_inters[pt][i];
            }

            prox_prev_known = prox_after_inters;
        }

        if (mll_side_icount[pt] == 2 && was_sorted) {
            // Sort back if necessary.
            DEBUG_PRINT("Sorted back! side == %d\n", pt);
            sort_with_max_2_elements<true>(pt, fac_arr, mll_side_inters, mll_side_icount);
        }

        // Walk the minuend and store the outside indices.
        if (pt_prox_flag == PROX_OUTSIDE) {
            minuend_outside_indices[minuend_outside_count++] = pt;
        }

        pt_prox_flag = (pt_prox_flag + mll_side_icount[pt]) % 2;
    }

    // Calculate most likely intersection indices.
    mll_inters_count = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < mll_side_icount[i]; j++) {
            mll_inters_indices[mll_inters_count++] = mll_side_inters[i][j];
        }
    }

    DEBUG_PRINT("mll_inters_count == %d\n", mll_inters_count);
}

void walk_subtr(const triangle& minuend, const triangle& subtr, int inters_indices[], int inters_count, 
    int subtr_inside_indices[3], int& subtr_inside_count, int side_inters[3][2], int side_icount[3]) {
    get_side_to_inters<subtr_side>(side_inters, side_icount, inters_indices, inters_count);

    int init_pt, prox_flag;
    tri_choose_start_point(subtr, minuend, init_pt, prox_flag);

    for (int i = 0; i < 3; i++) {
        int pt = (i + init_pt) % 3;

        if (prox_flag == PROX_INSIDE) {
            subtr_inside_indices[subtr_inside_count++] = pt;
        }

        prox_flag = (prox_flag + side_icount[pt]) % 2;
    }
}

void internal_subtract_triangles(const triangle& minuend, const triangle& subtr, std::vector<triangle>& tris) {
    if (!tri_is_winding_cc(minuend) || !tri_is_winding_cc(subtr)) {
        DEBUG_PRINT("Invalid winding as input. %d, %d\n", tri_is_winding_cc(minuend), tri_is_winding_cc(subtr));
        return;
    }

    // Other cases begin here.
    f32 fac_arr[fac_arr_size];
    int raw_inters_count = 0;
    int raw_inters_indices[fac_arr_size] = {};
    tris_get_inters(minuend, subtr, fac_arr, raw_inters_indices, raw_inters_count);

#if DEBUG
    DEBUG_PRINT("raw_inters_pts: ");
    for (int i = 0; i < raw_inters_count; i++) {
        DEBUG_PRINT("%s, ", to_string(tri_get_inters_point(minuend.pts, fac_arr, raw_inters_indices[i])).c_str());
    }
    DEBUG_PRINT("\n");
#endif

    int minuend_outside_indices[3] = {};
    int minuend_outside_count = 0;
    int inters_count = 0;
    int inters_indices[fac_arr_size] = {};
    int minuend_side_inters[3][2] = {};
    int minuend_side_icount[3] = {};
    get_mll_inters_and_walk_minuend(minuend, subtr, fac_arr, raw_inters_indices, raw_inters_count, inters_indices, inters_count, 
        minuend_outside_indices, minuend_outside_count, minuend_side_inters, minuend_side_icount);

    int subtr_inside_indices[3] = {};
    int subtr_inside_count = 0;
    int subtr_side_inters[3][2] = {}; 
    int subtr_side_icount[3] = {};
    walk_subtr(minuend, subtr, inters_indices, inters_count, subtr_inside_indices, subtr_inside_count, subtr_side_inters, subtr_side_icount);

    DEBUG_PRINT("inters_pts: ");
    for (int i = 0; i < inters_count; i++) {
        DEBUG_PRINT("%s, ", to_string(tri_get_inters_point(minuend.pts, fac_arr, inters_indices[i])).c_str());
    }
    DEBUG_PRINT("\n");

    DEBUG_PRINT("minuend_outside_count %d, subtr_inside_count %d, inters_count %d\n", minuend_outside_count, subtr_inside_count, inters_count);

    // assert(inters_count % 2 == 0);
    if (inters_count % 2 == 1) {
        printf("Failed on (dude): {%s}, {%s}\n", to_string(minuend).c_str(), to_string(subtr).c_str());
        return;
    }

    bool minuend_outside = (minuend_outside_count == 3);
    bool subtr_outside = (subtr_inside_count == 0);

    if (minuend_outside_count == 0) {
        // Minuend in subtrahend.
        
        return;
    } else if (subtr_inside_count == 3) {
        // Subtrahend in minuend.
        triangle subtr_aligned = triangle_align_corners(minuend, subtr);

        for (int i = 0; i < 3; i++) {
            tris_from_cc_quadrilateral(tris, {minuend.pts[i], minuend.pts[(i + 1) % 3], subtr_aligned.pts[(i + 1) % 3], subtr_aligned.pts[i]});
        }

        return;
    } else if (inters_count == 0) {
        tris.push_back(minuend);

        return;
    } else if (minuend_outside) {
        if (subtr_outside) {
            if (inters_count == 4) {
                int indices[4];
                get_near_and_far_minuend_4inters(fac_arr, inters_indices, indices);
                
                // Real logic begins here.
                line near_inters = tri_next_two_inters_points(minuend.pts, fac_arr, indices);
                line far_inters = tri_next_two_inters_points(minuend.pts, fac_arr, indices + 2);

                triangle minuend_winded = tri_align_as_common_side0_side1(minuend, minuend_side(indices[0]), minuend_side(indices[1]));

                line *lines[2] = {&near_inters, &far_inters};
                tris_first_to_cc_winding_others_simult<2>(minuend_winded, lines);

                tris.push_back({{minuend_winded.pts[0], near_inters.pts[0], near_inters.pts[1]}});
                tris_from_cc_quadrilateral(tris, {minuend_winded.pts[1], minuend_winded.pts[2], far_inters.pts[1], far_inters.pts[0]});

                return;
            } else if (inters_count == 6) {
                int common_points[3];
                for (int i = 0; i < 3; i++) {
                    common_points[i] = tri_get_common_point_of_sides(minuend_side(subtr_side_inters[i][0]), minuend_side(subtr_side_inters[i][1]));
                }

                for (int i = 0; i < 3; i++) {
                    line inters_line = tri_next_two_inters_points(minuend.pts, fac_arr, subtr_side_inters[i]);
                    triangle tri = {minuend.pts[common_points[i]], inters_line.pts[0], inters_line.pts[1]};
                    // TODO: Speed. This can probably be done once and simultaneously.
                    tris_first_to_cc_winding_others_simult<0>(tri, null);

                    tris.push_back(tri);
                }

                return;
            }
        } else if (subtr_inside_count == 1) {
            if (inters_count == 2) {
                triangle minuend_winded = tri_align_as_other_fac0_fac1(minuend, minuend_side(inters_indices[0]), fac_arr[inters_indices[0]], fac_arr[inters_indices[1]]);
                line inters = tri_next_two_inters_points(minuend.pts, fac_arr, inters_indices);

                line *lines[1] = {&inters};
                tris_first_to_cc_winding_others_simult<1>(minuend_winded, lines);

                glm::vec2 subtr_inside_pt = subtr.pts[subtr_inside_indices[0]];

                tris_from_cc_quadrilateral(tris, {minuend_winded.pts[0], minuend_winded.pts[1], inters.pts[0], subtr_inside_pt});
                tris_from_cc_quadrilateral(tris, {minuend_winded.pts[0], subtr_inside_pt, inters.pts[1], minuend_winded.pts[2]});

                return;
            } else if (inters_count == 4) {
                // TODO: Factor out.
                int indices[4];
                get_near_and_far_minuend_4inters(fac_arr, inters_indices, indices);
                
                // Real logic begins here.
                line near_inters = tri_next_two_inters_points(minuend.pts, fac_arr, indices);
                line far_inters = tri_next_two_inters_points(minuend.pts, fac_arr, indices + 2);

                triangle minuend_winded = tri_align_as_common_side0_side1(minuend, minuend_side(indices[0]), minuend_side(indices[1]));

                const glm::vec2& subtr_inside_pt = subtr.pts[subtr_inside_indices[0]];

                line *lines[2] = {&near_inters, &far_inters};
                tris_first_to_cc_winding_others_simult<2>(minuend_winded, lines);

                tris.push_back({{minuend_winded.pts[0], near_inters.pts[0], near_inters.pts[1]}});
                // TODO: Make use of a general n-gons -> triangles algorithm. 
                tris.push_back({{subtr_inside_pt, far_inters.pts[0], minuend_winded.pts[1]}});
                tris.push_back({{subtr_inside_pt, minuend_winded.pts[1], minuend_winded.pts[2]}});
                tris.push_back({{minuend_winded.pts[2], far_inters.pts[1], subtr_inside_pt}});

                return;
            } else {
                printf("Not a good lawyer 214256!\n");
            }
        } else if (subtr_inside_count == 2) {
            if (inters_count == 2) {
                triangle minuend_winded = tri_align_as_other_fac0_fac1(minuend, minuend_side(inters_indices[0]), fac_arr[inters_indices[0]], fac_arr[inters_indices[1]]);
                triangle subtr_winded = tri_align_as_common_side0_side1(subtr, subtr_side(inters_indices[0]), subtr_side(inters_indices[1]));
                line inters = tri_next_two_inters_points(minuend.pts, fac_arr, inters_indices);

                line subtr_insides = {subtr_winded.pts[1], subtr_winded.pts[2]};
                
                line *lines[2] = {&inters, &subtr_insides};
                tris_first_to_cc_winding_others_simult<2>(minuend_winded, lines);

                tris_from_cc_quadrilateral(tris, {minuend_winded.pts[0], minuend_winded.pts[1], inters.pts[0], subtr_insides.pts[0]});
                tris_from_cc_quadrilateral(tris, {minuend_winded.pts[0], subtr_insides.pts[1], inters.pts[1], minuend_winded.pts[2]});
                tris.push_back({{minuend_winded.pts[0], subtr_insides.pts[0], subtr_insides.pts[1]}});

                return;
            } else {
                printf("Not a good lawyer 987654!\n");
            }
        } else {
            printf("Not a good lawyer 376281!\n");
        }
    } else if (minuend_outside_count == 2) {
        if (subtr_outside) {
            if (inters_count == 2) {
                triangle minuend_winded = tri_align_as_common_side0_side1(minuend, minuend_side(inters_indices[0]), minuend_side(inters_indices[1]));
                line inters = tri_next_two_inters_points(minuend.pts, fac_arr, inters_indices);

                line *lines[1] = {&inters};
                tris_first_to_cc_winding_others_simult<1>(minuend_winded, lines);

                tris_from_cc_quadrilateral(tris, {minuend_winded.pts[1], minuend_winded.pts[2], inters.pts[1], inters.pts[0]});

                return;
            } else if (inters_count == 4) {
                int side[2];
                int side_counter = 0;
                for (int i = 0; i < 3; i++) {
                    if (subtr_side_icount[i] != 0) {
                        side[side_counter++] = i;
                    }
                }

                // Real logic begins here.
                int common_points[2] = {
                    tri_get_common_point_of_sides(minuend_side(subtr_side_inters[side[0]][0]), minuend_side(subtr_side_inters[side[0]][1])),
                    tri_get_common_point_of_sides(minuend_side(subtr_side_inters[side[1]][0]), minuend_side(subtr_side_inters[side[1]][1])),
                };

                line inters_cutoff0 = tri_next_two_inters_points(minuend.pts, fac_arr, subtr_side_inters[side[0]]);
                line inters_cutoff1 = tri_next_two_inters_points(minuend.pts, fac_arr, subtr_side_inters[side[1]]);
                
                triangle minuend_cutoff0 = {minuend.pts[common_points[0]], inters_cutoff0.pts[0], inters_cutoff0.pts[1]};
                tris_first_to_cc_winding_others_simult<0>(minuend_cutoff0, null);

                triangle minuend_cutoff1 = {minuend.pts[common_points[1]], inters_cutoff1.pts[1], inters_cutoff1.pts[0]};
                tris_first_to_cc_winding_others_simult<0>(minuend_cutoff1, null);

                tris.push_back(minuend_cutoff0);
                tris.push_back(minuend_cutoff1);

                return;
            }
        } else if (subtr_inside_count == 1) {
            if (inters_count == 2) {
                triangle minuend_winded = tri_align_as_common_side0_side1(minuend, minuend_side(inters_indices[0]), minuend_side(inters_indices[1]));
                line inters = tri_next_two_inters_points(minuend.pts, fac_arr, inters_indices);

                line *lines[1] = {&inters};
                tris_first_to_cc_winding_others_simult<1>(minuend_winded, lines);

                glm::vec2 subtr_inside_pt = subtr.pts[subtr_inside_indices[0]];

                tris.push_back({minuend_winded.pts[1], subtr_inside_pt, inters.pts[0]});
                tris.push_back({minuend_winded.pts[2], inters.pts[1], subtr_inside_pt});
                tris.push_back({minuend_winded.pts[2], subtr_inside_pt, minuend_winded.pts[1]});

                return;
            } else if (inters_count == 4) {
                // Seperate subtr. side with two inters. and the other two.
                int two_pt_side;
                int other_sides[2];
                int other_sides_count = 0;

                for (int i = 0; i < 3; i++) {
                    if (subtr_side_icount[i] == 2) {
                        two_pt_side = i;
                    } else {
                        assert(subtr_side_icount[i] == 1);
                        other_sides[other_sides_count++] = i;
                    }
                }

                assert(other_sides_count == 2);

                int common_points[2] = {
                    tri_get_common_point_of_sides(minuend_side(subtr_side_inters[two_pt_side][0]), minuend_side(subtr_side_inters[two_pt_side][1])),
                    tri_get_common_point_of_sides(minuend_side(subtr_side_inters[other_sides[0]][0]), minuend_side(subtr_side_inters[other_sides[1]][0])),
                };

                glm::vec2 subtr_inside_pt = subtr.pts[subtr_inside_indices[0]];

                line two_pt_line = tri_next_two_inters_points(minuend.pts, fac_arr, subtr_side_inters[two_pt_side]);
                line other_pt_line = {
                    tri_get_inters_point(minuend.pts, fac_arr, subtr_side_inters[other_sides[1]][0]),
                    tri_get_inters_point(minuend.pts, fac_arr, subtr_side_inters[other_sides[0]][0])
                };
                
                triangle minuend_cutoff0 = {minuend.pts[common_points[0]], two_pt_line.pts[0], two_pt_line.pts[1]};
                line *lines[1] = {&other_pt_line};
                tris_first_to_cc_winding_others_simult<1>(minuend_cutoff0, lines);

                tris.push_back(minuend_cutoff0);
                tris_from_cc_quadrilateral(tris, {minuend.pts[common_points[1]], other_pt_line.pts[0], subtr_inside_pt, other_pt_line.pts[1]});

                return;
            }
        } else if (subtr_inside_count == 2) {
            if (inters_count == 2) {
                triangle minuend_winded = tri_align_as_common_side0_side1(minuend, minuend_side(inters_indices[0]), minuend_side(inters_indices[1]));
                triangle subtr_winded = tri_align_as_common_side0_side1(subtr, subtr_side(inters_indices[0]), subtr_side(inters_indices[1]));
                line inters = tri_next_two_inters_points(minuend.pts, fac_arr, inters_indices);

                line subtr_insides = {subtr_winded.pts[1], subtr_winded.pts[2]};
                
                line *lines[2] = {&inters, &subtr_insides};
                tris_first_to_cc_winding_others_simult<2>(minuend_winded, lines);

                tris.push_back({minuend_winded.pts[1], subtr_insides.pts[0], inters.pts[0]});
                tris.push_back({minuend_winded.pts[2], inters.pts[1], subtr_insides.pts[1]});
                tris_from_cc_quadrilateral(tris, {minuend_winded.pts[1], minuend_winded.pts[2], subtr_insides.pts[1], subtr_insides.pts[0]});

                return;
            }
        }
    } else if (minuend_outside_count == 1) {
        if (subtr_outside) {
            assert(inters_count == 2);
            line inters = tri_next_two_inters_points(minuend.pts, fac_arr, inters_indices);

            triangle minuend_cutoff = {minuend.pts[minuend_outside_indices[0]], inters.pts[0], inters.pts[1]}; 
            tris_first_to_cc_winding_others_simult<0>(minuend_cutoff, null);

            tris.push_back(minuend_cutoff);

            return;
        } else if (subtr_inside_count == 2) {
            assert(inters_count == 2);
            line inters = tri_next_two_inters_points(minuend.pts, fac_arr, inters_indices);

            // TODO: Ugly! Make this nicer! marker
            glm::vec2 subtr_inside_pt;
            for (int i = 0; i < 2; i++) {
                const glm::vec2& inters_pt = inters.pts[i];

                for (int j = 0; j < 2; j++) {
                    f32 e = 1e-4;
                    if (glm::distance2(subtr.pts[subtr_inside_indices[j]], inters_pt) < e) {
                        subtr_inside_pt = subtr.pts[1 - j];
                        goto out;
                    }
                }
            }
            out:

            triangle minuend_cutoff = {minuend.pts[minuend_outside_indices[0]], inters.pts[0], inters.pts[1]}; 
            tris_first_to_cc_winding_others_simult<0>(minuend_cutoff, null);

            tris_from_cc_quadrilateral(tris, {minuend_cutoff.pts[0], minuend_cutoff.pts[1], subtr_inside_pt, minuend_cutoff.pts[2]});

            return;
        } else if (subtr_inside_count == 1) {
            triangle minuend_winded = tri_align_as_common_side0_side1(minuend, minuend_side(inters_indices[0]), minuend_side(inters_indices[1]));
            line inters = tri_next_two_inters_points(minuend.pts, fac_arr, inters_indices);

            line *lines[1] = {&inters};
            tris_first_to_cc_winding_others_simult<1>(minuend_winded, lines);

            glm::vec2 subtr_inside_pt = subtr.pts[subtr_inside_indices[0]];

            tris_from_cc_quadrilateral(tris, {minuend_winded.pts[0], inters.pts[0], subtr_inside_pt, inters.pts[1]});

            return;
        }
    } else {
        printf("Not a good lawyer 12938878!\n");
    }

    printf("Failed on (case missed): {%s}, {%s}\n", to_string(minuend).c_str(), to_string(subtr).c_str());
    tris.push_back(minuend); // Return something other than {} so tests fail initially.
}

void subtract_triangles(const triangle& minuend, const triangle& subtr, std::vector<triangle>& tris) {
    int start_idx = (int)tris.size();
    internal_subtract_triangles(minuend, subtr, tris);

    // Remove all zero area triangles.
    for (int i = (int)tris.size() - 1; i >= start_idx; i--) {
        const triangle& tri = tris[i];
            
        f32 e = 1e-7; // 1e-8 is smaller than the smallest f32 representable number other than 0. TODO: Numerically dubious.
        f32 min_ratio = 1e-2; // TODO: This might be a bad heuristic!
        if (tri_area(tri) < e || tri_min_height_to_ground_ratio(tri) < min_ratio || !tri_is_winding_cc(tri)) { // TODO: This might hide bugs.
            DEBUG_PRINT("removed zero area triangle %s\n", to_string(tri).c_str());
            tris.erase(tris.begin() + i);
        }
    }

    // Log failures.
    for (size_t i = start_idx; i < tris.size(); i++) {
        const triangle& tri = tris[i];
        
        if (!tri_is_winding_cc(tri)) {
            printf("Failed on (inv. winding): minuend {%s}, subtr. {%s}\n tris: ", to_string(minuend).c_str(), to_string(subtr).c_str());
            print(tris);
            printf("--------\n");
            break;
        }
    }
}

bool tri_in_mesh(const triangle& tri, const std::vector<triangle>& tris, f32 min_rem_area) {
    f32 intersecting_area = 0.0f;
    std::queue<triangle> intersecting;
    
    intersecting_area += tri_area(tri);
    intersecting.push(tri);

    while (intersecting_area >= min_rem_area) {
        f32 last_intersecting_area = intersecting_area;

        const triangle initial_rem = intersecting.front();
        intersecting_area -= tri_area(initial_rem);
        intersecting.pop();

        std::vector<triangle> curr_remainders = {initial_rem};
        for (size_t i = 0; i < tris.size(); i++) {
            std::vector<triangle> next_remainders;

            for (size_t j = 0; j < curr_remainders.size(); j++) {
                subtract_triangles(curr_remainders[j], tris[i], next_remainders);
            }

            curr_remainders = std::move(next_remainders);
        }

        for (triangle& rem: curr_remainders) {
            intersecting_area += tri_area(rem);
            intersecting.push(rem);
        }
 
        if (last_intersecting_area - intersecting_area <= min_rem_area) {
            return false;
        }
    }

    return true;
}


Occl_Mesh::Occl_Mesh(std::vector<glm::vec2> _convex_hull) : convex_hull(std::move(_convex_hull)) {
    bbox = {{99999.0f, 99999.0f}, {-99999.0f, -99999.0f}};

    for (const glm::vec2& p: convex_hull) {
        bbox.tl = {std::min(bbox.tl.x, p.x), std::min(bbox.tl.y, p.y)};
        bbox.br = {std::max(bbox.br.x, p.x), std::max(bbox.br.y, p.y)};
    }

    for (size_t i = 2; i < convex_hull.size(); i++) {
        mesh_proj.push_back({convex_hull[i - 1], convex_hull[i], convex_hull[0]});
    }
}

int Occl_Mesh::compare(f32 value, uint dim) const {
    if (bbox.br[dim] < value) {
        return -1;
    } else if(value < bbox.tl[dim]) {
        return 1;
    } else {
        return 0;
    }
}

bool Occl_Mesh::inside_fast(const Occl_Mesh *other) {
    // TODO: This could obviously be made even faster.
    // AVX2 for vectorization is possible.

    for (size_t i = 0; i < other->convex_hull.size(); i++) {
        const glm::vec2& curr = other->convex_hull[i];
        const glm::vec2& next = other->convex_hull[(i + 1) % other->convex_hull.size()];
        const glm::vec2 o = orth(next - curr);

        for (size_t j = 0; j < convex_hull.size(); j++) {
            f32 dot = glm::dot(convex_hull[j] - curr, o);

            if (dot > 0) {
                return false;
            }
        }
    }

    return true;
}

bool Occl_Mesh::intersect(const Occl_Mesh *other) {
    return ::bbox_intersect(bbox, other->bbox);
}

bool Occl_Mesh::bbox_intersect(const BBox& other_bbox) {
    return ::bbox_intersect(bbox, other_bbox);
}

bool Occl_Mesh::inside(Octree<Occl_Mesh *>& tree) {
    // Custom implementation of a concrete octree operation.

    std::queue<Octree<Occl_Mesh *>::Octree_Node *> queue;
    queue.push(tree.root);

    std::vector<Octree<Occl_Mesh *>::Octree_Node *> inters;

    while (queue.size() > 0) {
        Octree<Occl_Mesh *>::Octree_Node *node = queue.front();
        queue.pop();

        // Try to resolve using the fast method first.
        for (const Occl_Mesh *upon: node->upon_line) {
            if (this->inside_fast(upon)) {
                return true;
            }
        }

        inters.push_back(node);

        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                Octree<Occl_Mesh *>::Octree_Node *child = node->children[i][j];

                if (child != null && this->bbox_intersect(child->bbox)) {
                    queue.push(child);
                }
            }
        }
    }

    // Try the fast method using convexity on the indiviual triangles one last time.
    // Fallback to the slow method, if the fast one fails.
    std::vector<triangle> inters_tris;
    for (const Octree<Occl_Mesh *>::Octree_Node *node: inters) {
        for (const Occl_Mesh *mesh: node->upon_line) {
            if (!this->intersect(mesh)) {
                continue;
            }

            for (const triangle &tri: mesh->mesh_proj) {
                inters_tris.push_back(tri);
            }
        }
    }

    for (const triangle &tri: mesh_proj) {
        if (!tri_in_mesh(tri, inters_tris)) {
            return false;
        }
    }

    return true;
}

Occl_Cull_Context::Occl_Cull_Context(size_t reserve, const BBox& clip_box)
    : draw_tree_alloc(1024 * 512), occl_tree_alloc(1024 * 512), draw_tree(&draw_tree_alloc, clip_box), occluded_tree(&occl_tree_alloc, clip_box),
        reserved(reserve), total_occluded(0), total_fast(0), total_slow(0) {

    meshes.reserve(reserve); // TODO: Workaround so pointers stay valid!
    flags.reserve(reserve);
}

void Occl_Cull_Context::add_mesh(const Occl_Mesh&& mesh) {
    flags.push_back(0);
    meshes.push_back(mesh);

    assert(meshes.size() <= reserved); // TODO: Make memory move impossible.

    draw_tree.insert(&meshes[meshes.size() - 1]);
}

void Occl_Cull_Context::flag_mesh(int index, Occl_Cull_Flag flag) {
    flags[index] |= (u8)flag;

    if (flag == Occl_Cull_Flag::OCCLUDED) {
        Occl_Mesh& occl_mesh = meshes[index];

        if (occl_mesh.inside(occluded_tree)) {
            return;
        }
            
        occluded_tree.insert(&occl_mesh); // TODO: This might be wrong because of pointer to ref.!

        std::vector<Occl_Mesh *> inside_meshes;
        std::vector<Occl_Mesh *> affected_meshes;
        draw_tree.intersect(&occl_mesh, inside_meshes, affected_meshes);
        total_occluded++;

        // TODO: Duplicate code.
        for (Occl_Mesh *mesh: inside_meshes) {
            int i = mesh - meshes.data();
            assert(i >= 0 && i < (int)meshes.size()); // TODO: Make sure memory can't move!

            if (flags[i] != 0) continue;

            flags[i] |= (u8)Occl_Cull_Flag::OCCLUDED;
            total_fast++;
        }

        for (Occl_Mesh *mesh: affected_meshes) { 
            int i = mesh - meshes.data();
            assert(i >= 0 && i < (int)meshes.size()); // TODO: Make sure memory can't move!
            
            if (flags[i] != 0) continue;

            if (mesh->inside(occluded_tree)) {
                flags[i] |= (u8)Occl_Cull_Flag::OCCLUDED;
                total_slow++;
            }
        }
    }
}

u8 Occl_Cull_Context::get_flags(int index) {
    return flags[index];
}

size_t Occl_Cull_Context::get_total_tri_count() {
    size_t total_tris = 0;

    for (size_t i = 0; i < meshes.size(); i++) {        
        total_tris += meshes[i].mesh_proj.size();
    }

    return total_tris;
}