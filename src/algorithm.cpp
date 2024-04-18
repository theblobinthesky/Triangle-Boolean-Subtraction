#include "algorithm.h"
#include <glm/gtx/norm.hpp>
#include <functional>

int f32_compare_to(f32 a, f32 b) {
    if (f32_eq(a, b)) return 0;
    if (a < b) return -1;
    else return 1;
}

f32 pt_left_of(const glm::vec2& pt, const glm::vec2& p, const glm::vec2& q) {
	f32 a = (q.x - pt.x) * (p.y - pt.y);
	f32 b = (p.x - pt.x) * (q.y - pt.y);
	f32 cross_product = a - b;

	return f32_compare_to(cross_product, 0);
}

int pt_compare(const glm::vec2& a, const glm::vec2& b) {
    int c = f32_compare_to(a.x, b.x);

	if (c == 0) {
	    c = f32_compare_to(a.y, b.y);
	}

	return c;
}

void convex_hull_sort_and_filter(const glm::vec2& p0, std::vector<glm::vec2>& pts) {
    // Sort the points counter-clockwise around p0. Collinear points with
    // respect to p0 are sorted by distance to ensure none of them end up in
    // the convex hull later.
    std::sort(pts.begin(), pts.end(), [p0](const glm::vec2& p, const glm::vec2& q) {
        int c = pt_left_of(p0, p, q);
        
        if (c == 0) {
            f32 p0_to_p_2 = glm::distance2(p0, p);
            f32 p0_to_q_2 = glm::distance2(p0, q);

            // Since there can be no duplicate points and the reference point
            // has the smallest y value of the set of points with the smallest x
            // value, p and q can't be equal and compareTo cannot find p and q
            // to have the same distance. Assigning the compare value is fine.
            c = f32_compare_to(p0_to_p_2, p0_to_q_2);
        }

        return c < 0;
    });

    // The loop filters out collinear points with respect to p0 using an
    // array list and in-place for efficiency. We have to start at index 1
    // rather than index 0 to skip over p0 which is the first list element.
    int j = 1;
    for (int i = 1; i < (int)pts.size() - 1; i++) {
        const glm::vec2& curr_pt = pts[i];
        const glm::vec2& next_pt = pts[i + 1];

        // Move the current point forward if there is a gap caused by
        // removed collinear points with respect to p0.
        pts[j] = curr_pt;
        
        if (pt_left_of(p0, curr_pt, next_pt) != 0) {
            j++;
        }
    }

	// Move the last point forward as the loop might have created a gap
	// between it and its predecessor.
	if (pts.size() > 1) {
	    pts[j] = pts[(int)pts.size() - 1];
	}

	// Remove the unused gap at the end of the list efficiently.
	for (int i = (int)pts.size() - 1; i > j; i--) {
	    pts.erase(pts.begin() + i);
	}
}

void convex_hull_graham_scan(std::vector<glm::vec2>& pts) {
	// To implement the Graham-Scan algorithm efficiently, we want it to
	// operate in-place. Therefore we need to track the stack top within the
	// points array manually. During the loop it is always ensured that
	// stack_top < i and therefore the two parts of the stack are always
	// disjoint and we can think of them as two separate arrays.

	int stack_top = 2;
	for (size_t i = 3; i < pts.size(); i++) {
	    const glm::vec2& candidate = pts[i];

	    // At this point we imagine the candidate point was already added to
	    // the stack. Now the stack might not be a convex hull anymore.
	    // According to the Graham-Scan algorithm we need to remove the top
	    // stack points as long as they break the convex hull condition.
	    while (stack_top >= 2 && pt_left_of(candidate, pts[stack_top - 1], pts[stack_top]) >= 0) {
		    stack_top--;
	    }
 
	    // Now that the stack with the candidate is the convex hull of all
	    // previous points, we can add it.
	    pts[++stack_top] = candidate;
	}

	// Remove the unused gap at the end of the list efficiently.
	for (int i = (int)pts.size() - 1; i > stack_top; i--) {
	    pts.erase(pts.begin() + i);
	}
}


void inplace_convex_hull(std::vector<glm::vec2>& pts) {
    if (pts.size() > 0) {
	    // The point p0 is guaranteed to be in the convex hull and will be
	    // the first element of the output list.
        size_t min_idx = 0;
        for (size_t i = 1; i < pts.size(); i++) {
            const glm::vec2& pt = pts[i];

            if (pt_compare(pt, pts[min_idx]) <= 0) {
                min_idx = i;
            }
        }

	    // Sort the points counter-clockwise and collinear points by
	    // distance as it is described in the helper method.
	    glm::vec2 p0 = pts[min_idx];
	    convex_hull_sort_and_filter(p0, pts);

	    // Note that the Graham-Scan algorithm requires no special casing
	    // for one or two points. This is clarified in the helper method.
	    convex_hull_graham_scan(pts);
    }
}