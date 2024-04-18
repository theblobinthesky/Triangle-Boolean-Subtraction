#pragma once
#include "util.h"
#include "memory.h"
#include <vector>
#include <concepts>
#include <utility>
#include <queue>
#include <glm/vec2.hpp>

struct BBox {
    glm::vec2 tl;
    glm::vec2 br;

    inline glm::vec2 middle() const {
        return 0.5f * (tl + br);
    }
};

template<typename T>
concept Octree_Data = requires(f32 v, T a, T b, BBox bbox, f32 t, uint dim) {
    { a->compare(v, dim) } -> std::convertible_to<int>;
    { a->inside_fast(b) } -> std::same_as<bool>;
    { a->intersect(b) } -> std::same_as<bool>;
    { a->bbox_intersect(bbox) } -> std::same_as<bool>;
};

template<typename T>
class Octree {
public: // TODO: Revert!
    static_assert(Octree_Data<T>);

    struct Octree_Node {
        BBox bbox;
        std::vector<T> upon_line;
        Octree_Node *children[2][2];
    };

    bump_allocator *allocator;
    Octree_Node *root;

public:
    Octree(bump_allocator *allocator, BBox root_bbox) : allocator(allocator) {
        root = (Octree_Node *)allocator->allocate(sizeof(Octree_Node));
        new (root) Octree_Node{root_bbox, {}, {}};
    }

    void insert(const T t) {
        assert(t->bbox_intersect(root->bbox));

        Octree_Node *parent = null;
        Octree_Node **node = &root;

        for (;;) {
            int indices[2] = {};

            while (*node != null) {
                glm::vec2 middle = (*node)->bbox.middle();
                int compares[2] = {t->compare(middle.x, 0), t->compare(middle.y, 1)};

                if (compares[0] == 0 || compares[1] == 0) {
                    (*node)->upon_line.push_back(t);
                    return;
                }

                parent = *node;
                indices[0] = compares[0] >= 0;
                indices[1] = compares[1] >= 0;
                node = &(*node)->children[indices[0]][indices[1]];
            }

            // Compute new bounding box and create new node.
            const BBox& p_bbox = parent->bbox;
            glm::vec2 middle = p_bbox.middle();
            
            BBox bbox;
            switch ((indices[0] << 4) | indices[1]) {
                case (0 << 4) | 0: bbox = {p_bbox.tl, middle}; break;
                case (1 << 4) | 0: bbox = {{middle.x, p_bbox.tl.y}, {p_bbox.br.x, middle.y}}; break;
                case (0 << 4) | 1: bbox = {{p_bbox.tl.x, middle.y}, {middle.x, p_bbox.br.y}}; break;
                case (1 << 4) | 1: bbox = {middle, p_bbox.br}; break;
            }

            // TODO: Speed. Allocating all children upfront might be a good idea for intersection speed.
            *node = (Octree_Node *)allocator->allocate(sizeof(Octree_Node));
            new (*node) Octree_Node{bbox, {}, {}};
        }
    }

    void intersect(const T t, std::vector<T>& insides, std::vector<T>& inters) {
        assert(t->bbox_intersect(root->bbox));

        std::queue<Octree_Node *> queue;
        queue.push(root);

        while (queue.size() > 0) {
            Octree_Node *node = queue.front();
            queue.pop();

            for (const T& upon: node->upon_line) {
                if (upon->inside_fast(t)) {
                    insides.push_back(upon);
                } else if (upon->intersect(t)) {
                    inters.push_back(upon);
                }
            }

            for (int i = 0; i < 2; i++) {
                for (int j = 0; j < 2; j++) {
                    Octree_Node *child = node->children[i][j];

                    if (child != null && t->bbox_intersect(child->bbox)) {
                        queue.push(child);
                    }
                }
            }
        }
    }
};

struct triangle {
    // The points are in counter-clockwise order.
    glm::vec2 pts[3];
};

struct quadrilateral {
    glm::vec2 pts[4];
};

void tris_from_cc_quadrilateral(std::vector<triangle>& tris, const quadrilateral&& q);

bool tri_is_winding_cc(const triangle& tri);
f32 tri_area(const triangle& tri);

void subtract_triangles(const triangle& minuend, const triangle& subtr, std::vector<triangle>& tris);
bool tri_in_mesh(const triangle& tri, const std::vector<triangle>& tris, f32 min_rem_area = 1e-3);

std::string to_string(const glm::vec2& v);
std::string to_string(const triangle& tri);
void print(const std::vector<triangle>& tris);
void print(const std::vector<glm::vec2>& pts);

struct Occl_Mesh {
    BBox bbox;
    std::vector<glm::vec2> convex_hull;
    std::vector<triangle> mesh_proj;

    Occl_Mesh(std::vector<glm::vec2> _convex_hull);
    int compare(f32 value, uint dim) const;
    bool inside_fast(const Occl_Mesh *other);
    bool intersect(const Occl_Mesh *other);
    bool bbox_intersect(const BBox& other_bbox);
    bool inside(Octree<Occl_Mesh *>& tree);
};

enum class Occl_Cull_Flag : u8 {
    DRAWN = 1,
    OCCLUDED = 2
};

struct Occl_Cull_Context {
    bump_allocator draw_tree_alloc;
    bump_allocator occl_tree_alloc;

    Octree<Occl_Mesh *> draw_tree;
    Octree<Occl_Mesh *> occluded_tree;

    std::vector<u8> flags;
    std::vector<Occl_Mesh> meshes;
    size_t reserved;

    int total_occluded, total_fast, total_slow;
    
    Occl_Cull_Context(size_t reserve, const BBox& clip_box);
    void add_mesh(const Occl_Mesh&& mesh);
    void flag_mesh(int index, Occl_Cull_Flag flag);
    u8 get_flags(int index);
    size_t get_total_tri_count(); // TODO: Remove later.
};