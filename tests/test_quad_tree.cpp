#include "grtest.h"
#include "QuadTree.hpp"

#include <cassert>


TEST_CASE(test_coords_calcs) {
    assert((QuadTree<int>::children_coords(0, { 0,0 }) == std::array<Vec2i, 4>{Vec2i{ 0,0 }, { 1u<<31,0 }, { 0,1u<<31 }, { 1u<<31,1u<<31 }}));
    assert((QuadTree<int>::children_coords(30, { 0,0 }) == std::array<Vec2i, 4>{Vec2i{ 0,0 }, { 2,0 }, { 0,2 }, { 2,2 }}));
    assert((QuadTree<int>::children_coords(31, { 8,16 }) == std::array<Vec2i, 4>{Vec2i{ 8,16 }, { 9,16 }, { 8,17 }, { 9,17 }}));
    assert((QuadTree<int>::siblings_coords(31, { 8,16 }) == std::array<Vec2i, 3>{Vec2i{ 10,16 }, { 8,18 }, { 10,18 }}));
    assert((QuadTree<int>::siblings_coords(31, { 10,18 }) == std::array<Vec2i, 3>{Vec2i{ 8, 18 }, { 10,16 }, { 8,16 }}));
    assert((QuadTree<int>::parent_coords(31, { 8,16 }) == Vec2i{ 8,16 }));
    assert((QuadTree<int>::parent_coords(31, { 10,18 }) == Vec2i{ 8,16 }));
    assert(QuadTree<int>::is_parent(Vec2i{ 10,18 }, 29, Vec2i{ 8,16 }));
    assert(not QuadTree<int>::is_parent(Vec2i{ 1u<<31,1u<<31 }, 1, Vec2i{ 0,0 }));
}
