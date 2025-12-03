#ifndef VEC3_H
#define VEC3_H

// Small vector library
//  Represents a vector as 3 floats

struct vec3 {
  float x, y, z;

  vec3(float x, float y, float z) : x(x), y(y), z(z) {}
  vec3() : x(0), y(0), z(0) {}

  // Clamp each component (used to clamp pixel colors)
  vec3 clampTo1() { return vec3(fmin(x, 1), fmin(y, 1), fmin(z, 1)); }

  // Compute vector length (you may also want length squared)
  float length() { return sqrt(x * x + y * y + z * z); }

  // Create a unit-length vector
  vec3 normalized() const {
    float len = sqrt(x * x + y * y + z * z);
    return vec3(x / len, y / len, z / len);
  }
};

struct Triangle {
    int v1, v2, v3;     // vertex indices
    int n1, n2, n3;     // normal indices (-1 if unused)
    int matIndex;
    bool hasVertexNormals;
};


struct Sphere {
  vec3 pos;
  float radius;
  int matIndex; // Which material this sphere uses
};

struct material {
  float ar, ag, ab, dr, dg, db, sr, sg, sb, ns, tr, tg, tb, ior;

  material(float ar, float ag, float ab, float dr, float dg, float db, float sr,
           float sg, float sb, float ns, float tr, float tg, float tb,
           float ior)
      : ar(ar), ag(ag), ab(ab), dr(dr), dg(dg), db(db), sr(sr), sg(sg), sb(sb),
        ns(ns), tr(tr), tg(tg), tb(tb), ior(ior) {}

  material()
      : ar(0), ag(0), ab(0), dr(0), dg(0), sr(0), sg(0), sb(0), ns(0), tr(0),
        tg(0), tb(0), ior(0) {}
};

struct light {
  float r, g, b, x, y, z;

  light(float r, float g, float b, float x, float y, float z)
      : r(r), g(g), b(b), x(x), y(y), z(z) {}

  light() : r(0), g(0), b(0), x(0), y(0), z(0) {}
};

struct spotlight {
  float r, g, b, px, py, pz, dx, dy, dz, angle1, angle2;

  spotlight(float r, float g, float b, float px, float py, float pz, float dx,
            float dy, float dz, float angle1, float angle2)
      : r(r), g(g), b(b), px(px), py(py), pz(pz), dx(dx), dy(dy), dz(dz),
        angle1(angle1), angle2(angle2) {}

  spotlight()
      : r(0), g(0), b(0), px(0), py(0), pz(0), dx(0), dy(0), dz(0), angle1(0),
        angle2(0) {}
};

// Multiply float and vector
// TODO - Implement: you probably also want to implement multiply vector and
// float ... inline vec3 operator*(float f, vec3 a)
inline vec3 operator*(float f, vec3 a) {
  return vec3(a.x * f, a.y * f, a.z * f);
}

inline vec3 operator*(vec3 a, float f) {
  return vec3(a.x * f, a.y * f, a.z * f);
}

// Component-wise vector multiplication (Hadamard product)
inline vec3 operator*(vec3 a, vec3 b) {
  return vec3(a.x * b.x, a.y * b.y, a.z * b.z);
}

// Vector-vector dot product
inline float dot(vec3 a, vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline vec3 operator-(const vec3 &v) {
    return vec3(-v.x, -v.y, -v.z);
}

// Vector-vector cross product
inline vec3 cross(vec3 a, vec3 b) {
  return vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
              a.x * b.y - a.y * b.x);
}

// Vector addition
inline vec3 operator+(vec3 a, vec3 b) {
  return vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

// Vector subtraction
inline vec3 operator-(vec3 a, vec3 b) {
  return vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

// ============================================================================
// BVH (Bounding Volume Hierarchy) Acceleration Structure
// ============================================================================

// Axis-Aligned Bounding Box
struct AABB {
  vec3 min, max;

  AABB() : min(vec3(1e30f, 1e30f, 1e30f)), max(vec3(-1e30f, -1e30f, -1e30f)) {}
  AABB(vec3 min, vec3 max) : min(min), max(max) {}

  // Expand to include a point
  void expand(const vec3& p) {
    min.x = fmin(min.x, p.x);
    min.y = fmin(min.y, p.y);
    min.z = fmin(min.z, p.z);
    max.x = fmax(max.x, p.x);
    max.y = fmax(max.y, p.y);
    max.z = fmax(max.z, p.z);
  }

  // Expand to include another AABB
  void expand(const AABB& other) {
    expand(other.min);
    expand(other.max);
  }

  // Ray-AABB intersection test (slab method)
  bool intersect(const vec3& rayOrigin, const vec3& rayDirInv, float tMin, float tMax) const {
    for (int i = 0; i < 3; i++) {
      float t1 = ((&min.x)[i] - (&rayOrigin.x)[i]) * (&rayDirInv.x)[i];
      float t2 = ((&max.x)[i] - (&rayOrigin.x)[i]) * (&rayDirInv.x)[i];
      if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
      tMin = fmax(tMin, t1);
      tMax = fmin(tMax, t2);
      if (tMin > tMax) return false;
    }
    return true;
  }

  // Surface area (for SAH)
  float surfaceArea() const {
    vec3 d = max - min;
    return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
  }

  // Center point
  vec3 center() const {
    return vec3((min.x + max.x) * 0.5f, (min.y + max.y) * 0.5f, (min.z + max.z) * 0.5f);
  }
};

// Primitive wrapper (sphere or triangle)
struct Primitive {
  enum Type { SPHERE, TRIANGLE } type;
  int index; // Index into spheres[] or triangles[] array
  AABB bounds;

  Primitive() : type(SPHERE), index(-1) {}
};

// BVH Node
struct BVHNode {
  AABB bounds;
  int leftChild;   // Index of left child (or -1 if leaf)
  int rightChild;  // Index of right child (or -1 if leaf)
  int primStart;   // First primitive index (for leaves)
  int primCount;   // Number of primitives (for leaves)

  BVHNode() : leftChild(-1), rightChild(-1), primStart(0), primCount(0) {}

  bool isLeaf() const { return leftChild == -1; }
};

#endif