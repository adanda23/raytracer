// To Compile: g++ -fsanitize=address -std=c++11 rayTrace_vec3.cpp

#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS // For fopen and sscanf
#define _USE_MATH_DEFINES
#endif

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "image_lib.h"
#include "vec3.h"
#include "parse_vec3.h"
#include <atomic>
#include <algorithm>

#include <chrono>
#include <omp.h>

// Forward declaration for BVH shadow rays
bool isOccluded(const vec3& rayOrigin, const vec3& rayDir, float maxDist);

bool rayTriangleIntersect(const vec3& orig, const vec3& dir,
                          const vec3& v0, const vec3& v1, const vec3& v2,
                          float& t, float& u, float& v) {
    const float EPS = 1e-8f;
    vec3 edge1 = v1 - v0;
    vec3 edge2 = v2 - v0;
    vec3 h = cross(dir, edge2);
    float a = dot(edge1, h);
    if (fabs(a) < EPS) return false;
    float f = 1.0f / a;
    vec3 s = orig - v0;
    u = f * dot(s, h);
    if (u < 0.0f || u > 1.0f) return false;
    vec3 q = cross(s, edge1);
    v = f * dot(dir, q);
    if (v < 0.0f || u + v > 1.0f) return false;
    t = f * dot(edge2, q);
    return t > EPS;
}

// Ray–sphere intersection
bool raySphereIntersect(const vec3 &rayOrigin, const vec3 &rayDir,
                        const vec3 &sphereCenter, float radius, float &tNear) {
  vec3 L = rayOrigin - sphereCenter;
  float a = dot(rayDir, rayDir);
  float b = 2.0f * dot(rayDir, L);
  float c = dot(L, L) - radius * radius;

  float discriminant = b * b - 4 * a * c;
  if (discriminant < 0)
    return false;

  float sqrtDisc = sqrtf(discriminant);
  float t0 = (-b - sqrtDisc) / (2.0f * a);
  float t1 = (-b + sqrtDisc) / (2.0f * a);

  if (t0 > 0 && t1 > 0)
    tNear = fmin(t0, t1);
  else if (t0 > 0)
    tNear = t0;
  else if (t1 > 0)
    tNear = t1;
  else
    return false;

  return true;
}

// Structure to separate diffuse and specular contributions
struct LightingResult {
  vec3 diffuse;   // Ambient + diffuse (affected by surface color)
  vec3 specular;  // Specular highlights (should be added on top)
};

// Blinn-Phong shading (supports multiple point lights and directional lights,
// and does shadow checks against both spheres and triangles)
LightingResult calculateColor(const vec3 &point, const vec3 &normal, const material &mat,
                               const vec3 &ambientLight, const vec3 &eye) {
  vec3 N = normal.normalized();
  vec3 diffuseAccum(0, 0, 0);
  vec3 specularAccum(0, 0, 0);

  // Ambient contribution (part of diffuse)
  diffuseAccum = diffuseAccum + vec3(mat.ar, mat.ag, mat.ab) * ambientLight;

  // --- Point lights ---
  for (const auto &pointLight : point_lights) {
    vec3 Lvec = vec3(pointLight.x, pointLight.y, pointLight.z) - point;
    float L_len2 = dot(Lvec, Lvec);
    if (L_len2 <= 1e-8f) continue;
    vec3 L = Lvec * (1.0f / sqrtf(L_len2)); // normalized

    // Shadow check using BVH (much faster!)
    float distToLight = sqrtf(L_len2);
    bool inShadow = isOccluded(point + N * 1e-4f, L, distToLight);

    if (!inShadow) {
      float NdotL = fmax(dot(N, L), 0.0f);
      vec3 diffuse = vec3(mat.dr, mat.dg, mat.db) * NdotL;
      diffuse = diffuse * vec3(pointLight.r, pointLight.g, pointLight.b);

      vec3 V = (eye - point).normalized();
      vec3 H = (L + V).normalized();
      float NdotH = fmax(dot(N, H), 0.0f);
      vec3 specular = vec3(mat.sr, mat.sg, mat.sb) * powf(NdotH, mat.ns);
      specular = specular * vec3(pointLight.r, pointLight.g, pointLight.b);

      float attenuation = 1.0f / L_len2; // inverse-square falloff
      diffuse = diffuse * attenuation;
      specular = specular * attenuation;

      diffuseAccum = diffuseAccum + diffuse;
      specularAccum = specularAccum + specular;
    }
  }

  // --- Directional lights ---
  for (const auto &dirLight : directional_lights) {
    vec3 L = vec3(-dirLight.x, -dirLight.y, -dirLight.z).normalized();

    // Shadow check for directional lights using BVH
    bool inShadow = isOccluded(point + N * 1e-4f, L, 1e20f);

    if (!inShadow) {
      float NdotL = fmax(dot(N, L), 0.0f);
      vec3 diffuse = vec3(mat.dr, mat.dg, mat.db) * NdotL;
      diffuse = diffuse * vec3(dirLight.r, dirLight.g, dirLight.b);

      vec3 V = (eye - point).normalized();
      vec3 H = (L + V).normalized();
      float NdotH = fmax(dot(N, H), 0.0f);
      vec3 specular = vec3(mat.sr, mat.sg, mat.sb) * powf(NdotH, mat.ns);
      specular = specular * vec3(dirLight.r, dirLight.g, dirLight.b);

      diffuseAccum = diffuseAccum + diffuse;
      specularAccum = specularAccum + specular;
    }
  }

  LightingResult result;
  result.diffuse = diffuseAccum.clampTo1();
  result.specular = specularAccum.clampTo1();
  return result;
}


vec3 reflect(const vec3& I, const vec3& N) {
    return I - 2 * dot(I, N) * N;
}

bool refract(const vec3& I, const vec3& N, float n1, float n2, vec3& T) {
    float eta = n1 / n2;
    float cosi = -dot(N, I);
    float k = 1 - eta*eta * (1 - cosi*cosi);
    if (k < 0) return false;
    T = eta * I + (eta * cosi - sqrt(k)) * N;
    return true;
}

// ============================================================================
// BVH (Bounding Volume Hierarchy) Implementation
// ============================================================================

std::vector<Primitive> bvhPrimitives;
std::vector<BVHNode> bvhNodes;
int bvhRootIndex = -1;

// Compute bounding box for a primitive
AABB getPrimitiveBounds(const Primitive& prim) {
    AABB box;
    if (prim.type == Primitive::SPHERE) {
        const Sphere& s = spheres[prim.index];
        vec3 r(s.radius, s.radius, s.radius);
        box.min = s.pos - r;
        box.max = s.pos + r;
    } else { // TRIANGLE
        const Triangle& tri = triangles[prim.index];
        const vec3& v0 = vertices[tri.v1];
        const vec3& v1 = vertices[tri.v2];
        const vec3& v2 = vertices[tri.v3];
        box.expand(v0);
        box.expand(v1);
        box.expand(v2);
    }
    return box;
}

// Build BVH recursively using median split along longest axis
int buildBVH(int start, int end) {
    int nodeIdx = bvhNodes.size();
    bvhNodes.push_back(BVHNode());

    // Compute bounds for this node
    AABB bounds;
    for (int i = start; i < end; i++) {
        bounds.expand(bvhPrimitives[i].bounds);
    }
    bvhNodes[nodeIdx].bounds = bounds;

    int count = end - start;

    // Leaf node if few primitives
    if (count <= 2) {
        bvhNodes[nodeIdx].primStart = start;
        bvhNodes[nodeIdx].primCount = count;
        return nodeIdx;
    }

    // Find longest axis
    vec3 extent = bounds.max - bounds.min;
    int axis = 0;
    if (extent.y > extent.x) axis = 1;
    if (extent.z > (&extent.x)[axis]) axis = 2;

    // Sort primitives along axis using std::sort
    std::sort(bvhPrimitives.begin() + start, bvhPrimitives.begin() + end,
        [axis](const Primitive& a, const Primitive& b) {
            vec3 centerA = a.bounds.center();
            vec3 centerB = b.bounds.center();
            return (&centerA.x)[axis] < (&centerB.x)[axis];
        });

    // Split at median
    int mid = start + count / 2;

    // Build children
    int leftChild = buildBVH(start, mid);
    int rightChild = buildBVH(mid, end);

    // Update node (don't use reference as vector may have reallocated)
    bvhNodes[nodeIdx].leftChild = leftChild;
    bvhNodes[nodeIdx].rightChild = rightChild;

    return nodeIdx;
}

// Initialize BVH from scene geometry
void initBVH() {
    bvhPrimitives.clear();
    bvhNodes.clear();

    // Add all spheres
    for (size_t i = 0; i < spheres.size(); i++) {
        Primitive prim;
        prim.type = Primitive::SPHERE;
        prim.index = i;
        prim.bounds = getPrimitiveBounds(prim);
        bvhPrimitives.push_back(prim);
    }

    // Add all triangles
    for (size_t i = 0; i < triangles.size(); i++) {
        Primitive prim;
        prim.type = Primitive::TRIANGLE;
        prim.index = i;
        prim.bounds = getPrimitiveBounds(prim);
        bvhPrimitives.push_back(prim);
    }

    if (!bvhPrimitives.empty()) {
        bvhRootIndex = buildBVH(0, bvhPrimitives.size());
        printf("BVH built: %zu primitives, %zu nodes\n",
               bvhPrimitives.size(), bvhNodes.size());
    }
}

// Traverse BVH and find closest intersection
bool traverseBVH(const vec3& rayOrigin, const vec3& rayDir,
                 float& tNear, int& hitPrimIndex, float& baryU, float& baryV) {
    if (bvhRootIndex == -1) return false;

    // Precompute inverse ray direction for AABB tests
    vec3 rayDirInv(1.0f / rayDir.x, 1.0f / rayDir.y, 1.0f / rayDir.z);

    bool foundHit = false;
    tNear = 1e20f;
    hitPrimIndex = -1;

    // Stack-based traversal (avoid recursion)
    int stack[64];
    int stackPtr = 0;
    stack[stackPtr++] = bvhRootIndex;

    while (stackPtr > 0) {
        int nodeIdx = stack[--stackPtr];
        const BVHNode& node = bvhNodes[nodeIdx];

        // Test ray against node bounds
        if (!node.bounds.intersect(rayOrigin, rayDirInv, 1e-6f, tNear)) {
            continue;
        }

        if (node.isLeaf()) {
            // Test all primitives in leaf
            for (int i = 0; i < node.primCount; i++) {
                const Primitive& prim = bvhPrimitives[node.primStart + i];
                float t;

                if (prim.type == Primitive::SPHERE) {
                    if (raySphereIntersect(rayOrigin, rayDir,
                                          spheres[prim.index].pos,
                                          spheres[prim.index].radius, t)) {
                        if (t < tNear) {
                            tNear = t;
                            hitPrimIndex = node.primStart + i;
                            foundHit = true;
                        }
                    }
                } else { // TRIANGLE
                    const Triangle& tri = triangles[prim.index];
                    const vec3& v0 = vertices[tri.v1];
                    const vec3& v1 = vertices[tri.v2];
                    const vec3& v2 = vertices[tri.v3];
                    float u, v;
                    if (rayTriangleIntersect(rayOrigin, rayDir, v0, v1, v2, t, u, v)) {
                        if (t < tNear) {
                            tNear = t;
                            hitPrimIndex = node.primStart + i;
                            baryU = u;
                            baryV = v;
                            foundHit = true;
                        }
                    }
                }
            }
        } else {
            // Internal node: add children to stack
            stack[stackPtr++] = node.leftChild;
            stack[stackPtr++] = node.rightChild;
        }
    }

    return foundHit;
}

// Check if ray is occluded (for shadows) - early exit version
bool isOccluded(const vec3& rayOrigin, const vec3& rayDir, float maxDist) {
    if (bvhRootIndex == -1) return false;

    vec3 rayDirInv(1.0f / rayDir.x, 1.0f / rayDir.y, 1.0f / rayDir.z);

    int stack[64];
    int stackPtr = 0;
    stack[stackPtr++] = bvhRootIndex;

    while (stackPtr > 0) {
        int nodeIdx = stack[--stackPtr];
        const BVHNode& node = bvhNodes[nodeIdx];

        if (!node.bounds.intersect(rayOrigin, rayDirInv, 1e-6f, maxDist)) {
            continue;
        }

        if (node.isLeaf()) {
            for (int i = 0; i < node.primCount; i++) {
                const Primitive& prim = bvhPrimitives[node.primStart + i];
                float t;

                if (prim.type == Primitive::SPHERE) {
                    if (raySphereIntersect(rayOrigin, rayDir,
                                          spheres[prim.index].pos,
                                          spheres[prim.index].radius, t)) {
                        if (t > 1e-6f && t < maxDist) return true;
                    }
                } else { // TRIANGLE
                    const Triangle& tri = triangles[prim.index];
                    const vec3& v0 = vertices[tri.v1];
                    const vec3& v1 = vertices[tri.v2];
                    const vec3& v2 = vertices[tri.v3];
                    float u, v;
                    if (rayTriangleIntersect(rayOrigin, rayDir, v0, v1, v2, t, u, v)) {
                        if (t > 1e-6f && t < maxDist) return true;
                    }
                }
            }
        } else {
            stack[stackPtr++] = node.leftChild;
            stack[stackPtr++] = node.rightChild;
        }
    }

    return false;
}


vec3 traceRay(const vec3 &rayOrigin, const vec3 &rayDir, int depth) {
    if (depth > max_depth)
        return vec3(backgroundColor.x, backgroundColor.y, backgroundColor.z);

    // Use BVH for intersection
    float tNear;
    int hitPrimIndex;
    float baryU = 0, baryV = 0;

    if (!traverseBVH(rayOrigin, rayDir, tNear, hitPrimIndex, baryU, baryV)) {
        return vec3(backgroundColor.x, backgroundColor.y, backgroundColor.z);
    }

    // Get the primitive that was hit
    const Primitive& hitPrim = bvhPrimitives[hitPrimIndex];

    vec3 hitPoint = rayOrigin + rayDir * tNear;
    vec3 normal;
    const material* mat = nullptr;

    // Compute normal and material based on primitive type
    if (hitPrim.type == Primitive::SPHERE) {
        const Sphere& s = spheres[hitPrim.index];
        normal = (hitPoint - s.pos).normalized();
        mat = &materials[s.matIndex];
    } else { // TRIANGLE
        const Triangle& tri = triangles[hitPrim.index];
        const vec3& v0 = vertices[tri.v1];
        const vec3& v1 = vertices[tri.v2];
        const vec3& v2 = vertices[tri.v3];

        if (tri.hasVertexNormals) {
            const vec3& n0 = normals[tri.n1];
            const vec3& n1 = normals[tri.n2];
            const vec3& n2 = normals[tri.n3];
            normal = (n0 * (1 - baryU - baryV) + n1 * baryU + n2 * baryV).normalized();
        } else {
            normal = cross(v1 - v0, v2 - v0).normalized();
        }

        if (dot(normal, rayDir) > 0.0f) {
            normal = -normal;
        }

        mat = &materials[tri.matIndex];
    }

    vec3 offsetPoint = hitPoint + normal * 1e-4f;
    LightingResult lighting = calculateColor(hitPoint, normal, *mat, ambient_light, eye);

    // Reflection and Refraction - component-wise for physical accuracy
    vec3 reflectivity = vec3(mat->sr, mat->sg, mat->sb);
    vec3 transparency = vec3(mat->tr, mat->tg, mat->tb);

    vec3 reflectedColor(0,0,0);
    vec3 refractedColor(0,0,0);

    // Reflection - trace if any component is reflective
    float maxReflect = fmax(reflectivity.x, fmax(reflectivity.y, reflectivity.z));
    if (maxReflect > 0.0f) {
        vec3 R = reflect(rayDir, normal).normalized();
        reflectedColor = traceRay(offsetPoint, R, depth + 1) * reflectivity;
    }

    // Refraction - trace if any component is transparent
    float maxTransmit = fmax(transparency.x, fmax(transparency.y, transparency.z));
    if (maxTransmit > 0.0f && mat->ior > 0.0f) {
        vec3 T;
        float n1 = 1.0f;
        float n2 = mat->ior;
        vec3 N = normal;
        if (dot(rayDir, N) > 0) N = -N;
        if (refract(rayDir, N, n1, n2, T)) {
            T = T.normalized();
            refractedColor = traceRay(hitPoint - N*1e-4f, T, depth + 1) * transparency;
        }
    }

    // Energy conservation: diffuse is weighted, but specular highlights are added on top
    // This allows mirrors to show bright highlights while also reflecting
    vec3 totalReflectance = reflectivity + transparency;
    vec3 diffuseWeight = vec3(
        fmax(0.0f, 1.0f - totalReflectance.x),
        fmax(0.0f, 1.0f - totalReflectance.y),
        fmax(0.0f, 1.0f - totalReflectance.z)
    );

    // Combine: weighted diffuse + full specular highlights + reflections + refractions
    vec3 finalColor = lighting.diffuse * diffuseWeight
                    + lighting.specular
                    + reflectedColor
                    + refractedColor;

    return finalColor.clampTo1();
}

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cout << "Usage: ./a.out scenefile\n";
    return 0;
  }
  std::string sceneFileName = argv[1];
  parseSceneFile(sceneFileName);

  // Build BVH acceleration structure
  initBVH();

  float imgW = img_width, imgH = img_height;
  float halfW = imgW / 2, halfH = imgH / 2;
  float d = halfH / tanf(halfAngleVFOV * (M_PI / 180.0f));

  Image outputImg = Image(img_width, img_height);
  auto t_start = std::chrono::high_resolution_clock::now();

std::atomic<int> pixelsDone(0);
int totalPixels = img_width * img_height;

#pragma omp parallel for
  for (int i = 0; i < img_width; i++) {
    for (int j = 0; j < img_height; j++) {
      vec3 colorSum(0, 0, 0);

      // 2x2 sampling
      for (int sx = 0; sx < 2; sx++) {
        for (int sy = 0; sy < 2; sy++) {
          float u = (halfW - (imgW) * ((i + (sx + 0.5f) / 2) / imgW));
          float v = (halfH - (imgH) * ((j + (sy + 0.5f) / 2) / imgH));
          vec3 p = eye - d * forward + u * right + v * up;
          vec3 rayDir = (p - eye).normalized();

          vec3 sampleColor = traceRay(eye, rayDir, 0);
          colorSum = colorSum + sampleColor;
        }
      }

      vec3 color = colorSum * 0.25f;
      outputImg.setPixel(i, j, Color(color.x, color.y, color.z));

      // Extra for timer
      int done = ++pixelsDone; 
        if (done % 1000 == 0) { 
            float progress = (float)done / totalPixels * 100.0f;
            #pragma omp critical
            {
                printf("\rProgress: %.2f%%", progress);
                fflush(stdout);
            }
        }
    }
  }

  auto t_end = std::chrono::high_resolution_clock::now();
  printf("\nRendering took %.2f ms\n",
         std::chrono::duration<double, std::milli>(t_end - t_start).count());

  outputImg.write(imgName.c_str());
  return 0;
}
