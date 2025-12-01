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

#include <chrono>
#include <omp.h>

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

// Blinn-Phong shading (supports multiple point lights and directional lights,
// and does shadow checks against both spheres and triangles)
vec3 calculateColor(const vec3 &point, const vec3 &normal, const material &mat,
                    const vec3 &ambientLight, const vec3 &eye) {
  vec3 N = normal.normalized();
  vec3 color(0, 0, 0);

  // Ambient contribution
  color = color + vec3(mat.ar, mat.ag, mat.ab) * ambientLight;

  // --- Point lights ---
  for (const auto &pointLight : point_lights) {
    vec3 Lvec = vec3(pointLight.x, pointLight.y, pointLight.z) - point;
    float L_len2 = dot(Lvec, Lvec);
    if (L_len2 <= 1e-8f) continue;
    vec3 L = Lvec * (1.0f / sqrtf(L_len2)); // normalized

    // Shadow check: cast ray from slightly offset point toward light
    bool inShadow = false;
    float tNearShadow;

    // check spheres
    for (size_t k = 0; k < spheres.size(); ++k) {
      if (raySphereIntersect(point + N * 1e-4f, L, spheres[k].pos, spheres[k].radius, tNearShadow)) {
        float distToLight = sqrtf(L_len2);
        if (tNearShadow < distToLight) { inShadow = true; break; }
      }
    }

    // check triangles
    if (!inShadow) {
      for (size_t k = 0; k < triangles.size(); ++k) {
        const Triangle &tri = triangles[k];
        const vec3 &v0 = vertices[tri.v1];
        const vec3 &v1 = vertices[tri.v2];
        const vec3 &v2 = vertices[tri.v3];
        float t, u, v;
        if (rayTriangleIntersect(point + N * 1e-4f, L, v0, v1, v2, t, u, v)) {
          float distToLight = sqrtf(L_len2);
          if (t > 1e-6f && t < distToLight) { inShadow = true; break; }
        }
      }
    }

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

      color = color + diffuse + specular;
    }
  }

  // --- Directional lights ---
  for (const auto &dirLight : directional_lights) {
    vec3 L = vec3(-dirLight.x, -dirLight.y, -dirLight.z).normalized();

    // Shadow check for directional lights: treat as a ray in direction L and if it
    // intersects any geometry in front of the point (t > 0) then it's shadowed.
    bool inShadow = false;
    float tShadow;

    // check spheres
    for (size_t k = 0; k < spheres.size(); ++k) {
      if (raySphereIntersect(point + N * 1e-4f, L, spheres[k].pos, spheres[k].radius, tShadow)) {
        if (tShadow > 1e-6f) { inShadow = true; break; }
      }
    }

    // check triangles
    if (!inShadow) {
      for (size_t k = 0; k < triangles.size(); ++k) {
        const Triangle &tri = triangles[k];
        const vec3 &v0 = vertices[tri.v1];
        const vec3 &v1 = vertices[tri.v2];
        const vec3 &v2 = vertices[tri.v3];
        float t, u, v;
        if (rayTriangleIntersect(point + N * 1e-4f, L, v0, v1, v2, t, u, v)) {
          if (t > 1e-6f) { inShadow = true; break; }
        }
      }
    }

    if (!inShadow) {
      float NdotL = fmax(dot(N, L), 0.0f);
      vec3 diffuse = vec3(mat.dr, mat.dg, mat.db) * NdotL;
      diffuse = diffuse * vec3(dirLight.r, dirLight.g, dirLight.b);

      vec3 V = (eye - point).normalized();
      vec3 H = (L + V).normalized();
      float NdotH = fmax(dot(N, H), 0.0f);
      vec3 specular = vec3(mat.sr, mat.sg, mat.sb) * powf(NdotH, mat.ns);
      specular = specular * vec3(dirLight.r, dirLight.g, dirLight.b);

      color = color + diffuse + specular;
    }
  }

  return color.clampTo1();
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


vec3 traceRay(const vec3 &rayOrigin, const vec3 &rayDir, int depth) {
    if (depth > max_depth)
        return vec3(backgroundColor.x, backgroundColor.y, backgroundColor.z);

    float tNear = 1e20f;
    int hitSphereIndex = -1;
    int hitTriangleIndex = -1;
    float baryU = 0, baryV = 0;

    // Sphere intersections
    for (size_t k = 0; k < spheres.size(); k++) {
        float t;
        if (raySphereIntersect(rayOrigin, rayDir, spheres[k].pos, spheres[k].radius, t)) {
            if (t < tNear) {
                tNear = t;
                hitSphereIndex = k;
                hitTriangleIndex = -1;
            }
        }
    }

    // Triangle intersections
    for (size_t i = 0; i < triangles.size(); i++) {
        const Triangle& tri = triangles[i];
        const vec3& v0 = vertices[tri.v1];
        const vec3& v1 = vertices[tri.v2];
        const vec3& v2 = vertices[tri.v3];
        float t, u, v;
        if (rayTriangleIntersect(rayOrigin, rayDir, v0, v1, v2, t, u, v)) {
            if (t < tNear) {
                tNear = t;
                hitTriangleIndex = i;
                hitSphereIndex = -1;
                baryU = u;
                baryV = v;
            }
        }
    }

    // No intersection
    if (hitSphereIndex == -1 && hitTriangleIndex == -1)
        return vec3(backgroundColor.x, backgroundColor.y, backgroundColor.z);

    vec3 hitPoint = rayOrigin + rayDir * tNear;
    vec3 normal;
    const material* mat = nullptr;

    // normal 
    if (hitSphereIndex != -1) {
        const Sphere& s = spheres[hitSphereIndex];
        normal = (hitPoint - s.pos).normalized();
        mat = &materials[s.matIndex];
    } else {
        const Triangle& tri = triangles[hitTriangleIndex];
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
    vec3 localColor = calculateColor(hitPoint, normal, *mat, ambient_light, eye);

    // Reflection and Refraction 
    float reflectivity = fmax(mat->sr, fmax(mat->sg, mat->sb));
    float transparency = fmax(mat->tr, fmax(mat->tg, mat->tb));

    vec3 reflectedColor(0,0,0);
    vec3 refractedColor(0,0,0);

    // Reflection
    if (reflectivity > 0.0f) {
        vec3 R = reflect(rayDir, normal).normalized();
        reflectedColor = traceRay(offsetPoint, R, depth + 1) * reflectivity;
    }

    // Refraction
    if (transparency > 0.0f && mat->ior > 0.0f) {
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

    vec3 finalColor = localColor * (1 - reflectivity - transparency)
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
