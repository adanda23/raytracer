#ifndef PARSE_VEC3_H
#define PARSE_VEC3_H

#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

// Camera & Scene Parameters (Global Variables)
// Here we set default values, override them in parseSceneFile()

// Triangles
std::vector<vec3> vertices;
std::vector<vec3> normals;
std::vector<Triangle> triangles;


// Image Parameters
int img_width = 800, img_height = 600;
std::string imgName = "raytraced.png";

// Camera Parameters
vec3 eye = vec3(0, 0, 0);
vec3 forward = vec3(0, 0, -1).normalized();
vec3 up = vec3(0, 1, 0).normalized();
vec3 right;
float halfAngleVFOV = 35;

// Scene (Spheres) Parameters
std::vector<Sphere> spheres;

// Materials
std::vector<material> materials;

// Lighting
std::vector<light> directional_lights;
std::vector<light> point_lights;
spotlight spot_light;
vec3 ambient_light = vec3(0, 0, 0);

vec3 backgroundColor = vec3(0, 0, 0);
int max_depth = 5;

void parseSceneFile(std::string fileName) {
  std::ifstream inputFile(fileName);
  std::string line;

  while (std::getline(inputFile, line)) {
    // Skip empty lines
    size_t firstChar = line.find_first_not_of(" \t");
    if (firstChar == std::string::npos)
      continue;
    // Skip comment lines
    if (line[firstChar] == '#')
      continue;

    std::istringstream iss(line);
    std::string word;
    iss >> word;

    if (word == "camera_pos:") {
      iss >> eye.x >> eye.y >> eye.z;
    } else if (word == "camera_fwd:") {
      iss >> forward.x >> forward.y >> forward.z;
    } else if (word == "camera_up:") {
      iss >> up.x >> up.y >> up.z;
    } else if (word == "camera_fov_ha:") {
      iss >> halfAngleVFOV;
    } else if (word == "sphere:") {
      Sphere s;
      iss >> s.pos.x >> s.pos.y >> s.pos.z >> s.radius;
      s.matIndex = (materials.size() > 0) ? materials.size() - 1 : 0;
      spheres.push_back(s);
    } else if (word == "material:") {
      material matTemp;
      iss >> matTemp.ar >> matTemp.ag >> matTemp.ab >> matTemp.dr >>
          matTemp.dg >> matTemp.db >> matTemp.sr >> matTemp.sg >> matTemp.sb >>
          matTemp.ns >> matTemp.tr >> matTemp.tg >> matTemp.tb >> matTemp.ior;
      materials.push_back(matTemp);
    } else if (word == "directional_light:") {
    light l;
    iss >> l.r >> l.g >> l.b >> l.x >> l.y >> l.z;
    directional_lights.push_back(l);

    } else if (word == "point_light:") {
        light l;
        iss >> l.r >> l.g >> l.b >> l.x >> l.y >> l.z;
        point_lights.push_back(l);
    } else if (word == "spot_light:") {
      iss >> spot_light.r >> spot_light.g >> spot_light.b >> spot_light.px >>
          spot_light.py >> spot_light.pz >> spot_light.dx >> spot_light.dy >>
          spot_light.dz >> spot_light.angle1 >> spot_light.angle2;
    } else if (word == "ambient_light:") {
      iss >> ambient_light.x >> ambient_light.y >> ambient_light.z;
    } else if (word == "output_image:") {
      iss >> imgName;
    } else if (word == "background:") {
      iss >> backgroundColor.x >> backgroundColor.y >> backgroundColor.z;
    } else if (word == "max_depth:") {
      iss >> max_depth;
    } else if (word == "image_resolution:") {
      iss >> img_width >> img_height;
    }
    else if (word == "max_vertices:") {
    int n; iss >> n;
    vertices.reserve(n);
    }
    else if (word == "max_normals:") {
        int n; iss >> n;
        normals.reserve(n);
    }
    else if (word == "vertex:") {
        vec3 v; iss >> v.x >> v.y >> v.z;
        vertices.push_back(v);
    }
    else if (word == "normal:") {
        vec3 n; iss >> n.x >> n.y >> n.z;
        normals.push_back(n);
    }
    else if (word == "triangle:") {
        Triangle t;
        iss >> t.v1 >> t.v2 >> t.v3;
        t.hasVertexNormals = false;
        t.n1 = t.n2 = t.n3 = -1;
        t.matIndex = materials.size() - 1;
        triangles.push_back(t);
    }
    else if (word == "normal_triangle:") {
        Triangle t;
        iss >> t.v1 >> t.v2 >> t.v3 >> t.n1 >> t.n2 >> t.n3;
        t.hasVertexNormals = true;
        t.matIndex = materials.size() - 1;
        triangles.push_back(t);
    }

  }

  right = cross(up, forward).normalized();
  forward = forward.normalized();
  up = up.normalized();

  if (dot(right, forward) != 0 || dot(forward, up) != 0 ||
      dot(up, right) != 0) {
    forward = forward.normalized();
    up = (up - dot(up, forward) * forward).normalized();
    right = cross(up, forward).normalized();
  }

  printf("Orthogonal Camera Basis:\n");
  printf("forward: %f,%f,%f\n", forward.x, forward.y, forward.z);
  printf("right: %f,%f,%f\n", right.x, right.y, right.z);
  printf("up: %f,%f,%f\n", up.x, up.y, up.z);
}

#endif
