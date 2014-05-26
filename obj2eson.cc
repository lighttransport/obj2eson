/*
Copyright (c) 2014, Light Transport Entertainment, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Light Transport Entertainment, Inc nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "tiny_obj_loader.h"

#include <string>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>


#include "picojson.h"
#include "eson.h"

static std::string GetBaseFilename(const std::string &FileName) {
  if (FileName.find_last_of(".") != std::string::npos)
    return FileName.substr(0, FileName.find_last_of("."));
  return "";
}

class
vector3
{
public:

  vector3() : x(0), y(0), z(0) {}
  vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
  ~vector3() {}

  float& operator[](size_t i) {
    if (i == 0) return x;
    if (i == 1) return y;
    else return z;
  }

  void normalize() {
    double d = x * x + y * y + z * z;
    double len = sqrt(d);
    if (fabs(len) > 1.0e-30) {
      x /= len;
      y /= len;
      z /= len;
    }
  }

  vector3 operator-(const vector3& rhs) {
    return vector3(rhs.x - x, rhs.y - y, rhs.z - z);
  }

  float x, y, z;
};

inline vector3 operator-(const vector3& v) {
    return vector3(-v.x, -v.y, -v.z);
  }


inline vector3 cross(const vector3& a, const vector3& b)
{
  vector3 c;
  c.x = a.y * b.z - a.z * b.y;
  c.y = a.z * b.x - a.x * b.z;
  c.z = a.x * b.y - a.y * b.x;

  return c;
}

static vector3
ComputeNormal(
  vector3 v0,
  vector3 v1,
  vector3 v2)
{
  // Geometric normal.
  vector3 p10 = v1 - v0;
  vector3 p20 = v2 - v0;
  vector3 n   = cross(p20, p10);
  n.normalize();

  return -n;
}

static void
JSONFromDoubleArray(
    picojson::array& arr,
    std::vector<double>& values)
{
    for (size_t i = 0; i < values.size(); i++) {
        arr.push_back(picojson::value(values[i]));
    }
}


static bool
DumpMaterials(
  std::vector<tinyobj::shape_t>& shapes,
  const std::string& filename)
{

  picojson::array arr;
  for (size_t i = 0; i < shapes.size(); i++) {
    picojson::object obj;

    std::vector<double> kd;
    kd.push_back(shapes[i].material.diffuse[0]);
    kd.push_back(shapes[i].material.diffuse[1]);
    kd.push_back(shapes[i].material.diffuse[2]);

    std::vector<double> ka;
    ka.push_back(shapes[i].material.ambient[0]);
    ka.push_back(shapes[i].material.ambient[1]);
    ka.push_back(shapes[i].material.ambient[2]);

    std::vector<double> ks;
    ks.push_back(shapes[i].material.specular[0]);
    ks.push_back(shapes[i].material.specular[1]);
    ks.push_back(shapes[i].material.specular[2]);

    std::vector<double> kt;
    kt.push_back(shapes[i].material.transmittance[0]);
    kt.push_back(shapes[i].material.transmittance[1]);
    kt.push_back(shapes[i].material.transmittance[2]);

    picojson::object params;

    picojson::array kdArr;
    picojson::array kaArr;
    picojson::array ksArr;
    picojson::array ktArr;
    JSONFromDoubleArray(kdArr, kd);
    JSONFromDoubleArray(kaArr, ka);
    JSONFromDoubleArray(ksArr, ks);
    JSONFromDoubleArray(ktArr, kt);

    params["diffuse"] = picojson::value(kdArr);
    params["ambient"] = picojson::value(kaArr);
    params["reflection"] = picojson::value(ksArr);
    params["refraction"] = picojson::value(ktArr);

    obj["index"] = picojson::value((double)(i));
    obj["id"] = picojson::value("tmp"); // @todo
    obj["type"] = picojson::value("obj_material");
    obj["params"] = picojson::value(params);

    arr.push_back(picojson::value(obj));
  }

  std::ofstream ofs(filename.c_str());
  if (!ofs) return false;

  picojson::value v = picojson::value(arr);
  std::string str = v.serialize();
  ofs << str << std::endl;

  return true;
}

static bool
DumpMeshes(
  std::vector<tinyobj::shape_t>& shapes,
  const std::string& filename)
{
  // flatten
  std::vector<float> vertices;
  std::vector<float> normals;
  std::vector<float> tangents;
  std::vector<float> binormals;
  std::vector<float> uvs;
  std::vector<int> indices;
  std::vector<unsigned short> materialIDs;
  int foffset = 0;

  unsigned long long nverts = 0;
  unsigned long long nnormals = 0;
  unsigned long long nfaces = 0;

  int maxidx = 0;
  for (int i = 0; i < shapes.size(); i++) {
    int vsize = shapes[i].mesh.positions.size() / 3;
    int fsize = shapes[i].mesh.indices.size() / 3;

    // P
    for (int j = 0; j < vsize; j++) {
      vertices.push_back(shapes[i].mesh.positions[3*j+0]);
      vertices.push_back(shapes[i].mesh.positions[3*j+1]);
      vertices.push_back(shapes[i].mesh.positions[3*j+2]);
    }

    // facevarying N
    //printf("ns = %d, idxs = %d\n", meshes[i].normals.size(), meshes[i].indices.size());
    if ((shapes[i].mesh.normals.size() / 3) != shapes[i].mesh.indices.size()) {

      if (shapes[i].mesh.normals.empty()) {
        // Compute geometric normal
        for (int j = 0; j < fsize; j++) {
          
          int idx0 = shapes[i].mesh.indices[3*j+0];
          int idx1 = shapes[i].mesh.indices[3*j+1];
          int idx2 = shapes[i].mesh.indices[3*j+2];

          vector3 v0, v1, v2;
          for (int k = 0; k < 3; k++) {
            v0[k] = shapes[i].mesh.positions[3*idx0+k];
            v1[k] = shapes[i].mesh.positions[3*idx1+k];
            v2[k] = shapes[i].mesh.positions[3*idx2+k];
          }

          vector3 n = ComputeNormal(v0, v1, v2);

          // same N for all triangle vertex.
          for (int k = 0; k < 3; k++) {
            normals.push_back(n[0]);
            normals.push_back(n[1]);
            normals.push_back(n[2]);
          }
        }

      } else {
        // make N facevarying
        for (int j = 0; j < fsize; j++) {
          
          for (int k = 0; k < 3; k++) {
            int idx = shapes[i].mesh.indices[3*j+k];
            normals.push_back(shapes[i].mesh.normals[3*idx+0]);
            normals.push_back(shapes[i].mesh.normals[3*idx+1]);
            normals.push_back(shapes[i].mesh.normals[3*idx+2]);
          }
        }
      }
    } else {
      for (int j = 0; j < fsize * 3; j++) {
        normals.push_back(shapes[i].mesh.normals[3*j+0]);
        normals.push_back(shapes[i].mesh.normals[3*j+1]);
        normals.push_back(shapes[i].mesh.normals[3*j+2]);
      }
    }

    // facevaryingT(todo)
    {
      // Fill with zero values.
      for (int j = 0; j < fsize * 3; j++) {
        tangents.push_back(0.0f);
        tangents.push_back(0.0f);
        tangents.push_back(0.0f);
      }
    }

    // facevaryingB(todo)
    {

      // Fill with zero values.
      for (int j = 0; j < fsize * 3; j++) {
        binormals.push_back(0.0f);
        binormals.push_back(0.0f);
        binormals.push_back(0.0f);
      }

    }

    // facevarying UV
    if (shapes[i].mesh.texcoords.empty()) {

      // Fill with dummy value;
      for (int j = 0; j < fsize * 3; j++) {
        uvs.push_back(0.0f);
        uvs.push_back(0.0f);
      }

    } else {

      if ((shapes[i].mesh.texcoords.size() /2) != shapes[i].mesh.indices.size()) {
        if (shapes[i].mesh.texcoords.empty()) {
          for (int j = 0; j < fsize; j++) {
            
            for (int k = 0; k < 3; k++) {
              int idx = shapes[i].mesh.indices[3*j+k];
              uvs.push_back(0.0f);
              uvs.push_back(0.0f);
              //printf("uv[%d] = %f, %f\n", meshes[i].uvs[2*idx+0], meshes[i].uvs[2*idx+1]);
            }
          }
        } else {
          // make UV facevarying
          for (int j = 0; j < fsize; j++) {
            
            for (int k = 0; k < 3; k++) {
              int idx = shapes[i].mesh.indices[3*j+k];
              uvs.push_back(shapes[i].mesh.texcoords[2*idx+0]);
              uvs.push_back(shapes[i].mesh.texcoords[2*idx+1]);
              //printf("uv[%d] = %f, %f\n", meshes[i].uvs[2*idx+0], meshes[i].uvs[2*idx+1]);
            }
          }
        }
      } else {
        for (int j = 0; j < fsize * 3; j++) {
          assert(j < shapes[i].mesh.texcoords.size());
          uvs.push_back(shapes[i].mesh.texcoords[2*j+0]);
          uvs.push_back(shapes[i].mesh.texcoords[2*j+1]);
          //printf("uv[%d] = %f, %f\n", meshes[i].uvs[2*j+0], meshes[i].uvs[2*j+1]);
        }
      }
    }

    // face
    for (int j = 0; j < fsize; j++) {
      indices.push_back(foffset + shapes[i].mesh.indices[3*j+0]);
      indices.push_back(foffset + shapes[i].mesh.indices[3*j+1]);
      indices.push_back(foffset + shapes[i].mesh.indices[3*j+2]);
      //printf("%d, %d, %d\n",
      //  shapes[i].mesh.indices[3*j+0],
      //  shapes[i].mesh.indices[3*j+1],
      //  shapes[i].mesh.indices[3*j+2]);
    }

    for (int j = 0; j < fsize; j++) {
      materialIDs.push_back((unsigned short)(i));
    }

    foffset += vsize;
    nverts += vsize;
    nfaces += fsize;
    nnormals += 3 * fsize; // facevary

  }

  // mesh out
  eson::Object mesh;
  mesh["num_vertices"] = eson::Value((int64_t)nverts);
  mesh["num_faces"]    = eson::Value((int64_t)nfaces);
  mesh["vertices"]     = eson::Value((uint8_t*)&vertices[0], sizeof(float)*nverts*3);
  mesh["faces"]        = eson::Value((uint8_t*)&indices[0], sizeof(unsigned int)*nfaces*3);

  if (!normals.empty()) {
    mesh["facevarying_normals"] = eson::Value((uint8_t*)&normals[0], sizeof(float)*nnormals*3);
  }
  if (!tangents.empty()) {
    mesh["facevarying_tangents"] = eson::Value((uint8_t*)&tangents[0], sizeof(float)*nnormals*3);
  }
  if (!binormals.empty()) {
    mesh["facevarying_binormals"] = eson::Value((uint8_t*)&binormals[0], sizeof(float)*nnormals*3);
  }
  if (!uvs.empty()) {
    mesh["facevarying_uvs"] = eson::Value((uint8_t*)&uvs[0], sizeof(float)*nnormals*2);
  }

  mesh["material_ids"] = eson::Value((uint8_t*)&materialIDs[0], sizeof(unsigned short)*nfaces);
  
  eson::Value v = eson::Value(mesh);
  int64_t size = v.Size();
  
  std::vector<uint8_t> buf(size);
  uint8_t* ptr = &buf[0]; 

  ptr = v.Serialize(ptr);
  
  assert((ptr-&buf[0]) == size);

  FILE* fp = fopen(filename.c_str(), "wb");
  fwrite(&buf[0], 1, size, fp);
  fclose(fp);

  return true;
}

int
main(
  int argc,
  char **argv)
{
  if (argc < 3) {
    printf("Usage: obj2mesh input.obj output.eson\n");
    exit(-1);
  }

  std::string inputfile(argv[1]);
  std::string outputfile(argv[2]);

  std::vector<tinyobj::shape_t> shapes;

  std::string err = tinyobj::LoadObj(shapes, inputfile.c_str());

  if (!err.empty()) {
    std::cerr << err << std::endl;
    exit(-1);
  }

  printf("# of shapes = %ld\n", shapes.size());
  for (size_t i = 0; i < shapes.size(); i++) {
    printf("shape[%ld] # of indices = %ld\n", i, shapes[i].mesh.indices.size() / 3);
  }

  bool ret = DumpMeshes(shapes, outputfile);
  assert(ret);

  std::string basename = GetBaseFilename(outputfile);

  std::string materialfilename = basename + ".material.json";
  ret = DumpMaterials(shapes, materialfilename);
  assert(ret);

  return 0;
}
