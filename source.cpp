#include <igl/cotmatrix.h>
#include <igl/readOBJ.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "TexturedMesh.h"
#include "ChartConstruction.h"
#include "Atlas.h"

#include <iostream>


TexturedMesh mesh;
std::vector< TextureNodeInfo > textureNodes;
std::vector< BilinearElementIndex > bilinearElementIndices;
std::vector< AtlasChart > atlasCharts;

int main()
{

    mesh.read("Resource/dolphin/cube.obj", "Resource/dolphin/dolphin_Diffuse_low.jpg");

    Initialize(mesh, mesh.texture.width(), mesh.texture.height(), textureNodes, bilinearElementIndices, atlasCharts);
    
    return 0;
}
