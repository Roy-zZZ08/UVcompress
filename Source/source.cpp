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

   mesh.read("Resource/test/test.obj", "Resource/test/test1.jpg");

    Initialize(mesh, mesh.texture.width(), mesh.texture.height(), textureNodes, bilinearElementIndices, atlasCharts);
    
    return 0;
}
