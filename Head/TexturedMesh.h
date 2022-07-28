#ifndef TEXTURED_MESH_H
#define TEXTURED_MESH_H

#include <igl/readOBJ.h>
#include <Eigen/Sparse>
#include <Misha/Geometry.h>
#include <Misha/Image.h>
#include <Misha/Miscellany.h>
#include <Src/VectorIO.h>

#include "Atlas.h"

unsigned long long SetMeshEdgeKey(const unsigned long i0, const unsigned long i1)
{
	return (((static_cast<unsigned long long>(i0) << 32) & 0xFFFFFFFF00000000) | (static_cast<unsigned long long>(i1) & 0x00000000FFFFFFFF));
}

class TexturedMesh
{
public: 
	std::vector< Point3D< double > > vertices;
	std::vector< Point3D< double > > normals;
	std::vector< TriangleIndex > triangles;
	std::vector< Point2D< double > > textureCoordinates;
	Image< Point3D< float > > texture;
	
	void read(const char* meshName, const char* atlasName)
	{
		vertices.clear();
		triangles.clear();
		textureCoordinates.clear();

		Eigen::MatrixXd V, TC, N;
		Eigen::MatrixXi F, FTC, FN;

		igl::readOBJ(meshName, V, TC, N, F, FTC, FN);

		vertices.resize(V.rows());
		for (int i = 0; i < V.rows(); i++) vertices[i] = Point3D< double >(V(i, 0), V(i, 1), V(i, 2));

		triangles.resize(F.rows());
		textureCoordinates.resize(3 * F.rows());
		for (int i = 0; i < F.rows(); i++)
		{
			for (int j = 0; j < 3; j++)
			{
				triangles[i][j] = F(i, j);
				textureCoordinates[3 * i + j] = Point2D< double >(TC(FTC(i, j), 0), TC(FTC(i, j), 1));
			}
		}
		updateNormals();

		if (atlasName)
		{
			char* ext = GetFileExtension(atlasName);

			if (!strcasecmp(ext, "normap"))
			{
				printf("Reading normal texture\n");
				ReadBinaryImage(texture, atlasName);
			}
			else
			{
				printf("Reading color texture\n");
				texture.read(atlasName);
			}
			delete[] ext;
		}
	}

	void updateNormals(void)
	{
		normals.clear();
		normals.resize(vertices.size());
		for (int t = 0; t < triangles.size(); t++)
		{
			Point3D< double > d01 = vertices[triangles[t][1]] - vertices[triangles[t][0]];
			Point3D< double > d02 = vertices[triangles[t][2]] - vertices[triangles[t][0]];
			Point3D< double > n = Point3D< double >::CrossProduct(d01, d02);
			for (int v = 0; v < 3; v++) normals[triangles[t][v]] += n;
		}

		// normalization
		for (int i = 0; i < normals.size(); i++)
		{
			double l = Point3D< double >::Length(normals[i]);
			if (l > 0) normals[i] /= l;
		}
	}
};
#endif