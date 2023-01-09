#ifndef REMESH_MESH_H
#define REMESH_MESH_H

#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <Eigen/Sparse>
#include <Misha/Geometry.h>
#include <Misha/Image.h>
#include <Misha/Miscellany.h>
#include <Src/VectorIO.h>

class RemeshMesh
{
public:
	std::vector< Point3D< double > > vertices;
	std::vector< Point2D< double > > textureCoordinates;
	std::vector< Point3D< double > > normals; // Öð¶¥µã¼ÇÂ¼
	std::vector< TriangleIndex > triangles;
	std::vector< TriangleIndex > coordinateIndexs;
	std::vector< int > deleteTri;
	int deleteNum;

	void read(const char* meshName)
	{
		deleteNum = 0;
		vertices.clear();
		triangles.clear();
		normals.clear();
		textureCoordinates.clear();
		coordinateIndexs.clear();

		Eigen::MatrixXd V, TC, N;
		Eigen::MatrixXi F, FTC, FN;

		igl::readOBJ(meshName, V, TC, N, F, FTC, FN);

		vertices.resize(V.rows());
		for (int i = 0; i < V.rows(); i++) vertices[i] = Point3D< double >(V(i, 0), V(i, 1), V(i, 2));
		textureCoordinates.resize(TC.rows());
		for (int i = 0; i < TC.rows(); i++) textureCoordinates[i] = Point2D< double >(TC(i, 0), TC(i, 1));
		normals.resize(N.rows());
		for (int i = 0; i < N.rows(); i++) normals[i] = Point3D< double >(N(i, 0), N(i, 1), N(i, 2));
		triangles.resize(F.rows());
		coordinateIndexs.resize(F.rows());
		deleteTri.resize(F.rows());

		for (int i = 0; i < F.rows(); i++)
		{
			deleteTri[i] = 0;
			for (int j = 0; j < 3; j++)
			{
				triangles[i][j] = F(i, j);
				coordinateIndexs[i][j] = FTC(i, j);
			}
		}

		updateNormals();

	}

	void save(const char* meshName)
	{
		Eigen::MatrixXd newV, newTC, newN;
		Eigen::MatrixXi newF, newFTC;

		newV.resize(vertices.size(), 3);
		for (int i = 0; i < vertices.size(); i++) for (int k = 0; k < 3; k++) {
			newV(i, k) = vertices[i][k];
		}
		newTC.resize(textureCoordinates.size(), 2);
		for (int i = 0; i < textureCoordinates.size(); i++) for (int k = 0; k < 2; k++) {
			newTC(i, k) = textureCoordinates[i][k];
		}
		newN.resize(normals.size(), 3);
		for (int i = 0; i < normals.size(); i++) for (int k = 0; k < 3; k++) {
			newN(i, k) = normals[i][k];
		}

		int newTriNum = triangles.size() - deleteNum;
		newF.resize(newTriNum, 3);
		newFTC.resize(newTriNum, 3);

		int tmp = 0;
		for (int i = 0; i < triangles.size(); i++) {
			if (i >= deleteTri.size() || (i < deleteTri.size() && !deleteTri[i])) {
				for (int k = 0; k < 3; k++)
				{
					newF(tmp, k) = triangles[i][k];
					newFTC(tmp, k) = coordinateIndexs[i][k];
				}
				tmp++;
			}
		}

		igl::writeOBJ(meshName, newV, newF, newN, newF, newTC, newFTC);
	}

	// delete unused vertices
	void saveClean(const char* meshName)
	{
		Eigen::MatrixXd newV, newTC, newN;
		Eigen::MatrixXi newF, newFTC;

		// record used vertices
		set<int> usedVerts;

		for (int i = 0; i < triangles.size(); i++) {
			if (i >= deleteTri.size() || (i < deleteTri.size() && !deleteTri[i])) {
				for (int k = 0; k < 3; k++)
				{
					usedVerts.insert(triangles[i][k]);
				}
			}
		}

		int newVertsNum = usedVerts.size();
		vector<int> newVertIndex(vertices.size());
		newV.resize(newVertsNum, 3);
		newN.resize(newVertsNum, 3);
		int nowIndex = 0;
		for (int i = 0; i < vertices.size(); i++) {
			if (usedVerts.find(i) != usedVerts.end()) {
				newVertIndex[i] = nowIndex;
				for (int k = 0; k < 3; k++) {
					newV(nowIndex, k) = vertices[i][k];
					newN(nowIndex, k) = normals[i][k];
				}
				nowIndex++;
			}
		}

		newTC.resize(textureCoordinates.size(), 2);
		for (int i = 0; i < textureCoordinates.size(); i++) for (int k = 0; k < 2; k++) {
			newTC(i, k) = textureCoordinates[i][k];
		}

		int newTriNum = triangles.size() - deleteNum;
		newF.resize(newTriNum, 3);
		newFTC.resize(newTriNum, 3);

		int tmp = 0;
		for (int i = 0; i < triangles.size(); i++) {
			if (i >= deleteTri.size() || (i < deleteTri.size() && !deleteTri[i])) {
				for (int k = 0; k < 3; k++)
				{
					newF(tmp, k) = newVertIndex[triangles[i][k]];
					newFTC(tmp, k) = coordinateIndexs[i][k];
				}
				tmp++;
			}
		}

		igl::writeOBJ(meshName, newV, newF, newN, newF, newTC, newFTC);
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