#ifndef CHART_CONSTRUCTION_H
#define CHART_CONSTRUCTION_H

#include "TexturedMesh.h"
#include "Atlas.h"

#include <queue>
#include <set>

void AddComponent(std::vector< int >& vertexComponent, int vIndex, int currentComponent, const std::vector< std::vector< int > >& neighbours)
{
	vertexComponent[vIndex] = currentComponent;
	std::queue< int > visitingQueue;
	visitingQueue.push(vIndex);

	while (!visitingQueue.empty())
	{
		int currentVertex = visitingQueue.front();
		visitingQueue.pop();
		const std::vector< int >& vertexNeighbours = neighbours[currentVertex];
		for (int i = 0; i < vertexNeighbours.size(); i++)
		{
			if (vertexComponent[vertexNeighbours[i]] == -1)
			{
				vertexComponent[vertexNeighbours[i]] = currentComponent;
				visitingQueue.push(vertexNeighbours[i]);
			}
			else if (vertexComponent[vertexNeighbours[i]] == currentComponent);
			else Miscellany::Throw("Unexpected Condition on a connected component. Expected %d. Obtained %d.\n", currentComponent, vertexComponent[vertexNeighbours[i]]);
		}
	}
}

void InitializeTriangleChartIndexing
(
	const TexturedMesh &mesh,
	std::vector< int > &chartIndex,
	int &numCharts
)
{
	std::unordered_map< unsigned long long, int > edgeIndex;
	for (int i = 0; i < mesh.triangles.size(); i++) {
		for (int k = 0; k < 3; k++) {
			unsigned long long edgeKey = SetMeshEdgeKey(mesh.triangles[i][k], mesh.triangles[i][(k + 1) % 3]);
			if (edgeIndex.find(edgeKey) == edgeIndex.end())
				edgeIndex[edgeKey] = 3 * i + k;
			else std::cout << "Non manifold Mesh" << std::endl;
		}
	}

	// find neighbours triangles
	std::vector< std::vector< int > > neighbours(mesh.triangles.size());
	for (int i = 0; i < mesh.triangles.size(); i++) {
		for (int k = 0; k < 3; k++) {
			// find inverse half edge
			unsigned long long edgeKey = SetMeshEdgeKey(mesh.triangles[i][(k + 1) % 3], mesh.triangles[i][k]);
			if (edgeIndex.find(edgeKey) != edgeIndex.end()) {
				int tIndex = edgeIndex[edgeKey] / 3;
				int kIndex = edgeIndex[edgeKey] % 3;
				// make sure continuity in uv space (not the chart boundary)
				if (mesh.textureCoordinates[3 * i + (k + 1) % 3][0] == mesh.textureCoordinates[3 * tIndex + kIndex][0] &&
					mesh.textureCoordinates[3 * i + (k + 1) % 3][1] == mesh.textureCoordinates[3 * tIndex + kIndex][1] &&
					mesh.textureCoordinates[3 * i + k][0] == mesh.textureCoordinates[3 * tIndex + (kIndex + 1) % 3][0] &&
					mesh.textureCoordinates[3 * i + k][1] == mesh.textureCoordinates[3 * tIndex + (kIndex + 1) % 3][1])
				{
					neighbours[i].push_back(tIndex);
				}
			}
		}
	}
	chartIndex.clear();
	chartIndex.resize(mesh.triangles.size(), -1);
	int currentComponent = -1;
	for (int v = 0; v < mesh.triangles.size(); v++) if (chartIndex[v] == -1)
	{
		currentComponent++;
		AddComponent(chartIndex, v, currentComponent, neighbours);
	}
	numCharts = currentComponent + 1;
}

void InitializeAtlasMesh
(
	const TexturedMesh &inputMesh,
	AtlasMesh &outputMesh,
	const int width,const int height
)
{
	// get charts num and fine each triangle belong 
	InitializeTriangleChartIndexing(inputMesh, outputMesh.triangleChartIndex, outputMesh.numCharts);

	int lastVertexIndex = 0;
	std::set< IndexedVector2D, IndexedVector2DComparison > IndexedPointSet;
	typename std::set< IndexedVector2D, IndexedVector2DComparison >::iterator it;

	// compute the mesh assignment in UV space
	for (int t = 0; t < inputMesh.triangles.size(); t++)
	{
		int cornerIndices[3];
		for (int k = 0; k < 3; k++)
		{
			int currentCorner = -1;
			IndexedVector2D idxP(inputMesh.textureCoordinates[3 * t + k], lastVertexIndex, inputMesh.triangles[t][k]);
			it = IndexedPointSet.find(idxP);
			if (it == IndexedPointSet.end())
			{
				IndexedPointSet.insert(idxP);
				outputMesh.vertexMap.push_back(inputMesh.triangles[t][k]);
				currentCorner = lastVertexIndex;
				outputMesh.vertices.push_back(inputMesh.textureCoordinates[3 * t + k]);
				lastVertexIndex++;
			}
			else
			{
				IndexedVector2D indexPoint = *it;
				currentCorner = indexPoint.index;
			}
			cornerIndices[k] = currentCorner;
		}
		outputMesh.triangles.push_back(TriangleIndex(cornerIndices[0], cornerIndices[1], cornerIndices[2]));	
	}

	// Jitter vertices lying on the grid to avoid degeneracies
	if (true)
	{
		double precision = (double)1e-6;
		for (int i = 0; i < outputMesh.vertices.size(); i++) {
			for (int c = 0; c < 2; c++)
			{
				double dimSize = c == 0 ? (double)width : (double)height;
				double scaled = outputMesh.vertices[i][c] * dimSize - 0.5;
				double offset = scaled - round(scaled);
				if (fabs(offset) < precision)
				{
					if (offset > 0)
						scaled = round(scaled) + precision * (1 + Random<double>());
					else
						scaled = round(scaled) - precision * (1 + Random<double>());
					outputMesh.vertices[i][c] = (scaled + 0.5) / dimSize;
				}
			}
		}
	}

	std::unordered_map< unsigned long long, int > halfEdgeIndex;
	for (int i = 0; i < outputMesh.triangles.size(); i++) {
		for (int k = 0; k < 3; k++) {

		}
	}

}

void Initialize
(
	TexturedMesh &mesh,const int width,const int height,
	std::vector< TextureNodeInfo > &textureNodes,
	std::vector< BilinearElementIndex > &cellNode,
	std::vector< AtlasChart > &atlasCharts
)
{
	// Initialize Atlas Mesh
	AtlasMesh atlasMesh;
	std::vector< int > oppositeHalfEdge;
	std::unordered_map< int, int > boundaryVertucesIndices;
	int numBoundaryVertices;
	bool isClosedMesh;

	InitializeAtlasMesh(mesh, atlasMesh, width, height);
}

#endif