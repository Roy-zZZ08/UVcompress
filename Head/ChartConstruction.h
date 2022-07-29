#ifndef CHART_CONSTRUCTION_H
#define CHART_CONSTRUCTION_H

#include "TexturedMesh.h"
#include "Atlas.h"
#include "Util.h"

#include <queue>
#include <set>

// ----------------------------------------------------
// InitializeAtlasMesh
// ----------------------------------------------------

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

void InitializeTriangleChartIndexing(const TexturedMesh &mesh, std::vector< int > &chartIndex, int &numCharts)
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

void InitializeAtlasMesh(const TexturedMesh &inputMesh, AtlasMesh &outputMesh, const int width,const int height)
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
			unsigned long long edgeKey = SetMeshEdgeKey(outputMesh.triangles[i][k], outputMesh.triangles[i][(k + 1) % 3]);
			if (halfEdgeIndex.find(edgeKey) == halfEdgeIndex.end())
				halfEdgeIndex[edgeKey] = 3 * i + k;
			else Miscellany::Throw("Non oriented manifold mesh");
		}
	}

	int gridResolution = std::max< int >(width, height);
	int lastEdgeIndex = 2 * gridResolution * gridResolution * outputMesh.numCharts; // what for
	std::vector< int > halfEdgeToEdgeIndex(3 * outputMesh.triangles.size(), -1);

	for (int i = 0; i < outputMesh.triangles.size(); i++) {
		for (int k = 0; k < 3; k++) {
			int currentEdgeIndex = 3 * i + k;
			unsigned long long edgeKey = SetMeshEdgeKey(outputMesh.triangles[i][(k + 1) % 3], outputMesh.triangles[i][k]);
			if (halfEdgeIndex.find(edgeKey) != halfEdgeIndex.end())
			{
				int oppsiteEdgeIndex = halfEdgeIndex[edgeKey];

				if (currentEdgeIndex < oppsiteEdgeIndex)
				{
					halfEdgeToEdgeIndex[currentEdgeIndex] = halfEdgeToEdgeIndex[oppsiteEdgeIndex] = lastEdgeIndex;
					lastEdgeIndex++;
				}
			}
			else
			{
				halfEdgeToEdgeIndex[currentEdgeIndex] = lastEdgeIndex;
				lastEdgeIndex++;
			}
		}
	}

	for (int i = 0; i < outputMesh.triangles.size(); i++) {
		for (int k = 0; k < 3; k++) {
			if (halfEdgeToEdgeIndex[3 * i + k] == -1)
				Miscellany::Throw("Non indexed half edge");
		}
	}

	outputMesh.halfEdgeToEdgeIndex = halfEdgeToEdgeIndex;
}

// ----------------------------------------------------
// InitializeBoundaryHalfEdges
// ----------------------------------------------------

void InitializeBoundaryHalfEdges
(
	const TexturedMesh &mesh,
	std::vector< int > &boundaryHalfEdges,
	std::vector< int > &oppositeHalfEdge,
	std::vector< bool > &isBoundaryHalfEdge,
	bool  &isClosedMesh
)
{
	isClosedMesh = true;

	std::unordered_map< unsigned long long, int > edgeIndex;
	for (int i = 0; i < mesh.triangles.size(); i++) {
		for (int k = 0; k < 3; k++) {
			unsigned long long edgeKey = SetMeshEdgeKey(mesh.triangles[i][k], mesh.triangles[i][(k + 1) % 3]);
			if (edgeIndex.find(edgeKey) == edgeIndex.end())edgeIndex[edgeKey] = 3 * i + k;
			else Miscellany::Throw("Non manifold mesh");
		}
	}

	oppositeHalfEdge.resize(3 * mesh.triangles.size());
	isBoundaryHalfEdge.resize(3 * mesh.triangles.size(), false);

	for (int i = 0; i < mesh.triangles.size(); i++) {
		for (int k = 0; k < 3; k++) {
			int currentEdgeIndex = 3 * i + k;
			unsigned long long edgeKey = SetMeshEdgeKey(mesh.triangles[i][(k + 1) % 3], mesh.triangles[i][k]);
			if (edgeIndex.find(edgeKey) != edgeIndex.end())
			{
				int oppositeEdgeIndex = edgeIndex[edgeKey];

				if (currentEdgeIndex < oppositeEdgeIndex) {
					oppositeHalfEdge[currentEdgeIndex] = oppositeEdgeIndex;
					oppositeHalfEdge[oppositeEdgeIndex] = currentEdgeIndex;
				}

				int tIndex = oppositeEdgeIndex / 3;
				int kIndex = oppositeEdgeIndex % 3;
				if (mesh.textureCoordinates[3 * i + (k + 1) % 3][0] == mesh.textureCoordinates[3 * tIndex + kIndex][0] &&
					mesh.textureCoordinates[3 * i + (k + 1) % 3][1] == mesh.textureCoordinates[3 * tIndex + kIndex][1] &&
					mesh.textureCoordinates[3 * i + k][0] == mesh.textureCoordinates[3 * tIndex + (kIndex + 1) % 3][0] &&
					mesh.textureCoordinates[3 * i + k][1] == mesh.textureCoordinates[3 * tIndex + (kIndex + 1) % 3][1]);
				else // not continuous in UV space
				{
					if (currentEdgeIndex < oppositeEdgeIndex)
					{
						boundaryHalfEdges.push_back(currentEdgeIndex);
						boundaryHalfEdges.push_back(oppositeEdgeIndex);
						isBoundaryHalfEdge[currentEdgeIndex] = isBoundaryHalfEdge[oppositeEdgeIndex] = true;
					}
				}
			}
			else
			{
				isClosedMesh = false;
				oppositeHalfEdge[currentEdgeIndex] = -1;
				boundaryHalfEdges.push_back(currentEdgeIndex);
				isBoundaryHalfEdge[currentEdgeIndex] = true;
			}
		}
	}
}

// ----------------------------------------------------
// InitiallizeBoundaryVertices
// ----------------------------------------------------

void InitiallizeBoundaryVertices
(
	const TexturedMesh &mesh,
	const std::vector< int > &boundaryHalfEdges,
	std::unordered_map< int , int > &boundaryVerticesIndices,
	int &lastBoundaryIndex
)
{
	lastBoundaryIndex = 0;
	for (int b = 0; b < boundaryHalfEdges.size(); b++)
	{
		int halfEdgeIndex = boundaryHalfEdges[b];
		int i = halfEdgeIndex / 3;
		int k = halfEdgeIndex % 3;
		if (boundaryVerticesIndices.find(mesh.triangles[i][k]) == boundaryVerticesIndices.end())
			boundaryVerticesIndices[mesh.triangles[i][k]] = lastBoundaryIndex++;
		if (boundaryVerticesIndices.find(mesh.triangles[i][(k+1)%3]) == boundaryVerticesIndices.end())
			boundaryVerticesIndices[mesh.triangles[i][(k+1)%3]] = lastBoundaryIndex++;
	}
}

// ----------------------------------------------------
// InitializeAtlasCharts
// ----------------------------------------------------

void InitializeAtlasCharts
(
	AtlasMesh &atlasMesh,
	const std::vector< bool > &isBoundaryHalfEdge,
	int width,int height,
	std::vector< AtlasChart > &atlasCharts
)
{
	atlasCharts.resize(atlasMesh.numCharts);

	for (int i = 0; i < atlasMesh.numCharts; i++)
	{
		atlasCharts[i].minCorner = Point2D< double >(1, 1);
		atlasCharts[i].maxCorner = Point2D< double >(0, 0);
	}

	std::vector< int > lastVertexID(atlasMesh.numCharts, 0);
	std::vector< int > chartVertexID(atlasMesh.vertices.size(), -1);

	atlasMesh.triangleIndexInChart.resize(atlasMesh.triangles.size());
	std::vector< int > lastTriangleID(atlasMesh.numCharts, 0);

	for (int t = 0; t < atlasMesh.triangles.size(); t++)
	{
		int chartID = atlasMesh.triangleChartIndex[t];

		atlasMesh.triangleIndexInChart[t] = lastTriangleID[chartID];
		lastTriangleID[chartID]++;

		atlasCharts[chartID].meshTriangleIndices.push_back(t);
		int vertexID[3];
		for (int k = 0; k < 3; k++)
		{
			atlasCharts[chartID].atlasEdgeIndices.push_back(atlasMesh.halfEdgeToEdgeIndex[3 * t + k]);
			int atlasVertexID = atlasMesh.triangles[t][k];
			if (chartVertexID[atlasVertexID] == -1)
			{
				chartVertexID[atlasVertexID] = lastVertexID[chartID];
				lastVertexID[chartID]++;
				Point2D< double > vertexPos = atlasMesh.vertices[atlasVertexID];
				for (int c = 0; c < 2; c++)
				{
					atlasCharts[chartID].minCorner[c] = std::min< double >(vertexPos[c], atlasCharts[chartID].minCorner[c]);
					atlasCharts[chartID].maxCorner[c] = std::max< double >(vertexPos[c], atlasCharts[chartID].maxCorner[c]);
				}

				atlasCharts[chartID].vertices.push_back(vertexPos);
				atlasCharts[chartID].meshVertexIndices.push_back(atlasMesh.vertexMap[atlasVertexID]);
			}
			vertexID[k] = chartVertexID[atlasVertexID];
		}
		atlasCharts[chartID].triangles.push_back(TriangleIndex(vertexID[0], vertexID[1], vertexID[2]));
	}

	for (int i = 0; i < atlasCharts.size(); i++)
	{
		Point2D< double > midBBox = (atlasCharts[i].minCorner + atlasCharts[i].maxCorner) / 2;
		midBBox[0] *= width;
		midBBox[1] *= height;
		midBBox -= Point2D< double >(0.5, 0.5);
		midBBox = Point2D< double >(floor(midBBox[0]), floor(midBBox[1])); //?
		atlasCharts[i].originCoords[0] = (int)round(midBBox[0]);
		atlasCharts[i].originCoords[1] = (int)round(midBBox[1]);

		midBBox += Point2D< double >(0.5, 0.5);
		midBBox[0] /= width;
		midBBox[1] /= height;
		atlasCharts[i].gridOrigin = midBBox;
	}
}

// ----------------------------------------------------
// InitializeGridChartsActiveNodes
// ----------------------------------------------------

//Node type : inactive(-1) , exterior (0), interior boundary (1), interior deep (2) hybryd (both deep and boundary for the solver)(3).
//Cell type : inactive(-1) , boundary (0), interior (1).
void InitializeGridChartsActiveNodes
(
	const int chartID,
	const AtlasChart &atlasChart,
	GridChart &gridChart
)
{
	int width = gridChart.width;
	int height = gridChart.height;
	double cellSizeW = gridChart.cellSizeW;
	double cellSizeH = gridChart.cellSizeH;

	Image< int >& nodeType = gridChart.nodeType;
	nodeType.resize(width, height);
	for (int i = 0; i < nodeType.size(); i++) nodeType[i] = -1;

	Image< int > nodeOwner;
	nodeOwner.resize(width, height);

	Image< int > &cellType = gridChart.cellType;
	cellType.resize(width - 1, height - 1);
	for (int i = 0; i < cellType.size(); i++) cellType[i] = -1;

	Image< int > &triangleID = gridChart.triangleID;
	triangleID.resize(width, height);
	for (int i = 0; i < triangleID.size(); i++) triangleID[i] = -1;

	Image< Point2D< double > > &barycentricCoords = gridChart.barycentricCoords;
	barycentricCoords.resize(width, height);

	// 1.Add interior texels
	for (int t = 0; t < atlasChart.triangles.size(); t++)
	{
		Point2D< double > tPos[3];
		for (int i = 0; i < 3; i++) tPos[i] = atlasChart.vertices[atlasChart.triangles[t][i]] - gridChart.corner;

		int minCorner[2];
		int maxCorner[2];
		GetTriangleIntegerBBox(tPos, 1. / cellSizeW, 1. / cellSizeH, minCorner, maxCorner);

		SquareMatrix< double, 2 > barycentricMap = GetBarycentricMap(tPos);

		for (int j = minCorner[1]; j <= maxCorner[1]; j++) {
			for (int i = minCorner[0]; i <= maxCorner[0]; i++) {
				Point2D< double > texelPos = Point2D< double >(i * cellSizeW, j * cellSizeH) - tPos[0];
				Point2D< double > barycentricCoord = barycentricMap * texelPos;
				if (barycentricCoord[0] >= 0 && barycentricCoord[1] >= 0 && (barycentricCoord[0] + barycentricCoord[1]) <= 1)
				{
					if (nodeType(i, j) != -1) 
						Miscellany::Throw("Node ( %d , %d ) in chart %d already covered", i, j, chartID);

					nodeType(i, j) = 1;
					triangleID(i, j) = atlasChart.meshTriangleIndices[t];
					barycentricCoords(i, j) = barycentricCoord;
				}
			}
		}
		
	}

	// 2.Add texels adjacent to boundary cells;
	int r;
}

void Initialize
(
	TexturedMesh &mesh,const int width,const int height,
	std::vector< TextureNodeInfo > &textureNodes,
	std::vector< BilinearElementIndex > &cellNode,
	std::vector< AtlasChart > &atlasCharts
)
{
	AtlasMesh atlasMesh;
	std::vector< int > oppositeHalfEdge;
	std::unordered_map< int, int > boundaryVerticesIndices;
	int numBoundaryVertices;
	bool isClosedMesh;

	InitializeAtlasMesh(mesh, atlasMesh, width, height);

	std::vector< int > boundaryHalfEdges;
	std::vector< bool > isBoundaryHalfEdge;
	InitializeBoundaryHalfEdges(mesh, boundaryHalfEdges, oppositeHalfEdge, isBoundaryHalfEdge, isClosedMesh);

	int lastBoundaryIndex;
	InitiallizeBoundaryVertices(mesh, boundaryHalfEdges, boundaryVerticesIndices, lastBoundaryIndex);

	numBoundaryVertices = lastBoundaryIndex;

	InitializeAtlasCharts(atlasMesh, isBoundaryHalfEdge, width, height, atlasCharts);

	std::vector< GridChart > gridCharts;
	gridCharts.resize(atlasCharts.size());

	for (int i = 0; i < atlasCharts.size(); i++)
	{
		double cellSizeW = 1. / width;
		double cellSizeH = 1. / height;

		int halfSize[2][2];

		for (int c = 0; c < 2; c++)
		{
			if (c == 0)
			{
				halfSize[c][0] = (int)ceil((atlasCharts[i].gridOrigin[c] - atlasCharts[i].minCorner[c]) / cellSizeW);
				halfSize[c][1] = (int)ceil((atlasCharts[i].maxCorner[c] - atlasCharts[i].gridOrigin[c]) / cellSizeW);
				gridCharts[i].corner[c] = atlasCharts[i].gridOrigin[c] - cellSizeW * halfSize[c][0];
			}
			else
			{
				halfSize[c][0] = (int)ceil((atlasCharts[i].gridOrigin[c] - atlasCharts[i].minCorner[c]) / cellSizeH);
				halfSize[c][1] = (int)ceil((atlasCharts[i].maxCorner[c] - atlasCharts[i].gridOrigin[c]) / cellSizeH);
				gridCharts[i].corner[c] = atlasCharts[i].gridOrigin[c] - cellSizeH * halfSize[c][0];
			}
		}
		
		gridCharts[i].width = halfSize[0][0] + halfSize[0][1] + 1;
		gridCharts[i].height = halfSize[1][0] + halfSize[1][1] + 1;
		gridCharts[i].cellSizeW = cellSizeW;
		gridCharts[i].cellSizeH = cellSizeH;
		InitializeGridChartsActiveNodes(i, atlasCharts[i], gridCharts[i]);
	}
}

#endif