// NovyPokus.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <cmath>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <string>

using namespace std;
using namespace pcl;
using namespace Eigen;	

//typedef visualization::PointCloudColorHandlerCustom<PointXYZRGB> ColorHandlerXYZRGB;

void printVertices(PointCloud<PointXYZRGB>::Ptr cloud) {
	for (size_t i = 0; i < cloud->points.size(); i++)
		cout << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << endl;
}

PointCloud<PointXYZRGB>::Ptr cloudFromPLYfile(string filename) {
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

	if (io::loadPLYFile<PointXYZRGB>(filename.c_str(), *cloud) == -1) //* load the file
	{
		PCL_ERROR("Nepodarilo sa nacitat subor \n");
		return NULL;
	}
	cout << "Zo suboru " << filename << " bolo nacitanych " << cloud->points.size() << " bodov. " << endl;
	return cloud;
}

float minParam(PointCloud<PointXYZRGB>::Ptr cloud, char param) {
	float min = 100;
	if(param == 'x')
		for (size_t i = 0; i < cloud->points.size(); i++) {
			if (cloud->points[i].x < min) min = cloud->points[i].x;
		}
	else if (param == 'y')
		for (size_t i = 0; i < cloud->points.size(); i++) {
			if (cloud->points[i].y < min) min = cloud->points[i].y;
		}
	else if (param == 'z')
		for (size_t i = 0; i < cloud->points.size(); i++) {
			if (cloud->points[i].z < min) min = cloud->points[i].z;
		}
	else if (param == 'r')
		for (size_t i = 0; i < cloud->points.size(); i++) {
			if (cloud->points[i].r < min) min = cloud->points[i].r;
		}
	else if (param == 'g')
		for (size_t i = 0; i < cloud->points.size(); i++) {
			if (cloud->points[i].g < min) min = cloud->points[i].g;
		}
	else if (param == 'b')
		for (size_t i = 0; i < cloud->points.size(); i++) {
			if (cloud->points[i].b < min) min = cloud->points[i].b;
		}
	return min;
}

float maxParam(PointCloud<PointXYZRGB>::Ptr cloud, char param) {
	float max = -100;
	if (param == 'x')
		for (size_t i = 0; i < cloud->points.size(); i++) {
			if (cloud->points[i].x < max) max = cloud->points[i].x;
		}
	else if (param == 'y')
		for (size_t i = 0; i < cloud->points.size(); i++) {
			if (cloud->points[i].y < max) max = cloud->points[i].y;
		}
	else if (param == 'z')
		for (size_t i = 0; i < cloud->points.size(); i++) {
			if (cloud->points[i].z < max) max = cloud->points[i].z;
		}
	else if (param == 'r')
		for (size_t i = 0; i < cloud->points.size(); i++) {
			if (cloud->points[i].r < max) max = cloud->points[i].r;
		}
	else if (param == 'g')
		for (size_t i = 0; i < cloud->points.size(); i++) {
			if (cloud->points[i].g < max) max = cloud->points[i].g;
		}
	else if (param == 'b')
		for (size_t i = 0; i < cloud->points.size(); i++) {
			if (cloud->points[i].b < max) max = cloud->points[i].b;
		}
	return max;
}

PointCloud<PointXYZRGB>::Ptr filterCloud(PointCloud<PointXYZRGB>::Ptr cloud) {
	// Filtracia - vzorkovanie na 1cm
	VoxelGrid<PointXYZRGB> vg;
	PointCloud<PointXYZRGB>::Ptr cloud_filtered(new PointCloud<PointXYZRGB>);
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_filtered);
	cout << "Mracno ma po filtracii " << cloud_filtered->points.size() << " bodov." << endl;
	return cloud;
}

void planarSegmentation(PointCloud<PointXYZRGB>::Ptr filtered) {
	SACSegmentation<PointXYZRGB> seg;
	PointIndices::Ptr inliers(new PointIndices);
	ModelCoefficients::Ptr coefs(new ModelCoefficients);
	PointCloud<PointXYZRGB>::Ptr plane(new PointCloud<PointXYZRGB>), cloud_f(new PointCloud<PointXYZRGB>);
	seg.setOptimizeCoefficients(true);
	seg.setModelType(SACMODEL_PLANE);
	seg.setMethodType(SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.02);

	int i = 0, nr_points = (int)filtered->points.size();
	while (filtered->points.size() > 0.3*nr_points) {
		// Segmentacia najvacsieho plosneho komponentu zo zvysneho mracna
		seg.setInputCloud(filtered);
		seg.segment(*inliers, *coefs);
		if (inliers->indices.size() == 0) {
			cout << "Nepodarilo sa odhadnut plosny model pre zadane data..." << endl;
			break;
		}
		// Vyber bodov patriacich ploche
		ExtractIndices<PointXYZRGB> extract;
		extract.setInputCloud(filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*plane);
		cout << "Plosny komponent mracna ma " << plane->points.size() << " bodov." << endl;
		// Odstranenie bodov plochy, vyber zo zvysneho mracna
		extract.setNegative(true);
		extract.filter(*cloud_f);
		*filtered = *cloud_f;
	}
}

vector<PointCloud<PointXYZRGB>::Ptr> getClustersFromCloud(PointCloud<PointXYZRGB>::Ptr cloud) {
	// Hladacia metoda vyberu pomocou kdTree
	search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>);
	tree->setInputCloud(cloud);
	vector<PointIndices> cluster_indices;
	vector<PointCloud<PointXYZRGB>::Ptr> clusters_vector;

	EuclideanClusterExtraction<PointXYZRGB> ec;
	ec.setClusterTolerance(0.15); //10cm
	ec.setMinClusterSize(1000);
	ec.setMaxClusterSize(200000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	int j = 0;
	for (vector<PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
		PointCloud<PointXYZRGB>::Ptr cluster(new PointCloud<PointXYZRGB>);
		for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			cluster->points.push_back(cloud->points[*pit]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;
		clusters_vector.push_back(cluster);
		cout << "Zhluk bodov c." << j + 1 << " obsahuje " << cluster->points.size() << " bodov." << endl;
		j++;
	}
	return clusters_vector;
}

PointCloud<PointXYZRGB>::Ptr pCloudFromMinMax(const PointXYZRGB &minPoint, const PointXYZRGB &maxPoint) {
	// Kalkulacia vrcholov kvadra z minimalneho a maximalneho bodu
	PointXYZRGB A, B, C, D, E, F, G, H;
	A.x = maxPoint.x;
	A.y = maxPoint.y;
	A.z = maxPoint.z;
	A.r = 0;
	A.g = 255;
	A.b = 255;
	B.x = maxPoint.x;
	B.y = minPoint.y;
	B.z = maxPoint.z;
	B.r = 0;
	B.g = 255;
	B.b = 255;
	C.x = maxPoint.x;
	C.y = minPoint.y;
	C.z = minPoint.z;
	C.r = 0;
	C.g = 255;
	C.b = 255;
	D.x = maxPoint.x;
	D.y = maxPoint.y;
	D.z = minPoint.z;
	D.r = 0;
	D.g = 255;
	D.b = 255;
	E.x = minPoint.x;
	E.y = maxPoint.y;
	E.z = minPoint.z;
	E.r = 0;
	E.g = 255;
	E.b = 255;
	F.x = minPoint.x;
	F.y = maxPoint.y;
	F.z = maxPoint.z;
	F.r = 0;
	F.g = 255;
	F.b = 255;
	G.x = minPoint.x;
	G.y = minPoint.y;
	G.z = maxPoint.z;
	G.r = 0;
	G.g = 255;
	G.b = 255;
	H.x = minPoint.x;
	H.y = minPoint.y;
	H.z = minPoint.z;
	H.r = 0;
	H.g = 255;
	H.b = 255;

	// Vytvorenie pCloudu
	PointCloud<PointXYZRGB>::Ptr box(new PointCloud<PointXYZRGB>);
	box->points.reserve(8);
	box->push_back(A);
	box->push_back(B);
	box->push_back(C);
	box->push_back(D);
	box->push_back(E);
	box->push_back(F);
	box->push_back(G);
	box->push_back(H);

	return box;
}

int main(int argc, char** argv)
{
	// Inicializacia
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	cout << "Nacitava sa mracno zo suboru..." << endl;
	cloud = cloudFromPLYfile("spojene.ply");
	/*// Print pozicii vrcholov mracna
	printVertices(cloud);
	*/

	// Vytvorenie noveho suboru s orezanou podlahou
	PointCloud<PointXYZRGB>::Ptr floorless(new PointCloud<PointXYZRGB>);
	floorless->reserve(cloud->size());
	/*// Zistenie rozsahu na urcenie thresholdu pre orezanie podlahy
	float minZ = minParam(cloud, 'z');
	float maxZ = maxParam(cloud, 'z');
	cout << "Najnizsie 'z' z povodneho suboru: " << minZ << endl;
	cout << "Najvyssie 'z' z povodneho suboru: " << maxZ << endl;
	*/
	float thresholdZ = 0.001;
	cout << "Z mracna sa orezava podlaha..." << endl;
	for (size_t i = 0; i < cloud->points.size(); i++)
		if ((fabs(cloud->points[i].z) - thresholdZ) >= 0)
			floorless->push_back(cloud->points[i]);
	cout << "Mracno ma po orezani podlahy " << floorless->points.size() << " bodov." << endl;
	cout << "Nasleduje vytvorenie noveho floorles.ply suboru... [ENTER]" << endl;
	//cin.ignore();
	cout << "Orezane mracno sa zapisuje do suboru..." << endl;
	io::savePLYFileASCII("floorless.ply", *floorless);

	// Segmentacia
	PointCloud<PointXYZRGB>::Ptr filtered(new PointCloud<PointXYZRGB>);
	vector<PointCloud<PointXYZRGB>::Ptr> clusters;
	filtered->reserve(floorless->size());
	cout << "Prebieha filtracia mracna..." << endl;
	filtered = filterCloud(floorless);
	cout << "Filtrovane mracno sa zapisuje do suboru..." << endl;
	io::savePLYFileASCII("filtered.ply", *filtered);
	cout << "Prebieha plosna segmentacia..." << endl;
	planarSegmentation(filtered);
	cout << "Prebieha clustering..." << endl;
	clusters = getClustersFromCloud(filtered);
	cout << "Nasleduje vytvorenie samostatnych suborov pre zhluky... [ENTER]" << endl;
	//cin.ignore();

	// Prechadzanie kazdeho clustra
	cout << "Mracna clustrov sa zapisuju do suborov..." << endl;
	for (int j = 0; j < clusters.size();j++) {
		// Vytvorenie jednotlivych suborov s clustrami
		stringstream clusterfile;
		clusterfile << "cluster_" << j + 1 << ".ply";
		io::savePLYFileASCII(clusterfile.str(), *clusters[j]);

		// Vypocet normal
		Vector4f pcaCentroid;
		compute3DCentroid(*clusters[j], pcaCentroid);
		Matrix3f covariance;
		computeCovarianceMatrixNormalized(*clusters[j], pcaCentroid, covariance);
		SelfAdjointEigenSolver<Matrix3f> eigen_solver(covariance, ComputeEigenvectors);
		Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
		eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	
		// Transformacia mracna tak, aby hlavne komponenty korespondovali s priestorovymi osami
		Matrix4f projectionTransform(Matrix4f::Identity());
		projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
		projectionTransform.block<3, 1>(0, 3) = -1.f*(projectionTransform.block<3, 3>(0, 0)*pcaCentroid.head<3>());
		PointCloud<PointXYZRGB>::Ptr cloudPointsProjected(new PointCloud<PointXYZRGB>);
		transformPointCloud(*clusters[j], *cloudPointsProjected, projectionTransform);

		// Ziskanie minimalneho a maximalneho bodu transformovaneho mracna
		PointXYZRGB minPoint, maxPoint;
		getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
		const Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
		
		// Ziskanie mracna boxu v strede suradnicovej sustavy
		PointCloud<PointXYZRGB>::Ptr boxCloudCenter(new PointCloud<PointXYZRGB>);
		boxCloudCenter = pCloudFromMinMax(minPoint, maxPoint);
		//printVertices(boxCloudCenter);
		cout << "Body bounding boxu su stanovene pre stred suradnicovej sustavy... [ENTER]" << endl;
		cin.ignore();
		cout << "PCA centroid.head<3> : " << pcaCentroid.head<3>() << endl;

		// Spatna transformacia mracna
		const Quaternionf bboxQuaternion(eigenVectorsPCA);
		const Vector3f bboxTransform = eigenVectorsPCA*meanDiagonal + pcaCentroid.head<3>();

		PointCloud<PointXYZRGB>::Ptr boxCloud(new PointCloud<PointXYZRGB>);
		boxCloud->points.resize(8);
		for (int k = 0;k < 8;k++) {
			const Vector3f work ( boxCloudCenter->points[k].x, boxCloudCenter->points[k].y, boxCloudCenter->points[k].z);
			const Vector3f bboxPointTransform = eigenVectorsPCA * work + pcaCentroid.head<3>();
			boxCloud->points[k].x = bboxPointTransform[0];
			boxCloud->points[k].y = bboxPointTransform[1];
			boxCloud->points[k].z = bboxPointTransform[2];
		}
		stringstream boxfile;
		boxfile << "box_" << j + 1 << ".ply";
		io::savePLYFileASCII(boxfile.str(), *boxCloud);
		/*// Vizualizacia bounding boxu
		visualization::PCLVisualizer *visu;
		visu = new visualization::PCLVisualizer(argc, argv, "PlyViewer");
		int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
		visu->createViewPort(0.0, 0.5, 0.5, 1.0, mesh_vp_1);
		visu->createViewPort(0.5, 0.5, 1.0, 1.0, mesh_vp_2);
		visu->createViewPort(0.0, 0, 0.5, 0.5, mesh_vp_3);
		visu->createViewPort(0.5, 0, 1.0, 0.5, mesh_vp_4);
		visu->addPointCloud(clusters[j], ColorHandlerXYZRGB(clusters[j], 30, 144, 255), "bboxedCloud", mesh_vp_3);
		visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox", mesh_vp_3);
*/
	}


	
	

	cout << "!!!CONGRATS!!! Bol dosiahnuty koniec programu... [ENTER]";
	cin.ignore();

	return (0);
}