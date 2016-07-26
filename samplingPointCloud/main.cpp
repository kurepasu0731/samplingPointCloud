#pragma once

#define NOMINMAX
#include <Windows.h>
#include <math.h>

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>

//�_�E���T���v�����O
pcl::PointCloud<pcl::PointXYZ>::Ptr getDownSampledPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float size);
//�@���x�N�g�������߂�
pcl::PointCloud<pcl::Normal>::Ptr getNormalVectors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
//�O�p���b�V���𐶐��A���b�V������Ԃ�
pcl::PolygonMesh getMeshVectors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);
//PLY�`���ŕۑ�(�@������,mesh����)
void savePLY_with_normal_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PolygonMesh meshes, const std::string &fileName);

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	//ply�t�@�C���ǂݍ���
	pcl::PLYReader reader;
	std::string filename = "mesh.ply";
	reader.read<pcl::PointXYZ>(filename, *cloud);

	//�_�E���T���v�����O
	pcl::PointCloud<pcl::PointXYZ>::Ptr sampledCloud = getDownSampledPoints(cloud, 0.005f);

	//�@���x�N�g�������߂�
	pcl::PointCloud<pcl::Normal>::Ptr normalCloud = getNormalVectors(sampledCloud);

	//���b�V�������߂�
	pcl::PolygonMesh meshes = getMeshVectors(sampledCloud, normalCloud);

	//ply�`���ŕۑ�
	std::string dstfilename = "final_mesh.ply";
	savePLY_with_normal_mesh(sampledCloud, normalCloud, meshes, dstfilename);

	return 0;
}

//�_�E���T���v�����O
pcl::PointCloud<pcl::PointXYZ>::Ptr getDownSampledPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float size)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (size, size, size);
	sor.filter (*cloud_filtered);

	return cloud_filtered;
}

//�@���x�N�g�������߂�
pcl::PointCloud<pcl::Normal>::Ptr getNormalVectors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	  // Create the normal estimation class, and pass the input dataset to it
	  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	  ne.setInputCloud (cloud);

	  // Create an empty kdtree representation, and pass it to the normal estimation object.
	  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	  ne.setSearchMethod (tree);

	  // Output datasets
	  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	  // Use all neighbors in a sphere of radius 3cm
	  ne.setRadiusSearch (0.03);

	  // Compute the features
	  ne.compute (*cloud_normals);

	  return cloud_normals;
}

//�O�p���b�V���𐶐��A���b�V������Ԃ�
pcl::PolygonMesh getMeshVectors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *cloud_normals, *cloud_with_normals);

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.025);

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	return triangles;
}

//PLY�`���ŕۑ�(�@������,mesh����)
void savePLY_with_normal_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PolygonMesh meshes, const std::string &fileName)
{
	//�t�@�C���I�[�v��
	FILE *fp;
	fp = fopen(fileName.data(), "w");

	//�t�@�C���ɏ�������
	//�w�b�_�̐ݒ�
	fprintf(fp,"ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nelement face %d\nproperty list ushort int vertex_indices\nend_header\n", cloud->size(), meshes.polygons.size());

	//3�����_�Q
	//m�P�ʂŕۑ��ixml��mm�j
	for (int n = 0; n < cloud->size(); n++){
		fprintf(fp, "%f %f %f %f %f %f \n", cloud->at(n).x, cloud->at(n).y, cloud->at(n).z, 
															cloud_normals->at(n).normal_x, cloud_normals->at(n).normal_y, cloud_normals->at(n).normal_z);
	}
	//�ʏ��L�q
	for(int n = 0; n < meshes.polygons.size(); n++)
	{
	   fprintf(fp, "3 %d %d %d\n", meshes.polygons[n].vertices[0], meshes.polygons[n].vertices[1], meshes.polygons[n].vertices[2]);

	}
	//�t�@�C���N���[�Y
	fclose(fp);
}
