/*
 * SpectralClustering.cpp
 *
 *  Created on: 04-Mar-2009
 *      Author: sbutler
 */

#include "SkeletonFinder/spectral/SpectralClustering.h"
#include "SkeletonFinder/spectral/ClusterRotate.h"
#include "SkeletonFinder/spectral/Kmeans.h"

#include <Eigen/QR>

/**
* Performs eigenvector decomposition of an affinity matrix
*
* @param data 		the affinity matrix
* @param numDims	the number of dimensions to consider when clustering
*/
SpectralClustering::SpectralClustering(Eigen::MatrixXd& data, int numDims):
	mNumDims(numDims),
	mNumClusters(0)
{
	Eigen::VectorXd degrees = data.rowwise().sum();
	Eigen::MatrixXd DegInvSqrt = Eigen::MatrixXd::Zero(data.rows(),data.cols());

	if (!data.isApprox(data.transpose())) {
		std::cerr << "Input matrix is not symmetric!" << std::endl;
	}

	// calc normalised laplacian 
	for (int i = 0; i < data.rows(); ++i) {
        if (degrees(i) > 0) {
            DegInvSqrt(i, i) = 1.0 / std::sqrt(degrees(i));
        } else {
            DegInvSqrt(i, i) = 0; // Handle zero degree nodes
        }
    }
	Eigen::MatrixXd Lapla = Eigen::MatrixXd::Identity(data.rows(), data.cols()) - DegInvSqrt * data * DegInvSqrt;

	// Check if the Laplacian is symmetric
    if (!Lapla.isApprox(Lapla.transpose(), 1e-10)) { // Use tolerance for floating point comparisons
        std::cerr << "Laplacian matrix is not symmetric!" << std::endl;
    } else {
        std::cout << "Laplacian matrix is symmetric." << std::endl;
    }

	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> s(Lapla);
	Eigen::VectorXd val = s.eigenvalues();
	Eigen::MatrixXd vec = s.eigenvectors();

	//check for nans in the eigenvalues
	for (int i = 0; i < val.size(); i++) {
		if (std::isnan(val(i))) {
			std::cerr << "NaN in eigenvalues!" << std::endl;
		}
	}

	//sort eigenvalues/vectors
	int n = data.cols();
	for (int i = 0; i < n - 1; ++i) {
		int k;
		val.segment(i, n - i).maxCoeff(&k);
		if (k > 0) {
			std::swap(val[i], val[k + i]);
			vec.col(i).swap(vec.col(k + i));
		}
	}

	//choose the number of eigenvectors to consider
	if (mNumDims < vec.cols()) {
		mEigenVectors = vec.block(0,0,vec.rows(),mNumDims);
	} else {
		mEigenVectors = vec;
	}
}

SpectralClustering::~SpectralClustering() {
}

/**
 * Cluster by rotating the eigenvectors and evaluating the quality
 */
std::vector<std::vector<int> > SpectralClustering::clusterRotate() {

	ClusterRotate* c = new ClusterRotate();
	std::vector<std::vector<int> > clusters = c->cluster(mEigenVectors);

	mNumClusters = clusters.size();

	return clusters;
}

/**
 * Cluster by kmeans
 *
 * @param numClusters	the number of clusters to assign
 */
std::vector<std::vector<int> > SpectralClustering::clusterKmeans(int numClusters) {
	mNumClusters = numClusters;
	return Kmeans::cluster(mEigenVectors, numClusters);
}
