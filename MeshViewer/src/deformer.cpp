#include "deformer.h"
#include <iostream>

Deformer::Deformer() : mMesh(nullptr),
mCholeskySolver(nullptr) {
}

Deformer::~Deformer() {
	clear();
}

void Deformer::clear() {
	if (mCholeskySolver) {
		delete mCholeskySolver;
	}
	mCholeskySolver = nullptr;
	mRoiList.clear();
}

void Deformer::setMesh(Mesh* mesh) {
	mMesh = mesh;
	clear();
	// Record the handle vertices
	for (Vertex* vert : mMesh->vertices()) {
		if (vert->flag() > 0 || vert->isBoundary()) {
			mRoiList.push_back(vert);
		}
	}
	// Build system matrix for deformation
	buildSystemMat();
}

void Deformer::buildSystemMat() {
	/*====== Programming Assignment 2 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Build the matrix of the linear system for
	/* deformation and do factorization, in order
	/* to reuse and speed up in Deformer::deform().
	/* Handle vertices are maked by Vertex::flag() > 0
	/* Movements of the specified handle are already
	/* recorded in Vertex::position()
	/**********************************************/
	int numVertices = mMesh->vertices().size();
	int numConstraints = mRoiList.size();
	mA = Eigen::SparseMatrix<double>(numVertices + numConstraints, numVertices);
	mB = Eigen::MatrixX3d(numVertices + numConstraints, 3);
	for (Vertex *vert : mMesh->vertices()) {
		int i = vert->index();
		OneRingVertex orv(vert);
		Vertex *curr = nullptr;
		std::vector<Vertex *> adjVertices;
		while (curr = orv.nextVertex()) {
			adjVertices.push_back(curr);
		}
		int n = adjVertices.size();
		std::vector<double> weights(n);
		double weightSum = 0;
		for (int k = 0; k < n; ++k) {
			const Eigen::Vector3f &prev = adjVertices[k]->position();
			const Eigen::Vector3f &curr = adjVertices[(k + 1) % n]->position();
			const Eigen::Vector3f &next = adjVertices[(k + 2) % n]->position();
			double cot1 = triangleCot(vert->position(), prev, curr);
			double cot2 = triangleCot(vert->position(), next, curr);
			double weight = 0.5 * (cot1 + cot2);
			weights[(k + 1) % n] = weight;
			weightSum += weight;
		}
		Eigen::Vector3d delta_i = -vert->position().cast<double>();
		for (int k = 0; k < n; ++k) {
			int j = adjVertices[k]->index();
			mA.insert(i, j) = weights[k] / weightSum;
			delta_i += weights[k] / weightSum * adjVertices[k]->position().cast<double>();
		}
		mA.insert(i, i) = -1;
		mB.row(i) = delta_i;
	}

	for (int k = 0; k < numConstraints; ++k) {
		int i = mRoiList[k]->index();
		mA.insert(numVertices + k, i) = 1;
	}

	Eigen::SparseMatrix< double > systemMat = mA.transpose() * mA;
	/*====== Programming Assignment 2 ======*/

	// Please refer to the following link for the usage of sparse linear system solvers in Eigen
	// https://eigen.tuxfamily.org/dox/group__TopicSparseSystems.html

	// Do factorization
	if (systemMat.nonZeros() > 0) {
		mCholeskySolver = new Eigen::SimplicialLDLT< Eigen::SparseMatrix< double > >();
		mCholeskySolver->compute(systemMat);
		if (mCholeskySolver->info() != Eigen::Success) {
			// Decomposition failed
			std::cout << "Sparse decomposition failed\n";
		}
		else {
			std::cout << "Sparse decomposition succeeded\n";
		}
	}
}

void Deformer::deform() {
	if (mCholeskySolver == nullptr) {
		return;
	}

	/*====== Programming Assignment 2 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* This is the place where the editing techniques
	/* take place.
	/* Solve for the new vertex positions after the
	/* specified handles move using the factorized
	/* matrix from Deformer::buildSystemMat(), i.e.,
	/* mCholeskySolver defined in deformer.h
	/**********************************************/

	// Please refer to the following link for the usage of sparse linear system solvers in Eigen
	// https://eigen.tuxfamily.org/dox/group__TopicSparseSystems.html

	int numVertices = mMesh->vertices().size();
	int numConstraints = mRoiList.size();

	for (int k = 0; k < numConstraints; ++k) {
		mB.row(numVertices + k) = mRoiList[k]->position().cast<double>();
	}
	Eigen::MatrixX3f newPositions(numVertices, 3);
	for (int d = 0; d < 3; ++d) {
		Eigen::VectorXd rhs = mA.transpose() * mB.col(d);
		Eigen::VectorXd x = mCholeskySolver->solve(rhs);
		newPositions.col(d) = x.cast<float>();
	}

	for (Vertex *vert : mMesh->vertices()) {
		int i = vert->index();
		Eigen::Vector3f newPosition = newPositions.row(i);
		vert->setPosition(newPosition);
	}
	/*====== Programming Assignment 2 ======*/
}
