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

	Eigen::SparseMatrix< double > systemMat;

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


	/*====== Programming Assignment 2 ======*/
}
