#include "mex.h"
#include "matrix.h"
#include <iostream>
#include </usr/local/include/eigen3/Eigen/Eigen>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <ctime>
#include "/usr/local/include/nabo/nabo.h"
#include "stdlib.h"
#include <chrono>

Eigen::Matrix4d ICP_SVD_with_AnchorPoints(Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, int);
Eigen::MatrixXd get_rob_T_part(Eigen::MatrixXd, Eigen::MatrixXd);
Eigen::MatrixXd mean(Eigen::MatrixXd);
Eigen::MatrixXd apply_transformation(Eigen::MatrixXd, Eigen::Matrix4d);

void mexFunction (int _OutArgs, mxArray *MatlabOut[], int _InArgs, const mxArray *MatlabIn[] )
{
    // Define Input
    Eigen::Map<Eigen::ArrayXXd,Eigen::Aligned> part_pts (mxGetPr(MatlabIn[0]), mxGetNumberOfElements(MatlabIn[0])/3, 3); 
    Eigen::Map<Eigen::ArrayXXd,Eigen::Aligned> scan_pts (mxGetPr(MatlabIn[1]), mxGetNumberOfElements(MatlabIn[1])/3, 3); 
    Eigen::Map<Eigen::ArrayXXd,Eigen::Aligned> model_ptcloud (mxGetPr(MatlabIn[2]), mxGetNumberOfElements(MatlabIn[2])/3, 3); 
    Eigen::Map<Eigen::ArrayXXd,Eigen::Aligned> scan_ptcloud (mxGetPr(MatlabIn[3]), mxGetNumberOfElements(MatlabIn[3])/3, 3); 
    
    // Method 
    int max_iter = 300;
    Eigen::Matrix4d Transform_mat_new = ICP_SVD_with_AnchorPoints(model_ptcloud, scan_ptcloud, part_pts, scan_pts, max_iter);
    
    // Define Output
    MatlabOut[0] = mxCreateDoubleMatrix(4,4,mxREAL);
    Eigen::Map<Eigen::ArrayXXd,Eigen::Aligned> M0 ( mxGetPr(MatlabOut[0]),4,4);
    M0 = Transform_mat_new.array();  
}

Eigen::Matrix4d ICP_SVD_with_AnchorPoints(Eigen::MatrixXd model_ptcloud, Eigen::MatrixXd scan_ptcloud, Eigen::MatrixXd part_pts, Eigen::MatrixXd scan_pts, int max_iter)
{
    // NOTE: gives transformation matrix scan_T_model (part w.r.t. robot frame assuming that scan_ptcloud is with respect to robot frame) 

    Eigen::Matrix4d T_init = Eigen::Matrix4d::Identity();
    if (part_pts.rows()>0 && scan_pts.rows()>0)
    {
        T_init = get_rob_T_part(part_pts,scan_pts);
    }
    Eigen::MatrixXd scan_ptcloud_transformed = apply_transformation(scan_ptcloud,T_init.inverse());
    scan_ptcloud = scan_ptcloud_transformed;
    // create a kd-tree for M, note that M must stay valid during the lifetime of the kd-tree
    Eigen::MatrixXd model_ptcloud_t = model_ptcloud.transpose();
    Nabo::NNSearchD * nd = Nabo::NNSearchD::createKDTreeLinearHeap(model_ptcloud_t);
    Eigen::Matrix4d Transform_mat_new = T_init.inverse();
    Eigen::Matrix4d Transform_mat = Eigen::Matrix4d::Constant(0);
    int k = 1;
    double tol = 1e-6;
    int count = 0;
    Eigen::MatrixXi idx(k,scan_ptcloud.rows());
    Eigen::MatrixXd dist(k,scan_ptcloud.rows());
    Eigen::MatrixXd corresponding_val_from_model_ptcloud(scan_ptcloud.rows(), scan_ptcloud.cols());    
    for (unsigned int iter=0;iter<max_iter;++iter)
    {
        nd->knn(scan_ptcloud.transpose(), idx, dist, k);   
        for (unsigned int i=0;i<idx.cols();++i)
        {
            corresponding_val_from_model_ptcloud.row(i) = model_ptcloud.row(idx(0,i));    
        }
        // get transformation matrix
        Transform_mat = get_rob_T_part(corresponding_val_from_model_ptcloud,scan_ptcloud);
        Transform_mat_new = Transform_mat.inverse()*Transform_mat_new;       
        if(Transform_mat(0,3)<tol && Transform_mat(1,3)<tol && Transform_mat(2,3)<tol) 
        {
            count++;
            if(count>5)
            {
                break;
            }
        }
        scan_ptcloud_transformed = apply_transformation(scan_ptcloud,Transform_mat.inverse());
        scan_ptcloud = scan_ptcloud_transformed;
    }
    delete(nd);
    return Transform_mat_new.inverse();
}

Eigen::MatrixXd get_rob_T_part(Eigen::MatrixXd part_pts, Eigen::MatrixXd rob_pts)
{
// part_pts: co-ordinates with respect to CAD part frame
// rob_pts: points with respect to robot base frame (or world frame if it matches with robot base frame)
// input: part_pts = [x1, y1, z1;
//                             x2, y2, z2;
//                                :
//                                :
//                             xn, yn, zn]
// input: rob_pts = [x1, y1, z1;
//                            x2, y2, z2;
//                                :
//                                :
//                            xn, yn, zn]
    Eigen::MatrixXd centroid_part_pts(1,part_pts.cols());
    Eigen::MatrixXd centroid_rob_pts(1,rob_pts.cols());
    Eigen::MatrixXd shifted_part_pts(part_pts.rows(),part_pts.cols());
    Eigen::MatrixXd shifted_rob_pts(rob_pts.rows(),rob_pts.cols());
    Eigen::MatrixXd cros_cov_mat(part_pts.cols(),rob_pts.cols());
    Eigen::Matrix3d R;
    Eigen::Matrix3d U_T;
    Eigen::Matrix3d V;
    Eigen::Vector3d T;
    Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
    Eigen::Matrix4d transform_mat = Eigen::Matrix4d::Constant(0);
    if (part_pts.rows()==rob_pts.rows())
    { 
        centroid_part_pts = mean(part_pts);
        centroid_rob_pts = mean(rob_pts);
        shifted_part_pts.col(0) = part_pts.col(0).array() - centroid_part_pts(0,0);
        shifted_part_pts.col(1) = part_pts.col(1).array() - centroid_part_pts(0,1);
        shifted_part_pts.col(2) = part_pts.col(2).array() - centroid_part_pts(0,2);
		shifted_rob_pts.col(0) = rob_pts.col(0).array() - centroid_rob_pts(0,0);
		shifted_rob_pts.col(1) = rob_pts.col(1).array() - centroid_rob_pts(0,1);
		shifted_rob_pts.col(2) = rob_pts.col(2).array() - centroid_rob_pts(0,2);
		cros_cov_mat = shifted_part_pts.transpose()*shifted_rob_pts;
    
    	// Singular Value Decomposition
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(cros_cov_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    	
    	// Take care of reflection case due to negative eigen vectors
        U_T = svd.matrixU().transpose();    V = svd.matrixV();
        M(2,2) = (V*U_T).determinant();
        R = V*M*U_T;
        if (R.determinant()>0)
        {
            T = -R*centroid_part_pts.transpose() + centroid_rob_pts.transpose();
            transform_mat.block(0,0,3,3) = R;
            transform_mat.block(0,3,3,1) = T;
            transform_mat(3,3) = 1; 
        }
        else
        {
            std::cerr << "ERROR: Determinant of rotation matrix is negative..." << std::endl;
        }   
    }
    else
    {
        std::cerr << "ERROR: FUNCTION ERROR: For correspondance, number of rows of both matrices should be same..." << std::endl;
    }
    return transform_mat;
}

Eigen::MatrixXd mean(Eigen::MatrixXd mat)
{
    Eigen::VectorXd vec(mat.cols());
    for (int i=0;i<mat.cols();++i)
    {
        vec(i) =  mat.block(0,i,mat.rows(),1).sum()/mat.rows();
    }
    return vec.transpose();
}

Eigen::MatrixXd apply_transformation(Eigen::MatrixXd data, Eigen::Matrix4d T_mat)
{
    //NOTE: Homogeneous Tranformation Matrix (4x4)

    // putting data in [x, y, z, 1]' format
    Eigen::MatrixXd data_with_fourth_row(data.cols()+1,data.rows());
    Eigen::VectorXd ones_vec = Eigen::VectorXd::Constant(data.rows(),1);
    data_with_fourth_row.block(0,0,data.cols(),data.rows()) = data.transpose();
    data_with_fourth_row.block(data.cols(),0,1,data.rows()) = ones_vec.transpose();
    Eigen::MatrixXd transformed_data = T_mat*data_with_fourth_row;
    Eigen::MatrixXd transformed_data_mat(transformed_data.rows()-1,transformed_data.cols());
    transformed_data_mat = transformed_data.block(0,0,transformed_data.rows()-1,transformed_data.cols());
    return transformed_data_mat.transpose();
}