#pragma once

/// @note this needs to be placed outside of Fitting.h as these settings
///       are parsed by the CUDA portions of the code

namespace energy{
namespace fitting{
struct Settings{
    ///--- E_2D
    bool  fit2D_enable = true;
    float fit2D_weight = 1;

    ///--- E_3D
    bool  fit3D_enable = true;
    float fit3D_weight = 1.0f;
    bool  fit3D_backface_check = true;
    bool  fit3D_point2plane = true;
    bool  fit3D_reweight = true;
    /// (advanced)
    bool fit3D_show = false;
    bool fit3D_reweight_rigid = false;
    int  fit3D_stepsize = 1; ///< downsampling

    ///--- E_silhouette from [Wei SIGA'12]
    bool  silho_enable = false;
    float silho_weight = 1;
    bool  silho_show_constraints = false;

    ///--- E_depth from [Wei SIGA'12]
    bool  depth_enable = false;
    float depth_weight = 1;
    float depth_z_th = 3; ///< "depth/z_th"
    bool  depth_show_constraints = false; ///< "depth/show_constraints"

    /// DEBUG STUFF
    bool debug_show_constraints_image = false;
};
}} /// energy::fitting
