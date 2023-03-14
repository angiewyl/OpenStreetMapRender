#pragma once

#include <unordered_map>
#include <io2d.h>
#include "model.h"

namespace io2d = std::experimental::io2d;
namespace OSM_PathPlotter
{    

class DrawImage
{
    public:
        DrawImage(Model &model);
        void ExportImage(io2d::image_surface &surface, const std::vector<Model::Node>& input_route, const std::vector<Model::Node>& non_precise_points);
        
    private:
        void BuildRoadReps();
        void BuildLanduseBrushes();
        
        void DrawBuildings(io2d::image_surface &surface) const;
        void DrawHighways(io2d::image_surface &surface) const;
        void DrawRailways(io2d::image_surface &surface) const;
        void DrawLeisure(io2d::image_surface &surface) const;
        void DrawWater(io2d::image_surface &surface) const;
        void DrawLanduses(io2d::image_surface &surface) const;
        
        void DrawPath(io2d::image_surface &surface, const std::vector<Model::Node>& input_route);
        void DrawPoints(io2d::image_surface &surface, const float x, const float y, const io2d::brush& foreBrush);

        io2d::interpreted_path PathFromWay(const Model::Way &way) const;
        io2d::interpreted_path PathFromMP(const Model::Multipolygon &mp) const;
    
        Model &m_Model;
        float m_Scale = 1.0f;
        float m_PixelsInMeter = 1.0f;
        io2d::matrix_2d m_Matrix;
        
        

        io2d::brush m_BackgroundFillBrush{ io2d::rgba_color{238, 235, 227} };
        
        io2d::brush m_BuildingFillBrush{ io2d::rgba_color{208, 197, 190} };
        io2d::brush m_BuildingOutlineBrush{ io2d::rgba_color{181, 167, 154} };
        io2d::stroke_props m_BuildingOutlineStrokeProps{1.f};
        
        io2d::brush m_LeisureFillBrush{ io2d::rgba_color{189, 252, 193} };
        io2d::brush m_LeisureOutlineBrush{ io2d::rgba_color{160, 248, 162} };
        io2d::stroke_props m_LeisureOutlineStrokeProps{1.f};

        io2d::brush m_WaterFillBrush{ io2d::rgba_color{155, 201, 215} };    
            
        io2d::brush m_RailwayStrokeBrush{ io2d::rgba_color{93,93,93} };
        io2d::brush m_RailwayDashBrush{ io2d::rgba_color::white };
        io2d::dashes m_RailwayDashes{0.f, {3.f, 3.f}};
        float m_RailwayOuterWidth = 3.f;
        float m_RailwayInnerWidth = 2.f;
        
        struct RoadRep {
            io2d::brush brush{io2d::rgba_color::black};
            io2d::dashes dashes{};
            float metric_width = 1.f;
        };
        std::unordered_map<Model::Road::Type, RoadRep> m_RoadReps;
        
        std::unordered_map<Model::Landuse::Type, io2d::brush> m_LanduseBrushes;
};
}  // namespace OSM_PathPlotter