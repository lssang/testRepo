/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <sys/time.h>
#include <signal.h>
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
#include <sys/resource.h>
#endif

#include "utils/gettime.h"
#include "utils/logoutput.h"
#include "sliceDataStorage.h"

#include "modelFile/modelFile.h"
#include "optimizedModel.h"
#include "polygonOptimizer.h"
#include "slicer.h"
#include "layerPart.h"
#include "inset.h"
#include "skin.h"
#include "infill.h"
#include "bridge.h"
#include "support.h"
#include "pathOptimizer.h"
#include "skirt.h"
#include "raft.h"
#include "comb.h"
#include "gcodeExport.h"

//mk
#include "clipper/clipper.hpp"
using namespace ClipperLib;

#include <sstream>
#include <fstream>
#include <iomanip>
using namespace std;


#define VERSION "1.0"
class Config
{
public:
    int layerThickness;
    int initialLayerThickness;
    int filamentDiameter;
    int filamentFlow;
    int extrusionWidth;
    int insetCount;
    int downSkinCount;
    int upSkinCount;
    int sparseInfillLineDistance;
    int infillOverlap;
    int skirtDistance;
    int skirtLineCount;
    int retractionAmount;
    int retractionSpeed;
    
    int initialSpeedupLayers;
    int initialLayerSpeed;
    int printSpeed;
    int infillSpeed;
    int moveSpeed;
    int fanOnLayerNr;
    
    //Support material
    int supportAngle;
    int supportEverywhere;
    int supportLineWidth;

    //Cool settings
    int minimalLayerTime;
    int minimalFeedrate;
    int coolHeadLift;
    int fanSpeedMin;
    int fanSpeedMax;
    
    //Raft settings
    int raftMargin;
    int raftLineSpacing;
    int raftBaseThickness;
    int raftBaseLinewidth;
    int raftInterfaceThickness;
    int raftInterfaceLinewidth;
    
    FMatrix3x3 matrix;
    Point objectPosition;
    int objectSink;
    
    int fixHorrible;
    
    Point extruderOffset[16];
    const char* startCode;
    const char* endCode;

    //mk
    FMatrix3x3 matrix_identity;
    Point3 ledPosition;
    int posx;
    int posy;
    int lposx;
    int lposy;
};

int verbose_level;
int maxObjectHeight;

inline long64 Round(double val)
{
  if ((val < 0)) return (long64)(val - 0.5); else return (long64)(val + 0.5);
}

bool LoadFromFile(Polygons &ppg, char * filename, double scale= 1,
  int xOffset = 0, int yOffset = 0)
{
  ppg.clear();
  ifstream infile(filename);
  if (!infile.is_open()) return false;
  int polyCnt, vertCnt;
  double X, Y;
  
  infile >> polyCnt;
  infile.ignore(80, '\n');
  if (infile.good() && polyCnt > 0)
  {
    ppg.resize(polyCnt);
    for (int i = 0; i < polyCnt; i++) 
    {
      infile >> vertCnt;
      infile.ignore(80, '\n');
      if (!infile.good() || vertCnt < 0) break;
      ppg[i].resize(vertCnt);
      for (int j = 0; infile.good() && j < vertCnt; j++) 
      {
        infile >> X;
        while (infile.peek() == ' ') infile.ignore();
        if (infile.peek() == ',') infile.ignore();
        while (infile.peek() == ' ') infile.ignore();
        infile >> Y;
        ppg[i][j].X = Round((X + xOffset) * scale);
        ppg[i][j].Y = Round((Y + yOffset) * scale);
        infile.ignore(80, '\n');
      }
    }
  }
  infile.close();
  return true;
}

void SaveToFile(const char *filename, Polygons &pp, double scale = 1)
{
  ofstream of(filename);
  if (!of.is_open()) return;
  of << pp.size() << "\n";
  for (unsigned i = 0; i < pp.size(); ++i)
  {
    of << pp[i].size() << "\n";
    if (scale > 1.01 || scale < 0.99) 
      of << fixed << setprecision(6);
    for (unsigned j = 0; j < pp[i].size(); ++j)
      of << (double)pp[i][j].X /scale << ", " << (double)pp[i][j].Y /scale << ",\n";
  }
  of.close();
}

void MakePolygonFromInts(int *ints, int size, ClipperLib::Polygon &p)
{ log("MakePolygonFromInts...\n");
  p.clear();
  p.reserve(size / 2);
  for (int i = 0; i < size; i +=2) {
    log("MakePolygonFromInts... %dth loop\n", i);
    p.push_back(ClipperLib::IntPoint(ints[i], ints[i+1]));
  }
}

void TranslatePolygon(ClipperLib::Polygon &p, int dx, int dy)
{
  for (size_t i = 0; i < p.size(); ++i)
  {
    p[i].X += dx;
    p[i].Y += dy;
  }
}

void processFile(const char* input_filename, Config& config, GCodeExport& gcode, bool firstFile)
{
    config.startCode =
        ";Sliced at: {day} {date} {time}\n"
        ";Basic settings: Layer height: {layer_height} Walls: {wall_thickness} Fill: {fill_density}\n"
        ";Print time: {print_time}\n"
        ";Filament used: {filament_amount}m {filament_weight}g\n"
        ";Filament cost: {filament_cost}\n"
        "G29\n"
        "G1 Y169 F8000\n"
        "G92 E0\n"
        "G1 E20 F300\n"
        "G92 E0\n"
        ";Put printing message on LCD screen\n"
        "M117 Printing...\n";   
    config.endCode = 
        ";End GCode\n"
        "M104 S0                     ;extruder heater off\n"
        "M140 S0                     ;heated bed heater off (if you have it)\n"
        "G92 E0\n"
        "G1 E-3 F500\n"
        "G92 E0\n"
        "G28 Y\n"
        "M84                         ;steppers off\n"
        "G90                         ;absolute positioning\n";

    for(unsigned int n=1; n<16;n++)
        gcode.setExtruderOffset(n, config.extruderOffset[n]);
    
    double t = getTime();
    log("Loading %s from disk...\n", input_filename);
    SimpleModel* m = loadModel(input_filename, config.matrix);
    if (!m)
    {
        log("Failed to load model: %s\n", input_filename);
        return;
    }
    log("Loaded from disk in %5.3fs\n", timeElapsed(t));
    log("Analyzing and optimizing model...\n");
    OptimizedModel* om = new OptimizedModel(m, Point3(config.objectPosition.X, config.objectPosition.Y, -config.objectSink));
    for(unsigned int v = 0; v < m->volumes.size(); v++)
    {
        log("  Face counts: %i -> %i %0.1f%%\n", (int)m->volumes[v].faces.size(), (int)om->volumes[v].faces.size(), float(om->volumes[v].faces.size()) / float(m->volumes[v].faces.size()) * 100);
        log("  Vertex counts: %i -> %i %0.1f%%\n", (int)m->volumes[v].faces.size() * 3, (int)om->volumes[v].points.size(), float(om->volumes[v].points.size()) / float(m->volumes[v].faces.size() * 3) * 100);
    }
    delete m;
    log("Optimize model %5.3fs \n", timeElapsed(t));
    //om->saveDebugSTL("output.stl");
    
    log("Slicing model...\n");
    vector<Slicer*> slicerList;
    for(unsigned int volumeIdx=0; volumeIdx < om->volumes.size(); volumeIdx++)
    {
        slicerList.push_back(new Slicer(&om->volumes[volumeIdx], config.initialLayerThickness / 2, config.layerThickness, config.fixHorrible));
        //slicerList[volumeIdx]->dumpSegments("C:\\models\\output.html");
    }
    log("Sliced model in %5.3fs\n", timeElapsed(t));

    SliceDataStorage storage;
    if (config.supportAngle > -1)
    {
        fprintf(stdout,"Generating support map...\n");
        generateSupportGrid(storage.support, om, config.initialLayerThickness / 2, config.layerThickness);
    }
    storage.modelSize = om->modelSize;
    storage.modelMin = om->vMin;
    storage.modelMax = om->vMax;
    delete om;


    //mk: set identity for tool models
    config.matrix_identity.m[0][0]=1.0;
    config.matrix_identity.m[0][1]=0.0;
    config.matrix_identity.m[0][2]=0.0;
    config.matrix_identity.m[1][0]=0.0;
    config.matrix_identity.m[1][1]=1.0;
    config.matrix_identity.m[1][2]=0.0;
    config.matrix_identity.m[2][0]=0.0;
    config.matrix_identity.m[2][1]=0.0;
    config.matrix_identity.m[2][2]=1.0;

    //mk: proccess for battery model
    char* battery_filename; 
    battery_filename = (char*)"./parts/cr2032_hole.stl";
    printf("Loading %s from disk...\n", battery_filename);
    SimpleModel* m_b = loadModel(battery_filename, config.matrix_identity);
    if (!m_b)
    {
        log("Failed to load model: %s\n", battery_filename);
        return;
    }
    OptimizedModel* om_b = new OptimizedModel(m_b, Point3(config.objectPosition.X, config.objectPosition.Y, -config.objectSink));
    for(unsigned int v = 0; v < m_b->volumes.size(); v++)
    {
        log("  Face counts: %i -> %i %0.1f%%\n", (int)m_b->volumes[v].faces.size(), (int)om_b->volumes[v].faces.size(), float(om_b->volumes[v].faces.size()) / float(m_b->volumes[v].faces.size()) * 100);
        log("  Vertex counts: %i -> %i %0.1f%%\n", (int)m_b->volumes[v].faces.size() * 3, (int)om_b->volumes[v].points.size(), float(om_b->volumes[v].points.size()) / float(m_b->volumes[v].faces.size() * 3) * 100);
    }
    delete m_b;
    log("Optimize model %5.3fs \n", timeElapsed(t));
    //om_b->saveDebugSTL("output.stl");
    
    log("Slicing model...\n");
    vector<Slicer*> slicerList_b;
    for(unsigned int volumeIdx=0; volumeIdx < om_b->volumes.size(); volumeIdx++)
    {
        slicerList_b.push_back(new Slicer(&om_b->volumes[volumeIdx], config.initialLayerThickness / 2, config.layerThickness, config.fixHorrible));
        //slicerList_b[volumeIdx]->dumpSegments("C:\\models\\output.html");
    }
    log("Sliced model in %5.3fs\n", timeElapsed(t));

    SliceDataStorage storage_b;
    if (config.supportAngle > -1)
    {
        fprintf(stdout,"Generating support map...\n");
        generateSupportGrid(storage_b.support, om_b, config.initialLayerThickness / 2, config.layerThickness);

    }
    storage_b.modelSize = om_b->modelSize;
    storage_b.modelMin = om_b->vMin;
    storage_b.modelMax = om_b->vMax;
    delete om_b;

    // proccess for led model
    char* led_filename; 
    led_filename = (char*)"./parts/led_5pi.stl";
    log("Loading %s from disk...\n", led_filename);
    SimpleModel* m_l = loadModel(led_filename, config.matrix_identity);
    if (!m_l)
    {
        log("Failed to load model: %s\n", led_filename);
        return;
    }
    OptimizedModel* om_l = new OptimizedModel(m_l, Point3(config.objectPosition.X, config.objectPosition.Y, 0));
    for(unsigned int v = 0; v < m_l->volumes.size(); v++)
    {
        log("  Face counts: %i -> %i %0.1f%%\n", (int)m_l->volumes[v].faces.size(), (int)om_l->volumes[v].faces.size(), float(om_l->volumes[v].faces.size()) / float(m_l->volumes[v].faces.size()) * 100);
        log("  Vertex counts: %i -> %i %0.1f%%\n", (int)m_l->volumes[v].faces.size() * 3, (int)om_l->volumes[v].points.size(), float(om_l->volumes[v].points.size()) / float(m_l->volumes[v].faces.size() * 3) * 100);
    }
    delete m_l;
    log("Optimize model %5.3fs \n", timeElapsed(t));

    log("Slicing model...\n");
    vector<Slicer*> slicerList_l;
    for(unsigned int volumeIdx=0; volumeIdx < om_l->volumes.size(); volumeIdx++)
    {
        slicerList_l.push_back(new Slicer(&om_l->volumes[volumeIdx], config.initialLayerThickness / 2, config.layerThickness, config.fixHorrible));
        //slicerList_l[volumeIdx]->dumpSegments("C:\\models\\output.html");
    }
    log("Sliced model in %5.3fs\n", timeElapsed(t));

    SliceDataStorage storage_l;
    if (config.supportAngle > -1)
    {
        fprintf(stdout,"Generating support map...\n");
        generateSupportGrid(storage_l.support, om_l, config.initialLayerThickness / 2, config.layerThickness);

    }
    // log("M)Sliced led model vMin: %d, %d, %d\n", om_l->vMin.x, om_l->vMin.y, om_l->vMin.z);
    // log("M)Sliced led model vMax: %d, %d, %d\n", om_l->vMax.x, om_l->vMax.y, om_l->vMax.z);
    // log("M)Sliced led model modelSize: %d, %d, %d\n", om_l->modelSize.x, om_l->modelSize.y, om_l->modelSize.z);
    storage_l.modelSize = om_l->modelSize;
    storage_l.modelMin = om_l->vMin;
    storage_l.modelMax = om_l->vMax;
    delete om_l;

    
    log("Generating layer parts...\n");
    for(unsigned int volumeIdx=0; volumeIdx < slicerList.size(); volumeIdx++)
    {
        storage.volumes.push_back(SliceVolumeStorage());
        createLayerParts(storage.volumes[volumeIdx], slicerList[volumeIdx], config.fixHorrible);
        delete slicerList[volumeIdx];

        storage_b.volumes.push_back(SliceVolumeStorage());
        createLayerParts(storage_b.volumes[0], slicerList_b[0], config.fixHorrible);
        delete slicerList_b[0];
        
        storage_l.volumes.push_back(SliceVolumeStorage());
        createLayerParts(storage_l.volumes[0], slicerList_l[0], config.fixHorrible);
        delete slicerList_l[0];

        int storage_o_layer_size = storage.volumes[0].layers.size();
        int storage_b_layer_size = storage_b.volumes[0].layers.size();
        int storage_l_layer_size = storage_l.volumes[0].layers.size();


        int iLed = (int)(config.ledPosition.z/config.layerThickness);
        int intervalX = (config.ledPosition.x-config.objectPosition.X)/(iLed-storage_b_layer_size);
                    // (config.ledPosition.x-config.objectPosition.X)/(storage_o_layer_size-storage_l_layer_size-storage_b_layer_size); 
        int intervalY = (config.ledPosition.y-config.objectPosition.Y)/(iLed-storage_b_layer_size);
                    // (config.ledPosition.y-config.objectPosition.Y)/(storage_o_layer_size-storage_l_layer_size-storage_b_layer_size);
        printf("iLed: %d, intervalX: %d, intervalY: %d \n", iLed, intervalX, intervalY);
        int intpOffsetX = 0;
        int intpOffsetY = 0;        
        
        SliceLayer* layer_l;
        SliceLayer* layer_b;
        for(unsigned int layerNr=0; layerNr < storage.volumes[volumeIdx].layers.size(); layerNr++)
        {   // log("layerNr: %d ", layerNr);    
            SliceLayer* layer1 = &storage.volumes[volumeIdx].layers[layerNr];
            layer_b = &storage_b.volumes[0].layers[layerNr];
            Polygons holeWire1;
            Polygons holeWire2;
            Polygons holeLed;
            
            // if (layerNr > storage_o_layer_size-storage_l_layer_size )
            // {
            //     layer_l = &storage_l.volumes[0].layers[layerNr - storage_o_layer_size + storage_l_layer_size ];
            // }
                
            printf("intpOffsetX: %d, intpOffsetY: %d\n", intpOffsetX, intpOffsetY);
            if (layerNr > storage_b_layer_size+5 && layerNr < iLed )
            {
                intpOffsetX = intpOffsetX+intervalX;
                intpOffsetY = intpOffsetY+intervalY;    
            }

            for(unsigned int p1 = 0; p1 < layer1->parts.size(); p1++)
            {   // log("p1: %d ", p1);
                //Load hole info from file 
                LoadFromFile(holeWire1, (char*)"./parts/holewire.txt", 1, 0, 0);
                LoadFromFile(holeWire2, (char*)"./parts/holewire.txt", 1, 0, 0);
                LoadFromFile(holeLed, (char*)"./parts/holeLed.txt", 1, 0, 0);
                // hole set test1 
                int distance = 3200;
                int holeOffsetX = 46500;
                int holeOffsetY = 40500;
                
                int objectHoleOffsetX = config.objectPosition.X - holeOffsetX+intpOffsetX;
                int objectHoleOffsetY = config.objectPosition.Y - holeOffsetY+intpOffsetY;
                TranslatePolygon(holeWire1[0], objectHoleOffsetX-distance/2, objectHoleOffsetY);
                TranslatePolygon(holeWire2[0], objectHoleOffsetX+distance/2, objectHoleOffsetY);

                // clip holeWire
                // if (layerNr < storage.volumes[0].layers.size()-storage_l.volumes[0].layers.size() ) 
                if (layerNr > storage_b_layer_size/3 && layerNr <= iLed)
                {
                    ClipperLib::Clipper clipper;
                    clipper.AddPolygons(layer1->parts[p1].outline, ClipperLib::ptSubject);
                    clipper.AddPolygons(holeWire1, ClipperLib::ptClip);
                    clipper.AddPolygons(holeWire2, ClipperLib::ptClip);
                    clipper.Execute(ClipperLib::ctDifference, layer1->parts[p1].outline);
                }
                
                // clip bettry hole
                if (layerNr < storage_b_layer_size)
                {
                    ClipperLib::Clipper clipper2;
                    clipper2.AddPolygons(layer1->parts[p1].outline, ClipperLib::ptSubject);
                    clipper2.AddPolygons(layer_b->parts[p1].outline, ClipperLib::ptClip);
                    clipper2.Execute(ClipperLib::ctDifference, layer1->parts[p1].outline);
                }
                // clip led hole: method1
                // if (layerNr > om_o_layer_size-om_l_layer_size ) {
                //     // TranslatePolygon(layer_l->parts[0].outline[0], 
                //     //     config.ledPosition.x-config.objectPosition.X, config.ledPosition.y-config.objectPosition.Y);
                //     ClipperLib::Clipper clipper3;
                //     clipper3.AddPolygons(layer1->parts[p1].outline, ClipperLib::ptSubject);
                //     // clipper3.AddPolygons(layer_l->parts[0].outline, ClipperLib::ptClip);
                //     // TranslatePolygon(holeLed[0], 
                //         // config.ledPosition.x+config.objectPosition.X, config.ledPosition.y+config.objectPosition.Y);
                //     clipper3.AddPolygons(holeLed, ClipperLib::ptClip);
                //     clipper3.Execute(ClipperLib::ctDifference, layer1->parts[p1].outline);
                // }

                // clip led hole: method2
                if (layerNr > iLed )
                {   
                    // TranslatePolygon(layer_l->parts[0].outline[0], 
                        // config.ledPosition.x-80000, config.ledPosition.y-85000);
                    // SaveToFile("led.txt", layer_l->parts[0].outline, 1);
                    // TranslatePolygon(layer_l->parts[0].outline[0], 
                        // config.ledPosition.x+80000, config.ledPosition.y+85000);
                    
                    // TranslatePolygon(layer_l->parts[0].outline[0], 
                    //     config.ledPosition.x-config.objectPosition.X, config.ledPosition.y-config.objectPosition.Y);
                    ClipperLib::Clipper clipper3;
                    clipper3.AddPolygons(layer1->parts[p1].outline, ClipperLib::ptSubject);
                    // clipper3.AddPolygons(layer_l->parts[0].outline, ClipperLib::ptClip);
                    TranslatePolygon(holeLed[0], config.ledPosition.x, config.ledPosition.y);
                    // TranslatePolygon(holeLed[0], 
                        // config.ledPosition.x+config.objectPosition.X, config.ledPosition.y+config.objectPosition.Y);
                    clipper3.AddPolygons(holeLed, ClipperLib::ptClip);
                    clipper3.Execute(ClipperLib::ctDifference, layer1->parts[p1].outline);
                }

                // ClipperLib::Clipper clipper2;
                // clipper2.AddPolygons(layer1->parts[p1].outline, ClipperLib::ptSubject);
                // if (layerNr < storage_b.volumes[0].layers.size())
                //     clipper2.AddPolygons(layer_b->parts[0].outline, ClipperLib::ptClip);
                // if (layerNr > iLed ) 
                // {
                //     TranslatePolygon(layer_l->parts[0].outline[0], 
                //         config.ledPosition.x-config.objectPosition.X, config.ledPosition.y-config.objectPosition.Y);
                //     clipper2.AddPolygons(layer_l->parts[0].outline, ClipperLib::ptClip);
                // }
                // if (layerNr < storage.volumes[0].layers.size()-iLed )
                // // layerNr < storage.volumes[0].layers.size()-storage_l.volumes[0].layers.size() ) 
                // {
                //     clipper2.AddPolygons(holeWire1, ClipperLib::ptClip);
                //     clipper2.AddPolygons(holeWire2, ClipperLib::ptClip);
                // }
                // clipper2.Execute(ClipperLib::ctDifference, layer1->parts[p1].outline);

            } //log("\n");    
        }

        //Go trough all the volumes, and remove the previous volume outlines from our own outline, so we never have overlapped areas.
        for(unsigned int volumeIdx2=0; volumeIdx2<volumeIdx; volumeIdx2++)
        {
            for(unsigned int layerNr=0; layerNr < storage.volumes[volumeIdx].layers.size(); layerNr++)
            {
                SliceLayer* layer1 = &storage.volumes[volumeIdx].layers[layerNr];
                SliceLayer* layer2 = &storage.volumes[volumeIdx2].layers[layerNr];
                for(unsigned int p1 = 0; p1 < layer1->parts.size(); p1++)
                {
                    ClipperLib::Clipper clipper;
                    clipper.AddPolygons(layer1->parts[p1].outline, ClipperLib::ptSubject);
                    for(unsigned int p2 = 0; p2 < layer2->parts.size(); p2++)
                    {
                        clipper.AddPolygons(layer2->parts[p2].outline, ClipperLib::ptClip);
                    }
                    clipper.Execute(ClipperLib::ctDifference, layer1->parts[p1].outline);
                }
            }
        }
    }
    log("Generated layer parts in %5.3fs\n", timeElapsed(t));
    //dumpLayerparts(storage, "c:/models/output.html");
    
    const unsigned int totalLayers = storage.volumes[0].layers.size();
    for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
    {
        for(unsigned int volumeIdx=0; volumeIdx<storage.volumes.size(); volumeIdx++)
        {
            generateInsets(&storage.volumes[volumeIdx].layers[layerNr], config.extrusionWidth, config.insetCount);
        }
        logProgress("inset",layerNr+1,totalLayers);
    }
    log("Generated inset in %5.3fs\n", timeElapsed(t));
    //dumpLayerparts(storage, "c:/models/output.html");

    for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
    {
        for(unsigned int volumeIdx=0; volumeIdx<storage.volumes.size(); volumeIdx++)
        {
            generateSkins(layerNr, storage.volumes[volumeIdx], config.extrusionWidth, config.downSkinCount, config.upSkinCount, config.infillOverlap);
            generateSparse(layerNr, storage.volumes[volumeIdx], config.extrusionWidth, config.downSkinCount, config.upSkinCount);
        }
        logProgress("skin",layerNr+1,totalLayers);
    }
    log("Generated up/down skin in %5.3fs\n", timeElapsed(t));
    generateSkirt(storage, config.skirtDistance, config.extrusionWidth, config.skirtLineCount);
    generateRaft(storage, config.raftMargin);
    
    for(unsigned int volumeIdx=0; volumeIdx<storage.volumes.size(); volumeIdx++)
    {
        for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
        {
            for(unsigned int partNr=0; partNr<storage.volumes[volumeIdx].layers[layerNr].parts.size(); partNr++)
            {
                if (layerNr > 0)
                    storage.volumes[volumeIdx].layers[layerNr].parts[partNr].bridgeAngle = bridgeAngle(&storage.volumes[volumeIdx].layers[layerNr].parts[partNr], &storage.volumes[volumeIdx].layers[layerNr-1]);
                else
                    storage.volumes[volumeIdx].layers[layerNr].parts[partNr].bridgeAngle = -1;
            }
        }
    }

    gcode.setRetractionSettings(config.retractionAmount, config.retractionSpeed);
    if (firstFile)
    {
        gcode.addCode(config.startCode);
    }else{
        gcode.resetExtrusionValue();
        gcode.addRetraction();
        gcode.setZ(maxObjectHeight + 5000);
        gcode.addMove(config.objectPosition, config.moveSpeed, 0);
    }
    gcode.addComment("total_layers=%d",totalLayers);

    GCodePathConfig skirtConfig(config.printSpeed, config.extrusionWidth, "SKIRT");
    GCodePathConfig inset0Config(config.printSpeed, config.extrusionWidth, "WALL-OUTER");
    GCodePathConfig inset1Config(config.printSpeed, config.extrusionWidth, "WALL-INNER");
    GCodePathConfig fillConfig(config.infillSpeed, config.extrusionWidth, "FILL");
    GCodePathConfig supportConfig(config.printSpeed, config.supportLineWidth, "SUPPORT");
    
    if (config.raftBaseThickness > 0 && config.raftInterfaceThickness > 0)
    {
        GCodePathConfig raftBaseConfig(config.initialLayerSpeed, config.raftBaseLinewidth, "SUPPORT");
        GCodePathConfig raftInterfaceConfig(config.initialLayerSpeed, config.raftInterfaceLinewidth, "SUPPORT");
        {
            gcode.addComment("LAYER:-2");
            gcode.addComment("RAFT");
            GCodePlanner gcodeLayer(gcode, config.moveSpeed);
            gcode.setZ(config.raftBaseThickness);
            gcode.setExtrusion(config.raftBaseThickness, config.filamentDiameter, config.filamentFlow);
            gcodeLayer.addPolygonsByOptimizer(storage.raftOutline, &raftBaseConfig);
            
            Polygons raftLines;
            generateLineInfill(storage.raftOutline, raftLines, config.raftBaseLinewidth, config.raftLineSpacing, config.infillOverlap, 0);
            gcodeLayer.addPolygonsByOptimizer(raftLines, &raftBaseConfig);
            
            gcodeLayer.writeGCode(false);
        }

        {
            gcode.addComment("LAYER:-1");
            gcode.addComment("RAFT");
            GCodePlanner gcodeLayer(gcode, config.moveSpeed);
            gcode.setZ(config.raftBaseThickness + config.raftInterfaceThickness);
            gcode.setExtrusion(config.raftInterfaceThickness, config.filamentDiameter, config.filamentFlow);
            
            Polygons raftLines;
            generateLineInfill(storage.raftOutline, raftLines, config.raftInterfaceLinewidth, config.raftLineSpacing, config.infillOverlap, 90);
            gcodeLayer.addPolygonsByOptimizer(raftLines, &raftInterfaceConfig);
            
            gcodeLayer.writeGCode(false);
        }
    }

    int volumeIdx = 0;
    for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
    {
        logProgress("export", layerNr+1, totalLayers);
        
        GCodePlanner gcodeLayer(gcode, config.moveSpeed);
        gcode.addComment("LAYER:%d", layerNr);
        int32_t z = config.initialLayerThickness + layerNr * config.layerThickness;
        z += config.raftBaseThickness + config.raftInterfaceThickness;
        gcode.setZ(z);
        if (layerNr == 0)
            gcodeLayer.addPolygonsByOptimizer(storage.skirt, &skirtConfig);
        
        for(unsigned int volumeCnt = 0; volumeCnt < storage.volumes.size(); volumeCnt++)
        {
            if (volumeCnt > 0)
                volumeIdx = (volumeIdx + 1) % storage.volumes.size();
            SliceLayer* layer = &storage.volumes[volumeIdx].layers[layerNr];
            gcodeLayer.setExtruder(volumeIdx);
            
            PathOptimizer partOrderOptimizer(gcode.getPositionXY());
            for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
            {
                partOrderOptimizer.addPolygon(layer->parts[partNr].insets[0][0]);
            }
            partOrderOptimizer.optimize();
            
            for(unsigned int partCounter=0; partCounter<partOrderOptimizer.polyOrder.size(); partCounter++)
            {
                SliceLayerPart* part = &layer->parts[partOrderOptimizer.polyOrder[partCounter]];
                
                gcodeLayer.setCombBoundary(&part->insets[0]);
                for(int insetNr=part->insets.size()-1; insetNr>-1; insetNr--)
                {
                    if (insetNr == 0)
                        gcodeLayer.addPolygonsByOptimizer(part->insets[insetNr], &inset0Config);
                    else
                        gcodeLayer.addPolygonsByOptimizer(part->insets[insetNr], &inset1Config);
                }
                
                Polygons fillPolygons;
                int fillAngle = 45;
                if (layerNr & 1) fillAngle += 90;
                //int sparseSteps[1] = {config.extrusionWidth};
                //generateConcentricInfill(part->skinOutline, fillPolygons, sparseSteps, 1);
                generateLineInfill(part->skinOutline, fillPolygons, config.extrusionWidth, config.extrusionWidth, config.infillOverlap, (part->bridgeAngle > -1) ? part->bridgeAngle : fillAngle);
                //int sparseSteps[2] = {config.extrusionWidth*5, config.extrusionWidth * 0.8};
                //generateConcentricInfill(part->sparseOutline, fillPolygons, sparseSteps, 2);
                if (config.sparseInfillLineDistance > 0)
                {
                    if (config.sparseInfillLineDistance > config.extrusionWidth * 4)
                    {
                        generateLineInfill(part->sparseOutline, fillPolygons, config.extrusionWidth, config.sparseInfillLineDistance * 2, config.infillOverlap, 45);
                        generateLineInfill(part->sparseOutline, fillPolygons, config.extrusionWidth, config.sparseInfillLineDistance * 2, config.infillOverlap, 45 + 90);
                    }
                    else
                    {
                        generateLineInfill(part->sparseOutline, fillPolygons, config.extrusionWidth, config.sparseInfillLineDistance, config.infillOverlap, fillAngle);
                    }
                }

                gcodeLayer.addPolygonsByOptimizer(fillPolygons, &fillConfig);
            }
            gcodeLayer.setCombBoundary(NULL);
        }
        if (config.supportAngle > -1)
        {
            SupportPolyGenerator supportGenerator(storage.support, z, config.supportAngle, config.supportEverywhere > 0, true);
            gcodeLayer.addPolygonsByOptimizer(supportGenerator.polygons, &supportConfig);
            if (layerNr == 0)
            {
                SupportPolyGenerator supportGenerator2(storage.support, z, config.supportAngle, config.supportEverywhere > 0, false);
                gcodeLayer.addPolygonsByOptimizer(supportGenerator2.polygons, &supportConfig);
            }
        }

        //Finish the layer by applying speed corrections for minimal layer times and slowdown for the initial layer.
        if (int(layerNr) < config.initialSpeedupLayers)
        {
            int n = config.initialSpeedupLayers;
            int layer0Factor = config.initialLayerSpeed * 100 / config.printSpeed;
            gcodeLayer.setSpeedFactor((layer0Factor * (n - layerNr) + 100 * (layerNr)) / n);
        }
        gcodeLayer.forceMinimalLayerTime(config.minimalLayerTime, config.minimalFeedrate);
        if (layerNr == 0)
            gcode.setExtrusion(config.initialLayerThickness, config.filamentDiameter, config.filamentFlow);
        else
            gcode.setExtrusion(config.layerThickness, config.filamentDiameter, config.filamentFlow);
        if (int(layerNr) >= config.fanOnLayerNr)
        {
            int speed = config.fanSpeedMin;
            if (gcodeLayer.getSpeedFactor() <= 50)
            {
                speed = config.fanSpeedMax;
            }else{
                int n = gcodeLayer.getSpeedFactor() - 50;
                speed = config.fanSpeedMin * n / 50 + config.fanSpeedMax * (50 - n) / 50;
            }
            gcode.addFanCommand(speed);
        }else{
            gcode.addFanCommand(0);
        }
        gcodeLayer.writeGCode(config.coolHeadLift > 0);
    }

    /* support debug
    for(int32_t y=0; y<storage.support.gridHeight; y++)
    {
        for(int32_t x=0; x<storage.support.gridWidth; x++)
        {
            unsigned int n = x+y*storage.support.gridWidth;
            if (storage.support.grid[n].size() < 1) continue;
            int32_t z = storage.support.grid[n][0].z;
            gcode.addMove(Point3(x * storage.support.gridScale + storage.support.gridOffset.X, y * storage.support.gridScale + storage.support.gridOffset.Y, 0), 0);
            gcode.addMove(Point3(x * storage.support.gridScale + storage.support.gridOffset.X, y * storage.support.gridScale + storage.support.gridOffset.Y, z), z);
            gcode.addMove(Point3(x * storage.support.gridScale + storage.support.gridOffset.X, y * storage.support.gridScale + storage.support.gridOffset.Y, 0), 0);
        }
    }
    //*/
    
    log("Wrote layers in %5.2fs.\n", timeElapsed(t));
    gcode.tellFileSize();
    gcode.addFanCommand(0);

    logProgress("process", 1, 1);
    log("Total time elapsed %5.2fs.\n", timeElapsed(t,true));
    
    //Store the object height for when we are printing multiple objects, as we need to clear every one of them when moving to the next position.
    maxObjectHeight = std::max(maxObjectHeight, storage.modelSize.z);
}

void setConfig(Config& config, char* str)
{
    char* valuePtr = strchr(str, '=');
    if (!valuePtr) return;
    *valuePtr++ = '\0';
#define STRINGIFY(_s) #_s
#define SETTING(longName, shortName) if (strcasecmp(str, STRINGIFY(longName)) == 0 || strcasecmp(str, STRINGIFY(shortName)) == 0) { config.longName = atoi(valuePtr); }
    SETTING(layerThickness, lt);
    SETTING(initialLayerThickness, ilt);
    SETTING(filamentDiameter, fd);
    SETTING(filamentFlow, ff);
    SETTING(extrusionWidth, ew);
    SETTING(insetCount, ic);
    SETTING(downSkinCount, dsc);
    SETTING(upSkinCount, usc);
    SETTING(sparseInfillLineDistance, sild);
    SETTING(infillOverlap, iover);
    SETTING(skirtDistance, sd);
    SETTING(skirtLineCount, slc);

    SETTING(initialSpeedupLayers, isl);
    SETTING(initialLayerSpeed, ils);
    SETTING(printSpeed, ps);
    SETTING(infillSpeed, is);
    SETTING(moveSpeed, ms);
    SETTING(fanOnLayerNr, fl);
    
    SETTING(supportAngle, supa);
    SETTING(supportEverywhere, supe);
    SETTING(supportLineWidth, sulw);
    
    SETTING(retractionAmount, reta);
    SETTING(retractionSpeed, rets);
    SETTING(objectPosition.X, posx);
    SETTING(objectPosition.Y, posy);
    SETTING(objectSink, objsink);

    SETTING(ledPosition.x, lposx);
    SETTING(ledPosition.y, lposy);
    SETTING(ledPosition.z, lposz);

    SETTING(raftMargin, raftMar);
    SETTING(raftLineSpacing, raftLS);
    SETTING(raftBaseThickness, raftBaseT);
    SETTING(raftBaseLinewidth, raftBaseL);
    SETTING(raftInterfaceThickness, raftInterfaceT);
    SETTING(raftInterfaceLinewidth, raftInterfaceL);
    
    SETTING(minimalLayerTime, minLayTime);
    SETTING(minimalFeedrate, minFeed);
    SETTING(coolHeadLift, coolLift);
    SETTING(fanSpeedMin, fanMin);
    SETTING(fanSpeedMax, fanMax);
    
    SETTING(extruderOffset[1].X, eOff1X);
    SETTING(extruderOffset[1].Y, eOff1Y);
    SETTING(extruderOffset[2].X, eOff2X);
    SETTING(extruderOffset[2].Y, eOff2Y);
    SETTING(extruderOffset[3].X, eOff3X);
    SETTING(extruderOffset[3].Y, eOff3Y);
#undef SETTING
    if (strcasecmp(str, "startCode") == 0)
        config.startCode = valuePtr;
    if (strcasecmp(str, "endCode") == 0)
        config.endCode = valuePtr;
}

void print_usage()
{
    printf("TODO\n");
}

void signal_FPE(int n)
{
    printf("Floating point exception\n");
    exit(1);
}

int main(int argc, char **argv)
{
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
    //Lower the process priority on linux and mac.
    setpriority(PRIO_PROCESS, 0, 10);
#endif
    signal(SIGFPE, signal_FPE);

    GCodeExport gcode;
    Config config;
    int fileNr = 0;

    config.filamentDiameter = 2890;
    config.initialLayerThickness = 300;
    config.layerThickness = 100;
    config.extrusionWidth = 400;
    config.insetCount = 2;
    config.downSkinCount = 6;
    config.upSkinCount = 6;
    config.initialSpeedupLayers = 4;
    config.initialLayerSpeed = 20;
    config.printSpeed = 50;
    config.infillSpeed = 50;
    config.moveSpeed = 200;
    config.fanOnLayerNr = 2;
    config.skirtDistance = 6000;
    config.skirtLineCount = 1;
    config.sparseInfillLineDistance = 100 * config.extrusionWidth / 20;
    config.infillOverlap = 15;
    config.objectPosition = Point(102500, 102500);
    config.objectSink = 0;
    config.supportAngle = -1;
    config.supportEverywhere = 0;
    config.supportLineWidth = config.extrusionWidth;
    config.retractionAmount = 4.5;
    config.retractionSpeed = 45;

    config.minimalLayerTime = 5;
    config.minimalFeedrate = 10;
    config.coolHeadLift = 1;
    config.fanSpeedMin = 100;
    config.fanSpeedMax = 100;

    config.raftMargin = 5000;
    config.raftLineSpacing = 1000;
    config.raftBaseThickness = 0;
    config.raftBaseLinewidth = 0;
    config.raftInterfaceThickness = 0;
    config.raftInterfaceLinewidth = 0;

    config.fixHorrible = 0;
    
    config.startCode =
        "M109 S210     ;Heatup to 210C\n"
        "G21           ;metric values\n"
        "G90           ;absolute positioning\n"
        "G28           ;Home\n"
        "G1 Z15.0 F300 ;move the platform down 15mm\n"
        "G92 E0        ;zero the extruded length\n"
        "G1 F200 E5    ;extrude 5mm of feed stock\n"
        "G92 E0        ;zero the extruded length again\n";
    config.endCode = 
        "M104 S0                     ;extruder heater off\n"
        "M140 S0                     ;heated bed heater off (if you have it)\n"
        "G91                            ;relative positioning\n"
        "G1 E-1 F300                    ;retract the filament a bit before lifting the nozzle, to release some of the pressure\n"
        "G1 Z+0.5 E-5 X-20 Y-20 F9000   ;move Z up a bit and retract filament even more\n"
        "G28 X0 Y0                      ;move X/Y to min endstops, so the head is out of the way\n"
        "M84                         ;steppers off\n"
        "G90                         ;absolute positioning\n";

    fprintf(stdout,"WEngine version %s\n", VERSION);

    for(int argn = 1; argn < argc; argn++) {
        printf("%s ", argv[argn]);
    }   printf("\n");

    for(int argn = 1; argn < argc; argn++)
    {
        char* str = argv[argn];
        if (str[0] == '-')
        {
            for(str++; *str; str++)
            {
                switch(*str)
                {
                case 'h':
                    print_usage();
                    exit(1);
                case 'v':
                    verbose_level++;
                    break;
                case 'b':
                    argn++;
                    binaryMeshBlob = fopen(argv[argn], "rb");
                    break;
                case 'o':
                    argn++;
                    gcode.setFilename(argv[argn]);
                    if (!gcode.isValid())
                    {
                        logError("Failed to open %s for output.\n", argv[argn]);
                        exit(1);
                    }
                    gcode.addComment("Generated with Cura_SteamEngine %s", VERSION);
                    break;
                case 's':
                    argn++;
                    setConfig(config, argv[argn]);
                    break;
                case 'm':
                    argn++;
                    sscanf(argv[argn], "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                        &config.matrix.m[0][0], &config.matrix.m[0][1], &config.matrix.m[0][2],
                        &config.matrix.m[1][0], &config.matrix.m[1][1], &config.matrix.m[1][2],
                        &config.matrix.m[2][0], &config.matrix.m[2][1], &config.matrix.m[2][2]);
                    break;
                default:
                    logError("Unknown option: %c\n", *str);
                    break;
                }
            }
        }else{
            if (!gcode.isValid())
            {
                logError("No output file specified\n");
                return 1;
            }
            processFile(argv[argn], config, gcode, fileNr == 0);
            fileNr ++;
        }
    }
    if (gcode.isValid())
    {
        gcode.addFanCommand(0);
        gcode.addCode(config.endCode);
        log("Print time: %d\n", int(gcode.getTotalPrintTime()));
        log("Filament: %d\n", int(gcode.getTotalFilamentUsed()));
    }
}
