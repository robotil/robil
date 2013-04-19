#ifndef C23_Node_TLD_Handler_H_
#define C23_Node_TLD_Handler_H_

#include "libopentld/tld/TLD.h"
using namespace cv;
enum TldRetval {
    SUCCESS,
    INIT_ERROR,
    BB_ERROR
};
enum TldMode {
    TRACKING,
    LEARNING,
    NOT_INITIALIZED,
    
};

using namespace std;
class C23_Node_TLD_Handler {
    public:
        C23_Node_TLD_Handler(TldMode mode, const char* modelPath = NULL);
        ~C23_Node_TLD_Handler();
        TldRetval init(IplImage* img= NULL);
        TldRetval saveCurrentModel(const char*);
        TldRetval loadModel(string path);
        TldRetval setBB(IplImage* img, int x, int y, int width, int height);
        TldRetval processFrame(IplImage* img, int *x, int *y, int *width, int *height, float *confident);
        
    public:
        string _confFile;
        TldMode _mode;
        tld::TLD *tld;
        //ImAcq *imAcq;
        //tld::Gui *gui;
        //bool showOutput;
        //const char *printResults;
        //const char *saveDir;
        //double threshold;
        //bool showForeground;
        //bool showNotConfident;
        //bool selectManually;
        int *_initialBB;
        //bool reinit;
        //bool exportModelAfterRun;
        //bool loadModel;
        const char *_modelPath;
        const char *modelExportFile;
        const char *_window;
        //int seed;
        
    private:
        int getBBFromUser(IplImage*img, CvRect &rect);
        

};

#endif
