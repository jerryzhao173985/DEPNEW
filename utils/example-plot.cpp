#include "CImg.h"
#include <vector>

using namespace cimg_library;

int main(int argc, char** const argv)
{
    cimg_usage("Simple plotter of mathematical formulas");
    const char *const formula = cimg_option("-f", "sin(x)", "Formula to    plot");
    const float x0 = cimg_option("-x0", -5.0f, "Minimal X-value");
    const float x1 = cimg_option("-x1", 5.0f, "Maximal X-value");
    const int resolution = cimg_option("-r", 5000, "Plot resolution");
    const unsigned int nresolution = resolution>1 ? resolution : 5000;
    const unsigned int plot_type = cimg_option("-p", 1, "Plot type");
    const unsigned int vertex_type = cimg_option("-v", 1, "Vertex type");

    // Create plot data.
    CImg<double> values(1, nresolution, 1, 1, 0);

    const unsigned int r = nresolution - 1;

    for (int i1 = 0; i1 < resolution; ++i1)
    {
        double xtime = x0 + i1*(x1 - x0) / r;
        values(0, i1) = sin(xtime);
    }

    CImgDisplay disp;
    CImg<double> values2;
    values.display_graph(disp, plot_type, vertex_type, "X Axis", x0, x1, "Y Axis");
    disp.snapshot(values2);
    values2.save_bmp("result.bmp");
}
