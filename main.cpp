#define FSAMP 100
#include <fstream>
#include <vector>
#include <iostream>
#include "Butter1.h"
static bool getFileContent(std::string fileName, std::vector<float> &vecOfStrs);
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
int main()
{
    // plt::plot({1, 3, 2, 4});
    // plt::show();
        Butter1 filt1(5, FSAMP);
        std::vector<float> angle_vec;
        std::vector<float> angle_vec_filtered;
        std::vector<int> id_vec;
        getFileContent("test.csv", angle_vec);
    //     // Set the size of output image = 1200x780 pixels
    //     plt::figure_size(1200, 780);

    //     // Plot line from given x and y data. Color is selected automatically.
    //     plt::plot(angle_vec, angle_vec);

    // Prepare data.
    // int n = 5000;
    // std::vector<double> x(n), y(n), z(n), w(n, 2);
    for (int i = 0; i < angle_vec.size(); ++i)
    {
        float sample_filt = filt1.process(angle_vec[i]);
        angle_vec_filtered.push_back(sample_filt);
        id_vec.push_back(i);
    }

    // Set the size of output image = 1200x780 pixels
    plt::figure_size(1200, 780);

    // Plot line from given x and y data. Color is selected automatically.
    plt::plot(id_vec, angle_vec);

    // // Plot a red dashed line from given x and y data.
    plt::plot(id_vec, angle_vec_filtered, "r--");

    // // Plot a line whose name will show up as "log(x)" in the legend.
    // plt::named_plot("log(x)", x, z);

    // // Set x-axis to interval [0,1000000]
    // plt::xlim(0, 1000 * 1000);

    // // Add graph title
    // plt::title("Sample figure");

    // Enable legend.
    plt::show();
}

/*
 * It will iterate through all the lines in file and
 * put them in given vector
 */
static bool getFileContent(std::string fileName, std::vector<float> &vecOfStrs)
{

    // Open the File
    std::ifstream in(fileName.c_str());

    // Check if object is valid
    if (!in)
    {
        std::cout << "Cannot open the File : " << fileName << std::endl;
        return false;
    }

    std::string str;
    // Read the next line from File untill it reaches the end.
    while (std::getline(in, str))
    {
        // Line contains string of length > 0 then save it in vector
        if (str.size() > 0)
            vecOfStrs.push_back(std::stof(str));
    }
    // Close The File
    in.close();
    return true;
}