//
// Created by Kevin Yu on 2016-07-21.
//

#include <iostream>
#include <fstream>
#include <streambuf>
#include <sstream>
#include <string>
#include <falton/setting/general.h>

const int MAX_DATA = 1000;
const int MAX_SERIES = 100;

struct Axis {
    std::string title = "-";
    int32 tickInterval = 1;
};

struct Chart {
    std::string title;
    int nSeries = 0;
    int nData = MAX_DATA;
    std::string names[10];
    float data[MAX_SERIES][MAX_DATA];

    Axis xAxis, yAxis;

    void pushFile(std::string filename, std::string inputName) {
        names[nSeries] = inputName;
        std::ifstream file(filename);
        float inputData;
        int32 i = 0;
        std::string line;
        while (std::getline(file, line) && i < nData)  //read stream line by line
        {
            data[nSeries][i] = std::stof(line);
            ++i;
        }
        ++nSeries;
        file.close();
    }

    void clearData() {
        nSeries = 0;
    }

    void generateHTML(std::string filename) {
        std::string seriesString;
        seriesString = "series : [";
        for (int i = 0; i < nSeries; ++i) {
            seriesString.append("{ name :");
            seriesString.append("\"" + names[i] + "\"");
            seriesString.append(", data: [");
            for (int j = 0; j < nData; ++j) {
                seriesString.append(std::to_string(data[i][j]));
                seriesString.append(",\n");
            }
            seriesString.append("]},");
        }
        seriesString.append("]");

        std::string titleString = "title : { text : \'" + title +  "\'},";

        std::string xAxisString = "xAxis : { tickInterval : " + std::to_string(xAxis.tickInterval) +
                                  ", title : \'" + xAxis.title + "\' },";
        std::string yAxisString = "yAxis : { tickInterval : " + std::to_string(yAxis.tickInterval) +
                                  ", title : \'" + yAxis.title + "\' },";

        std::string legendString = "legend: {"
            "symbolWidth: 48,"
            "itemStyle: {"
                "fontWeight: 'bold',"
                "fontSize: '30px'"
            "}},";

        std::string chartString = "$('#container').highcharts({ chart: { type: 'line' }, \n" +
                titleString + legendString + xAxisString + yAxisString + seriesString + "});";

        std::string scriptString = "<script> $(function () { " + chartString + "}); </script>";

        std::string htmlString = "<html>\n"
                "<head><title>Chart 1</title></head>\n"
                "<body>\n"
                "<div id=\"container\" style=\"width:100%; height:100%;\"></div>\n"
                "</body>\n"
                "</html>";

        std::string additionalScriptString = "<script src=\"https://ajax.googleapis.com/ajax/libs/jquery/1.12.4/jquery.min.js\"></script>\n"
                "<script src=\"http://code.highcharts.com/highcharts.js\"></script>";

        std::string fileString = htmlString + additionalScriptString + scriptString;

        std::ofstream outputFile(filename);
        outputFile << fileString;
        outputFile.close();
    }
};

int main(int argc, char *argv[]) {
    std::string fileInputName(argv[1]);
    std::string fileOutputName(argv[2]);

    Chart chart;

    std::ifstream fileInput(fileInputName);
    std::string line;

    std::getline(fileInput, line);
    chart.title = line;

    std::getline(fileInput, line);
    chart.nData = std::stoi(line);

    std::getline(fileInput, line);
    chart.xAxis.title = line;
    std::getline(fileInput, line);
    chart.xAxis.tickInterval = stoi(line);

    std::getline(fileInput, line);
    chart.yAxis.title = line;
    std::getline(fileInput, line);
    chart.yAxis.tickInterval = stoi(line);

    while (std::getline(fileInput, line)) {
        std::istringstream in(line);
        std::string dataFileName;
        in >> dataFileName;

        std::string dataLabel;
        in >> dataLabel;

        chart.pushFile(dataFileName, dataLabel);
    }
    fileInput.close();
    chart.generateHTML(fileOutputName);

    return 0;


}