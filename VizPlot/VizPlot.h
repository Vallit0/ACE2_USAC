#ifndef VIZPLOT_H
#define VIZPLOT_H

#include <Arduino.h>
#include <vector>
#include <map>

// Estructura para estadísticas básicas
struct Stats {
    float mean;
    int min;
    int max;
    float stddev;
};

// Estructura para los logs
struct LogEntry {
    String tag;
    String message;
};

struct LogMetrics {
    int infoCount;
    int warningCount;
    int errorCount;
    int debugCount;
};

class VizPlot {
private:
    String label;                 // Etiqueta para el gráfico
    int *dataBuffer;              // Buffer para almacenar datos
    int dataIndex;                // Índice actual del buffer
    int maxPoints;                // Tamaño máximo del buffer
    Stream *output;               // Flujo de salida (e.g., Serial)
    std::vector<LogEntry> logs;   // Vector de logs
    LogMetrics logMetrics;        // Métricas de logs

public:
    // Constructor y Destructor
    VizPlot(String label, int maxPoints);
    ~VizPlot();

    // Inicialización
    void begin(Stream *stream);

    // Agregar datos
    void addData(int value);
    void addData(int x, int y);

    // Estadísticas descriptivas
    Stats computeStats();
    float computeMedian();
    int computeMode();
    float computeIQR();
    float computePercentile(float percentile);

    // Estadísticas inferenciales
    std::vector<int> detectAnomalies(float threshold);
    float computeTTest(float expectedMean);
    std::pair<float, float> computeConfidenceInterval(float confidenceLevel);
    float computePearsonCorrelation(const std::vector<int>& sampleA, const std::vector<int>& sampleB);
    float computeProportionTest(float observedProportion, float expectedProportion);

    // Exportación de datos
    void exportStatsToEEPROM(int startAddress);

    // Gráficos
    void plotLine();
    void plotBar();
    void plotScatter();
    void plotHistogram();
    void plotHeatmap();
    void plotBoxPlot();

    // Logging
    void logInfo(String message);
    void logWarning(String message);
    void logError(String message);
    void logDebug(String message);
    void showLogsByTag(String tag);
    LogMetrics getLogMetrics();
};

#endif // VIZPLOT_H
