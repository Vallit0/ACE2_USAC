// VizPlot.cpp
#include "VizPlot.h"
#include <math.h>

// Stats 
struct Stats {
    float mean;
    int min;
    int max;
    float stddev;
};
// Constructor
VizPlot::VizPlot(String label, int maxPoints) {
    this->label = label;
    this->maxPoints = maxPoints;
    this->dataBuffer = new int[maxPoints];
    this->dataIndex = 0;

    // Inicializamos metricas para los logs
    // Inicializar métricas de logs
    logMetrics.infoCount = 0;
    logMetrics.warningCount = 0;
    logMetrics.errorCount = 0;
    logMetrics.debugCount = 0;
}

// Inicialización
void VizPlot::begin(Stream *stream) {
    this->output = stream;
    for (int i = 0; i < maxPoints; i++) {
        dataBuffer[i] = 0;
    }
}

// Agregar datos al buffer
void VizPlot::addData(int value) {
    dataBuffer[dataIndex] = value;
    dataIndex = (dataIndex + 1) % maxPoints;
}

void VizPlot::addData(int x, int y) {
    // Implementación futura para gráficos de puntos
}

// PLOTS GENERALES 

// Graficar en modo línea
void VizPlot::plotLine() {
    output->println(label);
    for (int i = 0; i < maxPoints; i++) {
        int index = (dataIndex + i) % maxPoints;
        output->print("[" + String(i) + "]: ");
        for (int j = 0; j < dataBuffer[index]; j++) {
            output->print("█");
        }
        output->println();
    }
    output->println();
}

// Graficar en modo barras
void VizPlot::plotBar() {
    output->println(label);
    for (int i = 0; i < maxPoints; i++) {
        int index = (dataIndex + i) % maxPoints;
        output->print("[" + String(i) + "]: ");
        for (int j = 0; j < dataBuffer[index]; j++) {
            output->print("|");
        }
        output->println();
    }
    output->println();
}

// Graficar en modo dispersión
void VizPlot::plotScatter() {
    output->println(label + " Scatter Plot");
    for (int i = 0; i < maxPoints; i++) {
        output->print("[" + String(i) + "]: ");
        if (dataBuffer[i] > 0) {
            output->print("*");
        }
        output->println();
    }
    output->println();
}

// Graficar en modo histograma
void VizPlot::plotHistogram() {
    output->println(label + " Histogram");
    int frequency[10] = {0};
    for (int i = 0; i < maxPoints; i++) {
        int value = dataBuffer[i] / 10;
        if (value >= 0 && value < 10) {
            frequency[value]++;
        }
    }
    for (int i = 0; i < 10; i++) {
        output->print("[" + String(i * 10) + "-" + String((i + 1) * 10 - 1) + "]: ");
        for (int j = 0; j < frequency[i]; j++) {
            output->print("#");
        }
        output->println();
    }
    output->println();
}


// Advanced plots 
void VizPlot::plotHeatmap() {
    output->println(label + " Heatmap:");
    for (int i = 0; i < maxPoints; i++) {
        for (int j = 0; j < dataBuffer[i]; j++) {
            output->print("*");
        }
        output->println();
    }
}
// Errors 
void VizPlot::plotBoxPlot() {
    int sortedBuffer[maxPoints];
    memcpy(sortedBuffer, dataBuffer, maxPoints * sizeof(int));
    std::sort(sortedBuffer, sortedBuffer + maxPoints);

    int minVal = sortedBuffer[0];
    int maxVal = sortedBuffer[maxPoints - 1];
    int p25 = sortedBuffer[(int)(0.25 * maxPoints)];
    int p50 = sortedBuffer[(int)(0.5 * maxPoints)];
    int p75 = sortedBuffer[(int)(0.75 * maxPoints)];

    output->println(label + " Box Plot:");
    output->println("Min: " + String(minVal));
    output->println("P25: " + String(p25));
    output->println("Median: " + String(p50));
    output->println("P75: " + String(p75));
    output->println("Max: " + String(maxVal));

    output->print(" |");
    for (int i = minVal; i <= maxVal; i++) {
        if (i == minVal || i == p25 || i == p50 || i == p75 || i == maxVal) {
            output->print("+");
        } else {
            output->print("-");
        }
    }
    output->println("|");
}




// Logging
void VizPlot::logInfo(String message) {
    output->println("[INFO] " + message);
    logs.push_back({"[INFO]", message});
    logMetrics.infoCount++;
}

void VizPlot::logWarning(String message) {
    output->println("[WARNING]" + message);
    logs.push_back({"[WARNING]", message});
    logMetrics.warningCount++;
}

void VizPlot::logError(String message) {
    output->println("[ERROR]" + message);
    logs.push_back({"[ERROR]", message});
    logMetrics.errorCount++;
}

void VizPlot::logDebug(String message) {
    output->println("[DEBUG]" + message);
    logs.push_back({"[DEBUG]", message});
    logMetrics.debugCount++;
}

// Mostrar logs por etiqueta
void VizPlot::showLogsByTag(String tag) {
    output->println("Logs with tag: " + tag);
    for (const auto &log : logs) {
        if (log.tag == tag) {
            output->println(log.tag + ": " + log.message);
        }
    }
    output->println();
}

// Obtener métricas de logs
LogMetrics VizPlot::getLogMetrics() {
    return logMetrics;
}

// Exportar datos a EEPROM 
// Exportar estadísticas a EEPROM
void VizPlot::exportStatsToEEPROM(int startAddress) {
    int sum = 0;
    int minVal = INT_MAX;
    int maxVal = INT_MIN;

    for (int i = 0; i < maxPoints; i++) {
        sum += dataBuffer[i];
        if (dataBuffer[i] < minVal) minVal = dataBuffer[i];
        if (dataBuffer[i] > maxVal) maxVal = dataBuffer[i];
    }

    float mean = (float)sum / maxPoints;
    float variance = 0.0;
    for (int i = 0; i < maxPoints; i++) {
        variance += pow(dataBuffer[i] - mean, 2);
    }
    variance /= maxPoints;
    float stddev = sqrt(variance);

    // Guardar estadísticas en EEPROM
    EEPROM.put(startAddress, mean);
    EEPROM.put(startAddress + sizeof(mean), minVal);
    EEPROM.put(startAddress + sizeof(mean) + sizeof(minVal), maxVal);
    EEPROM.put(startAddress + sizeof(mean) + sizeof(minVal) + sizeof(maxVal), stddev);

    output->println("Statistics exported to EEPROM starting at address " + String(startAddress));
}


// ESTADISTICA DESCRIPTIVA 
// Calcular estadísticas y retornar valores
Stats VizPlot::computeStats() {
    int sum = 0;
    int minVal = INT_MAX;
    int maxVal = INT_MIN;

    for (int i = 0; i < maxPoints; i++) {
        sum += dataBuffer[i];
        if (dataBuffer[i] < minVal) minVal = dataBuffer[i];
        if (dataBuffer[i] > maxVal) maxVal = dataBuffer[i];
    }

    float mean = (float)sum / maxPoints;
    float variance = 0.0;
    for (int i = 0; i < maxPoints; i++) {
        variance += pow(dataBuffer[i] - mean, 2);
    }
    variance /= maxPoints;
    float stddev = sqrt(variance);

    return {mean, minVal, maxVal, stddev};
}
// Mediana 
float VizPlot::computeMedian() {
    std::vector<int> sortedBuffer(dataBuffer, dataBuffer + maxPoints);
    std::sort(sortedBuffer.begin(), sortedBuffer.end());

    if (maxPoints % 2 == 0) {
        return (sortedBuffer[maxPoints / 2 - 1] + sortedBuffer[maxPoints / 2]) / 2.0;
    } else {
        return sortedBuffer[maxPoints / 2];
    }
}
// Moda 
int VizPlot::computeMode() {
    std::map<int, int> frequency;
    for (int i = 0; i < maxPoints; i++) {
        frequency[dataBuffer[i]]++;
    }

    int mode = dataBuffer[0];
    int maxFreq = 0;
    for (const auto &entry : frequency) {
        if (entry.second > maxFreq) {
            maxFreq = entry.second;
            mode = entry.first;
        }
    }
    return mode;
}
// Compute IQR 
float VizPlot::computeIQR() {
    std::vector<int> sortedBuffer(dataBuffer, dataBuffer + maxPoints);
    std::sort(sortedBuffer.begin(), sortedBuffer.end());

    int q1 = sortedBuffer[(int)(0.25 * maxPoints)];
    int q3 = sortedBuffer[(int)(0.75 * maxPoints)];

    return q3 - q1;
}
// Percentiles 
float VizPlot::computePercentile(float percentile) {
    std::vector<int> sortedBuffer(dataBuffer, dataBuffer + maxPoints);
    std::sort(sortedBuffer.begin(), sortedBuffer.end());

    int index = (int)((percentile / 100.0) * maxPoints);
    return sortedBuffer[index];
}


// Detectar anomalías basadas en Z-Score y retornar índices
std::vector<int> VizPlot::detectAnomalies(float threshold) {
    Stats stats = computeStats();
    float mean = stats.mean;
    float stddev = stats.stddev;

    std::vector<int> anomalyIndices;
    for (int i = 0; i < maxPoints; i++) {
        float zScore = (dataBuffer[i] - mean) / stddev;
        if (fabs(zScore) > threshold) {
            anomalyIndices.push_back(i);
        }
    }
    return anomalyIndices;
}

// Calcular derivadas y retornar valores
std::vector<float> VizPlot::computeDerivative() {
    std::vector<float> derivatives;
    for (int i = 1; i < maxPoints; i++) {
        float derivative = dataBuffer[i] - dataBuffer[i - 1];
        derivatives.push_back(derivative);
    }
    return derivatives;
}

// Calcular prueba T-Student y retornar valor
float VizPlot::computeTTest(float expectedMean) {
    Stats stats = computeStats();
    float mean = stats.mean;
    float stddev = stats.stddev;
    float tStatistic = (mean - expectedMean) / (stddev / sqrt(maxPoints));

    return tStatistic;
}
// Coediciente de Variacion 
float VizPlot::computeCV() {
    Stats stats = computeStats();
    return (stats.stddev / stats.mean) * 100.0;
}

// ESTADISTICA INFERENCIAL 
// Calcular intervalo de confianza
std::pair<float, float> VizPlot::computeConfidenceInterval(float confidenceLevel) {
    Stats stats = computeStats();
    float mean = stats.mean;
    float stddev = stats.stddev;

    // Valor Z para nivel de confianza (asume distribución normal)
    float zValue = 1.96; // 95% de confianza por defecto
    if (confidenceLevel == 99) zValue = 2.576;
    else if (confidenceLevel == 90) zValue = 1.645;

    float marginOfError = zValue * (stddev / sqrt(maxPoints));
    return {mean - marginOfError, mean + marginOfError};
}

// Calcular prueba T para una muestra
float VizPlot::computeTTest(float expectedMean) {
    Stats stats = computeStats();
    float mean = stats.mean;
    float stddev = stats.stddev;
    float tStatistic = (mean - expectedMean) / (stddev / sqrt(maxPoints));

    return tStatistic;
}

// Correlacion de Pearson 
// Calcular correlación de Pearson
float VizPlot::computePearsonCorrelation(const std::vector<int>& sampleA, const std::vector<int>& sampleB) {
    if (sampleA.size() != sampleB.size()) {
        output->println("Error: Samples must have the same size.");
        return NAN;
    }

    int n = sampleA.size();
    float sumA = 0, sumB = 0, sumAB = 0, sumASq = 0, sumBSq = 0;

    for (int i = 0; i < n; i++) {
        sumA += sampleA[i];
        sumB += sampleB[i];
        sumAB += sampleA[i] * sampleB[i];
        sumASq += sampleA[i] * sampleA[i];
        sumBSq += sampleB[i] * sampleB[i];
    }

    float numerator = n * sumAB - sumA * sumB;
    float denominator = sqrt((n * sumASq - sumA * sumA) * (n * sumBSq - sumB * sumB));

    if (denominator == 0) {
        return 0; // No correlación
    }
    return numerator / denominator;
}

// Calcular prueba de proporciones
float VizPlot::computeProportionTest(float observedProportion, float expectedProportion) {
    float stddev = sqrt(expectedProportion * (1 - expectedProportion) / maxPoints);
    float zStatistic = (observedProportion - expectedProportion) / stddev;

    return zStatistic;
}

float VizPlot::computeChiSquareTest(const std::vector<int>& observed, const std::vector<int>& expected) {
    if (observed.size() != expected.size()) {
        output->println("Error: Observed and expected sizes must match.");
        return NAN;
    }

    float chiSquare = 0.0;
    for (size_t i = 0; i < observed.size(); i++) {
        chiSquare += pow(observed[i] - expected[i], 2) / expected[i];
    }
    return chiSquare;
}

// Kolmogorov-Smirnov Test
float VizPlot::computeKSStatistic(const std::vector<int>& sampleA, const std::vector<int>& sampleB) {
    std::vector<float> cumulativeA(sampleA.size()), cumulativeB(sampleB.size());
    std::partial_sum(sampleA.begin(), sampleA.end(), cumulativeA.begin());
    std::partial_sum(sampleB.begin(), sampleB.end(), cumulativeB.begin());

    float maxDiff = 0.0;
    for (size_t i = 0; i < cumulativeA.size(); i++) {
        float diff = fabs(cumulativeA[i] / cumulativeA.back() - cumulativeB[i] / cumulativeB.back());
        if (diff > maxDiff) {
            maxDiff = diff;
        }
    }
    return maxDiff;
}

// Regresion Lineal 
float VizPlot::computeMannWhitneyU(const std::vector<int>& sampleA, const std::vector<int>& sampleB) {
    std::vector<int> combined = sampleA;
    combined.insert(combined.end(), sampleB.begin(), sampleB.end());
    std::sort(combined.begin(), combined.end());

    float rankSumA = 0;
    for (int value : sampleA) {
        rankSumA += std::distance(combined.begin(), std::find(combined.begin(), combined.end(), value)) + 1;
    }

    float U1 = rankSumA - (sampleA.size() * (sampleA.size() + 1)) / 2.0;
    float U2 = sampleA.size() * sampleB.size() - U1;

    return std::min(U1, U2);
}

// Clasificacion utilizando 

std::pair<float, float> VizPlot::computeLinearRegression(const std::vector<int>& x, const std::vector<int>& y) {
    if (x.size() != y.size()) {
        output->println("Error: X and Y sizes must match.");
        return {NAN, NAN};
    }

    int n = x.size();
    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;

    for (int i = 0; i < n; i++) {
        sumX += x[i];
        sumY += y[i];
        sumXY += x[i] * y[i];
        sumX2 += x[i] * x[i];
    }

    float slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
    float intercept = (sumY - slope * sumX) / n;

    return {slope, intercept};
}

// CLasificacion K-Nearest Neighbors 
String VizPlot::classifyKNN(const std::vector<int>& exampleValues, const std::vector<String>& labels, int newValue, int k) {
    std::vector<std::pair<int, String>> distances;

    for (size_t i = 0; i < exampleValues.size(); i++) {
        int distance = abs(exampleValues[i] - newValue);
        distances.push_back({distance, labels[i]});
    }

    std::sort(distances.begin(), distances.end());
    std::map<String, int> labelCount;

    for (int i = 0; i < k; i++) {
        labelCount[distances[i].second]++;
    }

    String bestLabel;
    int maxCount = 0;
    for (const auto& label : labelCount) {
        if (label.second > maxCount) {
            maxCount = label.second;
            bestLabel = label.first;
        }
    }
    return bestLabel;
}

// Picos 
std::vector<int> VizPlot::detectPeaks(int threshold) {
    std::vector<int> peaks;

    for (int i = 1; i < maxPoints - 1; i++) {
        if (dataBuffer[i] > dataBuffer[i - 1] && dataBuffer[i] > dataBuffer[i + 1] && dataBuffer[i] > threshold) {
            peaks.push_back(i);
        }
    }
    return peaks;
}



// Destructor
VizPlot::~VizPlot() {
    delete[] dataBuffer;
}