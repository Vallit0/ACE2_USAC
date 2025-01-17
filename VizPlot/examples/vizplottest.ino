// VizPlotTest.ino
#include <VizPlot.h> // Asegúrate de que el archivo VizPlot.h esté en la misma carpeta o en la carpeta de bibliotecas

// Crear una instancia de VizPlot
VizPlot plotter("Example Plot", 10);

void setup() {
    Serial.begin(9600);
    plotter.begin(&Serial);

    // Agregar datos al buffer
    Serial.println("Adding Data...");
    for (int i = 1; i <= 10; i++) {
        plotter.addData(i * 2); // Agregar datos de prueba
    }

    // Calcular y mostrar estadísticas
    Serial.println("Computing Statistics...");
    Stats stats = plotter.computeStats();
    Serial.println("Mean: " + String(stats.mean));
    Serial.println("Min: " + String(stats.min));
    Serial.println("Max: " + String(stats.max));
    Serial.println("StdDev: " + String(stats.stddev));

    // Plot Line
    Serial.println("Plotting Line...");
    plotter.plotLine();

    // Plot Bar
    Serial.println("Plotting Bar...");
    plotter.plotBar();

    // Plot Scatter
    Serial.println("Plotting Scatter...");
    plotter.plotScatter();

    // Plot Histogram
    Serial.println("Plotting Histogram...");
    plotter.plotHistogram();

    // Plot Heatmap
    Serial.println("Plotting Heatmap...");
    plotter.plotHeatmap();

    // Plot BoxPlot
    Serial.println("Plotting BoxPlot...");
    plotter.plotBoxPlot();

    // Detectar anomalías
    Serial.println("Detecting Anomalies...");
    std::vector<int> anomalies = plotter.detectAnomalies(1.5); // Umbral de Z-Score
    for (int idx : anomalies) {
        Serial.println("Anomaly Detected at Index: " + String(idx));
    }

    // Calcular intervalo de confianza
    Serial.println("Computing Confidence Interval...");
    auto ci = plotter.computeConfidenceInterval(95);
    Serial.println("Confidence Interval (95%): [" + String(ci.first) + ", " + String(ci.second) + "]");

    // Calcular prueba T
    Serial.println("Computing T-Test...");
    float tTest = plotter.computeTTest(12.0);
    Serial.println("T-Test Result: " + String(tTest));

    // Calcular correlación de Pearson
    Serial.println("Computing Pearson Correlation...");
    std::vector<int> sampleA = {1, 2, 3, 4, 5};
    std::vector<int> sampleB = {2, 4, 6, 8, 10};
    float correlation = plotter.computePearsonCorrelation(sampleA, sampleB);
    Serial.println("Pearson Correlation: " + String(correlation));

    // Logging
    Serial.println("Logging Examples...");
    plotter.logInfo("System initialized.");
    plotter.logWarning("Low battery detected.");
    plotter.logError("Sensor disconnected.");
    plotter.logDebug("Debugging sensor readings.");
    plotter.showLogsByTag("INFO");

    // Exportar estadísticas a EEPROM (Solo si tienes hardware compatible)
    Serial.println("Exporting Stats to EEPROM...");
    plotter.exportStatsToEEPROM(0);
}

void loop() {
    // No actions in loop for testing
}
