import os
import statistics

# Lista de archivos a procesar
files = [
    "error_summary_byLength_Length_15m.txt",
    "error_summary_byFitting_Length_15m.txt",
    "error_summary_byFitting_Length_20m.txt",
    "error_summary_byLength_Length_20m.txt",
    "error_summary_byFitting_Length_25m.txt",
    "error_summary_byLength_Length_25m.txt"
]

def process_file(file_name):
    # Variables para almacenar los datos de cada métrica
    lengths = []
    error_avg = []
    error_max = []
    error_min = []

    # Leer el archivo de entrada
    with open(file_name, 'r') as file:
        lines = file.readlines()

        # Procesar cada línea
        for line in lines:
            if line.strip():  # Ignorar líneas vacías
                values = line.strip().split(',')
                lengths.append(float(values[0]))      # Primer valor: L
                error_avg.append(float(values[1]))    # Segundo valor: error_average
                error_max.append(float(values[2]))    # Tercer valor: error_max
                error_min.append(float(values[3]))    # Cuarto valor: error_min

    # Calcular promedio y desviación estándar para cada métrica
    length_mean = statistics.mean(lengths)
    length_stdev = statistics.stdev(lengths)

    error_avg_mean = statistics.mean(error_avg)
    error_avg_stdev = statistics.stdev(error_avg)

    error_max_mean = statistics.mean(error_max)
    error_max_stdev = statistics.stdev(error_max)

    error_min_mean = statistics.mean(error_min)
    error_min_stdev = statistics.stdev(error_min)

    return {
        "file": file_name,
        "length_mean": length_mean,
        "length_stdev": length_stdev,
        "error_avg_mean": error_avg_mean,
        "error_avg_stdev": error_avg_stdev,
        "error_max_mean": error_max_mean,
        "error_max_stdev": error_max_stdev,
        "error_min_mean": error_min_mean,
        "error_min_stdev": error_min_stdev
    }

def main():
    # Procesar cada archivo y calcular estadísticas
    for file in files:
        result = process_file(file)
        print(f"\nResultados para {result['file']}:")
        print(f"Promedio L: {result['length_mean']:.2f}, Desviación estándar L: {result['length_stdev']:.2f}")
        print(f"Promedio error_avg: {result['error_avg_mean']:.2f}, Desviación estándar error_avg: {result['error_avg_stdev']:.2f}")
        print(f"Promedio error_max: {result['error_max_mean']:.2f}, Desviación estándar error_max: {result['error_max_stdev']:.2f}")
        print(f"Promedio error_min: {result['error_min_mean']:.2f}, Desviación estándar error_min: {result['error_min_stdev']:.2f}")

if __name__ == "__main__":
    main()
