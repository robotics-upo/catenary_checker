import os
import matplotlib.pyplot as plt
import sys
import statistics as stats

# Función para leer y parsear los datos del archivo
def plot_both_error_vs_length(file_input1, file_input2, file_input3):
    lengths_bl = []
    errors_bl = []
    max_errors_bl = []
    errors_over_l_bl = []
    lengths_bf = []
    errors_bf = []
    max_errors_bf = []
    errors_over_l_bf = []
    lengths_bp = []
    errors_bp = []
    max_errors_bp = []
    errors_over_l_bp = []

    # Leer el archivo de entrada
    with open(file_input1, 'r') as infile:
        lines = infile.readlines()

        for line in lines:
            if line.strip():  # Ignorar líneas vacías
                values = line.strip().split(',')
                length = float(values[4])  # First value for Length 
                error = float(values[11])   # Second value for error avarage
                max_error = float(values[9])   # Third value for error max
                e_over_l = error/length
                lengths_bl.append(length)
                errors_bl.append(error)
                max_errors_bl.append(max_error)
                errors_over_l_bl.append(e_over_l)

     # Calcular promedio y desviación estándar para file_input1
    avg_error_bl = stats.mean(errors_bl)
    std_dev_error_bl = stats.stdev(errors_bl)
    avg_max_error_bl = stats.mean(max_errors_bl)
    std_dev_max_error_bl = stats.stdev(max_errors_bl)
    avg_error_over_l_bl = stats.mean(errors_over_l_bl)
    std_dev_error_over_l_bl = stats.stdev(errors_over_l_bl)
    print(f"byLength: Avg Error = {avg_error_bl:.4f}, Std Dev Error = {std_dev_error_bl:.4f}")
    print(f"byLength: Avg Max Error = {avg_max_error_bl:.4f}, Std Dev Max Error = {std_dev_max_error_bl:.4f}")
    print(f"byLength: Avg Error over Length = {avg_error_over_l_bl:.4f}, Std Dev Error over Length = {std_dev_error_over_l_bl:.4f}")
    
    # Combinar los valores de length y error en tuplas y ordenarlos por length
    data1 = sorted(zip(lengths_bl, errors_bl))
    data2 = sorted(zip(lengths_bl, max_errors_bl))
    data3 = sorted(zip(lengths_bl, errors_over_l_bl))

    # Desempaquetar los valores ya ordenados
    sorted_lengths_bl, sorted_errors_bl = zip(*data1)  
    sorted_lengths_bl, sorted_errors_max_bl = zip(*data2)  
    sorted_lengths_bl, sorted_errors_over_l_bl = zip(*data3)  

    # Leer el archivo de entrada
    with open(file_input2, 'r') as infile:
        lines = infile.readlines()

        for line in lines:
            if line.strip():  # Ignorar líneas vacías
                values = line.strip().split(',')
                length = float(values[4])  # First value for Length 
                error = float(values[11])   # Second value for error avarage
                max_error = float(values[9])   # Third value for error max
                e_over_l = error/length
                lengths_bf.append(length)
                errors_bf.append(error)
                max_errors_bf.append(max_error)
                errors_over_l_bf.append(e_over_l)

    # Calcular promedio y desviación estándar para file_input2
    avg_error_bf = stats.mean(errors_bf)
    std_dev_error_bf = stats.stdev(errors_bf)
    avg_max_error_bf = stats.mean(max_errors_bf)
    std_dev_max_error_bf = stats.stdev(max_errors_bf)
    avg_error_over_l_bf = stats.mean(errors_over_l_bf)
    std_dev_error_over_l_bf = stats.stdev(errors_over_l_bf)
    print(f"\nbyFitting: Avg Error = {avg_error_bf:.4f}, Std Dev Error = {std_dev_error_bf:.4f}")
    print(f"byFitting: Avg Max Error = {avg_max_error_bf:.4f}, Std Dev Max Error = {std_dev_max_error_bf:.4f}")
    print(f"byFitting: Avg Error over Length = {avg_error_over_l_bf:.4f}, Std Dev Error over Length = {std_dev_error_over_l_bf:.4f}")
    
    # Combinar los valores de length y error en tuplas y ordenarlos por length
    data1 = sorted(zip(lengths_bf, errors_bf))
    data2 = sorted(zip(lengths_bf, max_errors_bf))
    data3 = sorted(zip(lengths_bf, errors_over_l_bf))

    # Desempaquetar los valores ya ordenados
    sorted_lengths_bf, sorted_errors_bf = zip(*data1)  
    sorted_lengths_bf, sorted_errors_max_bf = zip(*data2)  
    sorted_lengths_bf, sorted_errors_over_l_bf = zip(*data3)  

    # Leer el archivo de entrada
    with open(file_input3, 'r') as infile:
        lines = infile.readlines()

        for line in lines:
            if line.strip():  # Ignorar líneas vacías
                values = line.strip().split(',')
                length = float(values[4])  # First value for Length 
                error = float(values[11])   # Second value for error avarage
                max_error = float(values[9])   # Third value for error max
                e_over_l = error/length
                lengths_bp.append(length)
                errors_bp.append(error)
                max_errors_bp.append(max_error)
                errors_over_l_bp.append(e_over_l)
    
    # Calcular promedio y desviación estándar para file_input3
    avg_error_bp = stats.mean(errors_bp)
    std_dev_error_bp = stats.stdev(errors_bp)
    avg_max_error_bp = stats.mean(max_errors_bp)
    std_dev_max_error_bp = stats.stdev(max_errors_bp)
    avg_error_over_l_bp = stats.mean(errors_over_l_bp)
    std_dev_error_over_l_bp = stats.stdev(errors_over_l_bp)
    print(f"\nbyPoints: Avg Error = {avg_error_bp:.4f}, Std Dev Error = {std_dev_error_bp:.4f}")
    print(f"byPoints: Avg Max Error = {avg_max_error_bp:.4f}, Std Dev Max Error = {std_dev_max_error_bp:.4f}")
    print(f"byPoints: Avg Error over Length = {avg_error_over_l_bp:.4f}, Std Dev Error over Length = {std_dev_error_over_l_bp:.4f}")
    
    # Combinar los valores de length y error en tuplas y ordenarlos por length
    data1 = sorted(zip(lengths_bp, errors_bp))
    data2 = sorted(zip(lengths_bp, max_errors_bp))
    data3 = sorted(zip(lengths_bp, errors_over_l_bp))

    # Desempaquetar los valores ya ordenados
    sorted_lengths_bp, sorted_errors_bp = zip(*data1)  
    sorted_lengths_bp, sorted_errors_max_bp = zip(*data2)  
    sorted_lengths_bp, sorted_errors_over_l_bp = zip(*data3)  


    # Crear la gráfica de error vs length para ambos casos
    plt.figure(figsize=(10, 6))
    # Plot for byLength (in blue)
    plt.plot(sorted_lengths_bl, sorted_errors_bl, marker='o', linestyle='-', color='blue', label=' byLength Method')
    # Plot for byFitting (in green)
    plt.plot(sorted_lengths_bf, sorted_errors_bf, marker='x', linestyle='-', color='green', label=' byFitting Method')
    # Plot for byFitting (in red)
    plt.plot(sorted_lengths_bp, sorted_errors_bp, marker='*', linestyle='-', color='magenta', label=' byPoints Method', linewidth=0.8)
    # Etiquetas y título
    plt.xlabel('Length')
    plt.ylabel('Average Error')
    plt.title('Average Error vs Length')
    plt.grid(True)
    plt.legend()
    # Mostrar la gráfica de manera no bloqueante
    plt.draw()           # Draw the plot
    plt.pause(0.001)     # Pause momentarily to ensure the plot is shown
    
    
    # Crear la gráfica de error vs length para ambos casos
    plt.figure(figsize=(10, 6))
    # Plot for byLength (in blue)
    plt.plot(sorted_lengths_bl, sorted_errors_max_bl, marker='o', linestyle='-', color='blue', label=' byLength Method')
    # Plot for byFitting (in green)
    plt.plot(sorted_lengths_bf, sorted_errors_max_bf, marker='x', linestyle='-', color='green', label=' byFitting Method')
    # Plot for byFitting (in red)
    plt.plot(sorted_lengths_bp, sorted_errors_max_bp, marker='*', linestyle='-', color='magenta', label=' byPoints Method', linewidth=0.8)
    # Etiquetas y título
    plt.xlabel('Length')
    plt.ylabel('Max Error')
    plt.title('Max Error vs Length')
    plt.grid(True)
    plt.legend()
    # Mostrar la gráfica de manera no bloqueante
    plt.draw()           # Draw the plot
    plt.pause(0.001)     # Pause momentarily to ensure the plot is shown


    # Crear la gráfica de error vs length para ambos casos
    plt.figure(figsize=(10, 6))
    # Plot for byLength (in blue)
    plt.plot(sorted_lengths_bl, sorted_errors_over_l_bl, marker='o', linestyle='-', color='blue', label=' byLength Method')
    # Plot for byFitting (in green)
    plt.plot(sorted_lengths_bf, sorted_errors_over_l_bf, marker='x', linestyle='-', color='green', label=' byFitting Method')
    # Plot for byFitting (in red)
    plt.plot(sorted_lengths_bp, sorted_errors_over_l_bp, marker='*', linestyle='-', color='magenta', label=' byPoints Method', linewidth=0.8)
    # Etiquetas y título
    plt.xlabel('Length')
    plt.ylabel('Error over Length')
    plt.title('Error over Length vs Length')
    plt.grid(True)
    plt.legend()

    # Mostrar la gráfica de manera no bloqueante
    plt.draw()           # Draw the plot
    plt.pause(0.001)     # Pause momentarily to ensure the plot is shown

print("\n\t *** INITIALIZING PYTHON SCRIPT FOR CATENARY AND PARABOLA PLOT, AND ERROR COMPUTATION AND PLOT ***\n")  # Warnning message


# print("\n\t *** IMPORTANT: Use this script given a parameter: byLength or byFitting ***\n")  # Warnning message
# mode = sys.argv[1] # number of position to analize


#### BY LENGHT STUFF
mode = 'byLength'
# Ruta del archivo en el directorio HOME
file_input1 = os.path.expanduser('~/results_error_catenary_vs_parabola_'+mode+'.txt')
#### BY LENGHT STUFF
mode = 'byFitting'
# Ruta del archivo en el directorio HOME
file_input2 = os.path.expanduser('~/results_error_catenary_vs_parabola_'+mode+'.txt')
#### BY LENGHT STUFF
mode = 'byPoints'
# Ruta del archivo en el directorio HOME
file_input3 = os.path.expanduser('~/results_error_catenary_vs_parabola_'+mode+'.txt')
# Llamar a la función
plot_both_error_vs_length(file_input1, file_input2, file_input3)


# Mantener las gráficas abiertas al final del programa
plt.show()