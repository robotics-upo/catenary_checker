import os
import matplotlib.pyplot as plt
import sys

# Función para leer y parsear los datos del archivo
def read_data(file_name):
    l = []
    x = []
    y_cat = []
    y_par = []
    error = []

    with open(file_name, 'r') as file:
        lines = file.readlines()
        for line in lines:
            if line.strip():  # Ignorar líneas vacías
                data = line.strip().split(';')
                l_line = []
                x_line = []
                y_cat_line = []
                y_par_line = []
                error_line = []
                for point in data:
                    if point.strip():  # Ignorar cualquier cadena vacía
                        values = point.split(',')
                        l_line.append(float(values[0]))
                        x_line.append(float(values[1]))
                        y_cat_line.append(float(values[2]))
                        y_par_line.append(float(values[3]))
                        error_line.append(float(values[4]))
                
                l.append(l_line)
                x.append(x_line)
                y_cat.append(y_cat_line)
                y_par.append(y_par_line)
                error.append(error_line)
    
    return l, x, y_cat, y_par, error

# Función para graficar los datos
def plot_data(x, y_cat, y_par, mode):
    plt.figure(figsize=(10, 6))

    for i in range(len(x)):
        # Graficar y_cat vs x con líneas más finas
        plt.plot(x[i], y_cat[i], label=f'y_cat_{i}', color='blue', linewidth=1, marker='o', markersize=3)

        # Graficar y_par vs x con líneas más finas
        plt.plot(x[i], y_par[i], label=f'y_par_{i}', color='red', linewidth=1, marker='x', markersize=3)

    # Configuración de la gráfica
    plt.xlabel('X')
    plt.ylabel('Y Values')
    plt.title('Comparación entre y_cat e y_par '+ mode)
    plt.legend()
    plt.grid(True)

    # # Mostrar la gráfica
    # plt.show(block=False)
    # Mostrar la gráfica de manera no bloqueante
    plt.draw()           # Draw the plot
    plt.pause(0.001)     # Pause momentarily to ensure the plot is shown

def calculate_write_errors(file_input, file_output):
    # Comprobar si el archivo de salida ya existe y eliminarlo si es necesario
    if os.path.exists(file_output):
        os.remove(file_output)
        print(f"El archivo {file_output} ya existía y ha sido reemplazado.")

    with open(file_input, 'r') as infile, open(file_output, 'w') as outfile:
        lines = infile.readlines()

        for line in lines:
            if line.strip():  # Ignorar líneas vacías
                # Dividir la línea por ";" para obtener los puntos
                points = line.strip().split(';')

                lengths = []
                errors = []

                # Iterar sobre cada punto
                for point in points:
                    if point.strip():  # Ignorar cualquier cadena vacía
                        values = point.split(',')
                        # Extraer el valor de error (último valor)
                        len_ = float(values[0])
                        error = float(values[4])
                        lengths.append(len_)
                        errors.append(error)

                # Calcular los errores requeridos
                if errors:
                    len_avg = sum(lengths) / len(lengths)
                    error_avg = sum(errors) / len(errors)  # Error promedio
                    error_max = max(errors)               # Error máximo
                    error_min = min(errors)               # Error mínimo

                    # Escribir los resultados en el archivo de salida
                    outfile.write(f"{round(len_avg,1)}, {round(error_avg,4)}, {round(error_max,4)}, {error_min}\n")

def plot_error_vs_length(file_input, mode):
    lengths = []
    errors = []

    # Leer el archivo de entrada
    with open(file_input, 'r') as infile:
        lines = infile.readlines()

        for line in lines:
            if line.strip():  # Ignorar líneas vacías
                values = line.strip().split(',')
                length = float(values[0])  # Primer valor es el length
                error = float(values[1])   # Segundo valor es el error
                lengths.append(length)
                errors.append(error)
    
    # Combinar los valores de length y error en tuplas y ordenarlos por length
    data = sorted(zip(lengths, errors))

    # Desempaquetar los valores ya ordenados
    sorted_lengths, sorted_errors = zip(*data)  

    # Crear la gráfica de error vs length
    plt.figure(figsize=(10, 6))
    plt.plot(sorted_lengths, sorted_errors, marker='o', linestyle='-', color='blue', label='Error')

    # Etiquetas y título
    plt.xlabel('Length')
    plt.ylabel('Error')
    plt.title('Error vs Length'+mode)
    plt.grid(True)
    plt.legend()

    # # Mostrar la gráfica
    # plt.show(block=False)
    
    # Mostrar la gráfica de manera no bloqueante
    plt.draw()           # Draw the plot
    plt.pause(0.001)     # Pause momentarily to ensure the plot is shown


def plot_both_error_vs_length(file_input1, file_input2):
    lengths_bl = []
    errors_bl = []
    max_errors_bl = []
    lengths_bf = []
    errors_bf = []
    max_errors_bf = []

    # Leer el archivo de entrada
    with open(file_input1, 'r') as infile:
        lines = infile.readlines()

        for line in lines:
            if line.strip():  # Ignorar líneas vacías
                values = line.strip().split(',')
                length = float(values[0])  # First value for Length 
                error = float(values[1])   # Second value for error avarage
                max_error = float(values[2])   # Third value for error max
                lengths_bl.append(length)
                errors_bl.append(error)
                max_errors_bl.append(max_error)
    
    # Combinar los valores de length y error en tuplas y ordenarlos por length
    data1 = sorted(zip(lengths_bl, errors_bl))
    data2 = sorted(zip(lengths_bl, max_errors_bl))

    # Desempaquetar los valores ya ordenados
    sorted_lengths_bl, sorted_errors_bl = zip(*data1)  
    sorted_lengths_bl, sorted_errors_max_bl = zip(*data2)  

    # Leer el archivo de entrada
    with open(file_input2, 'r') as infile:
        lines = infile.readlines()

        for line in lines:
            if line.strip():  # Ignorar líneas vacías
                values = line.strip().split(',')
                length = float(values[0])  # First value for Length 
                error = float(values[1])   # Second value for error avarage
                max_error = float(values[2])   # Third value for error max
                lengths_bf.append(length)
                errors_bf.append(error)
                max_errors_bf.append(max_error)
    
    # Combinar los valores de length y error en tuplas y ordenarlos por length
    data1 = sorted(zip(lengths_bf, errors_bf))
    data2 = sorted(zip(lengths_bf, max_errors_bf))

    # Desempaquetar los valores ya ordenados
    sorted_lengths_bf, sorted_errors_bf = zip(*data1)  
    sorted_lengths_bf, sorted_errors_max_bf = zip(*data2)  

    # Crear la gráfica de error vs length para ambos casos
    plt.figure(figsize=(10, 6))

    # Plot for byLength (in blue)
    plt.plot(sorted_lengths_bl, sorted_errors_bl, marker='o', linestyle='-', color='blue', label=' byLength Method')
    # Plot for byFitting (in red)
    plt.plot(sorted_lengths_bf, sorted_errors_bf, marker='x', linestyle='-', color='green', label=' byFitting Method')
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
    # Plot for byFitting (in red)
    plt.plot(sorted_lengths_bf, sorted_errors_max_bf, marker='x', linestyle='-', color='green', label=' byFitting Method')
    # Etiquetas y título
    plt.xlabel('Length')
    plt.ylabel('Max Error')
    plt.title('Max Error vs Length')
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
file_input = os.path.expanduser('~/results_error_catenary_vs_parabola_'+mode+'.txt')
# Leer los datos
l, x, y_cat, y_par, error = read_data(file_input)
# Graficar los datos
plot_data(x, y_cat, y_par, mode)
# Llamar a la función
file_output1 = os.path.expanduser('~/error_summary_'+mode+'.txt')
calculate_write_errors(file_input, file_output1)
# calculate_errors(file_input)
print(f"Los resultados han sido guardados en {file_output1}")
plot_error_vs_length(file_output1, mode)


#### BY LENGHT STUFF
mode = 'byFitting'
# Ruta del archivo en el directorio HOME
file_input = os.path.expanduser('~/results_error_catenary_vs_parabola_'+mode+'.txt')
# Leer los datos
l, x, y_cat, y_par, error = read_data(file_input)
# Graficar los datos
plot_data(x, y_cat, y_par, mode)
# Llamar a la función
file_output2 = os.path.expanduser('~/error_summary_'+mode+'.txt')
calculate_write_errors(file_input, file_output2)
# calculate_errors(file_input)
print(f"Los resultados han sido guardados en {file_output2}")
# Llamar a la función para crear la gráfica
plot_error_vs_length(file_output2, mode)


#### BOTH ERROS
plot_both_error_vs_length(file_output1, file_output2)

# Mantener las gráficas abiertas al final del programa
plt.show()