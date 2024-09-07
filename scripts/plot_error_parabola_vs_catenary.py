import os
import matplotlib.pyplot as plt
import sys


# Función para leer y parsear los datos del archivo
def read_data(file_name):
    x = []
    y_cat = []
    y_par = []
    error = []

    with open(file_name, 'r') as file:
        lines = file.readlines()
        for line in lines:
            if line.strip():  # Ignorar líneas vacías
                data = line.strip().split(';')
                x_line = []
                y_cat_line = []
                y_par_line = []
                error_line = []
                for point in data:
                    if point.strip():  # Ignorar cualquier cadena vacía
                        values = point.split(',')
                        x_line.append(float(values[0]))
                        y_cat_line.append(float(values[1]))
                        y_par_line.append(float(values[2]))
                        error_line.append(float(values[3]))
                
                x.append(x_line)
                y_cat.append(y_cat_line)
                y_par.append(y_par_line)
                error.append(error_line)
    
    return x, y_cat, y_par, error

# Función para graficar los datos
def plot_data(x, y_cat, y_par):
    plt.figure(figsize=(10, 6))

    for i in range(len(x)):
        # Graficar y_cat vs x con líneas más finas
        plt.plot(x[i], y_cat[i], label=f'y_cat_{i}', color='blue', linewidth=1, marker='o', markersize=3)

        # Graficar y_par vs x con líneas más finas
        plt.plot(x[i], y_par[i], label=f'y_par_{i}', color='red', linewidth=1, marker='x', markersize=3)

    # Configuración de la gráfica
    plt.xlabel('X')
    plt.ylabel('Y Values')
    plt.title('Comparación entre y_cat e y_par')
    plt.legend()
    plt.grid(True)

    # Mostrar la gráfica
    plt.show()


print("\n\t *** IMPORTANT: Use this script given one parameter: byLength or byFitting ***\n")  # Warnning message
m_ = sys.argv[1] # number of position to analize


# Ruta del archivo en el directorio HOME
file_name = os.path.expanduser('~/results_error_catenary_vs_parabola_'+m_+'.txt')

# Leer los datos
x, y_cat, y_par, error = read_data(file_name)

# Graficar los datos
plot_data(x, y_cat, y_par)
