import numpy as np
import matplotlib.pyplot as plt

def trilateracion(p1, p2, p3, r1, r2, r3):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3

    # Trasladar el sistema para que p1 quede en el origen
    dx = x2 - x1
    dy = y2 - y1
    d = np.sqrt(dx**2 + dy**2)

    # Coordenadas transformadas
    ex = [(x2 - x1) / d, (y2 - y1) / d]
    i = ex[0] * (x3 - x1) + ex[1] * (y3 - y1)

    aux = [(x3 - x1) - i * ex[0], (y3 - y1) - i * ex[1]]
    ey = aux / np.linalg.norm(aux)
    j = ey[0] * (x3 - x1) + ey[1] * (y3 - y1)

    x = (r1**2 - r2**2 + d**2) / (2 * d)
    y = (r1**2 - r3**2 + i**2 + j**2 - 2 * i * x) / (2 * j)

    final_x = x1 + x * ex[0] + y * ey[0]
    final_y = y1 + x * ex[1] + y * ey[1]

    return final_x, final_y


# Coordenadas de los nodos fijos
X = 100  # ancho en metros
Y = 80   # alto en metros

p1 = (0, 0)
p2 = (X, 0)
p3 = (0, Y)
p4 = (X, Y)

# Posición real del objeto (simulación)
real_pos = (40, 30)

# Calcular distancias reales al objeto
r1 = np.linalg.norm(np.subtract(real_pos, p1))
r2 = np.linalg.norm(np.subtract(real_pos, p2))
r3 = np.linalg.norm(np.subtract(real_pos, p3))
r4 = np.linalg.norm(np.subtract(real_pos, p4))  # no usado, solo para verificar

# Calcular posición estimada con trilateración
estimated_pos = trilateracion(p1, p2, p3, r1, r2, r3)

# Mostrar resultados
print(f"Posición real:      {real_pos}")
print(f"Posición estimada:  {estimated_pos}")

# Plot para visualización
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_title("Simulación de Trilateración en 2D")
ax.grid(True)
ax.set_xlim(-10, X + 20)
ax.set_ylim(-10, Y + 20)

# Dibujar nodos
for p, label in zip([p1, p2, p3, p4], ['P1', 'P2', 'P3', 'P4']):
    ax.plot(*p, 'ko')
    ax.text(p[0]+2, p[1]+2, label, fontsize=10)

# Dibujar círculos (distancias)
for p, r, color in zip([p1, p2, p3], [r1, r2, r3], ['r', 'g', 'b']):
    circle = plt.Circle(p, r, color=color, fill=False, linestyle='--')
    ax.add_patch(circle)

# Dibujar punto real y estimado
ax.plot(*real_pos, 'mo', label="Posición real")
ax.plot(*estimated_pos, 'c*', label="Posición estimada")

ax.legend()
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.show()
