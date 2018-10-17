import matplotlib.pyplot as plt
import matplotlib.patches as patches

SCREEN_X_RES = 128
SCREEN_Y_RES = 64
NUM_X_CELLS = 6
NUM_Y_CELLS = 6
PIXELS_PER_X_CELL = int(SCREEN_X_RES/NUM_X_CELLS)
PIXELS_PER_Y_CELL = int(SCREEN_Y_RES/NUM_Y_CELLS)
MAX_Y_COORD = PIXELS_PER_Y_CELL * NUM_Y_CELLS

d = {}

def overlap(l1, r1, l2, r2):
    if (l1[0] > r2[0] or l2[0] > r1[0]):
        return False
    if (l1[1] > r2[1] or l2[1] > r1[1]):
        return False
    return True

for i in range(NUM_X_CELLS):
    for j in range(NUM_Y_CELLS):
        x0 = PIXELS_PER_X_CELL * i
        y0 = MAX_Y_COORD - PIXELS_PER_Y_CELL * (j + 1)
        x1 = PIXELS_PER_X_CELL * (i + 1)
        y1 = MAX_Y_COORD - PIXELS_PER_Y_CELL * j
        d[(i, j)] = (x0, y0, x1, y1)

fig, ax = plt.subplots(1)
ax.set_xlim(0, SCREEN_X_RES)
ax.set_ylim(0, SCREEN_Y_RES)

for k in d:
    c = None
    l = None
    if (k[0] + k[1]) % 2 == 0:
        c = 'r'
        l = 2
    else:
        c = 'b'
        l = 4
    x = d[k][0]
    y = d[k][1]
    w = d[k][2]-d[k][0]
    h = d[k][3]-d[k][1]
    rect = patches.Rectangle((x, y), w, h, linewidth=l, edgecolor=c, facecolor='none', fill=False)
    ax.add_patch(rect)
    ax.annotate(k, ((x + w/2), (y + h/2)), color=c, weight='bold', fontsize=6, ha='center', va='center')

plt.gca().invert_yaxis()
plt.show()
