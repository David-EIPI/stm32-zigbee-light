import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


def arrow(ax, p1, p2, **props):
    ax.annotate("", p1, p2,
                arrowprops=dict(arrowstyle="<->", shrinkA=0, shrinkB=0, **props))

def rarrow(ax, p1, p2, **props):
    ax.annotate("", p1, p2,
                arrowprops=dict(arrowstyle="<-", shrinkA=0, shrinkB=0, **props))


# AC voltage magnitude, V
v_ac = 120 * np.sqrt(2)

# Optocoupler on current, A
i_oc = 1.6e-3

# x axis (time) units, ms
t_unit = 1e-3

# AC frequency, Hz
freq_ac = 60

# LED current limiting resistor value, Ohm
r_led = 80e3

# Duty time, fraction
duty = 0.2

full_period = 1/freq_ac/t_unit
x = np.linspace(full_period/8, 2.125*full_period, 500)

y = np.sin(x * freq_ac * 2 * np.pi * t_unit) * v_ac

# 3 plots: 1 = sine wave, 2 = output logic level, 3 = STM32 timers

# Plot 1. Sine wave.
ax_w, ax_v, ax_t = plt.figure(figsize=(7.5, 6)).subplots(
	3, 
	height_ratios = [0.6, 0.2, 0.2], 
	sharex='col', 
	gridspec_kw = { "hspace": 0.01 }
)

ax_w.set_ylabel("AC voltage", loc='center')
ax_w.set_yticks([])

ax_w.plot(x, y)

aymin, aymax = ax_w.get_ylim()
axmin, axmax = ax_w.get_xlim()
axmax += full_period / 1.5
axmin = axmin/2
aspect_ratio = (aymax - aymin)/(axmax - axmin)

# fix the Axes limits so that the following helper drawings
# cannot change them further.
ax_w.set(xlim=[axmin, axmax], ylim=[aymin, aymax])

# Fill the area where Opto is ON 
ax_w.fill_between(x, y, r_led * i_oc, where=y>(r_led * i_oc), facecolor="orange" )

# Detect optocoupler output edges
vout = y > r_led * i_oc
diff = np.logical_xor(np.append(vout, False), np.insert(vout, 0, False))
diff = np.delete(diff, -1)
edges = x[diff]

# Fill the area where TRIAC is ON
y_triac = np.sin(np.pi * duty) * v_ac
triac_on = np.logical_and(np.abs(y)<y_triac, x % (full_period/2) > full_period/4)
ax_w.fill_between(x, 0, y, where=triac_on, facecolor="red" )

# Add zero voltage line
ax_w.axhline(ls='--')
ax_w.text(40, 5, "V=0", size=14)

# Add Opto ON line
ax_w.axhline(r_led * i_oc, ls='--', color='orange')
ax_w.text(39, r_led * i_oc+10, r"$\tt{V=R_{led} I_{on}}$", size=14)
ax_w.annotate(r"$\rm{t_{on}}$", (edges[0], aymin), ((edges[0] + edges[1])/2, aymin*0.75),
                arrowprops=dict(arrowstyle="->", shrinkA=0, shrinkB=0))
ax_w.annotate(r"$\rm{t_{on}}$", (edges[2], aymin), ((edges[2] + edges[3])/2, aymin*0.75),
                arrowprops=dict(arrowstyle="->", shrinkA=0, shrinkB=0))


# Add zero crossing marks
zero_crossings = np.asarray([i * full_period / 2 for i in range(5)])
ax_w.vlines(zero_crossings, aymin, 0, linestyles='dotted')

# TRIAC ON marks
triac_edges = zero_crossings - duty * full_period / 2
ax_w.vlines(triac_edges, aymin, 0, linestyles='dotted', color='red')

# Add OC Vout edge lines
ax_w.vlines(edges, aymin, r_led * i_oc, linestyles='dotted', color='orange')

# Add circuit schematics
image = np.asarray(Image.open('ok_schematics.png'))
l, b, w, h = ax_w.get_position().bounds


ax_w2 = plt.axes((l + w - w/2.65, b, w/2.25, h/2.25),
	xlim=(0, 720), ylim=(480, 0),
	frame_on=False, xticks=[], yticks=[])

# Force transparency
im_copy = np.copy(image)
im_copy[image[:,:,3] < 17] = 0
# Draw
ax_w2.imshow(im_copy)

# Plot 2. Optical relay output.

ax_v.set_yticks([])
ax_v.set_ylabel("Vout")
ax_v.plot(x, vout, color="orange")

# Add arrows and text to indicate input timers
arrow(ax_v, (edges[0], 0.1), (edges[2], 0.1), color="sienna")
arrow(ax_v, (edges[2], 0.2), (edges[3], 0.2), color="olive")
ax_v.vlines(zero_crossings, 0, 1, linestyles='dotted')

ax_v.text((edges[0] + edges[2]) / 2, 0.14, r"$\tt{T1_{CH1}}$", size=12)
ax_v.text(edges[2], 0.24, r"$\tt{T1_{CH2}}$", size=12)

# Plot 3. TRIAC control
ax_t.set_yticks([])

ax_t.set_ylabel("Triac gate")
ax_t.plot(x, triac_on, color = 'red')
arrow(ax_t, (edges[2], 0.6), (triac_edges[3], 0.6), color="teal")
ax_t.text(edges[2], 0.64, r"$\tt{T1_{CH3}}$", size=12)
ax_t.vlines(zero_crossings, 0, 1, linestyles='dotted')
ax_v.vlines(triac_edges, 0, 1, linestyles='dotted', color='red')
ax_t.vlines(edges[2], 0, 1, linestyles='dotted', color='orange')

arrow(ax_t, (edges[2], 0.6), (triac_edges[3], 0.6), color="teal")
#arrow(ax_t, (edges[2], 0.2), (triac_edges[3], 0.2), color="cyan")

ax_t.text(zero_crossings[3], 0.24, r"$\tt{T2_{CH1}}$", size=12)
rarrow(ax_t, (triac_edges[3], 0.2), (zero_crossings[3], 0.2), color="cyan")
rarrow(ax_t, (zero_crossings[3], 0.2), (triac_edges[4], 0.2), color="cyan")

ax_t.set_xlabel("Time, ms")

plt.savefig("sinewave.png", dpi=200, pad_inches=0.0)
plt.show()
