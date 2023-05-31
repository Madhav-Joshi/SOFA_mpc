import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from mpc_func import lift

# The parametrized function to be plotted
def f(U0,U1,U2,X0,X1,L0,L1,L2):
    A = np.loadtxt('./Wakrabot/data_files/systemMatrix.csv',delimiter=',') # depends on dt
    B = np.loadtxt('./Wakrabot/data_files/controlMatrix.csv',delimiter=',') # depends on dt
    Xf = A@lift(np.array([[X0,X1,L0,L1,L2]]).T)+B@np.array([[U0,U1,U2]]).T
    return Xf

# Define range
U0_range = [0,0.4]
U1_range = [0,0.4]
U2_range = [0,0.4]
X0_range = [-0.3,0.3]
X1_range = [-0.3,0.3]
L0_range = [-0.2,0.2]
L1_range = [-0.2,0.2]
L2_range = [-0.2,0.2]

# Define initial parameters
init_U0 = 0.2-0.1
init_U1 = 0.26-0.1
init_U2 = 0.28-0.1
init_X0 = -0.0096
init_X1 = -0.0342
init_L0 = 0.266-0.24
init_L1 = 0.258-0.24
init_L2 = 0.255-0.24

t = np.array([0,0.05])
# Create the figure and the line0 that we will manipulate
fig, axs = plt.subplots(2,1)
y_out = f(init_U0, init_U1, init_U2, init_X0, init_X1, init_L0, init_L1, init_L2)
line0, = axs[0].plot(t, np.array([init_X0,y_out[0,0]]), lw=2)
axs[0].set_xlabel('Time [s]')
axs[0].set_ylabel('X [m]')

line1, = axs[1].plot(np.array([init_X0,y_out[0,0]]), np.array([init_X1,y_out[1,0]]), lw=2)
axs[1].set_xlabel('X [m]')
axs[1].set_ylabel('Z [m]')

# adjust the main plot to make room for the sliders
fig.subplots_adjust(left=0.45, bottom=0.25, top=0.95)

# Make a vertical slider to control the future number of steps.
axU0 = fig.add_axes([0.075, 0.3, 0.0225, 0.6])
U0_slider = Slider(ax=axU0,label='U0',valmin=U0_range[0],valmax=U0_range[1],
                    valinit=init_U0,orientation="vertical")

# Make a vertical slider to control the future number of steps.
axU1 = fig.add_axes([0.15, 0.3, 0.0225, 0.6])
U1_slider = Slider(ax=axU1,label='U1',valmin=U1_range[0],valmax=U1_range[1],
                    valinit=init_U1,orientation="vertical")

# Make a vertical slider to control the future number of steps.
axU2 = fig.add_axes([0.225, 0.3, 0.0225, 0.6])
U2_slider = Slider( ax=axU2,label='U2',valmin=U2_range[0],
                    valmax=U2_range[1],valinit=init_U2,orientation="vertical")

# Make a vertical slider to control the future number of steps.
axX0 = fig.add_axes([0.3, 0.3, 0.0225, 0.6])
X0_slider = Slider(  ax=axX0,label='X0',valmin=X0_range[0],valmax=X0_range[1],
                    valinit=init_X0,orientation="vertical")

# Make a horizontally oriented slider to control the amplitude
axX1 = fig.add_axes([0.5, 0.1, 0.4, 0.03])
X1_slider = Slider(  ax=axX1,label="X1",valmin=X1_range[0], valmax=X1_range[1],
                   valinit=init_X1)

# Make a horizontally oriented slider to control the amplitude
axL0 = fig.add_axes([0.075, 0.2, 0.3, 0.03])
L0_slider = Slider(  ax=axL0,label="L0",valmin=L0_range[0],
                    valmax=L0_range[1],valinit=init_L0)

# Make a horizontally oriented slider to control the amplitude
axL1 = fig.add_axes([0.075, 0.125, 0.3, 0.03])
L1_slider = Slider(  ax=axL1,label="L1",valmin=L1_range[0],
                    valmax=L1_range[1],valinit=init_L1)

# Make a horizontally oriented slider to control the amplitude
axL2 = fig.add_axes([0.075, 0.05, 0.3, 0.03])
L2_slider = Slider(  ax=axL2,label="L2",valmin=L2_range[0],
                    valmax=L2_range[1],valinit=init_L2)


# The function to be called anytime a slider's value changes
def update(val):
    predicted = f(  U0_slider.val, U1_slider.val, U2_slider.val, X0_slider.val, X1_slider.val,
                    L0_slider.val,L1_slider.val,L2_slider.val)
    print(predicted)
    y_out = np.array([X0_slider.val,predicted[0,0]])
    line0.set_xdata(t)
    line0.set_ydata(y_out)
    axs[0].set_xlim(np.min(t),np.max(t))
    axs[0].set_ylim(np.min(y_out),np.max(y_out))

    line1.set_xdata(np.array([X0_slider.val,predicted[0,0]]))
    line1.set_ydata(np.array([X1_slider.val,predicted[1,0]]))
    axs[1].set_xlim(-0.15,0.15)
    axs[1].set_ylim(-0.15,0.15)

    fig.canvas.draw_idle()


# register the update function with each slider
U0_slider.on_changed(update)
U1_slider.on_changed(update)
U2_slider.on_changed(update)
X0_slider.on_changed(update)
X1_slider.on_changed(update)
L0_slider.on_changed(update)
L1_slider.on_changed(update)
L2_slider.on_changed(update)

# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = fig.add_axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetax, 'Reset', hovercolor='0.975')


def reset(event):
    U0_slider.reset()
    U1_slider.reset()
    U2_slider.reset()
    X0_slider.reset()
    X1_slider.reset()
    L0_slider.reset()
    L1_slider.reset()
    L2_slider.reset()
button.on_clicked(reset)

plt.show()