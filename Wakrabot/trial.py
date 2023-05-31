import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from mpc_simple_test import CableController

# The parametrized function to be plotted
def f(i, N, wxt, wxc, wu, Sx=np.array([1]), Su=np.array([1])):
    c = CableController(i,N,wxt,wxc,wu,curr_coord=np.array([[0,0,0,0,0]]).T)
    c.problem.N=N
    c.problem.wu=wu
    c.i = i
    c.scale_problem(Sx,Su)
    c.solveMPC()
    c.descale_problem()
    return c.Xf[:,:]

def ref(i,N):
    c = CableController(i,N)
    return c.problem.xref

# Define range
N_range = [1,10]
wxt_range = [-3,0]
wxc_range = [-3,0]
wu_range = [-5,-3]
i_range = [0,400]
Sx0_range = [0.001,100]
Sx1_range = [0.001,100]
Su_range = [0.001,100]

# Define initial parameters
init_i = 0
init_N = 6
init_wxt = 0
init_wxc = 0
init_wu = -4.3
init_Sx = np.array([1])
init_Su = np.array([1])


dt = 0.05
t = np.arange(init_i*dt,dt*(init_i+init_N+1-1e-6),dt)
# Create the figure and the line0 that we will manipulate
fig, axs = plt.subplots(2,1)
y_out = f(init_i, init_N, np.array([10**init_wxt,10**init_wxt,0,0,0]), 
          np.array([10**init_wxc,10**init_wxc,0,0,0]), 
          10**init_wu, init_Sx, init_Su)

line0, = axs[0].plot(t, y_out[0,:], lw=2)
scatter0 = axs[0].scatter(t,ref(init_i,init_N)[0,:])
axs[0].set_xlabel('Time [s]')
axs[0].set_ylabel('X [m]')

line1, = axs[1].plot(t, y_out[1,:], lw=2)
scatter1 = axs[1].scatter(t,ref(init_i,init_N)[1,:])
axs[1].set_xlabel('Time [s]')
axs[1].set_ylabel('Z [m]')

# adjust the main plot to make room for the sliders
fig.subplots_adjust(left=0.45, bottom=0.25, top=0.95)

# Make a horizontally oriented slider to control the amplitude
axN = fig.add_axes([0.5, 0.1, 0.4, 0.03])
N_slider = Slider(  ax=axN,label="N",valmin=N_range[0],
                    valmax=N_range[1],valinit=init_N,valstep=1)

# Make a horizontally oriented slider to control the amplitude
axSx0 = fig.add_axes([0.075, 0.2, 0.3, 0.03])
Sx0_slider = Slider(  ax=axSx0,label="Sx0",valmin=Sx0_range[0],
                    valmax=Sx0_range[1],valinit=init_Sx[0])

# Make a horizontally oriented slider to control the amplitude
axSx1 = fig.add_axes([0.075, 0.125, 0.3, 0.03])
Sx1_slider = Slider(  ax=axSx1,label="Sx1",valmin=Sx1_range[0],
                    valmax=Sx1_range[1],valinit=init_Sx[0])

# Make a horizontally oriented slider to control the amplitude
axSu = fig.add_axes([0.075, 0.05, 0.3, 0.03])
Su_slider = Slider(  ax=axSu,label="Su",valmin=Su_range[0],
                    valmax=Su_range[1],valinit=init_Su[0])

# Make a vertical slider to control the future number of steps.
axwxt = fig.add_axes([0.075, 0.3, 0.0225, 0.6])
wxt_slider = Slider(ax=axwxt,label='wxt',valmin=wxt_range[0],valmax=wxt_range[1],
                    valinit=init_wxt,orientation="vertical")

# Make a vertical slider to control the future number of steps.
axwxc = fig.add_axes([0.15, 0.3, 0.0225, 0.6])
wxc_slider = Slider(ax=axwxc,label='wxc',valmin=wxc_range[0],valmax=wxc_range[1],
                    valinit=init_wxc,orientation="vertical")

# Make a vertical slider to control the future number of steps.
axwu = fig.add_axes([0.225, 0.3, 0.0225, 0.6])
wu_slider = Slider( ax=axwu,label='wu',valmin=wu_range[0],
                    valmax=wu_range[1],valinit=init_wu,orientation="vertical")

# Make a vertical slider to control the future number of steps.
axi = fig.add_axes([0.3, 0.3, 0.0225, 0.6])
i_slider = Slider(  ax=axi,label='i',valmin=i_range[0],valmax=i_range[1],
                    valinit=init_i,valstep=1,orientation="vertical")


# The function to be called anytime a slider's value changes
def update(val):
    new_t = np.arange(dt*i_slider.val,dt*(i_slider.val+N_slider.val+1-1e-6),dt)
    predicted = f(  i_slider.val, N_slider.val, np.array([10**(wxt_slider.val),10**(wxt_slider.val),0,0,0]), 
                    np.array([10**(wxc_slider.val),10**(wxc_slider.val),0,0,0]), 10**(wu_slider.val),
                    np.array([Sx0_slider.val]),np.array([Su_slider.val]))
    reference = ref(i_slider.val,N_slider.val)

    line0.set_xdata(new_t)
    line0.set_ydata(predicted[0,:])
    scatter0.set_offsets(np.c_[new_t[:],reference[0,:]])
    axs[0].set_xlim(np.min(new_t),np.max(new_t))
    axs[0].set_ylim(np.min(np.append(predicted[0,:],reference[0,:])),np.max(np.append(predicted[0,:],reference[0,:])))

    line1.set_xdata(new_t)
    line1.set_ydata(predicted[1,:])
    scatter1.set_offsets(np.c_[new_t[:],reference[1,:]])
    axs[1].set_xlim(np.min(new_t),np.max(new_t))
    axs[1].set_ylim(np.min(np.append(predicted[1,:],reference[1,:])),np.max(np.append(predicted[1,:],reference[1,:])))
    
    fig.canvas.draw_idle()


# register the update function with each slider
N_slider.on_changed(update)
Sx0_slider.on_changed(update)
Sx1_slider.on_changed(update)
Su_slider.on_changed(update)
wxt_slider.on_changed(update)
wxc_slider.on_changed(update)
wu_slider.on_changed(update)
i_slider.on_changed(update)

# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = fig.add_axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetax, 'Reset', hovercolor='0.975')


def reset(event):
    N_slider.reset()
    Sx0_slider.reset()
    Sx1_slider.reset()
    Su_slider.reset()
    wxt_slider.reset()
    wxc_slider.reset()
    wu_slider.reset()
    i_slider.reset()
button.on_clicked(reset)

plt.show()
