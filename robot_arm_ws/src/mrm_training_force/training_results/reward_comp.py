import numpy as np
import matplotlib.pyplot as plt

# range of distance
x1 = np.arange(0.0, 0.26, 0.001)

# linear loss 
linear_loss = -x1 

# Huber loss
delta = 0.13

huber = []
for x in x1:
	if x <= delta:
		huber.append(0.5*x**2)
	else:
		huber.append(delta*x - 0.5*delta**2)

huber = -np.asarray(huber)

# ln loss
ln_loss = -np.log(50*x1+0.1) + np.log(0.1)


# draw

fig = plt.figure()
ax = fig.add_subplot(111)

plt.plot(x1, 20*linear_loss, x1, 200*huber, x1, ln_loss)

ax.set_xlabel("Distance between ee and target",fontname="Arial", fontsize=13)
ax.set_ylabel("Reward",fontname="Arial", fontsize=13)

for label in (ax.get_xticklabels() + ax.get_yticklabels()):
	label.set_fontname('Arial')
	label.set_fontsize(12)

plt.rcParams.update({'font.size': 12})
plt.legend(['linear-loss reward function', 'huber-loss reward function', 'selected reward function'])
plt.show()

