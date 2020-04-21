import numpy as np
import matplotlib.pyplot as plt

# reward weights
r2 = [0.0, 0.3, 0.8]
r2 = [0.3]

r11 = 1.5

x1 = np.arange(0.0, 0.26, 0.001)


# ln loss
ln_loss = -np.log(50*x1+0.1) + np.log(0.8)
ln_loss *= r11

# obstacle penalty
offset = 0.005
beta = 5.0
x2 = np.arange(0.01, 0.1,0.001)

#plt.plot(x1, huber)
# plt.plot(x1, ln_loss)
for coff in r2:
	obs_p = -coff / (beta * x2 + offset)
	plt.plot(x2, obs_p, lw=2)


#plt.ylim([-5.0, 0])
# plt.xlim([0.008, 0.28])
plt.xlabel('Minimal value of the laser data')
plt.ylabel('Reward')
plt.title('Reward function for the laser data')
plt.show()
