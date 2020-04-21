import numpy as np
import matplotlib.pyplot as plt

D = [90, 100, 110, 120]
models = ['00', '02', '06', '04', '08']

labels = ['$C_2=0.0$', '$C_2=0.2$','$C_2=0.4$', '$C_2=0.6$', '$C_2=0.8$']
colors  = ['b','r','g','c','k']
markers = ['o', 'v', 's', 'p', '*']

def draw_plot(file_name, marker, color, label):
    data = np.loadtxt(file_name)
    data = np.asarray(data) * 100
    angles = range(55, 126, 5)
    plt.plot(angles, data, marker = marker, color = color, label = label)

i = 1
for d in D:
    f = plt.figure(i)
    for model, label, color, marker in zip(models, labels, colors, markers):
        file_name = 'success_rate_%s_%s.txt' % (model, d)
        draw_plot(file_name, marker, color, label)

    plt.rcParams.update({'font.size': 12})
    plt.legend()
    ax = plt.subplot(111)
    for item in ([ax.title, ax.xaxis.label, ax.yaxis.label] +
                 ax.get_xticklabels() + ax.get_yticklabels()):
        item.set_fontsize(12)

    ax.set_ylabel('Success rate %')
    ax.set_xlabel('Angles')
    ax.set_title('Success rate of turning within %smm pipes' % d)
    f.show()
    i+=1


print 'Press any key to quit'
x = input()
