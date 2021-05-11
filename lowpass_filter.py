from scipy.signal import filtfilt
from scipy import stats
import numpy as np
import matplotlib.pyplot as plt
import scipy


def bandpassfilter(signal):
	fs = 25.0
	lowcut = 2
	#highcut = 50.0

	nyq = 0.5*fs
	low = lowcut / nyq
	#high = highcut / nyq

	order = 6
	b,a = scipy.signal.butter(order, low, btype='low', analog=False)
	y = scipy.signal.filtfilt(b,a,signal, axis=0)

	return y



if __name__ == "__main__":
	
	data = np.load('sensor_data_1.npy')
	num = data.size
	print(num)
	print(data)
	data = data[20:num-20]

	time = np.linspace(0,num-40,num-40)

	filtered_data = bandpassfilter(data)

	fig = plt.figure()
	ax1 = fig.add_subplot(2,1,1)
	ax2 = fig.add_subplot(2,1,2)

	ax1.plot(time,data)
	ax2.plot(time,filtered_data)
	plt.show()