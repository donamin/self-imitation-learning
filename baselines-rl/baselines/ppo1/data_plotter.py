from __future__ import division, print_function
from matplotlib import pyplot as plt
import numpy as np
import matplotlib
import os

# Set some parameters to apply to all plots. These can be overridden
# in each plot if desired
# Plot size to 14" x 7"
matplotlib.rc('figure', figsize=(14, 7))
# Font size to 14
matplotlib.rc('font', size=14)
# Do not display top and right frame lines
matplotlib.rc('axes.spines', top=False, right=False)
# Remove grid lines
matplotlib.rc('axes', grid=False)
# Set background color to white
matplotlib.rc('axes', facecolor='white')

characters_list = ['Orc', 'Wolf', 'Mech']

plot_cost = False
constant_threshold = False

for character in characters_list:
	if plot_cost:
		data_series = [
			{
				'title': character + ' (Reward + Imitation)',
				'color': 'r',
				'draw': False,
				'main_dir': [character, 'NC'],
				'dir_list': ['1', '2', '3', '4', '5']
			}
			,
			{
				'title': character + ' (Reward)',
				'color': 'm',
				'draw': False,
				'main_dir': [character, 'Reward'],
				'dir_list': ['1', '2', '3', '4', '5']
			}
			,
			{
				'title': 'Using FDI-MCTS cost function as reward',#character + ' (Cost)',
				'color': '#913844',
				'draw': True,
				'main_dir': [character, 'NegCost'],
				'dir_list': ['1', '2', '3', '4', '5']
			}
			,
			{
				'title': character + ' (Cost + Imitation)',
				'color': 'c',
				'draw': False,
				'main_dir': [character, 'NegCost+Imitation'],
				'dir_list': ['1', '2', '3', '4', '5']
			}
			,
			{
				'title': character + ' (Cost + Imitation, 75-50)',
				'color': 'y',
				'draw': False,
				'main_dir': [character, 'NegCost+Imitation-75-50'],
				'dir_list': ['1', '2', '3', '4', '5']
			}
			,
			{
				'title': 'Using DeepMimic-style reward and termination curriculum (ours)',#character + ' (75-50)',
				'color': 'g',
				'draw': True,
				'main_dir': [character, '75-50'],
				'dir_list': ['1', '2', '3', '4', '5']
			}
		]
	elif constant_threshold:
		data_series = [
			{
				'title': character + '-Extra-Tight Threshold (0.85)',
				'color': 'c',
				'draw': True,
				'main_dir': [character, '85'],
				'dir_list': ['1', '2', '3', '4', '5']
			}
			,
			{
				'title': character + '-Tight Threshold (0.75)',
				'color': '#212cd1',
				'draw': True,
				'main_dir': [character, '75'],
				'dir_list': ['1', '2', '3', '4', '5']
			}
			,
			{
				'title': character + '-Medium Threshold (0.5)',
				'color': 'm',
				'draw': True,
				'main_dir': [character, '50'],
				'dir_list': ['1', '2', '3', '4', '5']
			}
			,
			{
				'title': character + '-Loose Threshold (0.25)',
				'color': '#913844',
				'draw': True,
				'main_dir': [character, '25'],
				'dir_list': ['1', '2', '3', '4', '5']
			}
			,
			{
				'title': character + '-No Curriculum',
				'color': 'r',
				'draw': True,
				'main_dir': [character, 'NC'],
				'dir_list': ['1', '2', '3', '4', '5']
			}
		]
	else:
		data_series = [
			{
				'title': 'Termination curriculum (ours)',#character + ' (75-50)',
				'color': 'g',
				'draw': True,
				'main_dir': [character, '75-50'],
				'dir_list': ['1', '2', '3', '4', '5']
			}
			,
			{
				'title': character + ' (85-50)',
				'color': '#5d258e',
				'draw': False,
				'main_dir': [character, '85-50'],
				'dir_list': ['1', '2', '3', '4', '5']
			}
			,
			{
				'title': character + '-Tight Threshold (0.85)',
				'color': 'c',
				'draw': False,
				'main_dir': [character, '85'],
				'dir_list': ['1', '2', '3', '4', '5']
			}
			,
			{
				'title': 'Tight threshold',#character + '-Tight Threshold (0.75)',
				'color': '#212cd1',
				'draw': True,
				'main_dir': [character, '75'],
				'dir_list': ['1', '2', '3', '4', '5']
			}
			,
			{
				'title': 'Medium threshold',#character + '-Medium Threshold (0.5)',
				'color': 'm',
				'draw': True,
				'main_dir': [character, '50'],
				'dir_list': ['1', '2', '3', '4', '5']
			}
			,
			{
				'title': 'Loose threshold',#character + '-Loose Threshold (0.25)',
				'color': '#913844',
				'draw': True,
				'main_dir': [character, '25'],
				'dir_list': ['1', '2', '3', '4', '5']
			}
			,
			{
				'title': 'No termination',#character + '-No Curriculum',
				'color': 'r',
				'draw': True,
				'main_dir': [character, 'NC'],
				'dir_list': ['1', '2', '3', '4', '5']
			}
		]

	base_dir = os.path.join(os.getcwd(), 'Results')

	fig = plt.figure()
	ax1 = fig.add_subplot(111)
	ax1.set_xlabel('Timesteps')
	ax1.set_ylabel('Average Cost' if plot_cost else 'Average Reward')
	if not plot_cost:
		ax1.set_ylim(0.0, 0.85)
	ax1.ticklabel_format(style='sci',scilimits=(-3,4),axis='both')
	# ax2 = fig.add_subplot(212)
	# ax2.set_xlabel('Timesteps')
	# ax2.set_ylabel('Average Episode Length')
	# ax2.ticklabel_format(style='sci',scilimits=(-3,4),axis='both')

	def find_indices(col_names):
		indices = {}
		for i in range(len(col_names)):
			indices[col_names[i]] = i
		return indices


	lines = []
	labels = []
	for data_pack in data_series:
		if data_pack['draw']:
			time_steps = None
			values = None
			values_col = 'AccumulatedCostMean' if plot_cost else 'AccumulatedRewMean'
			color = data_pack['color']
			main_dir = os.path.join(base_dir, *data_pack['main_dir'])

			for d in data_pack['dir_list']:
				path = os.path.join(os.path.join(main_dir, d), 'progress.csv')

				col_names = np.genfromtxt(path, max_rows=1, delimiter=',', dtype=str)
				# 'Iteration' 'MaxEpLen' 'MeanEpLen' 'MinEpLen' 'AccumulatedRewStd'
				# 'AccumulatedCostMean' 'AccumulatedRewMean' 'TimestepsSoFar' 'TimeElapsed' 'AccumulatedCostStd'
				col_indices = find_indices(col_names)
				new_data = np.genfromtxt(path, skip_header=1, delimiter=',', dtype=np.float32)

				new_values = new_data[:50, col_indices[values_col]] / new_data[:50, col_indices['MeanEpLen']]
				new_time_steps = new_data[:50, col_indices['TimestepsSoFar']]
				# print(len(new_time_steps))

				if time_steps is None:
					time_steps = new_time_steps
					values = new_values
				else:
					time_steps += new_time_steps
					values = np.row_stack((values, new_values))

			time_steps /= len(data_pack['dir_list'])
			if len(data_pack['dir_list']) > 1:
				mean = np.mean(values, axis=0)
				std = np.std(values, axis=0)
			else:
				mean = values
				std = np.zeros_like(values)
			# lines.append(ax1.plot(time_steps, mean, color=color, alpha=0.8, label=data_pack['title'])[0])
			lines.append(ax1.plot(time_steps, mean, color=color, alpha=0.8)[0])
			labels.append(data_pack['title'])
			ax1.fill_between(time_steps, mean-std/2, mean+std/2, color=color, alpha=0.2)

			'''
			if data_pack['curriculum'] and len(time_steps) > 4000:
				ax1.plot(time_steps[4499], mean[4499], 's', markersize=7.5, markerfacecolor=color,
						 markeredgewidth=2, markeredgecolor='k', alpha=0.8)
			ax2.plot(new_data[:, col_indices['TimestepsSoFar']], new_data[:, col_indices['MeanEpLen']]
					 , label=d, color=color, linewidth=0.25, alpha=1)
			ax2.legend(loc='best')
			'''

	# leg = ax1.legend(loc='best')
	# for line in leg.get_lines():
	# 	line.set_linewidth(10)
	# plt.show()
	name_prefix = ''
	if plot_cost:
		name_prefix = 'CostPlot'
	elif constant_threshold:
		name_prefix = 'ConstantThreshold'
	else:
		name_prefix = 'RewardPlot-TC'
	fig_leg = plt.figure()  # figsize=(1, 3))
	# fig_leg.legend(lines, labels=['one', 'two'], loc='center', ncol=2)
	leg = fig_leg.legend(lines, labels, loc='center', ncol=len(lines))
	for line in leg.get_lines():
		line.set_linewidth(10)
	fig_leg.savefig(os.path.join(base_dir, name_prefix + '-' + 'Legend.png'), bbox_inches='tight', pad_inches=0, dpi=200)
	fig.savefig(os.path.join(base_dir, name_prefix + '-' + character + '.png'), bbox_inches='tight', pad_inches=0, dpi=200)
