#!/usr/bin/env python
import zmq
import os
import sys
import numpy as np
import re
import time
import tensorflow as tf
from baselines import logger
from baselines.ppo1 import pposgd_ode
from matplotlib import pyplot as plt

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"  # disable Tensorflow GPU usage, these simple graphs run faster on CPU
tf.set_random_seed(int(time.time()))
np.random.seed(int(time.time()))
np.set_printoptions(threshold=sys.maxsize)  # Tell numpy not to summarize big arrays!


def main():
	save_path = logger.configure()

	print('Python: Setting server up...')
	context = zmq.Context()
	socket = context.socket(zmq.REP)
	socket.bind("tcp://*:{0}".format(5555))
	print('Python: Server set up successful.')

	plot = True
	interpolate = None
	data_per_iter = None

	state_dim = action_dim = action_min = action_max = None

	running = True
	rendering_policy = False
	training_completed = False

	evaluate_policy_every = 25
	render_policy_epochs = [0, 25, 50, 250, 500, 1000, 2000, 3500]

	ppo = nTrajectories = obs = rews = costs = vpreds = news = acs = eps_done = eps_len = ax1 = ax2 = None
	max_iters_ppo = n_data = tstart = 0
	_obs = _vpreds = _acs = _rews = _news = None
	_rews_accumulated = _costs_accumulated = []

	summary_ep = []
	summary_ts = []
	summary_mean_rews = []
	summary_std_low_rews = []
	summary_std_high_rews = []
	summary_min_ep_lens = []
	summary_mean_ep_lens = []
	summary_std_low_ep_lens = []
	summary_std_high_ep_lens = []
	summary_max_ep_lens = []
	iteration_ep_lens = np.zeros(0)

	if plot:
		plt.ion()
		fig = plt.figure()
		ax1 = fig.add_subplot(211)
		# ax1.set_title('Accumulated Reward')
		ax1.set_xlabel('Steps')
		ax1.set_ylabel('Accumulated Reward')
		ax1.ticklabel_format(style='sci', scilimits=(-3, 3), axis='both')
		ax2 = fig.add_subplot(212)
		# ax2.set_title('Episode Length')
		ax2.set_xlabel('Epochs')
		ax2.set_ylabel('Episode Length')
	while running:
		# print('Python: Listening...')
		message = socket.recv().decode("utf-8")
		# print('Python: Received request: %s' % message)

		msg_id = message[0]
		msg_body = message[1:-1]  # Last character is NULL
		if len(msg_body) > 0 and msg_body[0] == '|':
			msg_body = msg_body[1:]
		if len(msg_body) > 0 and msg_body[-1] == '|':
			msg_body = msg_body[:-1]
		reply = 'Empty reply'

		if msg_id == 'm':
			action_min = np.array(re.split('\|', msg_body), dtype=np.float32)
		elif msg_id == 'M':
			action_max = np.array(re.split('\|', msg_body), dtype=np.float32)
		elif msg_id == 'I':
			# Initialize
			options = np.array(re.split('\|', msg_body), dtype=np.int32)
			state_dim = int(options[0])
			action_dim = int(options[1])
			nTrajectories = int(options[2])
			max_ep_len_ppo = int(options[3])
			max_iters_ppo = int(options[4])

			interpolate = True
			data_per_iter = 4096

			ppo = pposgd_ode.ppo_learner(
				state_dim=state_dim
				, action_min=np.zeros_like(action_min) if interpolate else action_min
				, action_max=np.ones_like(action_max) if interpolate else action_max
				, clip_param=0.2
				, entcoeff=0.0
				, optim_epochs=25
				, optim_stepsize=1e-4
				, optim_batchsize=256
				, max_iters_ppo=max_iters_ppo
				, gamma=0.99
				, lam=0.95
				, interpolate=interpolate
				, hid_size=[64, 64]
				, activation='tanh'
				, schedule='linear'
			)

			tstart = time.time()

			obs = np.zeros((nTrajectories, max_ep_len_ppo, state_dim))
			rews = np.zeros((nTrajectories, max_ep_len_ppo))
			costs = np.zeros((nTrajectories, max_ep_len_ppo))
			vpreds = np.zeros((nTrajectories, max_ep_len_ppo))
			news = np.zeros((nTrajectories, max_ep_len_ppo))
			acs = np.zeros((nTrajectories, max_ep_len_ppo, action_dim))

			_obs = np.zeros((0, state_dim))
			_vpreds = np.zeros(0)
			_acs = np.zeros((0, action_dim))
			_rews = np.zeros(0)
			_news = np.zeros(0)

			eps_len = np.zeros(nTrajectories, dtype=np.int32)
		elif msg_id == 'D':
			# Report whether each episode has been done (=1) or not (=0)
			eps_done = np.array(re.split('\|', msg_body), dtype=np.float32)
			for i in range(nTrajectories):
				news[i, eps_len[i]] = 0 if eps_done[i] < 0.5 else 1
		elif msg_id == 'A':
			# Query the network for actions
			state_vector = np.array(re.split('\|', msg_body), dtype=np.float32)
			state_vector = np.reshape(state_vector, (-1, state_dim))
			ac, vpred = ppo.pi.act_batch(stochastic=not training_completed and not rendering_policy, ob=state_vector)

			if interpolate:
				ac_to_send = action_min + (np.copy(ac) + 1) / 2 * (action_max - action_min)
			else:
				ac_to_send = np.clip(a=ac, a_min=action_min, a_max=action_max)
			reply = np.array2string(ac_to_send.flatten(), formatter={'float': lambda x: "%.5f" % x}
									, precision=5, separator='|', max_line_width=100000)[1:-1]
			reply = reply.replace('[', '')
			reply = reply.replace(']', '')

			if not training_completed and not rendering_policy:
				j = 0
				for i in range(nTrajectories):
					if eps_done[i] < 0.5:
						obs[i, eps_len[i], :] = state_vector[j, :]
						vpreds[i, eps_len[i]] = vpred[j]
						acs[i, eps_len[i], :] = ac[j, :]
						j += 1
		elif msg_id == 'R':
			# Rewards
			rewards_costs = np.array(re.split('\|', msg_body), dtype=np.float32)
			j = 0
			for i in range(nTrajectories):
				if eps_done[i] < 0.5:
					rews[i, eps_len[i]] = rewards_costs[2 * j]
					costs[i, eps_len[i]] = rewards_costs[2 * j + 1]
					eps_len[i] += 1
					j += 1
		elif msg_id == '1':
			# Evaluating finished, switch to rendering or training mode
			if training_completed or ppo.iters_so_far in render_policy_epochs:
				reply = 'rendering'
				rendering_policy = True
			else:
				reply = 'training'

			_rews_accumulated = []
			_costs_accumulated = []

			for i in range(nTrajectories):
				_rews_accumulated.append(np.sum(rews[i, :eps_len[i]]))
				_costs_accumulated.append(np.sum(costs[i, :eps_len[i]]))

			summary_ep.append(ppo.iters_so_far)
			summary_ts.append(ppo.timesteps_so_far)

			cur_min_ep_len = np.min(eps_len)
			cur_mean_ep_len = np.mean(eps_len)
			cur_max_ep_len = np.max(eps_len)

			_mean = np.mean(_rews_accumulated)
			summary_mean_rews.append(_mean)
			_std = np.std(_rews_accumulated)
			summary_std_low_rews.append(max(_mean - _std / 2, np.min(_rews_accumulated)))
			summary_std_high_rews.append(min(_mean + _std / 2, np.max(_rews_accumulated)))

			summary_min_ep_lens.append(cur_min_ep_len)
			_mean = np.mean(eps_len)
			summary_mean_ep_lens.append(_mean)
			_std = np.std(eps_len)
			summary_std_low_ep_lens.append(max(_mean - _std / 2, cur_min_ep_len))
			summary_std_high_ep_lens.append(min(_mean + _std / 2, cur_max_ep_len))
			summary_max_ep_lens.append(cur_max_ep_len)

			logger.record_tabular("Iteration", ppo.iters_so_far)
			logger.record_tabular("AccumulatedRewMean", np.mean(_rews_accumulated))
			logger.record_tabular("AccumulatedRewStd", np.std(_rews_accumulated))
			logger.record_tabular("AccumulatedCostMean", np.mean(_costs_accumulated))
			logger.record_tabular("AccumulatedCostStd", np.std(_costs_accumulated))
			logger.record_tabular("MinEpLen", cur_min_ep_len)
			logger.record_tabular("MeanEpLen", cur_mean_ep_len)
			logger.record_tabular("MaxEpLen", cur_max_ep_len)
			logger.record_tabular("TimestepsSoFar", ppo.timesteps_so_far)
			logger.record_tabular("TimeElapsed", time.time() - tstart)
			logger.dump_tabular()

			if plot and ppo.iters_so_far % 50 == 0:
				# Mean reward
				ax1.plot(summary_ts, summary_mean_rews, color='b', linewidth=0.25, alpha=0.75)
				ax1.fill_between(summary_ts, summary_std_low_rews, summary_std_high_rews
								 , color='c', alpha=0.05)
				ax1.legend(loc='best')
				# Min episode length
				ax2.plot(summary_ep, summary_min_ep_lens, color='r', linewidth=0.25, alpha=0.75)
				# Mean episode length
				ax2.plot(summary_ep, summary_mean_ep_lens, color='b', linewidth=0.25, alpha=0.75)
				ax2.fill_between(summary_ep, summary_std_low_ep_lens, summary_std_high_ep_lens
								 , color='c', alpha=0.05)
				# Max episode length
				ax2.plot(summary_ep, summary_max_ep_lens, color='g', linewidth=0.25, alpha=0.75)
				ax2.legend(loc='best')
				plt.pause(0.001)

			_obs = np.zeros((0, state_dim))
			_vpreds = np.zeros(0)
			_acs = np.zeros((0, action_dim))
			_rews = np.zeros(0)
			_news = np.zeros(0)
			_rews_accumulated = []
			_costs_accumulated = []
			eps_len[:] = 0
			n_data = 0
			iteration_ep_lens = np.zeros(0)
		elif msg_id == '2':
			# Rendering finished, switch to training mode
			rendering_policy = False

			_obs = np.zeros((0, state_dim))
			_vpreds = np.zeros(0)
			_acs = np.zeros((0, action_dim))
			_rews = np.zeros(0)
			_news = np.zeros(0)
			_rews_accumulated = []
			_costs_accumulated = []
			eps_len[:] = 0
			n_data = 0
			iteration_ep_lens = np.zeros(0)
		elif msg_id == 'T':
			# Train
			accepted_ep = 0
			base = n_data
			for i in range(nTrajectories):
				base += eps_len[i]
				accepted_ep += 1
				if base > data_per_iter:
					break

			n_time_steps = np.sum(eps_len[:accepted_ep])
			_obs = np.append(_obs, np.zeros((n_time_steps, state_dim)), axis=0)
			_vpreds = np.append(_vpreds, np.zeros(n_time_steps), axis=0)
			_acs = np.append(_acs, np.zeros((n_time_steps, action_dim)), axis=0)
			_rews = np.append(_rews, np.zeros(n_time_steps), axis=0)
			_news = np.append(_news, np.zeros(n_time_steps), axis=0)

			base = n_data
			for i in range(accepted_ep):
				_start = base
				_news[_start] = 1
				_end = _start + eps_len[i]
				_obs[_start:_end, :] = obs[i, :eps_len[i], :]
				_vpreds[_start:_end] = vpreds[i, :eps_len[i]]
				_acs[_start:_end, :] = acs[i, :eps_len[i], :]
				_rews[_start:_end] = rews[i, :eps_len[i]]
				_rews_accumulated.append(np.sum(rews[i, :eps_len[i]]))
				base += eps_len[i]
			iteration_ep_lens = np.concatenate((iteration_ep_lens, eps_len[:accepted_ep]))
			n_data += np.sum(eps_len[:accepted_ep])
			eps_len[:] = 0
			assert base == n_data
			reply = 'skip'
			if n_data > data_per_iter:
				ppo.train({
					"ob": _obs
					, "vpred": _vpreds
					, "ac": _acs
					, "rew": _rews
					, "new": _news
					, "nextvpred": 0
					, "rewAccumulated": _rews_accumulated
				})

				if ppo.iters_so_far >= max_iters_ppo:
					ppo.save(os.path.join(save_path, 'model'))
					training_completed = True
					print('Training stopped after %d PPO iterations.' % ppo.iters_so_far)

				if training_completed:
					reply = 'final_evaluating'
				elif ppo.iters_so_far % evaluate_policy_every == 0:
					reply = 'evaluating'
				elif ppo.iters_so_far in render_policy_epochs:
					reply = 'rendering'
					rendering_policy = True
				else:
					reply = 'trained'

				_obs = np.zeros((0, state_dim))
				_vpreds = np.zeros(0)
				_acs = np.zeros((0, action_dim))
				_rews = np.zeros(0)
				_news = np.zeros(0)
				_rews_accumulated = []
				_costs_accumulated = []
				n_data = 0
				iteration_ep_lens = np.zeros(0)
		elif msg_id == 'E':
			# End
			running = False
			reply = 'End'
		socket.send_string(reply + '\0', encoding='utf-8')


if __name__ == '__main__':
	main()
