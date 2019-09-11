from baselines.common import Dataset, explained_variance, zipsame
from baselines import logger
import baselines.common.tf_util as U
import tensorflow as tf, numpy as np
import time
from baselines.common.mpi_adam import MpiAdam
from baselines.common.mpi_moments import mpi_moments
from gym import spaces


class ppo_learner:
    def __init__(self
                 , state_dim
                 , action_min, action_max
                 , clip_param, entcoeff  # clipping parameter epsilon, entropy coeff
                 , optim_epochs, optim_stepsize, optim_batchsize  # optimization hypers
                 , gamma, lam  # advantage estimation
                 , max_iters_ppo=5000
                 , adam_epsilon=1e-5
                 , schedule='constant'  # annealing for stepsize parameters (epsilon and adam)
                 , interpolate=False
                 , hid_size=None
                 , activation='tanh'
                 ):
        from baselines.ppo1 import mlp_policy
        U.make_session(num_cpu=1).__enter__()
        def policy_fn(name, ob_space, ac_space):
            return mlp_policy.MlpPolicy(name=name
                                        , ob_space=ob_space
                                        , ac_space=ac_space
                                        , hid_size=hid_size
                                        , activation=activation
                                        , interpolate=interpolate
                                        )

        high = 100 * np.ones(state_dim)
        low = -high
        self.ob_space = spaces.Box(low=low, high=high, dtype=np.float32)

        self.ac_space = spaces.Box(low=action_min, high=action_max, dtype=np.float32)

        self.pi = policy_fn("pi", self.ob_space, self.ac_space)  # Construct network for new policy
        self.oldpi = policy_fn("oldpi", self.ob_space, self.ac_space)  # Network for old policy

        self.atarg = tf.placeholder(dtype=tf.float32, shape=[None])  # Target advantage function (if applicable)
        self.ret = tf.placeholder(dtype=tf.float32, shape=[None])  # Empirical return

        self.lrmult = tf.placeholder(name='lrmult', dtype=tf.float32
                                     , shape=[])  # learning rate multiplier, updated with schedule
        clip_param = clip_param * self.lrmult  # Annealed cliping parameter epislon

        ob = U.get_placeholder_cached(name="ob")
        ac = self.pi.pdtype.sample_placeholder([None])

        kloldnew = self.oldpi.pd.kl(self.pi.pd)
        ent = self.pi.pd.entropy()
        meankl = tf.reduce_mean(kloldnew)
        meanent = tf.reduce_mean(ent)
        pol_entpen = (-entcoeff) * meanent

        ratio = tf.exp(self.pi.pd.logp(ac) - self.oldpi.pd.logp(ac))  # pnew / pold
        surr1 = ratio * self.atarg  # surrogate from conservative policy iteration
        surr2 = tf.clip_by_value(ratio, 1.0 - clip_param, 1.0 + clip_param) * self.atarg  #
        pol_surr = -tf.reduce_mean(tf.minimum(surr1, surr2))  # PPO's pessimistic surrogate (L^CLIP)
        vf_loss = tf.reduce_mean(tf.square(self.pi.vpred - self.ret))
        total_loss = pol_surr + pol_entpen + vf_loss
        losses = [pol_surr, pol_entpen, vf_loss, meankl, meanent]
        self.loss_names = ["pol_surr", "pol_entpen", "vf_loss", "kl", "ent"]

        var_list = self.pi.get_trainable_variables()
        self.lossandgrad = U.function([ob, ac, self.atarg, self.ret, self.lrmult]
                                 , losses + [U.flatgrad(total_loss, var_list)])
        self.adam = MpiAdam(var_list, epsilon=adam_epsilon)

        self.assign_old_eq_new = U.function([], [], updates=[tf.assign(oldv, newv)
                                                        for (oldv, newv) in
                                                        zipsame(self.oldpi.get_variables(), self.pi.get_variables())])
        self.compute_losses = U.function([ob, ac, self.atarg, self.ret, self.lrmult], losses)

        U.initialize()
        self.adam.sync()

        self.episodes_so_far = 0
        self.timesteps_so_far = 0
        self.iters_so_far = 0

        self.gamma = gamma
        self.lam = lam
        self.optim_epochs = optim_epochs
        self.optim_stepsize = optim_stepsize
        self.optim_batchsize = optim_batchsize
        self.max_iters_ppo = max_iters_ppo
        self.schedule = schedule

    def save(self, model_path):
        U.save_state(model_path)

    def load(self, model_path):
        U.load_state(model_path)

    def train(self, seg):
        # if callback: callback(locals(), globals())
        if self.schedule == 'constant':
            cur_lrmult = 1.0
        elif self.schedule == 'linear':
            cur_lrmult = max(1.0 - float(self.iters_so_far) / self.max_iters_ppo, 1e-3)
        else:
            raise NotImplementedError

        self.iters_so_far += 1
        logger.log("********** Iteration %i ************" % self.iters_so_far)

        add_vtarg_and_adv(seg, self.gamma, self.lam)

        # ob, ac, atarg, ret, td1ret = map(np.concatenate, (obs, acs, atargs, rets, td1rets))
        ob, ac, atarg, tdlamret = seg["ob"], seg["ac"], seg["adv"], seg["tdlamret"]
        vpredbefore = seg["vpred"]  # predicted value function before udpate
        atarg = (atarg - atarg.mean()) / atarg.std()  # standardized advantage function estimate
        d = Dataset(dict(ob=ob, ac=ac, atarg=atarg, vtarg=tdlamret), shuffle=True)
        optim_batchsize = min(self.optim_batchsize, ob.shape[0])

        if hasattr(self.pi, "ob_rms"): self.pi.ob_rms.update(ob)  # update running mean/std for policy

        self.assign_old_eq_new()  # set old parameter values to new parameter values
        # logger.log("Optimizing...")
        # logger.log(fmt_row(13, self.loss_names))
        # Here we do a bunch of optimization epochs over the data
        for _ in range(self.optim_epochs):
            losses = []  # list of tuples, each of which gives the loss for a minibatch
            for batch in d.iterate_once(optim_batchsize):
                *newlosses, g = self.lossandgrad(batch["ob"], batch["ac"], batch["atarg"], batch["vtarg"], cur_lrmult)
                self.adam.update(g, self.optim_stepsize * cur_lrmult)
                losses.append(newlosses)
            # logger.log(fmt_row(13, np.mean(losses, axis=0)))

        # logger.log("Evaluating losses...")
        losses = []
        for batch in d.iterate_once(optim_batchsize):
            newlosses = self.compute_losses(batch["ob"], batch["ac"], batch["atarg"], batch["vtarg"], cur_lrmult)
            losses.append(newlosses)
        meanlosses, _, _ = mpi_moments(losses, axis=0)

        n_data = seg["ob"].shape[0]
        self.timesteps_so_far += n_data
        for (lossval, name) in zipsame(meanlosses, self.loss_names):
            logger.log("loss_" + name + ": %f" % lossval)
        logger.log("ev_tdlam_before: %f" % explained_variance(vpredbefore, tdlamret))
        logger.log("EpisodesThisIter: %d" % len(seg["rewAccumulated"]))
        logger.log("TimestepsThisIter: %d" % n_data)
        logger.log("TimestepsSoFar: %d" % self.timesteps_so_far)
        logger.log("LearningRate: %f" % (self.optim_stepsize * cur_lrmult))

def add_vtarg_and_adv(seg, gamma, lam):
    """
    Compute target value using TD(lambda) estimator, and advantage with GAE(lambda)
    """
    new = np.append(seg["new"], 0) # last element is only used for last vtarg, but we already zeroed it if last new = 1
    vpred = np.append(seg["vpred"], seg["nextvpred"])
    T = len(seg["rew"])
    seg["adv"] = gaelam = np.empty(T, 'float32')
    rew = seg["rew"]
    lastgaelam = 0
    for t in reversed(range(T)):
        nonterminal = 1-new[t+1]
        delta = rew[t] + gamma * vpred[t+1] * nonterminal - vpred[t]
        gaelam[t] = lastgaelam = delta + gamma * lam * nonterminal * lastgaelam
    seg["tdlamret"] = seg["adv"] + seg["vpred"]
