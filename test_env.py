import numpy as np

# 2D
id_2D = [
'Reacher-v2',  #0
'Pusher-v2',
'Thrower-v2',
'Striker-v2',
'InvertedPendulum-v2',
'InvertedDoublePendulum-v2',  #5
'HalfCheetah-v2',
'HalfCheetah-v3',
'Hopper-v2',
'Hopper-v3',
'Swimmer-v2', #10
'Swimmer-v3',
'Walker2d-v2',
'Walker2d-v3',
'Ant-v2',
'Ant-v3',  #15
'Humanoid-v2',
'Humanoid-v3',
'HumanoidStandup-v2']

# robotics
id_robotics = [
'FetchSlide-v1',                                #0
'FetchPickAndPlace-v1',
'FetchReach-v1',
'FetchPush-v1',
'HandReach-v0',
'HandManipulateBlockRotateZ-v0',                #5
'HandManipulateBlockRotateZTouchSensors-v0',
'HandManipulateBlockRotateZTouchSensors-v1',
'HandManipulateBlockRotateParallel-v0',
'HandManipulateBlockRotateParallelTouchSensors-v0',
'HandManipulateBlockRotateParallelTouchSensors-v1', #10
'HandManipulateBlockRotateXYZ-v0',
'HandManipulateBlockRotateXYZTouchSensors-v0',
'HandManipulateBlockRotateXYZTouchSensors-v1',
'HandManipulateBlockFull-v0',
'HandManipulateBlock-v0',                       #15
'HandManipulateBlockTouchSensors-v0',
'HandManipulateBlockTouchSensors-v1',
'HandManipulateEggRotate-v0',
'HandManipulateEggRotateTouchSensors-v0',
'HandManipulateEggRotateTouchSensors-v1',   #20
'HandManipulateEggFull-v0',
'HandManipulateEgg-v0',
'HandManipulateEggTouchSensors-v0',
'HandManipulateEggTouchSensors-v1',
'HandManipulatePenRotate-v0',   #25
'HandManipulatePenRotateTouchSensors-v0',
'HandManipulatePenRotateTouchSensors-v1',
'HandManipulatePenFull-v0',
'HandManipulatePen-v0',
'HandManipulatePenTouchSensors-v0',
'HandManipulatePenTouchSensors-v1']




def test_env():

    import gym

    # env = gym.make('Hopper-v3')
    #env = gym.make('FetchPush-v1')
    # env = gym.make('Walker2d-v2')

    id = id_2D[16]
    # id = id_robotics[14]
    print('id:',id)

    env = gym.make(id)

    high = env.env.action_space.high
    low = env.env.action_space.low
    a = env.env.action_space.sample()
    m = len(a)

    from ipdb import set_trace;
    set_trace()

    o = env.reset()                 #### reset() in mb_env.py ####
    for i in range(2000):
        a = np.zeros_like(a)
        # a = low

        for k in range(m):
            # k = m-x-1
            n = 20
            for j in range(n):
                print(a)
                for t in range(10):
                    o, r, done, env_info = env.step(a)
                    env.env.render()

                if j <= n/4: a[k] -= high[k]/n*4
                if j > n/4 and j <= 2*n/4: a[k] += high[k]/n*4
                if j > 2*n/4 and j <= 3*n/4: a[k] += high[k]/n*4
                if j > 3*n/4 and j <= n: a[k] -= high[k]/n*4


            from ipdb import set_trace;
            set_trace()

if __name__ == '__main__':
    test_env()


