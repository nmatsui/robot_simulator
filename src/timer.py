import json
import signal
import time

import zmq


INTERVAL = 0.2
LIFETIME = 3600
PORT = 5556


def start(ekf):
    """
    Parameters:
    ----------
    ekf: src.filter.EKF
        an instance of EKF

    Notes:
    ----------
    execute EKF at specified intervals and sends estimated pose by using ZMQ
    """

    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.bind(f'tcp://*:{PORT}')

    def handler(signum, frame):
        ideal, xhat, P, K = ekf.step()

        msg = json.dumps({
            'ideal': {
                'x': ideal[0],
                'y': ideal[1],
                'theta': ideal[2],
            },
            'actual': {
                'x': ekf.agent.actual[0],
                'y': ekf.agent.actual[1],
                'theta': ekf.agent.actual[2],
            },
            'xhat': {
                'x': xhat[0],
                'y': xhat[1],
                'theta': xhat[2],
            },
            'observed': [{
                'landmark': {
                    'x': o[0][0],
                    'y': o[0][1],
                },
                'distance': o[1][0],
                'angle': o[1][1],
            } for o in ekf.agent.observed_list],
            'covariance': P.flatten().tolist(),
            'kalmanGain': K.flatten().tolist(),
        })
        publisher.send(msg.encode('utf-8'))
        print(f'send msg: {msg}')

    signal.signal(signal.SIGALRM, handler)
    signal.setitimer(signal.ITIMER_REAL, INTERVAL, INTERVAL)

    time.sleep(LIFETIME)
