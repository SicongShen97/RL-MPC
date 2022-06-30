import tensorflow as tf
from train import train

if __name__ == '__main__':
    tf.compat.v1.disable_eager_execution()
    # After eager execution is enabled, operations are executed as they are
    # defined and Tensor objects hold concrete values, which can be accessed as
    # numpy.ndarray`s through the numpy() method.
    train()
