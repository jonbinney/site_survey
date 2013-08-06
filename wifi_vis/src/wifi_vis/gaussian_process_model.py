from ppas import gp

class GaussianProcessModel:
    def __init__(self):
        self.kernel = gp.kernels.RBF

    def fit(self, X, Y):
        pass

    def predict(self, x):
        pass
