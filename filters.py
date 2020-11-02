MEAN_FILTER_SIZE = 15

class DCFilter:
    def __init__(self):
        self.w = 0.0
        self.result = 0.0
    
    def dc_removal(self, raw_data, alpha =0.95):
        prev_w = self.w
        self.w = raw_data + alpha * prev_w
        self.result = self.w - prev_w
        return self.result

class MeanDiffFilter:
    def __init__(self):
        self.values = [0] * MEAN_FILTER_SIZE
        self.index = 0
        self.sum = 0
        self.count = 0

    def mean_diff(self, m):
        avg = 0
        self.sum -= self.values[self.index]
        self.values[self.index] = m
        self.sum += self.values[self.index]

        self.index += 1
        self.index = self.index % MEAN_FILTER_SIZE

        if (self.count < MEAN_FILTER_SIZE):
            self.count += 1

        avg = self.sum/self.count
        return avg - m

class ButterworthFilter:
    def __init__(self):
        self.v = [0, 0]
        self.result = 0.0

    def lpb(self, raw_data):
        self.v[0] = self.v[1]
        self.v[1] =  (2.452372752527856026e-1 * raw_data) + (0.50952544949442879485 * self.v[0])
        self.result = self.v[0] + self.v[1]
        return self.result

