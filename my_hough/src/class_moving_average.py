class MovingAverage:

    def __init__(self, n):
        self.samples = n
        self.data = []
        self.weights = list(range(1, n + 1))

    def add_sample(self, new_sample):
        if len(self.data) < self.samples:
            self.data.append(new_sample)
        else:
            self.data = self.data[1:] + [new_sample]
            
    def get_sample_count(self):
        return len(self.data)
        
    # 이동평균값을 구하는 함수
    def get_mavg(self):
        return float(sum(self.data)) / len(self.data)

    # 중앙값을 사용해서 이동평균값을 구하는 함수
    def get_mmed(self):
        return float(np.median(self.data))

    # 가중치를 적용하여 이동평균값을 구하는 함수        
    def get_wmavg(self):
        s = 0
        for i, x in enumerate(self.data):
            s += x * self.weights[i]
        return float(s) / sum(self.weights[:len(self.data)])