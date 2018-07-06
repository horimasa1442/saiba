
class statistics():
    def __init__(self, ):
        self.min = 0.0;
        self.max = 0.0;
        self.avg = 0.0;
        self.stddev = 0.0;
        self.threshold = 0.0;
        self.numCorrect = 0;
        self.numTotal = 0;

    def __repr__(self):
        return self.genString()

    def saveText(self, path):
        with open(path, mode="w") as f:
            f.write(self.genString())

    def genString(self):
        res = f"min : {self.min}\r\n"
        res += f"max : {self.max}\r\n"
        res += f"avg : {self.avg}\r\n"
        res += f"stddev : {self.stddev}\r\n"
        res += f"threshold : {self.threshold}\r\n"
        res += f"numCorrect : {self.numCorrect}\r\n"
        res += f"numTotal : {self.numTotal}"
        return res