
class LowPassFilter(object):
    def __init__(self, weight):
        self.weight = weight
                                            

        self.last_value = .0
        self.ready = False

    def filter(self, value):
                            

                        
        if self.ready:
            value = self.weight * value + (1.-self.weight)* self.last_value
        else:
            self.ready = True

        self.last_value = value
        return value
